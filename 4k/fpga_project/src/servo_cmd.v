// =====================================================================
// servo_cmd —— 串口舵机微调控制
//
// 【协议】
//   每次发 4 个 ASCII 数字：  <舵机号><角度的百位><十位><个位>
//
//     第 1 位：舵机编号，只能是 1 / 2 / 3 / 4
//     后 3 位：角度值
//
//   只有这 4 位全部收齐才执行。其他字符（回车、换行、空格）一律忽略并
//   清空计数，所以带不带 \r\n 都行，发错了也不会误动作。
//   例：  2090 -> 2 号舵机转到 90
//         4360 -> 4 号舵机转到 360
//
// 【FPGA 主动发出的三条信息】—— 调试时靠这个判断链路
//   ① 开机横幅：复位后约 500ms，发两行
//        === Tang Nano 4K Servo Ready ===
//        Send 4 digits, e.g. 2090
//      只要看到横幅，就说明「FPGA 能发 + 波特率对 + 串口线对 + 你开的串口对」。
//      看不到横幅 → 问题在 TX 侧（线/口/波特率/没烧进去），跟收无关。
//
//   ② 心跳：每 HB_MS 毫秒发一行当前四路角度
//        A 090 090 090 090
//      能看到它一秒一行，说明 FPGA 活着、TX 一直在工作。
//      想在专心看回复时关掉，把 HB_MS 设 0。
//
//   ③ 动作回复：每收到一条命令回一行
//        S2=090 A045        收到原值 090，实际转到 45 度
//        ERR                编号不是 1-4，或角度超范围
//
//   ★ 这套设计的用意：把「收不到」和「发不出」分开定位。
//     如果横幅和心跳都有、但发指令没回复 → 问题在 RX 侧（线接反/没接 GND）。
//     如果连横幅都没有 → 问题在 TX 侧或根本没烧进 FPGA。
//
// 【角度标定 —— SCALE_360】
//   ⚠️ 你的舵机是 180 度的，所以这里有个映射问题，两种解释都实现了：
//
//     SCALE_360 = 0（默认）：收到的值**直接就是**舵机角度（0..180），1 度/单位
//                            发 090 -> 舵机 90 度；发 180 -> 舵机 180 度
//                            超过 180 判非法
//                            —— 舵机是 180 度的，所以用这个
//
//     SCALE_360 = 1：收到的 0..360 线性映射到舵机 0..180
//                            即 舵机角 = 收到值 / 2，分辨率 0.5 度/单位
//                            发 180 -> 舵机 90 度；发 360 -> 舵机 180 度
//
// 【上电默认】4 路都停在中位 CENTER_DEG（默认 90 度）。
//
// 【为什么要做成 3 级流水】
//   第一版把这些全写成组合逻辑，实测 Fmax 只有 36.5MHz，关键路径是
//       raw 乘加 -> /100 -> /10 -> 拼 ASCII -> 写 msg[]
//   27MHz 的 4K 上侥幸能过，移植到 50MHz 的 138K 必然失败。拆成 3 级后
//   关键路径大幅缩短。这些运算每条指令只跑一次，多几拍延迟毫无影响。
//
// 【打包顺序】
//   angle 是 32bit：[7:0]=舵机0 [15:8]=舵机1 [23:16]=舵机2 [31:24]=舵机3
//   和 138K 主工程 top.v 里 speed_cmd 的打包习惯一致。
// =====================================================================
`timescale 1ns / 1ps

module servo_cmd #(
    parameter CLK_FREQ      = 27_000_000,
    parameter BAUD          = 115200,
    parameter CENTER_DEG    = 90,     // 上电默认角度
    parameter SCALE_360     = 0,      // 0 = 值就是角度(0..180)；1 = 0..360 映射到 0..180
    parameter HB_MS         = 1000,   // 心跳周期 ms；设 0 关闭心跳
    parameter BOOT_DELAY_MS = 500     // 开机横幅延时 ms（给电脑开串口留时间）
)(
    input  wire       Clk,
    input  wire       Rst_n,
    input  wire       rx,      // 串口接收（来自上位机）
    output wire       tx,      // 串口发送（横幅 / 心跳 / 回复）
    output reg [31:0] angle    // 4 路目标角度，每路 8bit
);

    localparam [7:0] CENTER8 = CENTER_DEG;
    localparam [9:0] RAW_MAX = SCALE_360 ? 10'd360 : 10'd180;

    localparam CR = 8'h0D;
    localparam LF = 8'h0A;

    // =================================================================
    // 消息类型
    // =================================================================
    localparam K_REPLY = 2'd0;    // 动作回复
    localparam K_HB    = 2'd1;    // 心跳
    localparam K_BOOT  = 2'd2;    // 开机横幅

    localparam BANNER_LEN = 7'd60;
    localparam HB_LEN     = 7'd19;

    // =================================================================
    // 十进制转换（0..180 -> 3 个 BCD 数字）
    //
    // 这段**故意不用乘除法器**。走过的弯路（都是实测 Fmax，别重蹈）：
    //   ① ang100 = v/100; rem = v - ang100*100; t = rem/10; o = rem - t*10;
    //      -> ang100*100 / t*10 被综合成真乘法器，DSP 4->5.25，Fmax 36.5MHz
    //   ② 百位改比较器、乘 10 改移位相加（就是下面这版）-> Fmax 57.6MHz ✅
    //   ③ 再把 /10 换成魔数乘法 (v*205)>>11
    //      -> 15 位移位加法树反而更慢，Fmax 43.3MHz ❌ 已放弃
    // =================================================================
    function [11:0] dec3(input [7:0] v);
        reg [7:0] ge, rem, t, o;
        begin
            ge  = (v >= 8'd100) ? 8'd1 : 8'd0;
            rem = (v >= 8'd100) ? (v - 8'd100) : v;      // 0..80
            t   = rem / 8'd10;                           // 0..8，常量除法
            o   = rem - ((t << 3) + (t << 1));           // 余数，移位代替乘 10
            dec3 = {ge[3:0], t[3:0], o[3:0]};
        end
    endfunction

    // =================================================================
    // 开机横幅内容（编译期常量，用 case 生成，不占寄存器）
    //   "=== Tang Nano 4K Servo Ready ===\r\n"
    //   "Send 4 digits, e.g. 2090\r\n"
    // =================================================================
    function [7:0] banner_char(input [6:0] i);
        begin
            case (i)
                // '=' : "===" 和结尾的 "==="
                7'd0, 7'd1, 7'd2, 7'd29, 7'd30, 7'd31: banner_char = "=";
                // ' '
                7'd3, 7'd8, 7'd13, 7'd16, 7'd22, 7'd28, 7'd38, 7'd40, 7'd48, 7'd53: banner_char = " ";
                7'd4:  banner_char = "T";
                7'd5, 7'd10, 7'd25:                    banner_char = "a";
                7'd6, 7'd11, 7'd36:                    banner_char = "n";
                7'd7, 7'd43, 7'd51:                    banner_char = "g";
                7'd9:  banner_char = "N";
                7'd12, 7'd21:                          banner_char = "o";
                7'd14, 7'd39:                          banner_char = "4";
                7'd15: banner_char = "K";
                7'd17, 7'd34:                          banner_char = "S";
                7'd18, 7'd24, 7'd35, 7'd49:            banner_char = "e";
                7'd19: banner_char = "r";
                7'd20: banner_char = "v";
                7'd23: banner_char = "R";
                7'd26, 7'd37, 7'd41:                   banner_char = "d";
                7'd27: banner_char = "y";
                7'd32, 7'd58:                          banner_char = CR;
                7'd33, 7'd59:                          banner_char = LF;
                7'd42, 7'd44:                          banner_char = "i";
                7'd45: banner_char = "t";
                7'd46: banner_char = "s";
                7'd47: banner_char = ",";
                7'd50, 7'd52:                          banner_char = ".";
                7'd54: banner_char = "2";
                7'd55, 7'd57:                          banner_char = "0";
                7'd56: banner_char = "9";
                default: banner_char = " ";
            endcase
        end
    endfunction

    // =================================================================
    // UART
    // =================================================================
    wire [7:0] rx_data;
    wire       rx_valid;

    uart_rx #(.CLK_FREQ(CLK_FREQ), .BAUD(BAUD)) u_rx (
        .Clk   (Clk),
        .Rst_n (Rst_n),
        .rx    (rx),
        .data  (rx_data),
        .valid (rx_valid)
    );

    reg        tx_start;
    reg  [7:0] tx_data;
    wire       tx_done;

    uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD(BAUD)) u_tx (
        .Clk   (Clk),
        .Rst_n (Rst_n),
        .start (tx_start),
        .data  (tx_data),
        .tx    (tx),
        .done  (tx_done)
    );

    // =================================================================
    // 数字接收与组帧（收满 4 位 -> fire 单拍）
    // =================================================================
    wire is_digit = (rx_data >= 8'h30) && (rx_data <= 8'h39);

    // =================================================================
    // 第 1 级：接收组帧 + 锁存 4 位数字 + 算原值 raw
    //
    // ⚠️⚠️ 锁存**必须**和移位放在同一个 always 块里，这一条是踩过大坑的：
    //
    //   原来把"移位"放组帧块、"锁存"放另一个 always 块（想做成流水线），
    //   结果锁存发生在收到第 4 位之后的**下一拍**，那时 d1/d2/d3 已经被
    //   本次移位覆盖成了第 2/3/4 位。于是：
    //        cid（舵机号） 变成 原值的百位
    //        raw  变成 d3*100 + d4*10 + d4   —— 完全是垃圾
    //   现象：拖哪一路滑块结果都一样，因为舵机号只跟角度大小的百位有关；
    //         角度 0~49 时百位=0（非法编号）整段无反应。
    //
    //   正确做法：在 fire 那一拍、用**移位前**的旧值锁存。
    //   教训：跨 always 块的"前一拍数据"不能想当然，逐周期对时序。
    // =================================================================
    reg [3:0] d0, d1, d2, d3;   // 移位寄存器；收满时 d1=第1位 d2=第2位 d3=第3位
    reg [2:0] nd;
    reg       v1;               // 第 1 级有效（原 fire）

    reg [3:0] cid, cr1, cr2, cr3;
    reg [9:0] raw_r;

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            d0 <= 4'd0; d1 <= 4'd0; d2 <= 4'd0; d3 <= 4'd0;
            nd <= 3'd0; v1 <= 1'b0;
            cid <= 4'd0; cr1 <= 4'd0; cr2 <= 4'd0; cr3 <= 4'd0;
            raw_r <= 10'd0;
        end else begin
            v1 <= 1'b0;
            if (rx_valid) begin
                if (is_digit) begin
                    if (nd == 3'd3) begin
                        // 第 4 位到了。用移位**前**的旧值锁存：
                        //   d1=第1位(舵机号)  d2=第2位  d3=第3位  新字节=第4位
                        cid   <= d1;
                        cr1   <= d2;
                        cr2   <= d3;
                        cr3   <= rx_data[3:0];
                        raw_r <= {6'b0,d2} * 10'd100
                               + {6'b0,d3} * 10'd10
                               + {6'b0,rx_data[3:0]};
                        // 移位留给下一次组帧用（d0 只是习惯保留，不参与逻辑）
                        d0 <= d1; d1 <= d2; d2 <= d3; d3 <= rx_data[3:0];
                        nd <= 3'd0;
                        v1 <= 1'b1;
                    end else begin
                        d0 <= d1; d1 <= d2; d2 <= d3; d3 <= rx_data[3:0];
                        nd <= nd + 1'b1;
                    end
                end else begin
                    nd <= 3'd0;      // 非数字：重新同步，防止被拆断的帧误触发
                end
            end
        end
    end

    // =================================================================
    // 第 2 级：校验 + 原值 -> 舵机角度
    // =================================================================
    reg [7:0] ang_r;
    reg       ok_r;
    reg [3:0] id_r, r1_r, r2_r, r3_r;
    reg       v2;

    wire       id_ok_w  = (cid >= 4'd1) && (cid <= 4'd4);
    wire       raw_ok_w = (raw_r <= RAW_MAX);
    wire [7:0] ang_w    = SCALE_360 ? raw_r[8:1] : raw_r[7:0];

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            ang_r <= 8'd0; ok_r <= 1'b0;
            id_r <= 4'd0; r1_r <= 4'd0; r2_r <= 4'd0; r3_r <= 4'd0;
            v2 <= 1'b0;
        end else begin
            v2 <= v1;
            if (v1) begin
                ang_r <= ang_w;
                ok_r  <= id_ok_w && raw_ok_w;
                id_r  <= cid;
                r1_r  <= cr1; r2_r  <= cr2; r3_r  <= cr3;
            end
        end
    end

    // =================================================================
    // 第 3 级：角度拆十进制 + 填回复报文 + 更新目标角度
    // =================================================================
    wire [11:0] ang_d = dec3(ang_r);

    reg [7:0] rmsg [0:12];
    reg [3:0] rlen;
    reg       v3;

    // 正在流式发送一条回复？是的话就先别覆盖缓冲区（避免报文被改花）
    reg [1:0] st;          // 下面发送状态机用
    reg [1:0] tk;
    wire      replying = (st != 2'd0) && (tk == K_REPLY);

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            rlen <= 4'd0;
        end else if (v2 && !replying) begin
            if (ok_r) begin
                // S <号> = <三位原值> <空格> A <三位角度> CR LF   （13 字符）
                rmsg[0]  <= "S";
                rmsg[1]  <= 8'h30 + {4'b0, id_r};
                rmsg[2]  <= "=";
                rmsg[3]  <= 8'h30 + {4'b0, r1_r};
                rmsg[4]  <= 8'h30 + {4'b0, r2_r};
                rmsg[5]  <= 8'h30 + {4'b0, r3_r};
                rmsg[6]  <= " ";
                rmsg[7]  <= "A";
                rmsg[8]  <= 8'h30 + {4'b0, ang_d[11:8]};
                rmsg[9]  <= 8'h30 + {4'b0, ang_d[7:4]};
                rmsg[10] <= 8'h30 + {4'b0, ang_d[3:0]};
                rmsg[11] <= CR;
                rmsg[12] <= LF;
                rlen     <= 4'd13;
            end else begin
                // E R R CR LF   （5 字符）
                rmsg[0] <= "E";
                rmsg[1] <= "R";
                rmsg[2] <= "R";
                rmsg[3] <= CR;
                rmsg[4] <= LF;
                rlen    <= 4'd5;
            end
        end
    end

    always @(posedge Clk or negedge Rst_n)
        if (!Rst_n) v3 <= 1'b0;
        else        v3 <= v2 && !replying;

    // 目标角度：只改被点到的那一路，其余保持
    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            angle <= {4{CENTER8}};
        end else if (v2 && ok_r) begin
            case (id_r)
                4'd1: angle[7:0]   <= ang_r;
                4'd2: angle[15:8]  <= ang_r;
                4'd3: angle[23:16] <= ang_r;
                4'd4: angle[31:24] <= ang_r;
                default: ;
            endcase
        end
    end

    // =================================================================
    // 心跳报文：当前四路角度   "A 090 090 090 090\r\n"
    // =================================================================
    wire [11:0] hd0 = dec3(angle[7:0]);
    wire [11:0] hd1 = dec3(angle[15:8]);
    wire [11:0] hd2 = dec3(angle[23:16]);
    wire [11:0] hd3 = dec3(angle[31:24]);

    reg [7:0] hmsg [0:18];
    reg       hb_fill;      // 报文填好一拍后置位
    reg       hbpend;

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            hb_fill <= 1'b0;
        end else begin
            hb_fill <= hb_tick;
            if (hb_tick) begin
                hmsg[0]  <= "A";
                hmsg[1]  <= " ";
                hmsg[2]  <= 8'h30 + {4'b0, hd0[11:8]};
                hmsg[3]  <= 8'h30 + {4'b0, hd0[7:4]};
                hmsg[4]  <= 8'h30 + {4'b0, hd0[3:0]};
                hmsg[5]  <= " ";
                hmsg[6]  <= 8'h30 + {4'b0, hd1[11:8]};
                hmsg[7]  <= 8'h30 + {4'b0, hd1[7:4]};
                hmsg[8]  <= 8'h30 + {4'b0, hd1[3:0]};
                hmsg[9]  <= " ";
                hmsg[10] <= 8'h30 + {4'b0, hd2[11:8]};
                hmsg[11] <= 8'h30 + {4'b0, hd2[7:4]};
                hmsg[12] <= 8'h30 + {4'b0, hd2[3:0]};
                hmsg[13] <= " ";
                hmsg[14] <= 8'h30 + {4'b0, hd3[11:8]};
                hmsg[15] <= 8'h30 + {4'b0, hd3[7:4]};
                hmsg[16] <= 8'h30 + {4'b0, hd3[3:0]};
                hmsg[17] <= CR;
                hmsg[18] <= LF;
            end
        end
    end

    // ---- 心跳定时 ----
    localparam HB_TICKS = (HB_MS > 0) ? (CLK_FREQ / 1000) * HB_MS : 32'd0;

    reg [31:0] hb_cnt;
    reg        hb_tick;

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            hb_cnt <= 32'd0; hb_tick <= 1'b0;
        end else if (HB_MS == 0) begin
            hb_cnt <= 32'd0; hb_tick <= 1'b0;
        end else if (hb_cnt >= HB_TICKS - 1) begin
            hb_cnt <= 32'd0; hb_tick <= 1'b1;
        end else begin
            hb_cnt <= hb_cnt + 1'b1; hb_tick <= 1'b0;
        end
    end

    // =================================================================
    // 开机横幅延时（给电脑开串口留时间）
    // =================================================================
    localparam BOOT_TICKS = (CLK_FREQ / 1000) * BOOT_DELAY_MS;

    reg [31:0] boot_cnt;
    reg        boot_armed;
    reg        boot_pend;

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            boot_cnt <= 32'd0; boot_armed <= 1'b0;
        end else if (boot_cnt < BOOT_TICKS) begin
            boot_cnt <= boot_cnt + 1'b1;
        end else begin
            boot_armed <= 1'b1;
        end
    end

    // =================================================================
    // 发送仲裁与状态机
    //   优先级：动作回复 > 开机横幅 > 心跳
    // =================================================================
    localparam ST_IDLE = 2'd0, ST_LD = 2'd1, ST_TX = 2'd2;

    reg [6:0] clen;
    reg [6:0] tx_idx;

    // 按类型取第 i 个字符
    function [7:0] char_of(input [1:0] k, input [6:0] i);
        begin
            case (k)
                K_REPLY: char_of = rmsg[i];
                K_HB:    char_of = hmsg[i];
                default: char_of = banner_char(i);
            endcase
        end
    endfunction

    reg rpend;

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            st        <= ST_IDLE;
            tk        <= K_BOOT;
            clen      <= 7'd0;
            tx_idx    <= 7'd0;
            tx_start  <= 1'b0;
            tx_data   <= 8'd0;
            rpend     <= 1'b0;
            hbpend    <= 1'b0;
            boot_pend <= 1'b1;
        end else begin
            tx_start <= 1'b0;

            // 新的回复到达（正在发回复时忽略，避免把缓冲区改花）
            if (v3 && !replying) rpend <= 1'b1;

            case (st)
                ST_IDLE: begin
                    if (rpend) begin
                        tk     <= K_REPLY;
                        clen   <= {3'b0, rlen};
                        tx_idx <= 7'd0;
                        rpend  <= 1'b0;
                        st     <= ST_LD;
                    end else if (boot_pend && boot_armed) begin
                        tk        <= K_BOOT;
                        clen      <= BANNER_LEN;
                        tx_idx    <= 7'd0;
                        boot_pend <= 1'b0;
                        st        <= ST_LD;
                    end else if (hbpend) begin
                        tk     <= K_HB;
                        clen   <= HB_LEN;
                        tx_idx <= 7'd0;
                        hbpend <= 1'b0;
                        st     <= ST_LD;
                    end
                end
                ST_LD: begin
                    tx_data  <= char_of(tk, 7'd0);
                    tx_start <= 1'b1;
                    tx_idx   <= 7'd1;
                    st       <= ST_TX;
                end

                ST_TX: begin
                    if (tx_done) begin
                        if (tx_idx >= clen) begin
                            st <= ST_IDLE;
                        end else begin
                            tx_data  <= char_of(tk, tx_idx);
                            tx_start <= 1'b1;
                            tx_idx   <= tx_idx + 1'b1;
                        end
                    end
                end

                default: st <= ST_IDLE;
            endcase

            // 心跳报文填好后置位待发。
            // 必须放在 case 之后：这样若同一拍仲裁器刚把 hbpend 清掉
            // （开始发心跳），新的一拍不会把它又立刻置起来。
            if (hb_fill) hbpend <= 1'b1;
        end
    end

endmodule

// =====================================================================
// enc_report —— 把 4 路编码器速度通过串口打印出来
//
// 每 REPORT_MS 毫秒输出一行 ASCII，格式：
//     S0=+02200 S1=+02195 S2=-00004 S3=+02210\r\n
//       （每路 5 位数：符号 + 5 位，范围 ±32767）
//
// 数值单位 = **计数/秒**（counts per second，带符号）—— M/T 法测速的输出
// 换算成转速：RPM = cps * 60 / CPR
//   例：CPR=1320 -> RPM = cps * 0.04545
//       1100 cps -> 50 RPM
//
// 4 路都是带符号的（正转正、反转负），数值限幅显示到 ±32767。
// =====================================================================
module enc_report #(
    parameter CLK_FREQ  = 50_000_000,
    parameter BAUD      = 115200,
    parameter REPORT_MS = 200
) (
    input  wire               Clk,
    input  wire               Rst_n,
    input  wire signed [63:0] spd,      // 4 路，低 16bit = 轮0
    output wire               uart_tx
);

    // ---------------- 报告节拍 ----------------
    localparam TICK_MAX = CLK_FREQ / 1000 * REPORT_MS;
    reg [31:0] tick_cnt;
    reg        tick;
    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            tick_cnt <= 32'd0; tick <= 1'b0;
        end else if (tick_cnt >= TICK_MAX - 1) begin
            tick_cnt <= 32'd0; tick <= 1'b1;
        end else begin
            tick_cnt <= tick_cnt + 1'b1; tick <= 1'b0;
        end
    end

    // ---------------- UART ----------------
    reg        uart_start;
    reg  [7:0] uart_data;
    wire       uart_done;

    uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD(BAUD)) u_uart (
        .Clk  (Clk),
        .Rst_n(Rst_n),
        .start(uart_start),
        .data (uart_data),
        .tx   (uart_tx),
        .done (uart_done)
    );

    // ---------------- 状态 ----------------
    localparam S_IDLE = 4'd0,
               S_S    = 4'd1,   // 'S'
               S_CH   = 4'd2,   // 通道号
               S_EQ   = 4'd3,   // '='
               S_SGN  = 4'd4,   // '+' / '-'
               S_LOAD = 4'd5,   // 装载除数
               S_SUB  = 4'd6,   // 反复减
               S_DIG  = 4'd7,   // 输出该位数字
               S_SP   = 4'd8,   // 空格
               S_CR   = 4'd9,
               S_LF   = 4'd10;

    reg [3:0]  st;
    reg        go;        // 单拍：请求 UART 发一个字节
    reg        ew;        // 正在等待本字节发完
    reg [1:0]  ch;
    reg [2:0]  k;         // 0..3 -> 1000 / 100 / 10 / 1
    reg [3:0]  dcount;
    reg [15:0] vdiv;
    reg [63:0] vmag;      // 4 路绝对值
    reg [3:0]  neg;       // 4 路符号

    wire [15:0] cur = vmag[ch*16 +: 16];

    // 除数表
    always @(*) begin
        case (k)
            3'd0: vdiv = 16'd10000;
            3'd1: vdiv = 16'd1000;
            3'd2: vdiv = 16'd100;
            3'd3: vdiv = 16'd10;
            default: vdiv = 16'd1;
        endcase
    end

    // 当前要发的字节
    reg [7:0] eb;
    always @(*) begin
        case (st)
            S_S   : eb = 8'h53;                    // 'S'
            S_CH  : eb = 8'h30 + {6'b0, ch};       // '0'+ch
            S_EQ  : eb = 8'h3D;                    // '='
            S_SGN : eb = neg[ch] ? 8'h2D : 8'h2B;  // '-' : '+'
            S_DIG : eb = 8'h30 + {4'b0, dcount};   // '0'+dcount
            S_SP  : eb = 8'h20;
            S_CR  : eb = 8'h0D;
            S_LF  : eb = 8'h0A;
            default: eb = 8'h20;
        endcase
    end

    // UART 启动（go 为单拍脉冲）
    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            uart_start <= 1'b0;
            uart_data  <= 8'd0;
        end else begin
            uart_start <= 1'b0;
            if (go) begin
                uart_data  <= eb;
                uart_start <= 1'b1;
            end
        end
    end

    // ---------------- 主状态机 ----------------
    integer i;
    reg [15:0] mag16;

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            st <= S_IDLE; go <= 1'b0; ew <= 1'b0;
            ch <= 2'd0; k <= 3'd0; dcount <= 4'd0;
            vmag <= 64'd0; neg <= 4'd0;
        end else begin
            go <= 1'b0;

            case (st)
                // ---------------- 等节拍，锁存 4 路速度 ----------------
                S_IDLE: begin
                    ew <= 1'b0;
                    if (tick) begin
                        for (i = 0; i < 4; i = i + 1) begin
                            neg[i] <= spd[i*16 + 15];
                            // 先在 16 位无符号域内求绝对值，再扩展到 vmag。
                            // 不能把 16 位补码表达式直接赋给更宽的 mag：
                            // 负数会先被零扩展，-2200 会错误变成 65536-2200。
                            if (spd[i*16 + 15]) begin
                                // -32768 无法在 16 位有符号数中表示绝对值，
                                // 上报协议统一限幅到 32767。
                                mag16 = (spd[i*16 +: 16] == 16'h8000) ?
                                        16'd32767 : (~spd[i*16 +: 16] + 16'd1);
                            end else begin
                                mag16 = spd[i*16 +: 16];
                            end
                            vmag[i*16 +: 16] <= (mag16 > 16'd32767) ?
                                                16'd32767 : mag16;
                        end
                        ch <= 2'd0;
                        k  <= 3'd0;
                        st <= S_S;
                    end
                end

                // ---------------- 发射类状态 ----------------
                S_S, S_CH, S_EQ, S_SGN, S_SP, S_CR, S_LF, S_DIG: begin
                    if (!ew) begin
                        go <= 1'b1;
                        ew <= 1'b1;
                    end else if (uart_done) begin
                        ew <= 1'b0;
                        case (st)
                            S_S  : st <= S_CH;
                            S_CH : st <= S_EQ;
                            S_EQ : st <= S_SGN;
                            S_SGN: begin k <= 3'd0; st <= S_LOAD; end
                            S_DIG: begin
                                if (k == 3'd4) begin     // 5 位数字发完
                                    st <= S_SP;
                                end else begin
                                    k  <= k + 1'b1;
                                    st <= S_LOAD;
                                end
                            end
                            S_SP : begin
                                if (ch == 2'd3) begin
                                    st <= S_CR;
                                end else begin
                                    ch <= ch + 1'b1;
                                    st <= S_S;
                                end
                            end
                            S_CR : st <= S_LF;
                            default: st <= S_IDLE;   // S_LF
                        endcase
                    end
                end

                // ---------------- 减法取数字 ----------------
                S_LOAD: begin
                    dcount <= 4'd0;
                    st     <= S_SUB;
                end

                S_SUB: begin
                    if (cur >= vdiv) begin
                        vmag[ch*16 +: 16] <= cur - vdiv;
                        dcount <= dcount + 1'b1;
                    end else begin
                        st <= S_DIG;
                    end
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule

// =====================================================================
// cmd_rx —— 上位机遥控指令接收 + 掉线保护
//
// 【在整条链路里的位置】
//   cmd_rx ──(vx,vy,w)──> mecanum_mixer ──> motor_quad ──> AT8236
//   （替换原来的 motor_demo；mecanum_mixer 以下完全不用改）
//
// 【帧格式】14 字节 ASCII，方便直接用串口助手手打调试
//
//     '@'  <svx><d><d><d>  <svy><d><d><d>  <sw><d><d><d>  '\n'
//      0     1   2  3  4     5   6  7  8     9  10 11 12    13
//
//   <s> = '+' 或 '-'      <d> = '0'..'9'      每个分量 ±000..±999
//
//   例：
//     "@+300+000+000\n"   前进 300
//     "@+300-200+000\n"   右前斜移（vy 负 = 右）
//     "@+000+000+250\n"   原地逆时针转
//     "@+000+000+000\n"   停
//
//   为什么用 ASCII 不用二进制：
//     出问题时可以直接在串口助手里看到、手打一帧来验证，不用写脚本。
//     每帧 14 字节，20Hz 时只占 115200 带宽的 2.4%。
//
// 【掉线保护】—— 遥控车必须有
//   超过 TIMEOUT_MS 没收到**合法**帧 -> 输出强制归零，车自己停。
//   上电时计数器就是"已超时"状态，所以**上电后车不动**，
//   直到收到第一条指令 —— 不会像 motor_demo 那样一上电就冲出去。
//   GUI 冻结、USB 掉了、线松了，都会触发。
//
// 【帧同步】
//   靠 '@' 找帧头；载荷字符逐个校验（符号位必须是 +/-，数字位必须是 0-9），
//   任何一个不对立刻回 S_IDLE 重新找 '@'。所以被截断/串扰的帧不会被误认，
//   最坏情况只是丢一帧，下一帧就恢复。
//
// 【时序】
//   全程无乘法器：acc*10 用 (acc<<3)+(acc<<1)，而且是每个数字一拍、
//   分散在 3 拍里做，没有长组合路径。
//
// 【移植】CLK_FREQ 参数化，27MHz / 50MHz 都能直接用。
// =====================================================================
`timescale 1ns / 1ps

module cmd_rx #(
    parameter CLK_FREQ   = 50_000_000,
    parameter BAUD       = 115200,
    parameter VMAX       = 1000,    // 每个分量的限幅（与 mecanum_mixer 的 OUT_MAX 对齐）
    parameter TIMEOUT_MS = 500      // 掉线保护时间（毫秒）
) (
    input  wire               Clk,
    input  wire               Rst_n,
    input  wire               rx,        // 串口输入（空闲高）
    output wire signed [15:0] vx,        // 前后：前进为正
    output wire signed [15:0] vy,        // 左右：左移为正
    output wire signed [15:0] w,         // 自旋：逆时针为正
    output wire               link_ok    // 1 = 最近 TIMEOUT_MS 内收到过合法帧
);

    // ---------------- 字节流 ----------------
    wire [7:0] b;
    wire       bv;

    uart_rx #(.CLK_FREQ(CLK_FREQ), .BAUD(BAUD)) u_uart (
        .Clk   (Clk),
        .Rst_n (Rst_n),
        .rx    (rx),
        .data  (b),
        .valid (bv)
    );

    // ---------------- 帧解析 ----------------
    localparam S_IDLE = 2'd0,   // 等 '@'
               S_PAY  = 2'd1,   // 收 12 个载荷字符
               S_END  = 2'd2;   // 等 '\n'

    reg [1:0] st;
    reg [3:0] k;                // 载荷序号 0..11
    reg       n0, n1, n2;       // 三个分量的符号（1 = 负）
    reg [9:0] a0, a1, a2;       // 三个分量的累加器（0..999）

    reg               frame_ok;                 // 单拍：收到一个完整合法帧
    reg signed [15:0] fvx, fvy, fw;             // 最后一次收到的有效指令

    // k[3:2] = 第几个分量（0=vx 1=vy 2=w），k[1:0] = 分量内位置（0=符号 1..3=数字）
    wire [1:0] fld    = k[3:2];
    wire [1:0] pos    = k[1:0];
    wire       is_dig = (b >= 8'h30) && (b <= 8'h39);
    wire       is_sgn = (b == 8'h2B) || (b == 8'h2D);   // '+' / '-'

    // 当前累加的那一路，做 acc*10 + digit（移位加法，无乘法器）
    wire [9:0] acc_cur = (fld == 2'd0) ? a0 : (fld == 2'd1) ? a1 : a2;
    wire [9:0] acc_nxt = (acc_cur << 3) + (acc_cur << 1) + {6'b0, b[3:0]};

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            st <= S_IDLE; k <= 4'd0; frame_ok <= 1'b0;
            n0 <= 1'b0; n1 <= 1'b0; n2 <= 1'b0;
            a0 <= 10'd0; a1 <= 10'd0; a2 <= 10'd0;
            fvx <= 16'sd0; fvy <= 16'sd0; fw <= 16'sd0;
        end else begin
            frame_ok <= 1'b0;
            if (bv) begin
                case (st)
                    // ---------- 找帧头 ----------
                    S_IDLE: begin
                        if (b == 8'h40) begin           // '@'
                            st <= S_PAY;
                            k  <= 4'd0;
                            a0 <= 10'd0; a1 <= 10'd0; a2 <= 10'd0;
                        end
                    end

                    // ---------- 收 12 个载荷字符 ----------
                    S_PAY: begin
                        if (pos == 2'd0) begin
                            // 符号位
                            if (is_sgn) begin
                                case (fld)
                                    2'd0:    n0 <= (b == 8'h2D);
                                    2'd1:    n1 <= (b == 8'h2D);
                                    default: n2 <= (b == 8'h2D);
                                endcase
                                k <= k + 1'b1;
                            end else begin
                                st <= S_IDLE;           // 格式不对，重新找 '@'
                            end
                        end else begin
                            // 数字位
                            if (is_dig) begin
                                case (fld)
                                    2'd0:    a0 <= acc_nxt;
                                    2'd1:    a1 <= acc_nxt;
                                    default: a2 <= acc_nxt;
                                endcase
                                k <= k + 1'b1;
                                if (k == 4'd11) st <= S_END;   // 这是最后一个载荷字符
                            end else begin
                                st <= S_IDLE;
                            end
                        end
                    end

                    // ---------- 等帧尾，整帧合法才生效 ----------
                    S_END: begin
                        if (b == 8'h0A) begin           // '\n'
                            fvx <= n0 ? -$signed({6'b0, a0}) : $signed({6'b0, a0});
                            fvy <= n1 ? -$signed({6'b0, a1}) : $signed({6'b0, a1});
                            fw  <= n2 ? -$signed({6'b0, a2}) : $signed({6'b0, a2});
                            frame_ok <= 1'b1;
                        end
                        st <= S_IDLE;                   // 不管对错都回去找下一个 '@'
                    end

                    default: st <= S_IDLE;
                endcase
            end
        end
    end

    // ---------------- 掉线保护 ----------------
    localparam [31:0] TD_MAX = CLK_FREQ / 1000 * TIMEOUT_MS;

    reg [31:0] td;
    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n)           td <= TD_MAX;     // 上电即"已超时"
        else if (frame_ok)    td <= 32'd0;
        else if (td < TD_MAX) td <= td + 1'b1;
    end

    assign link_ok = (td < TD_MAX);

    // ---------------- 限幅 + 输出 ----------------
    localparam signed [15:0] VMAX_C = VMAX;

    function signed [15:0] clampv;
        input signed [15:0] x;
        begin
            if      (x >  VMAX_C) clampv =  VMAX_C;
            else if (x < -VMAX_C) clampv = -VMAX_C;
            else                  clampv = x;
        end
    endfunction

    wire signed [15:0] cvx = clampv(fvx);
    wire signed [15:0] cvy = clampv(fvy);
    wire signed [15:0] cw  = clampv(fw);

    assign vx = link_ok ? cvx : 16'sd0;
    assign vy = link_ok ? cvy : 16'sd0;
    assign w  = link_ok ? cw  : 16'sd0;

endmodule

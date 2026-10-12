// =====================================================================
// speed_pi —— 单路转速闭环 PI 控制器
//
// 量纲约定：
//   target / actual 量纲必须一致（现在 encoder_reader 输出的是 **计数/秒**）；
//   注意 PI 的 out 是 motor_channel 的占空比刻度（±SPEED_MAX），
//   即 encoder_reader 输出的 enc_speed 的原始单位，不需要做 RPM 换算。
//   参考换算：counts/闸门 = RPM * CPR * GATE_MS / 60000
//   例：CPR=1320、GATE_MS=10ms 时，counts = RPM * 0.22
//       -> 500 RPM 对应 110 counts/闸门
//
// 控制律（每来一个 update 脉冲算一次，update 即测速闸门结束标志）：
//   err   = target - actual
//   integ = integ + err        （限幅 ±I_MAX，抗积分饱和）
//   out   = (KP*err + KI*integ) >>> GAIN_SHIFT   （限幅 ±SPEED_MAX）
//
// 参数整定（GATE_MS=10 -> 控制频率 100Hz）：
//   1) 先把 KI 置 0，只加 KP：从 1 开始加，直到响应够快但不振荡
//   2) 再加 KI：从 0 开始加，消掉稳态误差
//   3) GAIN_SHIFT 用来给 KP/KI 增加小数分辨率（两者同时被右移）
//
// 反馈丢失保护（FEEDBACK_TIMEOUT > 0 时生效，默认 100 个闸门 = 1s）：
//   目标非零、但实测速度连续 FEEDBACK_TIMEOUT 个闸门一直为 0
//   -> 判定编码器未接/线断，输出强制归零并置 fault，避免 PI 空转到满占空比。
//   目标回到 0 时自动清除故障。设为 0 可关闭该保护。
// =====================================================================
module speed_pi #(
    parameter SPEED_MAX        = 1000,   // 输出限幅，需与 motor_channel 的 SPEED_MAX 一致
    parameter KP               = 4,
    parameter KI               = 1,
    parameter GAIN_SHIFT       = 0,
    parameter I_MAX            = 2000,   // 积分器限幅
    parameter FEEDBACK_TIMEOUT = 100,    // 0 = 关闭反馈丢失保护
    parameter TARGET_DEADBAND  = 0       // |target| 小于该值视为"零指令"
) (
    input  wire               Clk,
    input  wire               Rst_n,
    input  wire               enable,
    input  wire               update,    // 闸门结束脉冲
    input  wire signed [15:0] target,
    input  wire signed [15:0] actual,
    output reg  signed [15:0] out,
    output reg                fault
);

    localparam signed [31:0] SPEED_MAX_C = SPEED_MAX;
    localparam signed [31:0] I_MAX_C     = I_MAX;
    localparam               FB_PROT_EN  = (FEEDBACK_TIMEOUT != 0);

    // 误差用 17bit 存，避免 -32768 取反溢出
    wire signed [16:0] err = target - actual;

    // ---------------- 积分器 ----------------
    reg  signed [31:0] integ;
    wire signed [31:0] integ_nxt = integ + err;
    wire signed [31:0] integ_lim = (integ_nxt >  I_MAX_C) ?  I_MAX_C :
                                   (integ_nxt < -I_MAX_C) ? -I_MAX_C : integ_nxt;

    // ---------------- 输出 ----------------
    wire signed [31:0] raw = (KP * err + KI * integ) >>> GAIN_SHIFT;
    wire signed [31:0] lim = (raw >  SPEED_MAX_C) ?  SPEED_MAX_C :
                             (raw < -SPEED_MAX_C) ? -SPEED_MAX_C : raw;

    // ---------------- 反馈丢失检测 ----------------
    reg [15:0] zero_cnt;
    wire       tgt_nz  = (target > TARGET_DEADBAND) || (target < -TARGET_DEADBAND);
    wire       fb_nz   = (actual != 0);
    wire       fb_lost = FB_PROT_EN && (zero_cnt >= FEEDBACK_TIMEOUT);

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            integ    <= 32'sd0;
            out      <= 16'sd0;
            fault    <= 1'b0;
            zero_cnt <= 16'd0;
        end else if (!enable) begin
            integ    <= 32'sd0;
            out      <= 16'sd0;
            fault    <= 1'b0;
            zero_cnt <= 16'd0;
        end else if (update) begin
            // 有目标但一直测不到转速 -> 累计；否则清零
            if (FB_PROT_EN && tgt_nz && !fb_nz) begin
                if (zero_cnt != 16'hFFFF)
                    zero_cnt <= zero_cnt + 1'b1;
            end else begin
                zero_cnt <= 16'd0;
            end

            // 目标归零自动清故障；超时则置故障
            if (!tgt_nz)
                fault <= 1'b0;
            else if (fb_lost)
                fault <= 1'b1;

            // 故障时输出归零并清积分，防止继续空转
            if (fault) begin
                integ <= 32'sd0;
                out   <= 16'sd0;
            end else begin
                integ <= integ_lim;
                out   <= lim[15:0];
            end
        end
        // update 之外的周期：out / integ / fault 保持不变
    end

endmodule

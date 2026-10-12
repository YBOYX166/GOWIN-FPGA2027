// =====================================================================
// 舵机 PWM 驱动（单路）
//
// 标准模拟/数字舵机都是同一套控制方式：
//   周期 20ms（50Hz），高电平脉宽决定角度：
//     0.5ms ->   0 度
//     1.5ms ->  90 度（中位）
//     2.5ms -> 180 度
//
// 【单位与算法】
//   全程整数定点，**没有除法器**（除法在编译期算完）。
//   这正是 138K 主工程 MOTOR_DRIVER.md 里记的教训：motor_channel 里
//   留了运行时除法器，clk50 的 Fmax 从 87MHz 掉到 29MHz。这里照抄
//   那边修好后的做法 —— 编译期算好「每度多少 tick」（Q12 定点），
//   运行时只剩一次乘法。
//
//   pulse_ticks = PULSE_MIN_TICKS + ((angle * TICKS_PER_DEG_Q) >> 12)
//
// 【为什么做成 3 级流水】
//   第一版把乘法、加法、比较全写成组合逻辑，实测 Fmax 只有 41.9MHz
//   （4K 上跑 27MHz 够用，但移植到 50MHz 的 138K 就悬了）。
//   拆成 3 级后关键路径只剩一段，Fmax 明显改善（见 README 验证记录）：
//       第 1 级：乘法        （交给 DSP 硬核）
//       第 2 级：加常数 + 定点移位
//       第 3 级：与周期计数器比较，输出 PWM
//   角度变化很慢（秒级），多 2 个时钟延迟完全无感。
//
// 【参数化 CLK_FREQ —— 不许写死】
//   4K 板 27MHz，138K 板 50MHz。两种频率下 TICKS_PER_US 和 PERIOD_TICKS
//   都是整数，输出波形完全一致。移植时只改顶层传进来的值。
// =====================================================================
`timescale 1ns / 1ps

module servo_pwm #(
    parameter CLK_FREQ     = 27_000_000,  // 系统时钟频率
    parameter PWM_HZ       = 50,          // 舵机标准 50Hz（20ms）
    parameter PULSE_MIN_US = 500,         // 0 度对应脉宽
    parameter PULSE_MAX_US = 2500,        // 180 度对应脉宽
    parameter ANGLE_MAX    = 180          // 角度上限（度）
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] angle,   // 0 .. 180（度）
    output reg        pwm
);

    // ---------------- 编译期常量 ----------------
    localparam TICKS_PER_US    = CLK_FREQ / 1_000_000;
    localparam PERIOD_TICKS    = CLK_FREQ / PWM_HZ;
    localparam PULSE_MIN_TICKS = PULSE_MIN_US * TICKS_PER_US;
    localparam SPAN_TICKS      = (PULSE_MAX_US - PULSE_MIN_US) * TICKS_PER_US;

    localparam SHIFT = 12;                        // Q12 定点
    localparam HALF  = (1 << (SHIFT - 1));        // 四舍五入
    localparam TICKS_PER_DEG_Q = ((SPAN_TICKS << SHIFT) + (ANGLE_MAX / 2)) / ANGLE_MAX;

    // 计数器位宽按实际需要收窄 —— 比较器越窄，时序越好
    localparam CNT_W = $clog2(PERIOD_TICKS);

    localparam [31:0] PULSE_BIAS    = (PULSE_MIN_TICKS << SHIFT) + HALF;
    localparam [CNT_W-1:0] PULSE_RST = PULSE_MIN_TICKS;

    // ---------------- 角度限幅 ----------------
    // 超范围时脉宽可能超过一个周期 -> 舵机收到 100% 占空比 -> 堵转发热。
    // 这里硬性夹住，算一种保护。
    localparam [7:0] ANGLE_CLAMP = ANGLE_MAX;
    wire [7:0] ang = (angle > ANGLE_CLAMP) ? ANGLE_CLAMP : angle;

    // ---------------- 第 1 级：乘法（DSP）----------------
    reg [31:0] mult_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) mult_r <= 32'd0;
        else        mult_r <= ang * TICKS_PER_DEG_Q;
    end

    // ---------------- 第 2 级：加常数 + 定点移位 ----------------
    // 脉宽 tick 数 = (PULSE_MIN_TICKS<<SHIFT + ang*TICKS_PER_DEG_Q + HALF) >> SHIFT
    reg [CNT_W-1:0] pulse_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) pulse_r <= PULSE_RST;
        else        pulse_r <= (mult_r + PULSE_BIAS) >> SHIFT;
    end

    // ---------------- 20ms 周期计数器 ----------------
    reg [CNT_W-1:0] cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cnt <= {CNT_W{1'b0}};
        else if (cnt >= PERIOD_TICKS - 1)
            cnt <= {CNT_W{1'b0}};
        else
            cnt <= cnt + 1'b1;
    end

    // ---------------- 第 3 级：输出比较 ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) pwm <= 1'b0;
        else        pwm <= (cnt < pulse_r);
    end

endmodule

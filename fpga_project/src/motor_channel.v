// =====================================================================
// motor_channel —— AT8236 单路电机驱动（IN1 / IN2 双 PWM 接口）
//
// 【重要】AT8236 和 TB6612 接口完全不同：
//   TB6612：PWMA + AIN1 + AIN2 + STBY  （PWM 单独一根脚）
//   AT8236：只有 IN1 / IN2 两个脚，PWM 直接灌到这两个脚上，没有 STBY
//
// 本模块的真值表（与 AT8236 官方一致）：
//   speed > 0（正转）：IN1 = PWM(占空比)，IN2 = 0
//   speed < 0（反转）：IN1 = 0，          IN2 = PWM(占空比)
//   speed = 0 且 BRAKE_ON_ZERO=1：IN1 = 1，IN2 = 1（刹车）
//   speed = 0 且 BRAKE_ON_ZERO=0：IN1 = 0，IN2 = 0（滑行）
//
// ⚠️ AT8236 在 IN1=IN2=0 持续 >1ms 后会进入休眠，再次唤醒约需 1ms。
//    如果发现"停车后再起步有延迟"，把 BRAKE_ON_ZERO 改成 1 即可
//    （刹车态 IN1=IN2=1，芯片不会休眠）。
//
// PWM 默认 20kHz（在可闻频段外），占空比 = |speed| / SPEED_MAX。
// 占空比用"定点常量乘 + 移位"实现，不用除法器。
// =====================================================================
module motor_channel #(
    parameter CLK_FREQ      = 50_000_000,
    parameter PWM_FREQ      = 20_000,
    parameter SPEED_MAX     = 1000,
    parameter BRAKE_ON_ZERO = 0,
    // 1 = TB6612FNG / DRV8833（PWM + IN1 + IN2，方向脚静态）
    // 0 = AT8236（IN1 / IN2 双 PWM，无独立 PWM 脚）
    parameter IFACE_TB6612   = 1
) (
    input  wire               Clk,
    input  wire               Rst_n,
    input  wire signed [15:0] speed,   // 速度指令，正转正 / 负转反
    output wire               pwm,     // TB6612/DRV8833 的 PWMA/PWMB（AT8236 模式恒 0）
    output wire               in1,     // TB6612: AIN1   AT8236: IN1
    output wire               in2      // TB6612: AIN2   AT8236: IN2
);

    localparam PERIOD = CLK_FREQ / PWM_FREQ;   // 50MHz/20kHz = 2500 计数

    localparam [15:0] SPEED_MAX_C = SPEED_MAX;

    // ---------------- 取绝对值并限幅 ----------------
    wire        sign = speed[15];
    wire [15:0] mag  = sign ? (~speed + 1'b1) : speed;
    wire [15:0] mag_c = (mag > SPEED_MAX_C) ? SPEED_MAX_C : mag;

    // ---------------- 占空比：定点常量乘 + 移位 ----------------
    // duty = |speed| * PERIOD / SPEED_MAX = (|speed| * DUTY_MUL) >> DUTY_SHIFT
    // 默认 PERIOD=2500、SPEED_MAX=1000 -> DUTY_MUL = 2560（正好整除，无误差）
    localparam DUTY_SHIFT = 10;
    localparam DUTY_MUL   = (PERIOD << DUTY_SHIFT) / SPEED_MAX;

    wire [31:0] duty_full = mag_c * DUTY_MUL;

    reg [15:0] duty_r;
    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n)
            duty_r <= 16'd0;
        else
            duty_r <= duty_full[DUTY_SHIFT+15 : DUTY_SHIFT];
    end

    // ---------------- PWM 计数器 ----------------
    reg [15:0] cnt;
    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n)
            cnt <= 16'd0;
        else if (cnt >= PERIOD - 1)
            cnt <= 16'd0;
        else
            cnt <= cnt + 1'b1;
    end

    wire pwm_r = (cnt < duty_r);
    wire zero = (mag_c == 16'd0);

    // ---------------- 输出 ----------------
    wire stop = zero ? (BRAKE_ON_ZERO ? 1'b1 : 1'b0) : 1'b0;

    // TB6612 / DRV8833：PWM 单独一根脚，IN1/IN2 给静态方向
    assign pwm = IFACE_TB6612 ? (zero ? 1'b0 : pwm_r) : 1'b0;
    assign in1 = IFACE_TB6612 ? (zero ? stop : ~sign)
                              : (zero ? stop : (sign ? 1'b0 : pwm_r));
    assign in2 = IFACE_TB6612 ? (zero ? stop :  sign)
                              : (zero ? stop : (sign ? pwm_r : 1'b0));

endmodule

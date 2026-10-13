// =====================================================================
// motor_quad —— 四驱电机驱动 + 四路编码器（2 片 AT8236，每片 2 路）
//
// 【AT8236 接口】每路只有 IN1 / IN2 两个控制脚，PWM 直接灌进去，
//   没有独立的 PWM 脚，也没有 STBY（与 TB6612 不同）。
//   所以 4 路电机一共占 8 个 IO，每路 2 个。
//
// 端口约定（总线按小端打包：低 16/32 bit = 轮 0，依次向上）：
//   speed_cmd[63:0]   4 路带符号速度指令，每路 16bit
//                     闭环时 = 目标转速（编码器计数/测速闸门）
//                     开环时 = 直接 PWM 指令，范围 ±SPEED_MAX
//   in1[3:0]          接各 AT8236 的 AIN1/BIN1
//   in2[3:0]          接各 AT8236 的 AIN2/BIN2
//   enc_a/enc_b[3:0]  4 路编码器 A/B 相
//   enc_pos[127:0]    位置累计，每路 32bit 带符号
//   enc_speed[63:0]   每闸门脉冲数，每路 16bit 带符号（= PI 的反馈量）
//   speed_valid       测速闸门结束标志（4 路同拍，取轮 0 的输出）
//   fault[3:0]        各路反馈丢失告警（编码器未接/断线），见 speed_pi
//
// 接线：轮 0 -> 片0 的 IN1/IN2，轮 1 -> 片0 的另一路，
//       轮 2 -> 片1 的 IN1/IN2，轮 3 -> 片1 的另一路
//
// CLOSED_LOOP = 1：每路插入 speed_pi，speed_cmd 当目标转速；
// CLOSED_LOOP = 0：开环，speed_cmd 直接当 PWM 指令。
// =====================================================================
module motor_quad #(
    parameter CLK_FREQ         = 50_000_000,
    parameter PWM_FREQ         = 20_000,
    parameter SPEED_MAX        = 1000,
    parameter GATE_MS          = 10,
    // 编码器方向修正：bit0..bit3 对应轮0..轮3；1 表示交换该路正负方向
    parameter [3:0] DIR_SWAP    = 4'b0000,
    parameter CLOSED_LOOP      = 1,
    // 1 = TB6612FNG / DRV8833（PWM + IN1 + IN2 + STBY）
    // 0 = AT8236（IN1 / IN2 双 PWM，无独立 PWM 脚、无 STBY）
    parameter IFACE_TB6612     = 1,
    // 速度为 0 时：1 = 刹车（IN1=IN2=1，轮子锁死，斜坡不溜车）
    //               0 = 滑行（IN1=IN2=0，轮子自由，且 AT8236 会在 1ms 后休眠）
    // 遥控车要"按什么动什么"，用 1。
    parameter BRAKE_ON_ZERO    = 1,
    // ---- speed_pi 参数 ----
    parameter KP               = 4,
    parameter KI               = 1,
    parameter GAIN_SHIFT       = 0,
    parameter I_MAX            = 2000,
    parameter FEEDBACK_TIMEOUT = 100
) (
    input  wire                      Clk,
    input  wire                      Rst_n,
    input  wire                      enable,

    input  wire signed [63:0]        speed_cmd,
    output wire [3:0]                pwm,     // TB6612/DRV8833 用；AT8236 模式恒 0
    output wire [3:0]                in1,
    output wire [3:0]                in2,
    output wire [1:0]                stby,    // TB6612 用；AT8236 无此脚

    input  wire [3:0]                enc_a,
    input  wire [3:0]                enc_b,
    output wire signed [127:0]       enc_pos,
    output wire signed [63:0]        enc_speed,
    output wire                      speed_valid,
    output wire [3:0]                fault
);

    // TB6612 的 STBY 高有效；AT8236 无 STBY，此输出可悬空
    assign stby = enable ? 2'b11 : 2'b00;

    wire [3:0]         valid_w;
    wire signed [63:0] enc_speed_w;

    genvar i;
    generate
        for (i = 0; i < 4; i = i + 1) begin : g_ch

            // ⚠️ CLOSED_LOOP=1 时 target 必须与 meas 同量纲（meas 现在是"计数/秒"）
            wire signed [15:0] target = speed_cmd[i*16 +: 16];
            wire signed [15:0] meas   = enc_speed_w[i*16 +: 16];
            wire signed [15:0] pi_out;

            (* syn_preserve = 1 *) encoder_reader #(
                .CLK_FREQ(CLK_FREQ),
                .GATE_MS (GATE_MS),
                .DIR_SWAP(DIR_SWAP[i])
            ) u_enc (
                .Clk        (Clk),
                .Rst_n      (Rst_n),
                .enc_a      (enc_a[i]),
                .enc_b      (enc_b[i]),
                .pos        (enc_pos[i*32 +: 32]),
                .speed      (enc_speed_w[i*16 +: 16]),
                .speed_valid(valid_w[i])
            );

            speed_pi #(
                .SPEED_MAX       (SPEED_MAX),
                .KP              (KP),
                .KI              (KI),
                .GAIN_SHIFT      (GAIN_SHIFT),
                .I_MAX           (I_MAX),
                .FEEDBACK_TIMEOUT(FEEDBACK_TIMEOUT)
            ) u_pi (
                .Clk    (Clk),
                .Rst_n  (Rst_n),
                .enable (enable),
                .update (valid_w[i]),
                .target (target),
                .actual (meas),
                .out    (pi_out),
                .fault  (fault[i])
            );

            // 闭环用 PI 输出；开环直接用指令；enable=0 时强制 0
            // （速度 0 时是刹车还是滑行由 BRAKE_ON_ZERO 决定）
            wire signed [15:0] cmd = enable ? (CLOSED_LOOP ? pi_out : target) : 16'sd0;

            motor_channel #(
                .CLK_FREQ      (CLK_FREQ),
                .PWM_FREQ      (PWM_FREQ),
                .SPEED_MAX     (SPEED_MAX),
                .IFACE_TB6612  (IFACE_TB6612),
                .BRAKE_ON_ZERO (BRAKE_ON_ZERO)
            ) u_motor (
                .Clk  (Clk),
                .Rst_n(Rst_n),
                .speed(cmd),
                .pwm  (pwm[i]),
                .in1  (in1[i]),
                .in2  (in2[i])
            );
        end
    endgenerate

    assign enc_speed = enc_speed_w;

    // 4 路闸门同参数同拍对齐，取轮 0 的 valid 作为整体标志
    assign speed_valid = valid_w[0];

endmodule

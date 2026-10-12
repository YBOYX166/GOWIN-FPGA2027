// =====================================================================
// top_pwm —— 最小 PWM 测试工程（排除摄像头/LCD/PLL/复位依赖）
//
// 目的：单独验证 FPGA 的 PWM 引脚与外部接线。
// 上电即输出：PWM=20kHz/30% 占空比，IN1=1 IN2=0 STBY=1，LED 闪烁。
// 不依赖任何按键/摄像头/上位机，只要时钟 Y18 正常就一定会出波形。
// =====================================================================
module top_pwm (
    input  wire       Clk,     // 50MHz (Y18)
    output wire [3:0] pwm,     // Gpio1[3:0]  F13/F14/E13/E14
    output wire [3:0] in1,     // Gpio1[7:4]  D14/D15/F16/E17
    output wire [3:0] in2,     // Gpio1[11:8] E16/D16/A13/A14
    output wire [1:0] stby,    // Gpio1[13:12] C13/B13
    output wire [1:0] led      // Led[1:0]  Y19/AA18
);

    localparam PERIOD = 2500;   // 50MHz / 20kHz
    localparam DUTY   = 750;    // 30% 占空比

    reg [15:0] cnt = 0;
    always @(posedge Clk) begin
        if (cnt >= PERIOD - 1)
            cnt <= 16'd0;
        else
            cnt <= cnt + 1'b1;
    end

    wire pwm_o = (cnt < DUTY);
    assign pwm  = {4{pwm_o}};
    assign in1  = 4'b1111;      // 正转
    assign in2  = 4'b0000;
    assign stby = 2'b11;        // 使能

    reg [24:0] blink = 0;
    always @(posedge Clk) blink <= blink + 1'b1;
    assign led = {blink[24], ~blink[23]};   // 两路 LED 交替闪烁

endmodule

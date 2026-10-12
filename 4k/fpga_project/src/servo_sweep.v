// =====================================================================
// 舵机扫描信号发生器（三角波）
//
// 产生一个在 [CENTER_DEG - HALF_SWEEP_DEG, CENTER_DEG + HALF_SWEEP_DEG]
// 之间往复的角度值，喂给 servo_pwm。
//
// 默认：中心 90 度，正负 45 度 -> 在 45 度 <-> 135 度 之间往复。
//       单程 1000ms，一个完整来回 2000ms。
//
// 【输出波形】
//       135 ─┐         ┌─────────┐
//            │         │         │
//        90  │         │         │
//            │         │         │
//        45 ─┘─────────┘         └──
//            <-- 1s --><-- 1s -->
//          角度每 STEP_TICKS 个时钟变化 1 度，所以是平滑移动不是跳变。
//
// 【调速度】改 SWEEP_MS 即可（单程毫秒数）。
// 【调幅度】改 HALF_SWEEP_DEG。
// 【想错开多路】给不同实例传不同的 SWEEP_MS，或改下面的 PHASE_DEG 初值。
//
// 全程单时钟域、无乘法无除法（STEP_TICKS 在编译期算完）。
// =====================================================================
`timescale 1ns / 1ps

module servo_sweep #(
    parameter CLK_FREQ       = 27_000_000,
    parameter CENTER_DEG     = 90,     // 中心角度
    parameter HALF_SWEEP_DEG = 45,     // 中心点左右各摆多少度
    parameter SWEEP_MS       = 1000,   // 单程（45->135 或 135->45）耗时 ms
    parameter PHASE_DEG      = 0       // 起始相位偏移（0 = 从最低点起步）
)(
    input  wire       clk,
    input  wire       rst_n,
    output reg  [7:0] angle
);

    localparam LOW_DEG  = CENTER_DEG - HALF_SWEEP_DEG;
    localparam HIGH_DEG = CENTER_DEG + HALF_SWEEP_DEG;

    // 每变化 1 度需要多少个时钟。
    // 注意是 2*HALF_SWEEP_DEG —— 单程要跨过"中心点两侧各 HALF_SWEEP_DEG"，
    // 总共 2*HALF_SWEEP_DEG 度。写成 HALF_SWEEP_DEG 会让单程慢一倍。
    localparam STEP_TICKS = (CLK_FREQ / 1000) * SWEEP_MS / (2 * HALF_SWEEP_DEG);

    reg [31:0] cnt;
    reg        dir;      // 0 = 向上扫, 1 = 向下扫

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt   <= 32'd0;
            dir   <= 1'b0;
            angle <= LOW_DEG + PHASE_DEG;
        end else if (cnt >= STEP_TICKS - 1) begin
            cnt <= 32'd0;
            if (dir == 1'b0) begin
                if (angle >= HIGH_DEG) begin
                    dir   <= 1'b1;
                    angle <= HIGH_DEG - 1;
                end else begin
                    angle <= angle + 1'b1;
                end
            end else begin
                if (angle <= LOW_DEG) begin
                    dir   <= 1'b0;
                    angle <= LOW_DEG + 1'b1;
                end else begin
                    angle <= angle - 1'b1;
                end
            end
        end else begin
            cnt <= cnt + 1'b1;
        end
    end

endmodule

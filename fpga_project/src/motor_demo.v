// =====================================================================
// motor_demo —— 调试用速度指令源（无上位机时的占位控制源）
//
// MODE = 1：固定指令（默认，台架调试用）
//           恒定输出 vx = V, vy = 0, w = 0  -> 四轮同速前进
// MODE = 0：8 相位循环演示序列，用于验接线/转向/编码器方向
//           前进 -> 原地左转 -> 左横移 -> 后退 -> 原地右转 -> 右横移
//           -> 斜移+自转 -> 停止
//           每相 2^27 / 50MHz ≈ 2.684s，整周期 ≈ 21.47s
//
// 量纲：V 的单位与 mecanum_mixer 的输出一致。
//   闭环（推荐）：V = 目标"编码器计数 / 测速闸门"
//                 V = RPM * CPR * GATE_MS / 60000
//                 例：CPR=1320、GATE_MS=10ms -> V = RPM * 0.22
//                     500 RPM -> V = 110
//   开环        ：V 直接是 ±SPEED_MAX 范围内的 PWM 指令
//
// 接入 UART / 蓝牙 / 上位机后，用实际控制信号替换本模块输出即可。
// =====================================================================
module motor_demo #(
    parameter CLK_FREQ = 50_000_000,
    parameter V        = 110,   // 固定指令幅值
    parameter MODE     = 1      // 0 = 演示序列, 1 = 固定指令
) (
    input  wire               Clk,
    input  wire               Rst_n,
    output reg  signed [15:0] vx,
    output reg  signed [15:0] vy,
    output reg  signed [15:0] w
);

    reg [29:0] cnt;
    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n)
            cnt <= 30'd0;
        else
            cnt <= cnt + 1'b1;
    end

    // 高 3 位作相位：每相 2^27/50MHz ≈ 2.684s，整周期 ≈ 21.47s
    wire [2:0] phase = cnt[29:27];

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            vx <= 16'sd0;
            vy <= 16'sd0;
            w  <= 16'sd0;
        end else if (MODE == 1) begin
            // ---- 固定指令：四轮同速前进 ----
            vx <= V;
            vy <= 16'sd0;
            w  <= 16'sd0;
        end else begin
            case (phase)
                3'd0: begin vx <=  V;     vy <=  0;     w <=  0;     end // 前进
                3'd1: begin vx <=  0;     vy <=  0;     w <=  V;     end // 原地左转
                3'd2: begin vx <=  0;     vy <=  V;     w <=  0;     end // 左横移
                3'd3: begin vx <= -V;     vy <=  0;     w <=  0;     end // 后退
                3'd4: begin vx <=  0;     vy <=  0;     w <= -V;     end // 原地右转
                3'd5: begin vx <=  0;     vy <= -V;     w <=  0;     end // 右横移
                3'd6: begin vx <=  V/2;   vy <=  0;     w <=  V/2;   end // 斜移+自转
                default: begin vx <= 0;   vy <=  0;     w <=  0;     end // 停止
            endcase
        end
    end

endmodule

// =====================================================================
// mecanum_mixer —— 麦克纳姆轮速度解算（X 型安装）
// 把整车速度指令 (vx, vy, w) 解算成 4 轮速度指令，接 motor_quad 的
// speed_cmd。结果四路均已限幅到 ±OUT_MAX。
//
// 量纲：vx / vy / w 与四轮输出用同一整数单位，本模块不做单位换算。
//   开环时    ：任意"相对速度"单位即可
//   闭环时推荐：用"每个测速闸门内的编码器计数"（= encoder_reader 的
//               enc_speed 单位），这样解算结果可以直接当 PI 的目标值。
//
// 输入：
//   vx : 前后速度，前进为正
//   vy : 左右速度，左移为正
//   w  : 自旋角速度，逆时针为正
//
// X 型安装标准解算：
//   轮0 (左前) = vx - vy - kw
//   轮1 (右前) = vx + vy + kw
//   轮2 (左后) = vx + vy - kw
//   轮3 (右后) = vx - vy + kw
//   其中 kw = (K_GAIN * w) >>> GAIN_SHIFT，增益 = K_GAIN / 2^GAIN_SHIFT。
//
//   ⚠️ 这里原来写的是 kw = K_GAIN * w / 1000。**除以非 2 的幂是灾难**：
//      综合器要用 DSP 做倒数乘法再加约 30 级归一化逻辑，实测关键路径
//      slack = -2.294ns、Logic Level = 30，clk50 只能跑到 44.8MHz，
//      17 条 setup 违规。改成右移 10 位后是纯布线，零组合逻辑。
//      K_GAIN 从 300 相应提到 307，增益只差 0.065%。
//
// 时序：内部 3 级流水（乘法 / 除法 / 加法+限幅 各占一级），
//       避免 "乘 + 除 + 加法树 + 限幅" 挤在一个 20ns 周期内。
//       流水带来的 3 拍延迟对速度环无影响。
// =====================================================================
module mecanum_mixer #(
    parameter OUT_MAX   = 1000, // 输出限幅（= motor_quad 的 SPEED_MAX）
    // ---- 旋转增益 ----
    // 等效增益 = K_GAIN / 2^GAIN_SHIFT
    // 默认 307/1024 = 0.29980，和原来的 300/1000 = 0.30000 只差 0.065%
    parameter K_GAIN    = 307,
    parameter GAIN_SHIFT = 10,
    // ---- 逐轮输出反向（修接线用）----
    //   bit0=轮0(左前) bit1=轮1(右前) bit2=轮2(左后) bit3=轮3(右后)
    //   某个轮子的 +/- 接反了，就把对应的位置 1，不用重新焊线。
    //   四个全接反（整车反着跑）就填 4'b1111。
    //   诊断方法见 top.v 的 INV_* 说明。
    parameter [3:0] INVERT_WHEEL = 4'b0000
) (
    input  wire                      Clk,
    input  wire                      Rst_n,
    input  wire signed [15:0]        vx,
    input  wire signed [15:0]        vy,
    input  wire signed [15:0]        w,
    output reg  signed [15:0]        spd_fl,   // 轮0 左前
    output reg  signed [15:0]        spd_fr,   // 轮1 右前
    output reg  signed [15:0]        spd_rl,   // 轮2 左后
    output reg  signed [15:0]        spd_rr    // 轮3 右后
);

    localparam signed [31:0] OUT_MAX_C = OUT_MAX;

    // ---------------- 流水级 1：kw 的乘法 ----------------
    reg signed [31:0] kw_mul;
    reg signed [15:0] vx_d1, vy_d1;
    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            kw_mul <= 32'sd0;
            vx_d1  <= 16'sd0;
            vy_d1  <= 16'sd0;
        end else begin
            kw_mul <= K_GAIN * w;
            vx_d1  <= vx;
            vy_d1  <= vy;
        end
    end

    // ---------------- 流水级 2：kw 的除法 ----------------
    reg signed [31:0] kw;
    reg signed [15:0] vx_d2, vy_d2;
    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            kw    <= 32'sd0;
            vx_d2 <= 16'sd0;
            vy_d2 <= 16'sd0;
        end else begin
            kw    <= kw_mul >>> GAIN_SHIFT;   // 原来是 /1000，见文件头说明
            vx_d2 <= vx_d1;
            vy_d2 <= vy_d1;
        end
    end

    // ---------------- 流水级 3：解算 + 限幅 ----------------
    function signed [15:0] clamp;
        input signed [31:0] v;
        begin
            if (v >  OUT_MAX_C)      clamp =  OUT_MAX;
            else if (v < -OUT_MAX_C) clamp = -OUT_MAX;
            else                     clamp = v[15:0];
        end
    endfunction

    // 显式声明 32bit 有符号中间量，避免位宽/符号推断问题
    wire signed [31:0] raw_fl = vx_d2 - vy_d2 - kw;
    wire signed [31:0] raw_fr = vx_d2 + vy_d2 + kw;
    wire signed [31:0] raw_rl = vx_d2 + vy_d2 - kw;
    wire signed [31:0] raw_rr = vx_d2 - vy_d2 + kw;

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            spd_fl <= 16'sd0;
            spd_fr <= 16'sd0;
            spd_rl <= 16'sd0;
            spd_rr <= 16'sd0;
        end else begin
            // INVERT_WHEEL 的对应位置 1 就把该轮取反（取反后再限幅，
            // 所以 -(-1000) 也不会溢出）
            spd_fl <= clamp(INVERT_WHEEL[0] ? -raw_fl : raw_fl);
            spd_fr <= clamp(INVERT_WHEEL[1] ? -raw_fr : raw_fr);
            spd_rl <= clamp(INVERT_WHEEL[2] ? -raw_rl : raw_rl);
            spd_rr <= clamp(INVERT_WHEEL[3] ? -raw_rr : raw_rr);
        end
    end

endmodule

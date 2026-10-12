// =====================================================================
// 4K 侧时钟生成（GW1NSR-LV4C / Tang Nano 4K，板载 27MHz 晶振）
//
// 本文件提供两个可互换的模块，端口形状完全相同：
//
//     clk_gen   —— 27MHz 直通，零资源。★ top.v 默认用这个
//     pll_27    —— PLLVR 锁相环，27MHz 进 / 27MHz 出（1:1），走全局时钟网络
//
// 切换方式：改 top.v 里唯一一处例化的模块名即可。
//     clk_gen u_clk (...);   ← 默认
//     pll_27  u_clk (...);   ← 换成 PLL 版本
//
// ---------------------------------------------------------------------
// 【为什么默认不用 PLL】
//   首次上电调试时，直通方案失败模式最少：LED 不闪就一定是器件/时钟/工具链
//   的问题，不会牵扯 PLL 有没有锁。两种方案都已实测通过综合+P&R+时序。
//
// 【什么时候换成 pll_27】
//   引脚 45 在器件里是 LPLL_T_in —— **专用 PLL 输入脚**，说明板子设计意图
//   就是让 27MHz 先进 PLL。PLL 的 CLKOUT 走专用全局时钟网络，而直通方案
//   在 P&R 时会报：
//       WARN (PR1014) Generic routing resource will be used to clock
//                     signal 'Clk_d' ... may lead to excessive delay or skew
//   实测这条警告在两种方案下都会出现，当前规模下时序是 0 违规、Fmax 远超
//   27MHz，无实际影响。但机械臂规模变大后若出现 hold 问题，优先查这条，
//   那时切到 pll_27。
//
// ---------------------------------------------------------------------
// 【重要：这颗芯片的 PLL 原语是 PLLVR，不是 rPLL，也不是 PLL】
//   实测教训：
//     - 用 rPLL  ->  ERROR (EX0312) There is no rPLL resource in current device
//     - 用 PLL   ->  不适用（那是 GW5AT-138 的原语，见 138K 的 pll.v）
//     - 用 PLLVR ->  ✅ 正确
//   在 Gowin EDA 的 primitive.xml 里，GW1NSR-4C 的 PLL 只由 PLLVR 支持。
//
//   PLLVR 与 rPLL 的两点差异（踩过才知道）：
//     1. PLLVR 多一个 VREN 输入（内部稳压器使能），必须接 1'b1；
//     2. **VCO 公式不同**，PLLVR 是：
//          fVCO    = FCLKIN * (FBDIV_SEL+1) * ODIV_SEL / (IDIV_SEL+1)
//          fCLKOUT = fVCO / ODIV_SEL
//                  = FCLKIN * (FBDIV_SEL+1) / (IDIV_SEL+1)
//        PLLVR 的 fVCO 允许范围是 **600 ~ 1200MHz**（比 rPLL 的 400 更窄）。
//
//   本文件用的一组（已实测，输出精确 27.000MHz）：
//     IDIV_SEL=0  -> N = 1
//     FBDIV_SEL=0 -> M = 1
//     ODIV_SEL=32
//     fVCO    = 27 * 1 * 32 / 1 = 864MHz     ✅ 落在 600~1200 内
//     fCLKOUT = 864 / 32        = 27.000MHz  ✅ 精确 1:1
//
//   ⚠️ 不要手改这组数去"凑别的频率"然后直接烧板 —— 先用
//      Gowin IDE 的 IP Core Generator 生成 PLLVR，它会校验 VCO 范围。
//      手写的错误参数只会得到一条 WARN，仍然能生成比特流，但时钟是错的。
//
// ---------------------------------------------------------------------
// 【移植到 138K 时怎么处理本文件】
//   138K 侧对应 fpga_project/src/pll.v 里的 sys_pll（Gowin_PLL 硬核，
//   50MHz -> 33.33MHz LCD + 24MHz XCLK）。两边模块名、原语、参数都不同，
//   但 top.v 里的用法形状是一致的：
//       wire sys_clk, clk_lock;
//       <时钟模块> u_clk (.clkin(Clk), .clkout(sys_clk), .lock(clk_lock));
//   移植时只替换这一处例化。★ 本文件不要往 138K 搬。
// =====================================================================
`timescale 1ns / 1ps

// ---------------------------------------------------------------------
// 方案 A（默认）：27MHz 直通，不用 PLL
// ---------------------------------------------------------------------
module clk_gen (
    input  wire clkin,   // 板载 27MHz 晶振，Tang Nano 4K: 引脚 45
    output wire clkout,  // = clkin
    output wire lock     // 恒 1（无 PLL，永远"锁定"）
);
    assign clkout = clkin;
    assign lock   = 1'b1;
endmodule

// ---------------------------------------------------------------------
// 方案 B：PLLVR 锁相环，27MHz -> 27MHz（1:1）
//
// 端口与 clk_gen 相同，可直接替换 top.v 里的例化。
// 上电后需等 lock 拉高才算时钟稳定 —— top.v 里
// `sys_rst_n = por_n & clk_lock` 已经处理了这件事。
// ---------------------------------------------------------------------
module pll_27 (
    input  wire clkin,   // 27MHz
    output wire clkout,  // 27.000MHz（走全局时钟网络）
    output wire lock     // PLL 锁定指示
);
    wire clkout_o;
    wire lock_o;

    PLLVR u_pll (
        .CLKOUT   (clkout_o),
        .LOCK     (lock_o),
        .CLKOUTP  (),
        .CLKOUTD  (),
        .CLKOUTD3 (),
        .RESET    (1'b0),
        .RESET_P  (1'b0),
        .VREN     (1'b1),      // 内部稳压器使能，必须为 1
        .CLKIN    (clkin),
        .CLKFB    (1'b0),      // CLKFB_SEL = internal，不用外部反馈
        .FBDSEL   (6'b0),      // 动态重配置关闭，全 0
        .IDSEL    (6'b0),
        .ODSEL    (6'b0),
        .PSDA     (4'b0),
        .DUTYDA   (4'b0),
        .FDLY     (4'b0)
    );

    defparam u_pll.FCLKIN         = "27";
    defparam u_pll.DEVICE         = "GW1NSR-4C";
    defparam u_pll.IDIV_SEL       = 0;      // N = 1
    defparam u_pll.FBDIV_SEL      = 0;      // M = 1
    defparam u_pll.ODIV_SEL       = 32;     // fVCO = 27*1*32/1 = 864MHz
    defparam u_pll.DYN_IDIV_SEL   = "false";
    defparam u_pll.DYN_FBDIV_SEL  = "false";
    defparam u_pll.DYN_ODIV_SEL   = "false";
    defparam u_pll.PSDA_SEL       = "0000";
    defparam u_pll.DUTYDA_SEL     = "1000";
    defparam u_pll.DYN_DA_EN      = "true";
    defparam u_pll.CLKOUT_FT_DIR  = 1'b1;
    defparam u_pll.CLKOUTP_FT_DIR = 1'b1;
    defparam u_pll.CLKOUT_DLY_STEP  = 0;
    defparam u_pll.CLKOUTP_DLY_STEP = 0;
    defparam u_pll.CLKFB_SEL      = "internal";
    defparam u_pll.CLKOUT_BYPASS  = "false";
    defparam u_pll.CLKOUTP_BYPASS = "false";
    defparam u_pll.CLKOUTD_BYPASS = "false";
    defparam u_pll.DYN_SDIV_SEL   = 2;
    defparam u_pll.CLKOUTD_SRC    = "CLKOUT";
    defparam u_pll.CLKOUTD3_SRC   = "CLKOUT";

    assign clkout = clkout_o;
    assign lock   = lock_o;
endmodule

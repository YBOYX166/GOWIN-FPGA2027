# =====================================================================
# 时序约束 —— OV5640 摄像头 + LCD + 电机（2×AT8236 闭环）工程
# =====================================================================

# 主时钟 50MHz（Y18）
create_clock -name clk50 -period 20.000 [get_ports {Clk}]

# 摄像头 PCLK 为外部异步时钟
create_clock -name cmos_pclk -period 20.000 [get_ports {camera_io[12]}]

# clk50 与外部摄像头 PCLK 互异步。
#
# 【已知的 1 条 hold 违规说明】
# PLL 生成时钟（lcd_pclk = CLKOUT0、xclk = CLKOUT1）由工具自动派生，
# 在 SDC 解析阶段还不存在，因此无法写进 group：
#   - get_clocks {u_pll/.../CLKOUT0.default_gen_clk} -> 报 TA2004
#   - get_clocks -include_generated_clocks {clk50}   -> 报 TA2000 语法错误
# 结果是 cmos_pclk <-> lcd_pclk 的跨时钟路径仍按同步关系分析，产生：
#   cam_frame_ok_s0 -> cam_valid_sync_0_s0   Relation 0.000  slack -0.724ns
# 该路径在 RTL 中是标准的 3 级同步器（top.v 里的 cam_valid_sync），
# 属于正常 CDC，这条 hold 违规是相位关系的假报警，不影响功能。
set_clock_groups -asynchronous -group [get_clocks {clk50}] -group [get_clocks {cmos_pclk}]

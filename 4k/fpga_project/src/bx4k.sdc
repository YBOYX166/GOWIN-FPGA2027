# =====================================================================
# Tang Nano 4K (GW1NSR-LV4C) 时序约束
#
# 与 138K 主工程的 src/fpga_project.sdc 对应的 4K 版本。
# ★ 本文件不要往 138K 搬 —— 时钟频率和约束对象都不同。
#
# 注意：本文件是 Tcl 语法，注释用 # ，不是 // 。
# =====================================================================

# 主时钟 27MHz（Tang Nano 4K 板载晶振，引脚 45）
#   周期 = 1 / 27MHz = 37.037ns
#
# 这个数字必须和 top.v 里的 localparam CLK_FREQ = 27_000_000 一致。
# 两者不一致时工具不会报错，但时序分析结果是错的（约束无效），
# 属于"看起来通过了、其实没分析"的坑。
create_clock -name clk27 -period 37.037 [get_ports {Clk}]

# =====================================================================
# 将来如果加了 PLL（改 clk_gen.v），需要在这里补生成时钟的约束，
# 否则 PLL 输出域的逻辑不会被正确分析。
#
# 两选一：
#   1) 让工具自动派生（推荐，简单）—— 不用写任何东西，
#      Gowin 会把 rPLL 的 CLKOUT 自动识别为生成时钟；
#   2) 显式声明：
#        create_generated_clock -name sys_clk \
#            -source [get_ports {Clk}] -divide_by 1 [get_pins {u_clk/u_rpll/CLKOUT}]
#
# 138K 主工程在 fpga_project.sdc 里记过一个真实教训：
#   PLL 生成时钟在 SDC 解析阶段还不存在，因此无法写进 set_clock_groups：
#       get_clocks {u_pll/.../CLKOUT0.default_gen_clk} -> TA2004（找不到）
#       get_clocks -include_generated_clocks {clk50}   -> TA2000（语法不支持）
#   结果跨时钟路径仍按同步关系分析，产生一条 hold 假报警。
#   这边将来如果引入多时钟域，会踩同一个坑，先在文档里记着。
# =====================================================================

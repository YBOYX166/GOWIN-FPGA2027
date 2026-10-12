# =====================================================================
# P&R 前运行：把 GW1NSR-4C 的 MODE 专用引脚放开为普通 IO。
#
# 为什么需要：
#   Tang Nano 4K 的板载红色 LED 落在引脚 10 上，而引脚 10 默认是
#   MODE（配置模式选择）专用脚。不放开就约束不了，P&R 会报
#   "dedicated pin (MODE)" 之类的错误。
#
# 在 Gowin IDE 中的等效操作：
#   Process -> Place & Route -> 运行本脚本
#   或 Project -> Configuration -> Dual-Purpose Pin
#      勾选 "Use MODE as regular IO"
#
# 这一条对应 138K 主工程的 src/pnr_dual_purpose.tcl：
#   138K 那边是 set_option -use_cpu_as_gpio 1
#                set_option -use_sspi_as_gpio 1
#   （摄像头 SCCB 的 SDA/SCLK 落在 CPU/SSPI 复用脚上）
#   两边问题同源、选项不同。★ 本文件不要往 138K 搬。
# =====================================================================

set_option -use_mode_as_gpio 1

# ---------------------------------------------------------------------
# GW1NSR-4C 全部可用的"专用脚放开为普通 IO"选项（从 Gowin EDA 中提取），
# 以后排针不够时按需打开：
#
#   set_option -use_jtag_as_gpio    1   # JTAG 四个脚（注意：会失去 JTAG 下载）
#   set_option -use_mspi_as_gpio    1   # 外部 SPI Flash 配置脚
#   set_option -use_sspi_as_gpio    1   # SSPI 配置脚
#   set_option -use_done_as_gpio    1   # DONE 脚
#   set_option -use_ready_as_gpio   1   # READY 脚
#   set_option -use_reconfig_as_gpio 1  # RECONFIG_N 脚
#
# ⚠️ 打开 use_jtag_as_gpio 后无法再用板载 BL702 通过 JTAG 下载，
#    只能用其他配置方式，请谨慎。
#
# ⚠️ Gowin 还有一条静态检查开关：
#      set_option -use_strict 1
#    打开后未约束/未使用的端口检查更严格，适合在移植到 138K 之前
#    用来找出"忘了传 CLK_FREQ"或"忘了约束管脚"的问题。
# ---------------------------------------------------------------------

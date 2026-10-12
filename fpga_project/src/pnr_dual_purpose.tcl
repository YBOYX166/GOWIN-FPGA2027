# =====================================================================
# P&R 前运行：把 GW5AT-138 的 CPU(JTAG) / SSPI 配置引脚复用为普通 IO。
# 摄像头 SCCB 的 SDA(T21) / SCLK(U21) 分别落在 CPU 与 SSPI 复用脚上，
# 不启用这两项会报 PR2017 "dedicated pin (CPU/SSPI)"。
# 在 Gowin IDE 中：Process -> Place & Route -> 运行本脚本（或等效勾选
# "Use JTAG as Regular IO" / "Use SSPI as Regular IO"）。
# =====================================================================
set_option -use_cpu_as_gpio 1
set_option -use_sspi_as_gpio 1

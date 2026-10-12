# Gowin gw_sh 全流程脚本 —— 最小 PWM 测试工程
set_device -name GW5AT-138C GW5AT-LV138PG484AC1/I0
set_option -top_module top_pwm
set_option -output_base_name pwm_test
set_option -verilog_std v2001
add_file -type verilog /Volumes/YBOYX/FPGA2027/fpga_project/test_pwm/top_pwm.v
add_file -type cst /Volumes/YBOYX/FPGA2027/fpga_project/test_pwm/pwm.cst
add_file -type sdc /Volumes/YBOYX/FPGA2027/fpga_project/test_pwm/pwm.sdc
run all

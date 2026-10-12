// =====================================================================
// 4K 侧顶层 —— GW1NSR-LV4C / Tang Nano 4K
//
// 这个工程的用途：在没有 138K 板子的这段时间里，把机械臂控制模块写完、
// 调通。**这里只放可移植的纯 RTL**，唯一设备相关的部分都隔离在
//   clk_gen.v / bx4k.cst / bx4k.sdc / pnr_dual_purpose.tcl
// 四个文件里，移植时这四个不搬。
//
// 当前 top 做：内部上电复位 + LED 心跳 + 4 路舵机 + 串口微调。
//
// ★ LED 心跳是 1Hz。烧进去之后数 LED 闪的速度：
//     1 秒一次 -> CLK_FREQ = 27_000_000 正确
//     0.54 秒一次 -> 你误用了 50_000_000，快 1.85 倍，必须改回来
//
// ⚠️⚠️ 如果烧录后舵机**自己平滑地来回摆、约 2 秒一个来回、4 路错开**，
//        那不是本文件的行为 —— 本文件在 SERVO_CTRL_MODE=0 时 4 路恒定
//        停在 90 度不动。那个现象是「模式 1 自动扫描」的签名
//        （HALF=45 + SWEEP_MS=1000 双程 = 2 秒 + PHASE_STEP=22 错开）。
//
//        原因几乎一定是：**板上配置 Flash 里还是旧固件**。
//        Gowin Programmer 默认只烧 SRAM（掉电即失），一断电/复位就
//        回到 Flash 里的旧程序。
//
//        确认方法：接串口跑 tools/servo_send.py -m。
//                  本版本有开机横幅 + 每秒心跳，那个扫描版没有串口。
//        修法：Programmer -> Edit -> Configure Device -> 选 Flash 再烧。
//
//        完整记录见 README 第 7.7 节。
// =====================================================================
`timescale 1ns / 1ps

module top (
    input  wire       Clk,     // Tang Nano 4K 板载 27MHz 晶振（引脚 45）
    input  wire       uart_rx, // 串口接收，接上位机（引脚 46）
    output wire       uart_tx, // 串口发送，回复上位机（引脚 44）
    output wire       Led,     // 板载红色 LED（引脚 10，MODE 复用脚）
    output wire [3:0] servo    // 4 路舵机 PWM（引脚 43/42/41/40）
);

    // =================================================================
    // 系统频率 —— 这里是全工程唯一需要区分 4K / 138K 的地方
    //
    //   4K  (GW1NSR-LV4C, Tang Nano 4K) = 27_000_000
    //   138K(GW5AT-138, BX72)           = 50_000_000
    //
    // 所有子模块都必须把 CLK_FREQ 传下去，**绝不允许在模块内部写死**。
    // 这样移植到 138K 时只改这一个数字，行为完全等价。
    // =================================================================
    localparam CLK_FREQ = 27_000_000;

    // ---------------- 内部上电复位（约 20ms 低脉冲）----------------
    // 刻意不使用外部复位按键：Tang Nano 4K 的按键引脚各版本资料不一致，
    // 而内部 POR 不依赖任何外部管脚，两边工程都能用。
    localparam POR_CNT_MAX = CLK_FREQ / 50;   // 27MHz -> 540_000 (约 20ms)

    reg [23:0] por_cnt = 24'd0;
    reg        por_n   = 1'b0;
    always @(posedge Clk) begin
        if (por_cnt < POR_CNT_MAX) begin
            por_cnt <= por_cnt + 1'b1;
            por_n   <= 1'b0;
        end else begin
            por_n   <= 1'b1;
        end
    end

    // ---------------- 时钟 ----------------
    // 当前 clk_gen 是 27MHz 直通；将来需要 PLL 时只改 clk_gen.v。
    wire sys_clk, clk_lock;
    clk_gen u_clk (
        .clkin  (Clk),
        .clkout (sys_clk),
        .lock   (clk_lock)
    );

    wire sys_rst_n = por_n & clk_lock;

    // =================================================================
    // LED 心跳：0.5s 亮 / 0.5s 灭
    // =================================================================
    localparam HALF_SEC = CLK_FREQ / 2;       // 13_500_000
    localparam HB_W     = $clog2(HALF_SEC);   // 24

    reg [HB_W-1:0] hb_cnt = 0;
    reg            led_reg = 1'b0;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            hb_cnt  <= 0;
            led_reg <= 1'b0;
        end else if (hb_cnt >= HALF_SEC - 1) begin
            hb_cnt  <= 0;
            led_reg <= ~led_reg;
        end else begin
            hb_cnt  <= hb_cnt + 1'b1;
        end
    end

    assign Led = led_reg;

    // =================================================================
    // ============  机械臂模块挂载点（在这里加例化）  ================
    // =================================================================
    //
    // 约定（务必遵守，否则移植时会痛）：
    //
    //   1. 模块名 / 端口名 / 参数名 尽量沿用 138K 主工程 src/ 里的风格，
    //      这样合并 = 复制文件 + gprj 加一行 + 这里加例化。
    //
    //   2. 每个模块的第一个参数一律是 CLK_FREQ，并且必须把上面的
    //      localparam CLK_FREQ 传进去。**不许用默认值。**
    //
    //   3. 时钟一律用 sys_clk，复位一律用 sys_rst_n（低有效）。
    //      不要在子模块里再生成时钟；需要分频就用使能脉冲（tick）。
    //
    //   4. 单位写进注释。"关节角是 0.01 度还是 raw count"、
    //      "PID 周期是 1ms 还是 10ms" 必须写死，别靠猜。
    //      （138K 主工程在这上面踩过坑：闸门计数 vs 计数/秒 差 100 倍。）
    //
    //   5. 输出信号统一走这里的 IO 展开区，不要在子模块里直接 assign
    //      到顶层端口。
    //
    // 例化模板（把注释去掉并改名字即可）：
    //
    //   localparam ARM_JOINTS = 6;
    //
    //   wire signed [15:0] joint_angle [0:ARM_JOINTS-1];
    //   wire signed [15:0] joint_speed [0:ARM_JOINTS-1];
    //
    //   arm_kinematics #(
    //       .CLK_FREQ (CLK_FREQ),          // ← 必须传，别用默认值
    //       .JOINTS   (ARM_JOINTS)
    //   ) u_kin (
    //       .clk      (sys_clk),
    //       .rst_n    (sys_rst_n),
    //       .angle    (joint_angle),
    //       .speed    (joint_speed)
    //   );
    //
    //   arm_joint_pid #(
    //       .CLK_FREQ (CLK_FREQ),
    //       .CTRL_MS  (1)                  // 1kHz 控制周期
    //   ) u_pid0 (
    //       .clk      (sys_clk),
    //       .rst_n    (sys_rst_n),
    //       .target   (target0),
    //       .actual   (joint_speed[0]),
    //       .out      (cmd0)
    //   );
    //
    // =================================================================
    // ============  4 路舵机：中心 ±45 度往复扫描测试  =================
    // =================================================================
    //
    //   两套角度源，用 SERVO_CTRL_MODE 二选一（未选中的会被裁掉）：
    //
    //   【模式 0 · 串口手动微调】当前默认
    //     上电 4 路停在中位 90 度。发 4 个 ASCII 数字控制指定舵机：
    //         <舵机号 1-4><角度百位><十位><个位>
    //         例：2090 -> 2 号舵机转到 90
    //     带不带回车都行，非数字字符会清空计数（防止误触发）。
    //     每次回复一行：  S2=090 A045  /  ERR
    //     详细协议见 src/servo_cmd.v 顶部注释。
    //
    //   【模式 1 · 自动扫描】
    //     4 路在 45<->135 度（中心 ±45）往复，单程 1000ms。
    //     相位差 22 度，任意时刻 4 路位置都不同。
    //     用 SERVO_HALF / SERVO_SWEEP_MS 调幅度和速度。
    //
    //   ⚠️ 相位错开（SERVO_PHASE_STEP）不要随便改成 0：
    //      4 路参数完全一样时，Gowin 会判定它们"逻辑等价"并**合并成一路**
    //      再扇出到 4 个脚（实测综合日志报 g_servo[1..3] is swept），
    //      4 个脚输出完全相同的波形。功能没错，但你就无法判断
    //      "3 号舵机到底接在 41 还是 42 上"—— 排查接线时就没用了。
    //      （模式 0 下各路角度来自串口、本来就互不相同，不受影响。）
    //
    //   接线：servo[0..3] -> 舵机信号线（橙/黄线）
    //         舵机电源（红/棕）**不要接 FPGA 板**，见 README 的供电警告。
    // =================================================================
    localparam SERVO_N          = 4;
    localparam SERVO_CENTER     = 90;    // 中心角度 / 上电默认角度
    localparam SERVO_HALF       = 45;    // 扫描幅度 -> 45..135 度
    localparam SERVO_SWEEP_MS   = 1000;  // 单程耗时
    localparam SERVO_PHASE_STEP = 22;    // 每路相位差（度）；0 = 完全同步

    // ---- 控制模式（改这一行切换）----
    //   0 = 串口手动微调（当前）：上电 4 路停在中位 90 度，
    //       发 4 位数字控制指定舵机，其余不动
    //   1 = 自动扫描测试：4 路在中心 ±45 度往复（焊线/接线排查用）
    //
    // SERVO_CTRL_MODE 是常量，未选中的那一路会被综合器整体裁掉，不占资源。
    localparam SERVO_CTRL_MODE = 0;

    // ---- 角度源 A：串口手动 ----
    wire [31:0] cmd_angle;      // [7:0]=舵机0 [15:8]=舵机1 [23:16]=舵机2 [31:24]=舵机3
    servo_cmd #(
        .CLK_FREQ   (CLK_FREQ),      // ← 必须传，别用默认值
        .BAUD       (115200),
        .CENTER_DEG (SERVO_CENTER),
        .SCALE_360  (0)              // 0 = 收到的值直接就是舵机角度(0..180)
                                     //     （舵机是 180 度的，不用 ×2 那层映射）
    ) u_cmd (
        .Clk   (sys_clk),
        .Rst_n (sys_rst_n),
        .rx    (uart_rx),
        .tx    (uart_tx),
        .angle (cmd_angle)
    );

    wire [SERVO_N-1:0] servo_pwm_w;

    genvar si;
    generate
        for (si = 0; si < SERVO_N; si = si + 1) begin : g_servo
            // ---- 角度源 B：自动扫描（模式 0 时会被整体裁掉）----
            wire [7:0] sweep_ang;
            servo_sweep #(
                .CLK_FREQ       (CLK_FREQ),      // ← 必须传，别用默认值
                .CENTER_DEG     (SERVO_CENTER),
                .HALF_SWEEP_DEG (SERVO_HALF),
                .SWEEP_MS       (SERVO_SWEEP_MS),
                .PHASE_DEG      (si * SERVO_PHASE_STEP)
            ) u_sweep (
                .clk   (sys_clk),
                .rst_n (sys_rst_n),
                .angle (sweep_ang)
            );

            // ---- 二选一 ----
            wire [7:0] cmd_ang = cmd_angle[si*8 +: 8];
            wire [7:0] ang     = SERVO_CTRL_MODE ? sweep_ang : cmd_ang;

            servo_pwm #(
                .CLK_FREQ     (CLK_FREQ),        // ← 必须传
                .PWM_HZ       (50),              // 20ms 周期
                .PULSE_MIN_US (500),             // 0 度
                .PULSE_MAX_US (2500),            // 180 度
                .ANGLE_MAX    (180)
            ) u_pwm (
                .clk   (sys_clk),
                .rst_n (sys_rst_n),
                .angle (ang),
                .pwm   (servo_pwm_w[si])
            );
        end
    endgenerate

    assign servo = servo_pwm_w;

    // =================================================================
    // 以后加更多外设时的可用排针（已扣除板载占用）
    // =================================================================
    //   Slot 1: [43 42 41 40] ← 已被 4 路舵机占用
    //   Slot 2:  39 35 34 32
    //   Slot 3:  31 33 16 13
    //   Slot 4:  17 18 19 20
    //   Slot 5:  21 22 23 27
    //   Slot 6:  28 29 30
    //
    //   已占用：45 = 27MHz 时钟，44 = UART TX，46 = UART RX，10 = LED
    //
    // ⚠️ 这块板有 **三种不同的 IO 电压**，部分排针槽内混压。加外设前务必
    //    对照原理图 Tang_Nano_4K_3603_Schematic_.pdf 确认 BANK_VCCIO。
    // =================================================================

endmodule

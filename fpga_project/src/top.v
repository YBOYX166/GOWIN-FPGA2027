// =====================================================================
// BX72 (GW5AT-138) OV5640 DVP RGB565 -> 800x480 RGB 屏
// 采用 ch41 官方 camera_init + DVP_Capture，像素写入单帧缓冲后给 LCD。
// LCD 复位/时钟不依赖 KEY2 和摄像头，上电先出彩条。
// 另：Gpio1 接 2 片 AT8236 驱动四驱麦轮（motor_demo -> mecanum_mixer ->
//     motor_quad），Gpio1[21:14] 读 4 路编码器。
// =====================================================================
module top (
    input  wire        Clk,          // 50MHz (Y18)
    input  wire        Rst_n,        // KEY2 低有效 (V17)，仅复位摄像头链路
    inout  wire [13:0] camera_io,    // CAMERA 接口，见 BX72管脚分配表V2.1.md
    output wire        camera_rst_n, // CAMERA_RST  P3.15 / F15，低有效
    output wire        camera_pwdn,  // CAMERA_PWDN P3.16 / F21，高有效
    input  wire        uart_rx,      // 板载 USB-UART 接收 N13，上位机遥控指令
    output wire        uart_tx,      // 板载 USB-UART 发送 N14，编码器速度上报
    inout  wire [35:0] Gpio0,
    inout  wire [35:0] Gpio1,
    output wire [1:0]  Led
);
    localparam IMG_W     = 800;
    localparam IMG_H     = 480;
    localparam IMG_PIXEL = IMG_W * IMG_H;

    // ---------------- 上电复位（内部产生 ~20ms 低脉冲） ----------------
    reg [20:0] por_cnt = 21'd0;
    reg        por_n   = 1'b0;
    always @(posedge Clk) begin
        if (por_cnt < 21'd1_000_000) begin
            por_cnt <= por_cnt + 1'b1;
            por_n   <= 1'b0;
        end else begin
            por_n <= 1'b1;
        end
    end

    // ---------------- 时钟：PLL 33.33MHz LCD + 24MHz XCLK ----------------
    wire lcd_pclk, clk_cmos, pll_lock;
    sys_pll u_pll (
        .clkin  (Clk),
        .clkout (lcd_pclk),
        .xclk   (clk_cmos),
        .lock   (pll_lock)
    );

    wire cam_sys_rst_n = Rst_n & por_n & pll_lock;

    // LCD 域复位：只等 PLL lock，不跟 KEY2
    reg [3:0] rst_lcd_sync = 4'b0000;
    always @(posedge lcd_pclk) rst_lcd_sync <= {rst_lcd_sync[2:0], pll_lock};
    wire rst_lcd_n = rst_lcd_sync[3];

    // TFT 面板硬件复位：上电后保持约 10ms 低，再拉高释放
    reg [18:0] tft_rst_cnt = 19'd0;
    always @(posedge lcd_pclk or negedge rst_lcd_n) begin
        if (!rst_lcd_n)
            tft_rst_cnt <= 19'd0;
        else if (tft_rst_cnt < 19'd333_333)
            tft_rst_cnt <= tft_rst_cnt + 1'b1;
    end
    wire tft_rst_n = (tft_rst_cnt >= 19'd333_333);

    // ---------------- 摄像头引脚 ----------------
    // camera_io: [0]SCLK [1]HREF [2]D7 ... [9]D0 [10]SDA [11]XCLK [12]PCLK [13]VSYNC
    wire        cmos_sclk;
    wire        cmos_pclk  = camera_io[12];
    wire        cmos_href  = camera_io[1];
    wire        cmos_vsync = camera_io[13];
    wire [7:0]  cmos_din   = {camera_io[2], camera_io[3], camera_io[4], camera_io[5],
                              camera_io[6], camera_io[7], camera_io[8], camera_io[9]};
    assign camera_io[0]  = cmos_sclk;
    assign camera_io[11] = clk_cmos;

    wire camera_init_done;
    camera_init #(
        .SYS_CLOCK       (50_000_000),
        .SCL_CLOCK       (400_000),
        .CAMERA_TYPE     ("ov5640"),
        .IMAGE_TYPE      (0),
        .IMAGE_WIDTH     (IMG_W),
        .IMAGE_HEIGHT    (IMG_H),
        .IMAGE_FLIP_EN   (0),
        .IMAGE_MIRROR_EN (0)
    ) u_camera_init (
        .Clk          (Clk),
        .Rst_n        (cam_sys_rst_n),
        .Init_Done    (camera_init_done),
        .camera_rst_n (camera_rst_n),
        .camera_pwdn  (camera_pwdn),
        .i2c_sclk     (cmos_sclk),
        .i2c_sdat     (camera_io[10])
    );

    wire        dvp_valid;
    wire [15:0] dvp_pixel;
    wire        dvp_vs;
    wire [10:0] dvp_x;
    wire [10:0] dvp_y;
    DVP_Capture u_dvp (
        .Rst_n      (cam_sys_rst_n),
        .PCLK       (cmos_pclk),
        .Vsync      (cmos_vsync),
        .Href       (cmos_href),
        .Data       (cmos_din),
        .ImageState (),
        .DataValid  (dvp_valid),
        .DataPixel  (dvp_pixel),
        .DataHs     (),
        .DataVs     (dvp_vs),
        .Xaddr      (dvp_x),
        .Yaddr      (dvp_y)
    );

    // 4x4 Bayer dither while packing RGB565 -> RGB444, reduces brightness banding.
    wire [3:0] bayer = {dvp_y[1], dvp_x[1], dvp_y[0], dvp_x[0]} ^ {dvp_y[1:0], 2'b00};
    wire [5:0] r5 = {1'b0, dvp_pixel[15:11]};
    wire [6:0] g6 = {1'b0, dvp_pixel[10:5]};
    wire [5:0] b5 = {1'b0, dvp_pixel[4:0]};
    wire [5:0] r_d = r5 + {4'd0, bayer[3:2]};
    wire [6:0] g_d = g6 + {5'd0, bayer[3:2]};
    wire [5:0] b_d = b5 + {4'd0, bayer[3:2]};
    wire [3:0] r4 = r_d[5] ? 4'hF : r_d[4:1];
    wire [3:0] g4 = g_d[6] ? 4'hF : g_d[5:2];
    wire [3:0] b4 = b_d[5] ? 4'hF : b_d[4:1];
    wire [11:0] pixel444 = {r4, g4, b4};

    wire cap_en = camera_init_done;
    reg [18:0] wr_pix;
    reg        frame_full;
    reg        cam_frame_ok;
    always @(posedge cmos_pclk or negedge cam_sys_rst_n) begin
        if (!cam_sys_rst_n) begin
            wr_pix <= 19'd0;
            frame_full <= 1'b0;
            cam_frame_ok <= 1'b0;
        end else if (!cap_en) begin
            wr_pix <= 19'd0;
            frame_full <= 1'b0;
        end else if (!dvp_vs) begin
            wr_pix <= 19'd0;
            frame_full <= 1'b0;
        end else if (fb_wr_en) begin
            if (wr_pix == IMG_PIXEL - 1) begin
                frame_full <= 1'b1;
                cam_frame_ok <= 1'b1;
            end else
                wr_pix <= wr_pix + 1'b1;
        end
    end
    wire        fb_wr_en   = cap_en && dvp_valid && !frame_full;
    wire [18:0] fb_wr_addr = wr_pix;

    reg [2:0] cam_valid_sync;
    always @(posedge lcd_pclk or negedge rst_lcd_n) begin
        if (!rst_lcd_n) cam_valid_sync <= 3'b000;
        else            cam_valid_sync <= {cam_valid_sync[1:0], cam_frame_ok};
    end
    reg cam_valid;
    wire lcd_frame_start;
    always @(posedge lcd_pclk or negedge rst_lcd_n) begin
        if (!rst_lcd_n)
            cam_valid <= 1'b0;
        else if (!cam_valid_sync[2])
            cam_valid <= 1'b0;
        else if (lcd_frame_start)
            cam_valid <= 1'b1;
    end

    wire [18:0] lcd_rd_addr;
    wire [11:0] fb_rd_data;
    dpram_async #(
        .DATA_W (12),
        .ADDR_W (19),
        .DEPTH  (IMG_PIXEL)
    ) u_fb (
        .wr_clk  (cmos_pclk),
        .wr_en   (fb_wr_en),
        .wr_addr (fb_wr_addr),
        .wr_data (pixel444),
        .rd_clk  (lcd_pclk),
        .rd_addr (lcd_rd_addr),
        .rd_data (fb_rd_data)
    );

    wire [23:0] lcd_rgb;
    wire        lcd_de, lcd_hs, lcd_vs, lcd_clk;
    lcd_rgb_driver u_lcd (
        .pclk        (lcd_pclk),
        .rst_n       (rst_lcd_n),
        .rd_data     (fb_rd_data),
        .cam_valid   (cam_valid),
        .rd_addr     (lcd_rd_addr),
        .lcd_rgb     (lcd_rgb),
        .lcd_de      (lcd_de),
        .lcd_hs      (lcd_hs),
        .lcd_vs      (lcd_vs),
        .lcd_clk     (lcd_clk),
        .frame_start (lcd_frame_start)
    );

    // =====================================================================
    // 电机驱动（2 片 AT8236，四驱麦轮，闭环速度 PI）
    //   cmd_rx    —— 指令源（上位机遥控；带掉线保护，断连自动停车）
    //   motor_demo —— 备用指令源（台架演示，CTRL_SRC_UART=0 时启用）
    //   mecanum_mixer —— (vx,vy,w) -> 四轮目标转速
    //   motor_quad —— 4×(encoder_reader -> speed_pi -> motor_channel)
    // 占用 Gpio1[13:0] 输出（PWM/IN1/IN2/STBY），Gpio1[21:14] 输入（编码器 A/B）
    // 轮序: 轮0=左前 轮1=右前 轮2=左后 轮3=右后（motor_quad 注释有接线图）
    // =====================================================================

    // ---- 闭环 PI 整定参数（GATE_MS=10ms -> 控制频率 100Hz）----
    // KP 先单独调（KI 置 0），再加 KI 消稳态误差。
    localparam PI_KP     = 4;      // 比例增益
    localparam PI_KI     = 1;      // 积分增益
    localparam PI_SHIFT  = 0;      // 增益右移（给 KP/KI 加小数分辨率）
    localparam PI_I_MAX  = 2000;   // 积分器限幅（抗饱和）
    localparam PI_FB_TO  = 100;    // 反馈丢失保护：100 个闸门 = 1s（0=关闭）

    localparam MOTOR_ENABLE = 1'b1;

    // ---- 指令源 ----
    // MODE=1 固定指令，V 的单位 = 编码器计数/闸门
    //   V = RPM * CPR * GATE_MS / 60000
    //   CPR=1320、GATE_MS=10ms 时 V = RPM*0.22，例 500RPM -> V=110
    // ---- 指令源选择 ----
    //   1 = 上位机遥控（cmd_rx，正常使用）
    //   0 = motor_demo 台架演示（不接上位机时验接线/转向/编码器方向用）
    localparam CTRL_SRC_UART = 1;

    // ---- 指令源 A：上位机遥控 ----
    //   帧格式见 src/cmd_rx.v 头部。每帧 14 字节 ASCII，方便串口助手手打调试。
    //   **掉线保护**：500ms 收不到合法帧就输出 0，车自己停；上电时也是停止态，
    //   不会像 motor_demo 那样一上电就往前冲。
    wire signed [15:0] cmd_vx, cmd_vy, cmd_w;
    wire               cmd_link_ok;

    cmd_rx #(
        .CLK_FREQ   (50_000_000),
        .BAUD       (115200),
        .VMAX       (1000),
        .TIMEOUT_MS (500)
    ) u_cmd_rx (
        .Clk     (Clk),
        .Rst_n   (por_n),
        .rx      (uart_rx),
        .vx      (cmd_vx),
        .vy      (cmd_vy),
        .w       (cmd_w),
        .link_ok (cmd_link_ok)      // 需要时可用 GAO 观察；暂未接出
    );

    // ---- 指令源 B：motor_demo 台架演示（CTRL_SRC_UART=0 时启用）----
    localparam DEMO_V    = 700;
    localparam DEMO_MODE = 1;      // 0 = 8 相位演示序列

    wire signed [15:0] demo_vx, demo_vy, demo_w;
    motor_demo #(
        .CLK_FREQ (50_000_000),
        .V        (DEMO_V),
        .MODE     (DEMO_MODE)
    ) u_motor_demo (
        .Clk   (Clk),
        .Rst_n (por_n),
        .vx    (demo_vx),
        .vy    (demo_vy),
        .w     (demo_w)
    );

    // ---- 二选一（常数选择，综合时没被选中的那路会被优化掉）----
    wire signed [15:0] src_vx = CTRL_SRC_UART ? cmd_vx : demo_vx;
    wire signed [15:0] src_vy = CTRL_SRC_UART ? cmd_vy : demo_vy;
    wire signed [15:0] src_w  = CTRL_SRC_UART ? cmd_w  : demo_w;

    wire signed [15:0] spd_fl, spd_fr, spd_rl, spd_rr;
    // =====================================================================
    // 【方向修正开关】—— 接线/电机极性不对时在这里改，不用重新焊线
    //
    //   怎么判断该改哪个？把车架空（轮子悬空），用上位机发单个方向，
    //   看四个轮子的**实际转向**，然后对下表：
    //
    //   现象                                  要设的开关
    //   ─────────────────────────────────────────────────────────
    //   前进变后退                             INV_VX = 1
    //   左移变右移                             INV_VY = 1
    //   左转变右转                             INV_W  = 1
    //   以上全部反过来（整个镜像了）            INV_VX/INV_VY/INV_W 全设 1
    //                                          等价于 INV_WHEEL = 4'b1111
    //   只有某一个轮子转反了                   INV_WHEEL 对应位置 1
    //                                          轮0=左前 bit0 … 轮3=右后 bit3
    //
    //   ⚠️ 注意：纯前进时四轮指令完全相同，所以**"前后轮接反"不会导致
    //      前进变后退** —— 那种情况是横移和自转会反，不是前后。
    //      如果前进变后退，一定是电机 +/- 极性反了。
    // =====================================================================
    localparam INV_VX     = 0;            // 前后反向
    localparam INV_VY     = 0;            // 左右反向
    localparam INV_W      = 0;            // 自转反向
    localparam [3:0] INV_WHEEL = 4'b0000; // 逐轮反向（见上表）

    wire signed [15:0] mix_vx = INV_VX ? -src_vx : src_vx;
    wire signed [15:0] mix_vy = INV_VY ? -src_vy : src_vy;
    wire signed [15:0] mix_w  = INV_W  ? -src_w  : src_w;

    mecanum_mixer #(
        .OUT_MAX      (1000),
        .INVERT_WHEEL (INV_WHEEL),
        // K_GAIN/2^GAIN_SHIFT = 1024/1024 = 1.0 -> kw 就等于 w，
        // 也就是教科书公式:  FL=vx-vy-w  FR=vx+vy+w  RL=vx+vy-w  RR=vx-vy+w
        // 这样 vx/vy/w 三者量纲一致，"转 600" 和 "走 600" 一样快。
        // 上位机 car_gui.py 里按同一套公式做预缩放（防饱和失真），两边必须一致。
        .K_GAIN     (1024),
        .GAIN_SHIFT (10)
    ) u_mixer (
        .Clk    (Clk),
        .Rst_n  (por_n),
        .vx     (mix_vx),
        .vy     (mix_vy),
        .w      (mix_w),
        .spd_fl (spd_fl),
        .spd_fr (spd_fr),
        .spd_rl (spd_rl),
        .spd_rr (spd_rr)
    );

    wire        [3:0]         mt_pwm, mt_in1, mt_in2;
    wire        [1:0]         mt_stby;
    wire        [3:0]         mt_fault;
    // syn_keep: 便于 GAO 在线逻辑分析仪观察
    (* syn_keep = 1 *) wire signed [127:0] mt_pos;
    (* syn_keep = 1 *) wire signed [63:0]  mt_speed;
    (* syn_keep = 1 *) wire                mt_speed_valid;
    (* syn_keep = 1 *) wire        [3:0]   mt_fault_keep;
    motor_quad #(
        .CLK_FREQ         (50_000_000),
        .PWM_FREQ         (20_000),
        .SPEED_MAX        (1000),
        .GATE_MS          (10),
        // S0 和 S3 的编码器相序与另外两路相反，统一交换这两路正负号。
        .DIR_SWAP         (4'b1001),
        .CLOSED_LOOP      (0),
        .IFACE_TB6612     (MOTOR_IFACE),
        // 速度 0 时刹车（不是滑行）：轮子锁死、斜坡不溜车，
        // 且 AT8236 不会休眠 -> 起步没有 ~1ms 延迟。
        .BRAKE_ON_ZERO    (1),
        .KP               (PI_KP),
        .KI               (PI_KI),
        .GAIN_SHIFT       (PI_SHIFT),
        .I_MAX            (PI_I_MAX),
        .FEEDBACK_TIMEOUT (PI_FB_TO)
    ) u_motors (
        .Clk        (Clk),
        .Rst_n      (por_n),
        .enable     (MOTOR_ENABLE),
        .speed_cmd  ({spd_rr, spd_rl, spd_fr, spd_fl}),  // [15:0]=轮0=左前
        .pwm        (mt_pwm),
        .in1        (mt_in1),
        .in2        (mt_in2),
        .stby       (mt_stby),
        // 编码器按"相邻对"接线：轮n 的 A/B 挨在一起
        //   轮0: A=P2-17(Gpio1[14])  B=P2-18(Gpio1[15])
        //   轮1: A=P2-19(Gpio1[16])  B=P2-20(Gpio1[17])
        //   轮2: A=P2-21(Gpio1[18])  B=P2-22(Gpio1[19])
        //   轮3: A=P2-23(Gpio1[20])  B=P2-24(Gpio1[21])
        .enc_a      ({Gpio1[20], Gpio1[18], Gpio1[16], Gpio1[14]}),
        .enc_b      ({Gpio1[21], Gpio1[19], Gpio1[17], Gpio1[15]}),
        .enc_pos    (mt_pos),
        .enc_speed  (mt_speed),
        .speed_valid(mt_speed_valid),
        .fault      (mt_fault)
    );
    assign mt_fault_keep = mt_fault;

    // =====================================================================
    // 编码器速度上报：每 200ms 通过板载 USB-UART 打印一行
    //   S0=+0123 S1=+0119 S2=-0004 S3=+0121\r\n
    //   单位：**计数/秒**（counts per second，带符号）—— M/T 法测速输出
    //   RPM = cps * 60 / CPR
    //        例 CPR=1320 -> RPM = cps * 0.04545   （cps=2200 -> 100 RPM）
    //
    //   ⚠️ 单位已从"每 10ms 计数"改成"计数/秒"，数值大了 100 倍。
    //      将来开闭环时，speed_cmd(PI 的 target) 必须也用 cps，
    //      否则 target 和 actual 量纲不一致，闭环不成立。
    // =====================================================================
    enc_report #(
        .CLK_FREQ (50_000_000),
        .BAUD     (115200),
        .REPORT_MS(200)
    ) u_enc_report (
        .Clk    (Clk),
        .Rst_n  (por_n),
        .spd    (mt_speed),
        .uart_tx(uart_tx)
    );

    reg [35:0] gpio0_out, gpio0_oe;
    reg [35:0] gpio1_out, gpio1_oe;
    genvar g;
    generate
        for (g = 0; g < 36; g = g + 1) begin : g0
            assign Gpio0[g] = gpio0_oe[g] ? gpio0_out[g] : 1'bz;
        end
        for (g = 0; g < 36; g = g + 1) begin : g1
            assign Gpio1[g] = gpio1_oe[g] ? gpio1_out[g] : 1'bz;
        end
    endgenerate

    // =====================================================================
    // Gpio1 输出模式开关
    //   0 = 正常电机驱动（motor_quad -> PWM / IN1 / IN2 / STBY）
    //   1 = 【当前】4 路 PWM 验证：Gpio1[3:0] = 20kHz / 50%，
    //       其余排针脚 = 2Hz / 50% 身份标记（P2-33 留给屏）
    //       完全不依赖编码器 / PI / 电机 / 摄像头
    //   2 = 全组调试：Gpio1[35:0] 全部输出同一个固定 PWM
    //   3 = 【AT8236 转轮自检】直接给 4 路 50% 固定 PWM（不走 mixer/PI/demo）：
    //       Gpio1[0/2/4/6] = IN1 = 20kHz 50%，Gpio1[1/3/5/7] = IN2 = 0
    //
    // 模式 1/2 下 Gpio1[21:14]（编码器输入）会变成输出，
    // 若编码器已接线请先断开，避免两边对驱。
    // =====================================================================
    // 驱动模块接口：1 = TB6612FNG / DRV8833（PWM+IN1+IN2+STBY）
    //               0 = AT8236（IN1/IN2 双 PWM，无 STBY）
    localparam MOTOR_IFACE    = 1;       // ← 换模块时改这里
    //   ★ 遥控必须用模式 0。模式 1/2/3 是 Gpio1 自检，会**完全绕过**整条电机
    //     控制链（motor_demo/cmd_rx -> mecanum_mixer -> motor_quad 全被综合器
    //     优化掉，电机只是固定方波）。
    localparam GPIO1_MODE     = 0;       // 0=电机驱动 1=PWM+标记 2=全组PWM 3=转轮自检
    localparam GPIO1_PWM_HZ   = 20_000;  // 4 路 PWM 频率（模式 1/2）
    localparam GPIO1_PWM_DUTY = 50;      // 占空比 %
    localparam GPIO1_MARK_HZ  = 2;       // 模式 1 其余脚的标记频率

    // ---- 4 路 PWM 用的固定方波 ----
    localparam TPWM_PERIOD = 50_000_000 / GPIO1_PWM_HZ;
    localparam TPWM_HIGH   = TPWM_PERIOD * GPIO1_PWM_DUTY / 100;

    reg [31:0] tpwm_cnt = 32'd0;
    always @(posedge Clk) begin          // 自由运行，无复位，只依赖 50MHz
        if (tpwm_cnt >= TPWM_PERIOD - 1) tpwm_cnt <= 32'd0;
        else                             tpwm_cnt <= tpwm_cnt + 1'b1;
    end
    wire gpio1_test_pwm = (tpwm_cnt < TPWM_HIGH);

    // ---- 模式 3 自检用的 50% 方波 ----
    wire gpio1_half_pwm = (tpwm_cnt < (TPWM_PERIOD / 2));

    // ---- 模式 1 的 2Hz 身份标记 ----
    localparam TMARK_PERIOD = 50_000_000 / GPIO1_MARK_HZ;
    localparam TMARK_HIGH   = TMARK_PERIOD / 2;

    reg [31:0] tmark_cnt = 32'd0;
    always @(posedge Clk) begin
        if (tmark_cnt >= TMARK_PERIOD - 1) tmark_cnt <= 32'd0;
        else                               tmark_cnt <= tmark_cnt + 1'b1;
    end
    wire gpio1_mark_pwm = (tmark_cnt < TMARK_HIGH);

    always @(*) begin
        gpio0_oe = 36'd0; gpio0_out = 36'd0;
        gpio1_oe = 36'd0; gpio1_out = 36'd0;

        gpio0_oe[1]  = 1'b1; gpio0_out[1]  = tft_rst_n;
        gpio0_oe[7]  = 1'b1; gpio0_out[7]  = 1'b1;
        gpio0_oe[6]  = 1'b1; gpio0_out[6]  = lcd_de;
        gpio0_oe[8]  = 1'b1; gpio0_out[8]  = lcd_hs;
        gpio0_oe[9]  = 1'b1; gpio0_out[9]  = lcd_vs;
        gpio0_oe[11] = 1'b1; gpio0_out[11] = lcd_clk;

        // ---------------- Gpio1 ----------------
        if (GPIO1_MODE == 1) begin
            // [模式 1] P2-1~4 固定 20kHz；其余排针脚 2Hz 身份标记
            gpio1_oe[3:0]    = 4'hF;      gpio1_out[3:0]    = {4{gpio1_test_pwm}};
            gpio1_oe[13:4]   = 10'h3FF;   gpio1_out[13:4]   = {10{gpio1_mark_pwm}};
            gpio1_oe[27:14]  = 14'h3FFF;  gpio1_out[27:14]  = {14{gpio1_mark_pwm}};
            gpio1_oe[35:29]  = 7'h7F;     gpio1_out[35:29]  = {7{gpio1_mark_pwm}};
            gpio1_oe[28] = 1'b1; gpio1_out[28] = lcd_rgb[21];   // LCD_R5 留给屏
        end else if (GPIO1_MODE == 3) begin
            // [模式 3] 转轮自检：直接固定 50% 正转，不经过任何运动逻辑
            if (MOTOR_IFACE == 1) begin
                // TB6612 / DRV8833：PWM=50%，AIN1=1 AIN2=0 BIN1=1 BIN2=0，STBY=1
                //   片0 = P2-5~8   -> 0101 (bit4=AIN1,5=AIN2,6=BIN1,7=BIN2)
                //   片1 = P2-9/10/13/14 -> 0101
                gpio1_oe[3:0]    = 4'hF;  gpio1_out[3:0]    = {4{gpio1_half_pwm}};
                gpio1_oe[7:4]    = 4'hF;  gpio1_out[7:4]    = 4'b0101;
                gpio1_oe[11:8]   = 4'hF;  gpio1_out[11:8]   = 4'b0101;
                gpio1_oe[13:12]  = 2'b11; gpio1_out[13:12]  = 2'b11;
            end else begin
                // AT8236：IN1=50%（偶数脚），IN2=0（奇数脚）
                gpio1_oe[7:0]  = 8'hFF;
                gpio1_out[7:0] = {1'b0, gpio1_half_pwm, 1'b0, gpio1_half_pwm,
                                  1'b0, gpio1_half_pwm, 1'b0, gpio1_half_pwm};
            end
            gpio1_oe[28] = 1'b1; gpio1_out[28] = lcd_rgb[21];   // LCD_R5
        end else if (GPIO1_MODE == 2) begin
            // [模式 2] 全组 36 根输出同一个固定 PWM
            gpio1_oe  = {36{1'b1}};
            gpio1_out = {36{gpio1_test_pwm}};
        end else begin
            // [模式 0] 电机驱动输出，按 MOTOR_IFACE 选择接法
            if (MOTOR_IFACE == 1) begin
                // TB6612FNG / DRV8833：**按模块分组**，一个模块的 4 根方向线挨在一起
                //   P2-1~4        PWM0 PWM1 PWM2 PWM3
                //   P2-5~8        片0：AIN1 AIN2 BIN1 BIN2
                //   P2-9,10,13,14 片1：AIN1 AIN2 BIN1 BIN2   （P2-11/12 是 5V/GND，跳过）
                //   P2-15,16      STBY0 STBY1
                gpio1_oe[3:0]    = 4'hF;  gpio1_out[3:0]    = mt_pwm;
                gpio1_oe[7:4]    = 4'hF;
                gpio1_out[7:4]   = {mt_in2[1], mt_in1[1], mt_in2[0], mt_in1[0]};  // 片0
                gpio1_oe[11:8]   = 4'hF;
                gpio1_out[11:8]  = {mt_in2[3], mt_in1[3], mt_in2[2], mt_in1[2]};  // 片1
                gpio1_oe[13:12]  = 2'b11; gpio1_out[13:12]  = mt_stby;
            end else begin
                // AT8236：每轮 2 个脚（IN1 / IN2），共 8 个
                //   Gpio1[7:0] -> P2-1 ~ P2-8
                //     轮0: [0]=IN1 [1]=IN2      轮1: [2]=IN1 [3]=IN2
                //     轮2: [4]=IN1 [5]=IN2      轮3: [6]=IN1 [7]=IN2
                gpio1_oe[7:0]  = 8'hFF;
                gpio1_out[7:0] = {mt_in2[3], mt_in1[3], mt_in2[2], mt_in1[2],
                                  mt_in2[1], mt_in1[1], mt_in2[0], mt_in1[0]};
            end
            // Gpio1[21:14] 为编码器输入，oe 保持 0（高阻输入）
            gpio1_oe[28] = 1'b1; gpio1_out[28] = lcd_rgb[21];   // LCD_R5
        end

        gpio0_oe[26] = 1'b1; gpio0_out[26] = lcd_rgb[23];
        gpio0_oe[29] = 1'b1; gpio0_out[29] = lcd_rgb[22];
        gpio0_oe[31] = 1'b1; gpio0_out[31] = lcd_rgb[20];
        gpio0_oe[30] = 1'b1; gpio0_out[30] = lcd_rgb[19];
        gpio0_oe[33] = 1'b1; gpio0_out[33] = lcd_rgb[18];
        gpio0_oe[32] = 1'b1; gpio0_out[32] = lcd_rgb[17];
        gpio0_oe[35] = 1'b1; gpio0_out[35] = lcd_rgb[16];
        gpio0_oe[18] = 1'b1; gpio0_out[18] = lcd_rgb[15];
        gpio0_oe[21] = 1'b1; gpio0_out[21] = lcd_rgb[14];
        gpio0_oe[20] = 1'b1; gpio0_out[20] = lcd_rgb[13];
        gpio0_oe[23] = 1'b1; gpio0_out[23] = lcd_rgb[12];
        gpio0_oe[22] = 1'b1; gpio0_out[22] = lcd_rgb[11];
        gpio0_oe[25] = 1'b1; gpio0_out[25] = lcd_rgb[10];
        gpio0_oe[24] = 1'b1; gpio0_out[24] = lcd_rgb[9];
        gpio0_oe[27] = 1'b1; gpio0_out[27] = lcd_rgb[8];
        gpio0_oe[10] = 1'b1; gpio0_out[10] = lcd_rgb[7];
        gpio0_oe[13] = 1'b1; gpio0_out[13] = lcd_rgb[6];
        gpio0_oe[12] = 1'b1; gpio0_out[12] = lcd_rgb[5];
        gpio0_oe[15] = 1'b1; gpio0_out[15] = lcd_rgb[4];
        gpio0_oe[14] = 1'b1; gpio0_out[14] = lcd_rgb[3];
        gpio0_oe[17] = 1'b1; gpio0_out[17] = lcd_rgb[2];
        gpio0_oe[16] = 1'b1; gpio0_out[16] = lcd_rgb[1];
        gpio0_oe[19] = 1'b1; gpio0_out[19] = lcd_rgb[0];
    end

    // Led[0]: 初始化完成常亮，否则慢闪
    // Led[1]: 有完整帧则随场翻转；否则 PCLK 活动快闪 / PLL lock 常亮
    reg [24:0] blink;
    always @(posedge Clk) blink <= blink + 1'b1;
    assign Led[0] = camera_init_done ? 1'b1 : blink[24];

    reg [2:0] vs_sys, pclk_sys;
    always @(posedge Clk or negedge cam_sys_rst_n) begin
        if (!cam_sys_rst_n) begin
            vs_sys   <= 3'b000;
            pclk_sys <= 3'b000;
        end else begin
            vs_sys   <= {vs_sys[1:0],   cmos_vsync};
            pclk_sys <= {pclk_sys[1:0], cmos_pclk};
        end
    end
    wire vs_edge  = vs_sys[2] ^ vs_sys[1];
    wire pclk_tog = pclk_sys[2] ^ pclk_sys[1];
    reg led1_tog, pclk_seen;
    always @(posedge Clk or negedge cam_sys_rst_n) begin
        if (!cam_sys_rst_n) begin
            led1_tog <= 1'b0;
            pclk_seen <= 1'b0;
        end else begin
            if (pclk_tog)
                pclk_seen <= 1'b1;
            if (vs_edge)
                led1_tog <= ~led1_tog;
        end
    end
    assign Led[1] = cam_frame_ok ? led1_tog : (pclk_seen ? blink[23] : pll_lock);
endmodule

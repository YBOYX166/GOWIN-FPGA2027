# 电机驱动说明（2 片 AT8236 · 四驱麦轮 · 闭环速度 PI）

> 本文描述重写后的电机链路。`PROJECT_INTERFACE.md` 第 6 节描述的是旧的纯开环版本，
> 已被本文取代。

## 1. 数据通路

```
motor_demo            指令源（当前=固定指令，可切 8 相位演示序列）
   │  vx, vy, w            单位：编码器计数 / 测速闸门
   ▼
mecanum_mixer         麦轮 X 型解算（3 级流水），输出四轮目标转速 ±1000
   │  spd_fl/fr/rl/rr
   ▼
motor_quad            4 路，每路：
   ├─ encoder_reader  A/B 3 拍同步 + 4 倍频解码 + 10ms 闸门测速
   ├─ speed_pi        err=目标-实测 → PI → ±1000 指令（带抗饱和 + 反馈丢失保护）
   └─ motor_channel   |指令| → 20kHz PWM 占空比 + IN1/IN2 方向
```

- 轮序：轮0=左前、轮1=右前、轮2=左后、轮3=右后。
- 轮 0/1 接第 1 片 AT8236 的 A/B 通道，轮 2/3 接第 2 片。

## 2. 量纲（最关键的一点）

**闭环模式下，`vx/vy/w` 和四轮指令的单位都是"每个测速闸门内的编码器计数"**，
也就是 `encoder_reader` 输出的 `enc_speed` 的原始单位。这样解算结果可以直接当 PI 目标值，
不需要任何换算。

换算公式：

```
计数/闸门 = RPM × CPR × GATE_MS / 60000
```

- `CPR`：编码器每转 4 倍频计数。310 电机常见 A 相 11 PPR、减速比 30:1
  → CPR = 11 × 30 × 4 = **1320**
- `GATE_MS`：测速闸门，默认 **10 ms**

所以 CPR=1320、GATE_MS=10ms 时：

```
计数/闸门 = RPM × 0.22
```

| 目标转速 | `V`（= 计数/闸门） |
| ---: | ---: |
| 100 RPM | 22 |
| 300 RPM | 66 |
| 500 RPM | 110 |
| 1000 RPM | 220 |

`src/top.v` 里改这一行即可：

```verilog
localparam DEMO_V    = 110;   // 500 RPM
localparam DEMO_MODE = 1;     // 1=固定指令  0=8 相位演示序列
```

## 3. 引脚（未变）

| 信号 | Gpio1 位 | FPGA 管脚 | P2 排针 |
| --- | --- | --- | --- |
| PWM0..3 | [3:0] | F13/F14/E13/E14 | P2-1~4 |
| IN1_0..3 | [7:4] | D14/D15/F16/E17 | P2-5~8 |
| IN2_0..3 | [11:8] | E16/D16/A13/A14 | P2-9/10/13/14 |
| STBY0/1 | [13:12] | C13/B13 | P2-15/16 |
| ENC_A0..3 | [17:14] | C14/C15/B15/B16 | P2-17~20 |
| ENC_B0..3 | [21:18] | A15/A16/B17/B18 | P2-21~24 |
| LCD_R5（屏占用，勿动） | [28] | D20 | P2-33 |

## 4. PI 整定步骤

控制频率 = 1 / GATE_MS = **100 Hz**。参数在 `src/top.v`：

```verilog
localparam PI_KP    = 4;      // 比例增益
localparam PI_KI    = 1;      // 积分增益
localparam PI_SHIFT = 0;      // 增益右移（给 KP/KI 加小数分辨率）
localparam PI_I_MAX = 2000;   // 积分器限幅（抗饱和）
```

整定方法：

1. **先把 `PI_KI` 置 0**，只加 `PI_KP`：从 1 → 2 → 4 → 8 逐步加，
   直到轮子响应够快、但停下时不来回振荡。
2. **再加 `PI_KI`**：从 0 → 1 → 2 逐步加，消掉稳态误差（堵转/摩擦导致的转速偏低）。
3. 需要更细的分辨率时，把 `PI_KP`/`PI_KI`/`PI_SHIFT` 同时放大，
   例如 `KP=32, KI=8, SHIFT=3` 等价于 `KP=4, KI=1`，但可调步进更细。

旋转增益在 `mecanum_mixer` 的 `K_GAIN`（千分比）：

```verilog
mecanum_mixer #( .OUT_MAX(1000), .K_GAIN(300) )   // kw = 0.30 * w
```

## 5. 安全特性

### 反馈丢失保护

`PI_FB_TO = 100`（100 个闸门 = 1 s）：**目标非零、但实测转速连续 1 秒一直是 0**
→ 判定编码器未接/断线，该路输出强制归零并置 `fault`，目标回 0 时自动清除。

这一条很重要：如果没有它，编码器没插时 PI 会认为"转速不够"而一路加油门到 100% 占空比，
四轮直接满速冲出去。置 0 可关闭。

故障状态可以用 GAO 观察 `mt_fault_keep[3:0]`（`top.v` 里已加 `syn_keep`）。

### 使能

```verilog
localparam MOTOR_ENABLE = 1'b1;   // 置 0 时 stby 拉低，两片 AT8236 全部断电
```

## 6. Gpio1 输出模式开关（`src/top.v`）

顶层有一个三档开关，改一行就能切换 Gpio1 的行为：

```verilog
localparam GPIO1_MODE = 1;   // ← 改这里
```

| 模式 | 行为 | 用途 |
| ---: | --- | --- |
| **0** | 正常电机驱动：`Gpio1[3:0]`=PWM、`[7:4]`=IN1、`[11:8]`=IN2、`[13:12]`=STBY、`[21:14]`=编码器输入 | 实际跑电机 |
| **1** | `Gpio1[3:0]` = **20 kHz / 50% 固定方波**；其余排针脚 = **2 Hz / 50% 身份标记**（P2-33 仍给 LCD_R5） | **验证 4 路 PWM 与排针**，完全不依赖编码器/PI/电机/摄像头 |
| **2** | `Gpio1[35:0]` 全部输出同一个固定 PWM | 排查排针/焊接 |

模式 1 的两路方波发生器都是**自由运行、无复位、只依赖 50 MHz 时钟**的，所以 FPGA 一配置好就出波形：

```verilog
reg [31:0] tpwm_cnt = 32'd0;
always @(posedge Clk) begin          // 没有 Rst_n
    if (tpwm_cnt >= 2500-1) tpwm_cnt <= 0;
    else                    tpwm_cnt <= tpwm_cnt + 1;
end
wire gpio1_test_pwm = (tpwm_cnt < 1250);   // 20kHz / 50%
```

频率和占空比也可以调：

```verilog
localparam GPIO1_PWM_HZ   = 20_000;  // 4 路 PWM 频率
localparam GPIO1_PWM_DUTY = 50;      // 占空比 %
localparam GPIO1_MARK_HZ  = 2;       // 模式 1 其余脚的标记频率
```

> ⚠️ 模式 1/2 下 `Gpio1[21:14]`（编码器输入）会变成输出。若编码器已接线，请先断开，避免两边对驱。

## 7. 时序（重写前 vs 重写后，均为实际 PnR 报告数据）

旧版本 `mecanum_mixer` 把 `raw*1000/1000`（数学上是恒等变换）也综合成了乘除法，
`motor_channel` 里还留了一个 32bit 除法器，导致严重时序失败。

| 指标 | 重写前 | 重写后 |
| --- | ---: | ---: |
| `clk50` 实测 Fmax | 29.275 MHz ❌ | **87.586 MHz** ✅ |
| 最长逻辑级数 | 42 | **11** |
| setup 失败端点 | 44 | **0** |
| `clk50` setup TNS | −613.180 ns | **0.000 ns** |
| 最差 slack | −14.159 ns | 0（最紧正 slack +0.207 ns） |
| LUT | 4477 | 2409 |
| DSP | 8.5 | 4.5 |
| Register | 844 | 963 |
| BSRAM | 290/340 | 290/340（帧缓存占满，未变） |

三处改动：

1. `mecanum_mixer`：去掉恒等乘除，只留 `kw = K_GAIN*w/1000`，
   拆成 **乘法 / 除法 / 加法+限幅 三级流水**；
2. `motor_channel`：把 `|speed|*PERIOD/SPEED_MAX` 改写成
   **定点常量乘 + 移位**（`DUTY_MUL = (PERIOD<<10)/SPEED_MAX`，
   默认 2500/1000 → 2560，正好整除无误差），**彻底去掉除法器**；
3. `speed_pi`：只在测速闸门结束（100 Hz）更新一次，组合路径为
   乘法→加法→移位→限幅。

### 唯一剩下的一条负面路径（已知假报警）

```
slack = -0.439 ns   hold   cam_frame_ok_s0 -> cam_valid_sync_0_s0
                    cmos_pclk:[R] -> u_pll/.../CLKOUT0.default_gen_clk
```

这是 `top.v` 里标准的 3 级同步器 CDC，属于正常跨时钟传递。
工具之所以还分析它，是因为 PLL 生成时钟在 SDC 解析阶段尚不存在，
无法写进 `set_clock_groups`：

- `get_clocks {u_pll/.../CLKOUT0.default_gen_clk}` → 报 **TA2004**（找不到该时钟）
- `get_clocks -include_generated_clocks {clk50}` → 报 **TA2000**（语法错误，Gowin 不支持）

因此 `cmos_pclk <-> lcd_pclk` 的跨时钟路径仍按同步关系分析，产生这条相位假报警。
**不影响功能**，详见 `src/fpga_project.sdc` 中的注释。

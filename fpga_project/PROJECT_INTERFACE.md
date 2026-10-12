# BX72 FPGA 工程接口说明

本文档对应当前工程中的 `src/top.v`、相关 RTL、`src/bx72.cst` 和
`src/fpga_project.sdc`。顶层模块名为 `top`，器件为
`GW5AT-LV138PG484AC1/I0`（GW5AT-138）。

## 1. 顶层功能

当前 `top` 同时实现两条链路：

```text
OV5640 DVP
  -> camera_init（SCCB/I2C、复位、寄存器表）
  -> DVP_Capture（RGB565、丢弃前 10 帧）
  -> RGB565 转 RGB444
  -> 800x480 单帧双口 RAM
  -> lcd_rgb_driver（800x480 TTL LCD）

motor_demo
  -> mecanum_mixer
  -> motor_quad（4 路 PWM/方向 + 4 路正交编码器）
```

工程文件入口为 `fpga_project.gprj`。RTL 文件全部位于 `src/`；行为模型和测试平台位于 `sim/`。

## 2. 顶层端口

| 端口 | 方向 | 当前 RTL 行为 |
| --- | --- | --- |
| `Clk` | input | 50MHz 板载时钟，CST 管脚 Y18 |
| `Rst_n` | input | KEY2 低有效，CST 管脚 V17；只参与摄像头链路复位 |
| `camera_io[13:0]` | inout | 摄像头 DVP、SCCB、XCLK、PCLK 接口 |
| `camera_rst_n` | output | 摄像头复位输出，低有效，CST 管脚 F15 |
| `camera_pwdn` | output | 摄像头掉电控制输出，CST 管脚 F21；当前 RTL 固定输出 0 |
| `Gpio0[35:0]` | inout | TFT/LCD 控制和 RGB 数据，部分管脚保持高阻 |
| `uart_rx` | input | **上位机遥控指令输入**，CST 管脚 N13 |
| `uart_tx` | output | 编码器速度上报，CST 管脚 N14 |
| `Gpio1[35:0]` | inout | 电机控制、编码器输入，以及 LCD_R5 |
| `Led[1:0]` | output | 摄像头初始化、PCLK/帧活动状态指示 |

`Rst_n` 不会复位 LCD 计数器和电机控制链。LCD 只等待 PLL lock；电机使用内部约 20ms 上电复位 `por_n`。

## 3. 时钟和复位

`src/pll.v` 中的 `Gowin_PLL` 使用 50MHz `Clk` 产生：

| 时钟 | 频率 | 用途 |
| --- | ---: | --- |
| `Clk` | 50MHz | 摄像头初始化、电机和编码器 |
| `lcd_pclk` | 约 33.33MHz | LCD 时序和帧缓存读口 |
| `clk_cmos` | 24MHz | OV5640 XCLK |

复位行为：

- `por_n` 在 `Clk` 下保持低电平 1,000,000 个周期，约 20ms。
- `cam_sys_rst_n = Rst_n & por_n & pll_lock`，用于 `camera_init`、`DVP_Capture` 和摄像头写 RAM 控制。
- `rst_lcd_n` 是 `pll_lock` 在 `lcd_pclk` 域的 4 级同步结果。
- TFT `RESET`（`Gpio0[1]`）在 LCD 复位释放后继续保持约 10ms 低电平。
- `camera_init` 在 `cam_sys_rst_n` 释放后等待约 21ms，再开始写 OV5640 初始化寄存器，完成后拉高 `Init_Done`。

SDC 当前只显式约束 50MHz `Clk` 和外部摄像头 `camera_io[12]` PCLK，PCLK 周期设置为 20ns，并将这两个时钟组声明为异步。

## 4. 摄像头接口

### 4.1 管脚和位序

| 摄像头信号 | RTL 信号 | FPGA 管脚 |
| --- | --- | --- |
| SCLK | `camera_io[0]` | U21 |
| HREF | `camera_io[1]` | G21 |
| D7 | `camera_io[2]` | D22 |
| D6 | `camera_io[3]` | E22 |
| D5 | `camera_io[4]` | D21 |
| D4 | `camera_io[5]` | E21 |
| D3 | `camera_io[6]` | B22 |
| D2 | `camera_io[7]` | C22 |
| D1 | `camera_io[8]` | A21 |
| D0 | `camera_io[9]` | B21 |
| SDA | `camera_io[10]` | T21 |
| XCLK | `camera_io[11]` | C19 |
| PCLK | `camera_io[12]` | C18 |
| VSYNC | `camera_io[13]` | G22 |
| RESET | `camera_rst_n` | F15 |
| PWDN | `camera_pwdn` | F21 |

`camera_io[11]` 由 FPGA 输出 24MHz XCLK；`camera_io[0]` 和 `camera_io[10]` 由 SCCB 控制器驱动。数据输入在 `top.v` 中按
`{camera_io[2], ..., camera_io[9]}` 组成 `cmos_din[7:0]`。

### 4.2 初始化参数

`top.v` 实例化 `camera_init` 时使用：

```text
CAMERA_TYPE     = "ov5640"
IMAGE_TYPE      = RGB
IMAGE_WIDTH     = 800
IMAGE_HEIGHT    = 480
IMAGE_FLIP_EN   = 0
IMAGE_MIRROR_EN = 0
SYS_CLOCK       = 50_000_000
SCL_CLOCK       = 400_000
```

`camera_pwdn` 在 `camera_init.v` 中固定为 `0`，因此当前设计不通过该信号执行掉电控制。`camera_rst_n` 由初始化模块的延时计数器产生。

### 4.3 采集和帧缓存

`DVP_Capture` 在摄像头 PCLK 域完成以下工作：

- 每两个 8bit DVP 数据拼成一个 RGB565 像素。
- 统计行列地址，并丢弃上电后的前 10 帧。
- `DataValid` 只在丢帧结束后有效。
- `DataVs` 的实现是 `~r_Vsync & dump_frame`。

`top.v` 再执行以下处理：

- 从 RGB565 提取 RGB 分量，并加入简单 Bayer 抖动。
- 每个像素压缩为 RGB444，共 `800 * 480 = 384000` 个 12bit 像素。
- `dpram_async` 使用摄像头 PCLK 写入、LCD PCLK 读出。
- `wr_pix` 在 `!dvp_vs` 时清零，在写到地址 `383999` 时置 `frame_full` 和 `cam_frame_ok`。
- `cam_frame_ok` 通过 3 级寄存器同步到 LCD 域；LCD 在帧边界切换到摄像头图像，否则显示测试彩条。

当前实现只有一块帧 RAM。下一场开始时 `frame_full` 会再次清零并复用该 RAM，因此连续采集期间摄像头写入和 LCD 读取可能同时访问同一帧数据；代码没有实现 ping-pong 双缓冲。

`camera_init_done` 位于 `Clk` 域，但当前代码直接作为 `cmos_pclk` 域的 `cap_en` 使用；`cam_frame_ok` 进入 LCD 域时有 3 级同步。这里的跨时钟行为仍应结合板级时序重新验证。

需要注意，`sim/chain_sim.py` 当前模型把 VSYNC 极性写成了与 `DVP_Capture.DataVs` 相反的形式，实际运行结果为所有场景“未写满一帧”。这是仿真模型与 RTL 接口约定尚未统一的状态，不能据此证明板级摄像头极性正确。

## 5. LCD 接口

`lcd_rgb_driver` 的默认时序参数为：

| 参数 | 值 |
| --- | ---: |
| 水平有效区 | 800 |
| 水平前沿/同步/后沿 | 40 / 128 / 88 |
| 水平总计 | 1056 |
| 垂直有效区 | 480 |
| 垂直前沿/同步/后沿 | 10 / 2 / 33 |
| 垂直总计 | 525 |

LCD 像素时钟输出为 `lcd_clk = ~lcd_pclk`。HSYNC 和 VSYNC 使用低有效极性，DE 在有效图像区域为高。

### 5.1 控制信号

| LCD 信号 | RTL 输出 | 物理管脚 |
| --- | --- | --- |
| RESET | `Gpio0[1]` | G18 |
| BL | `Gpio0[7]`，固定为 1 | J17 |
| DE | `Gpio0[6]` | K17 |
| HSYNC | `Gpio0[8]` | K18 |
| VSYNC | `Gpio0[9]` | K19 |
| CLK | `Gpio0[11]` | L20 |

### 5.2 RGB 数据

| 颜色位 | RTL 输出 | 物理管脚 |
| --- | --- | --- |
| R7..R0 | `Gpio0[26]`, `Gpio0[29]`, `Gpio1[28]`, `Gpio0[31]`, `Gpio0[30]`, `Gpio0[33]`, `Gpio0[32]`, `Gpio0[35]` | K13, H15, D20, G16, G15, H14, J14, G13 |
| G7..G0 | `Gpio0[18]`, `Gpio0[21]`, `Gpio0[20]`, `Gpio0[23]`, `Gpio0[22]`, `Gpio0[25]`, `Gpio0[24]`, `Gpio0[27]` | L16, M16, M15, L15, L14, L13, M13, K14 |
| B7..B0 | `Gpio0[10]`, `Gpio0[13]`, `Gpio0[12]`, `Gpio0[15]`, `Gpio0[14]`, `Gpio0[17]`, `Gpio0[16]`, `Gpio0[19]` | L19, M20, N20, N19, N18, L18, M18, K16 |

其中 `LCD_R5` 特意跨接到 `Gpio1[28]`（D20），该管脚不参与电机控制。

## 6. 电机和编码器接口

### 6.1 控制链路

**控制源已从 `motor_demo` 换成 `cmd_rx`（上位机遥控）。**

`top.v` 里用 `CTRL_SRC_UART` 选择：

| 值 | 指令源 |
|---:|---|
| **1（当前）** | `cmd_rx` —— 上位机遥控，**带掉线保护**（500ms 收不到合法帧就输出 0，车自己停）|
| 0 | `motor_demo` —— 台架演示序列，不接上位机时验接线/转向/编码器方向用 |

**⚠️ 还有一个必须注意的开关：`GPIO1_MODE`**

| 值 | 行为 |
|---:|---|
| **0（遥控必须）** | 电机驱动输出，`cmd_rx -> mecanum_mixer -> motor_quad -> Gpio1` |
| 1 / 2 / 3 | Gpio1 自检模式，**完全绕过整条电机控制链** |

> **踩过的坑**：工程原先是 `GPIO1_MODE = 3`（转轮自检），这个模式直接输出固定
> 50% 方波，综合器会把 `cmd_rx`/`mecanum_mixer`/`motor_quad` **全部优化掉**
> （日志里一堆 `NL0002 ... is swept`），任何运动控制逻辑都不生效。
> 遥控前务必确认它是 **0**。

`motor_demo` 在 `CTRL_SRC_UART=0` 时的行为：默认 `V=300`，输出 8 个阶段：

1. 前进：`vx=+300`
2. 原地左转：`w=+300`
3. 左移：`vy=+300`
4. 后退：`vx=-300`
5. 原地右转：`w=-300`
6. 右移：`vy=-300`
7. 斜移加自转：`vx=+150, w=+150`
8. 停止

代码使用 `cnt[29:27]` 作为阶段号。每阶段为 `2^27 / 50MHz`，约 2.684 秒；完整 8 阶段周期约 21.47 秒。

`mecanum_mixer` 的例化参数为 `OUT_MAX=1000`、`K_GAIN=1024`、`GAIN_SHIFT=10`，
即增益 1024/1024 = **1.0**，`kw` 就等于 `w`，公式退化为教科书形式：

```text
kw    = (K_GAIN * w) >>> GAIN_SHIFT     // = w
FL    = vx - vy - kw
FR    = vx + vy + kw
RL    = vx + vy - kw
RR    = vx - vy + kw
```

> **⚠️ 时序大坑（已修）**：这里原来写的是 `kw = K_GAIN * w / 1000`。
> **除以非 2 的幂是灾难** —— 综合器要用 DSP 做倒数乘法再加约 **30 级**归一化逻辑，
> 实测关键路径 `slack = -2.294ns`、`Logic Level = 30`，`clk50` 只能跑到
> **44.836 MHz**，17 条 setup 违规（TNS −37.5）。
> 改成算术右移后是纯布线、零组合逻辑：
>
> | | Fmax | setup 违规 | DSP |
> |---|---:|---:|---:|
> | `/1000` | 44.836 MHz ❌ | 17 | 9.5 |
> | `>>> GAIN_SHIFT` | **87.265 MHz** ✅ | **0** | **8.5** |
>
> 而且增益设成 1.0 后 `(1024*w)>>10` 直接化简成 `w`，**乘法器都省掉了**。
>
> **通用教训：定点缩放一律用 2 的幂（移位），不要用非 2 的幂的除法。**

最终速度限制在 `[-1000, +1000]`，并打包为 `motor_quad.speed_cmd` 的低位到高位：轮 0、轮 1、轮 2、轮 3。
轮 0/1 接第 1 片 AT8236 的 A/B 通道，轮 2/3 接第 2 片 AT8236 的 A/B 通道。

### 6.2 Gpio1 映射

| Gpio1 | RTL 信号 | 方向 | 说明 |
| --- | --- | --- | --- |
| `[3:0]` | `pwm[3:0]` | output | 4 路 AT8236 PWM，默认 PWM 频率 20kHz |
| `[7:4]` | `in1[3:0]` | output | 4 路方向控制 IN1 |
| `[11:8]` | `in2[3:0]` | output | 4 路方向控制 IN2 |
| `[13:12]` | `stby[1:0]` | output | 两片 AT8236 STBY，RTL 固定为 `2'b11` |
| `[17:14]` | `enc_a[3:0]` | input | A 相：轮 0/1/2/3 对应 `[14]/[15]/[16]/[17]` |
| `[21:18]` | `enc_b[3:0]` | input | B 相：轮 0/1/2/3 对应 `[18]/[19]/[20]/[21]` |
| `[28]` | LCD_R5 | output | LCD 红色数据位，不属于电机接口 |
| 其他 | 未使用 | high-Z/input | 当前 RTL 未驱动 |

每路 `motor_channel` 的方向真值：

```text
speed > 0: IN1=PWM, IN2=0
speed < 0: IN1=0,   IN2=PWM
speed = 0: 见下面的 BRAKE_ON_ZERO
```

**速度 0 时是刹车还是滑行 —— `BRAKE_ON_ZERO`**

| 值 | 输出 | 效果 |
|---:|---|---|
| **1（本工程）** | `IN1=IN2=1` | **刹车**：轮子锁死，斜坡不溜车；AT8236 不休眠，**起步无延迟** |
| 0 | `IN1=IN2=0` | 滑行：轮子自由，斜坡会溜；**AT8236 在 1ms 后休眠**，再起步有约 1ms 延迟 |

> **踩过的坑**：这个参数原来**没有从 `top.v` 一路传到 `motor_channel`**
> （`motor_quad` 也没暴露它），所以一直是默认的「滑行」。
> 现在 `motor_quad` 加了 `BRAKE_ON_ZERO` 参数、`top.v` 显式传 `1`。
>
> 遥控车要「按什么动什么」，必须用刹车 —— 否则松手后车会靠惯性继续滑，
> 而且每次起步都被芯片休眠拖 1ms。

**上电静止**：`cmd_rx` 的掉线保护计数器上电初值就是「已超时」，
所以 `link_ok=0` -> `vx=vy=w=0` -> 四轮速度 0 -> 刹车。
**上电即锁死静止，不接上位机也永远不会自己动。**

`encoder_reader` 使用 3 级输入同步和四倍频正交解码。默认测速闸门为 10ms，输出的是每个闸门内的有符号计数 `enc_speed`，RTL 没有直接输出 RPM。若使用 1320 CPR 电机，可在上层按实际闸门时间换算 RPM。

### 6.3 遥控指令协议（`cmd_rx.v`）

上位机用 `tools/car_gui.py`（键盘遥控）。

**帧格式：14 字节 ASCII**，方便直接用串口助手手打调试：

```text
'@'  <svx><d><d><d>  <svy><d><d><d>  <sw><d><d><d>  '\n'
 0     1   2  3  4     5   6  7  8     9  10 11 12    13

<s> = '+' 或 '-'        <d> = '0'..'9'        每个分量 ±000..±999
```

| 帧 | 含义 |
|---|---|
| `@+600+000+000\n` | 前进 600 |
| `@+000+600+000\n` | 左横移 600 |
| `@+000+000+600\n` | 原地左转 600 |
| `@+000+000+000\n` | 停 |

**参数**：`CLK_FREQ=50MHz`、`BAUD=115200`、`VMAX=1000`、`TIMEOUT_MS=500`

**掉线保护**（遥控车必须有）：超过 500ms 没收到**合法**帧就强制输出 0。
上电时计数器就是"已超时"状态，所以**上电后车不动**，
直到收到第一条指令 —— 不会像 `motor_demo` 那样一上电就冲出去。

**帧同步**：靠 `'@'` 找帧头；载荷字符逐个校验（符号位必须 `+/-`，
数字位必须 `0-9`），任何一个不对立刻回到找帧头的状态。
所以被截断/串扰的帧不会被误认，最坏只丢一帧。

**时序**：全程无乘法器 —— `acc*10` 用 `(acc<<3)+(acc<<1)`，
而且每个数字一拍、分散在 3 拍里，没有长组合路径。

### 上位机 `tools/car_gui.py`

```bash
/usr/bin/python3 tools/car_gui.py
```

| 按键 | 作用 | 指令 |
|---|---|---|
| `↑` / `W` | 前进 | `vx = +V` |
| `↓` / `S` | 后退 | `vx = -V` |
| `←` / `A` | **左横移** | `vy = +V` |
| `→` / `D` | **右横移** | `vy = -V` |
| `Q` / `E` | 原地左转 / 右转 | `w = ±V` |
| `空格` | 急停（清空所有按键）| 全零 |

**方向键是平移不是转向**，所以 `↑+←` 就是"左前方"。转向用 `Q/E`，
可以和方向键叠加（`↑+Q` = 边前进边左转 = 画弧）。

三个设计要点：

1. **20Hz 持续发帧**（哪怕按键没变），喂 FPGA 的掉线保护。
   上位机一崩 / USB 一掉 / 线一松，车 0.5 秒内自己停。
2. **窗口失焦自动停车** —— 切到别的程序时清空按键，不然车会一直冲。
3. **防饱和预缩放**：上位机按同一套混控公式预判四轮速度，
   如果会超限就**整体等比缩小**。因为 FPGA 的 clamp 是**逐轮**夹的，
   一旦有一轮先到顶，四轮比例就破坏了（"原地左转"会变成"边走边转"）。
   遍历 166375 组组合验证：预缩放后零超限，且运动方向完全不变。

**串口回包**（`enc_report.v`，每 200ms 一行）：

```text
S0=+02200 S1=+02195 S2=-00004 S3=+02210
```

单位 = **计数/秒**，带符号。`RPM = cps * 60 / CPR`（CPR=1320 时 cps 2200 = 100 RPM）。
GUI 默认显示 cps，勾选"同时显示 RPM"后一起显示。

## 7. LED 状态

| LED | 当前行为 |
| --- | --- |
| `Led[0]` | `camera_init_done=1` 时输出高电平，否则输出 `blink[24]` |
| `Led[1]` | `cam_frame_ok=1` 时输出随 VSYNC 边沿翻转的 `led1_tog`；此前 PCLK 活动时输出 `blink[23]`，否则输出 `pll_lock` |

## 8. 构建和验证边界

- Gowin 工程：打开 `fpga_project.gprj`，顶层为 `top`。
- 物理约束：`src/bx72.cst`。
- 时序约束：`src/fpga_project.sdc`。
- P&R 前脚本：`src/pnr_dual_purpose.tcl`，用于将 CPU/SSPI 专用管脚复用为普通 IO。
- 摄像头行为模型：`python3 sim/chain_sim.py`。该模型不是 RTL 仿真，使用 16bit RGB565 字典模拟写口，没有建模 RGB444 Bayer 转换、SCCB、PLL、LCD 输出时序和 CDC。
- `sim/tb_hcsr04.v` 依赖仓库中未提供的 `us_tick` 和 `hcsr04_unit`，不属于当前 `src/` 顶层工程。

当前综合/P&R 报告显示帧缓存占用约 290/340 个 BSRAM；`mecanum_mixer` 的组合乘除法也需要在满足 50MHz 时序前重新优化。

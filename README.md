# GOWIN-FPGA2027

基于**高云（GOWIN）FPGA** 的两套机器人控制工程 —— 一个四驱麦轮移动平台，一个四路舵机机械臂。

| | |
|---|---|
| **主工程** | GW5AT-138（BX72 开发板）· 麦轮移动平台 |
| **子工程** | GW1NSR-4C（Tang Nano 4K）· 四路舵机机械臂 |
| **工具链** | Gowin EDA V1.9.12.03 |
| **语言** | Verilog-2001 + Python 3（上位机）|

---

## 目录

- [这是什么](#这是什么)
- [两个子工程](#两个子工程)
- [硬件](#硬件)
- [快速开始](#快速开始)
- [上位机](#上位机)
- [文档导航](#文档导航) ← **第一次看建议直接跳这里**
- [仓库里为什么没有比特流](#仓库里为什么没有比特流)
- [已知问题](#已知问题)

---

## 这是什么

一块 FPGA 同时承担**感知、显示、运动、通信**四件事，构成一个可以被人遥控的移动机器人平台。

```text
                    ┌──────────────────────────────────────┐
   OV5640 摄像头 ──> │  camera_init / DVP_Capture           │
   (DVP + SCCB)      │  RGB565 -> RGB444 -> 800x480 帧缓存  │
                    │            ↓                         │
   TTL LCD 800x480 <─│  lcd_rgb_driver                      │
                    │                                      │
   键盘遥控 ───────> │  cmd_rx ─> mecanum_mixer ─> motor_quad│──> 4x AT8236
   (USB-UART)        │  麦轮解算      闭环/开环    4路PWM   │──> 四驱麦轮
                    │            ↑                         │
   编码器上报 <──────│  enc_report  <── encoder_reader x4   │<── 4路正交编码器
   (USB-UART)        └──────────────────────────────────────┘
                             GW5AT-138 (50MHz)
```

第二套工程（4K）是独立的手臂控制，见下。

---

## 两个子工程

### 一、`fpga_project/` —— GW5AT-138 麦轮移动平台

| 模块 | 实现 |
|---|---|
| **图像采集** | OV5640 DVP 接口，SCCB 初始化，RGB565 → RGB444，丢弃前 10 帧 |
| **显示** | 800×480 TTL LCD，单帧双口 RAM（BSRAM）|
| **运动学** | 麦克纳姆轮 X 型解算，`(vx, vy, w)` → 四轮速度 |
| **电机驱动** | 2 片 AT8236 驱动四驱麦轮，20kHz PWM |
| **速度反馈** | 4 路正交编码器，四倍频解码，M/T 法测速 |
| **闭环** | 增量式 PI（当前为开环，闭环代码已就绪）|
| **遥控** | 板上 USB-UART 接收指令帧，**带 500ms 掉线保护** |

**关键设计**：掉线保护 —— 超过 500ms 收不到合法指令就强制输出 0 并刹车，上电即为停止态。程序崩溃、USB 脱落、线松都会让车自己停下。

### 二、`4k/` —— GW1NSR-4C 四路舵机机械臂

在 **Tang Nano 4K** 上做的四路舵机控制，与上面的麦轮平台**并存、互不依赖**。

| 功能 | 说明 |
|---|---|
| **PWM** | 4 路独立 50Hz / 500–2500µs / 0–180°。定点数学，实测脉宽误差 0 |
| **串口协议** | 4 位数字：首位舵机号 + 后三位角度，如 `2090` = 2 号转 90° |
| **主动上报** | 开机横幅 + 每秒心跳 + 每条指令回复 |
| **上位机** | 滑块微调 + **录制/回放** + **时间轴编辑**（可让四路同时动）+ 起始位 |

> 为什么单独做一个 4K 工程：138K 的板子当时还没到货，但舵机控制逻辑与芯片无关，先在便宜的板子上调通。后来决定两者并存，不做合并。

---

## 硬件

| | 主工程 | 子工程 |
|---|---|---|
| **芯片** | GW5AT-LV138PG484AC1/I0 | GW1NSR-LV4CQN48PC6/I5 |
| **开发板** | BX72 | Tang Nano 4K |
| **时钟** | 50 MHz（Y18）| 27 MHz（板载）|
| **封装** | PG484A | QFN48P |

### 主工程外设

| 外设 | 接口 | 说明 |
|---|---|---|
| OV5640 | DVP + SCCB | 摄像头 |
| TTL LCD | 并行 RGB | 800×480 |
| AT8236 ×2 | PWM + IN1/IN2 | 四驱麦轮 |
| 正交编码器 ×4 | A/B 相 | 测速 |
| USB-UART | 115200 8N1 | 遥控收发（TX=N14, RX=N13）|

管脚细节见 [BX72管脚分配表V2.1.md](fpga_project/BX72管脚分配表V2.1.md) 和 [BX72接线说明.md](fpga_project/BX72接线说明.md)。

### ⚠️ 供电

```
舵机/电机必须用【独立电源】，绝不能从 FPGA 板取电
电源负极必须和 FPGA 的 GND 接在一起（共地）
```

---

## 快速开始

### 环境

- **Gowin EDA V1.9.12.03**（含 Education 版）
- **Python 3**（上位机需要 `pyserial`；用系统自带 `/usr/bin/python3` 才有 tkinter）

### 主工程（138K）

用 Gowin IDE 打开 `fpga_project/fpga_project.gprj`，顶层 `top`，直接综合+布局布线。

命令行方式（macOS）：

```bash
GW="/Applications/GowinIDE.app/Contents/Resources/Gowin_EDA/IDE"
DYLD_LIBRARY_PATH="$GW/lib" DYLD_FRAMEWORK_PATH="$GW/lib" "$GW/bin/gw_sh" your_build.tcl
```

### 子工程（4K）

```bash
cd 4k
./tools/build_fs.sh servo_serial      # 一键编译，产物在 fs/
```

脚本会从 `.gprj` 读器件和文件列表，在 `/tmp` 里编译（不污染工程目录），
并打印资源占用、时序和比特流 CheckSum。

---

## 上位机

两套工程各有自己的上位机，都是 Python + Tkinter。

### 麦轮小车遥控 —— `fpga_project/tools/car_gui.py`

```bash
/usr/bin/python3 fpga_project/tools/car_gui.py
```

| 按键 | 作用 |
|---|---|
| `↑↓←→` / `WASD` | **平移**（麦轮可横移，`↑+←` = 左前方）|
| `Q` / `E` | 原地左转 / 右转，可与方向键叠加画弧 |
| `空格` | 急停 |

**按键组合是严格校验的**：2 个键必须在不同轴上（`↑+←` 合法，`↑+↓` 置零），3 个及以上直接置零 —— 按错键只会停车，不会跑出意外方向。

### 舵机臂调试 —— `4k/tools/servo_gui.py`

```bash
/usr/bin/python3 4k/tools/servo_gui.py
```

4 个滑块 + 编码器回读 + **录制/回放** + **时间轴编辑**（把四路动作对齐到同一时刻，让舵机同时动）。

命令行版：`4k/tools/servo_send.py`

---

## 文档导航

**第一次看就从这里挑：**

| 我想知道…… | 看这个 |
|---|---|
| **整个主工程怎么接线的** | [BX72接线说明.md](fpga_project/BX72接线说明.md) |
| **顶层接口、协议、管脚映射** | [PROJECT_INTERFACE.md](fpga_project/PROJECT_INTERFACE.md) |
| **电机驱动怎么调、PI 怎么整定** | [MOTOR_DRIVER.md](fpga_project/MOTOR_DRIVER.md) |
| **某个引脚具体是什么** | [BX72管脚分配表V2.1.md](fpga_project/BX72管脚分配表V2.1.md) |
| **速查管脚** | [PINOUT.md](fpga_project/PINOUT.md) |
| **4K 舵机工程的完整技术文档** | [4k/fpga_project/README.md](4k/fpga_project/README.md) |
| **4K 怎么烧录、有哪些坑** | [4k/fs/README.md](4k/fs/README.md) |
| **4K 工程的交接说明** | [4k/交接文档.md](4k/交接文档.md) |

### 各文档一句话

| 文件 | 内容 |
|---|---|
| `PROJECT_INTERFACE.md` | 顶层端口、时钟复位、摄像头/LCD 接口、电机编码器链路、遥控协议 |
| `MOTOR_DRIVER.md` | 数据通路、量纲换算、PI 整定步骤、安全特性、时序优化记录 |
| `PINOUT.md` | 管脚速查 |
| `BX72接线说明.md` | 连接器总览、摄像头座、LCD 排针、电机排针、空闲资源 |
| `BX72管脚分配表V2.1.md` | 从厂商 .cst 和 xlsx 对齐整理的完整管脚表（413 行）|
| `4k/fpga_project/README.md` | 4K 工程器件/功能/协议/硬规矩/移植/排错/踩坑（8 大节）|
| `4k/fs/README.md` | 烧录指南 + 实测更正 |
| `4k/交接文档.md` | 交接文档（含未解决问题的完整证据链）|

---

## 目录结构

```text
.
├── fpga_project/                 # 【主工程】GW5AT-138 麦轮移动平台
│   ├── fpga_project.gprj         #   Gowin 工程入口
│   ├── src/                      #   RTL 源码（20 个模块）
│   │   ├── top.v                 #     顶层
│   │   ├── camera_init.v         #     摄像头 SCCB 初始化
│   │   ├── DVP_Capture.v         #     DVP 采集
│   │   ├── lcd_rgb_driver.v      #     LCD 时序
│   │   ├── cmd_rx.v              #     遥控指令接收 + 掉线保护
│   │   ├── mecanum_mixer.v       #     麦轮运动学解算
│   │   ├── motor_quad.v          #     四路电机通道
│   │   ├── motor_channel.v       #     AT8236 单路驱动
│   │   ├── encoder_reader.v      #     正交编码器
│   │   ├── speed_pi.v            #     速度 PI
│   │   ├── enc_report.v          #     编码器串口上报
│   │   └── bx72.cst              #     物理约束
│   ├── tools/car_gui.py          #   键盘遥控上位机
│   ├── sim/                      #   行为模型与测试平台
│   └── test_pwm/                 #   PWM 自检小工程
│
├── 4k/                           # 【子工程】GW1NSR-4C 舵机机械臂
│   ├── fpga_project/
│   │   ├── src/                  #   RTL 源码
│   │   │   ├── servo_pwm.v       #     单路舵机 PWM（可移植核心）
│   │   │   ├── servo_cmd.v       #     串口协议解析
│   │   │   └── ...
│   │   └── README.md             #   详细技术文档
│   ├── fs/                       #   烧录说明
│   ├── tools/                    #   上位机 + 录制数据
│   └── 交接文档.md
│
└── .gitignore
```

---

## 仓库里为什么没有比特流

你可能会好奇为什么找不到 `.fs` 文件。**这是故意的：**

| 类别 | 体积 | 处理 |
|---|---:|---|
| 源码 + 文档 + 脚本 | **1.3 MB** | ✅ 已提交 |
| 比特流 `.fs`（12 个）| 482 MB | ❌ 忽略，可重新编译 |
| 构建目录 `impl/` | 271 MB | ❌ 忽略，编译产物 |
| 历史快照 `.zip`（11 个）| 1.2 GB | ❌ 忽略，加 git 之前的手工备份 |

**全部重新编译一次就能得到比特流**，没必要塞进仓库（那会让 clone 慢到没法用）。

需要固件的话：

```bash
cd 4k && ./tools/build_fs.sh servo_serial    # 4K 的
# 138K 用 Gowin IDE 打开 fpga_project.gprj 综合
```

---

## 已知问题

诚实列出，方便接手的人知道从哪里开始。

### 4K 舵机臂

| 问题 | 状态 | 说明 |
|---|---|---|
| **夹子会周期性松开** | 🔴 未定位 | 夹住物体并命令到 180° 后，动其他舵机时夹子会反复松一下。**已从网表层面证明固件不产生这个效果**（3 号 PWM 的每个输入都只依赖它自己的角度寄存器），怀疑是供电瞬态/地线反弹/舵机自身堵转保护 |
| **掉电不保存** | 🟡 部分 | 板上无外部 SPI Flash；片内 Flash 校验失败。目前只能烧 SRAM，掉电需重烧（约 2.4 秒）|
| **串口未验证成功** | 🟡 部分 | 板载 USB 桥的串口通道读不到数据，怀疑引脚 44/46 未连到板载桥。建议用外接 USB-TTL |

### 138K 主工程

| 问题 | 状态 | 说明 |
|---|---|---|
| **BSRAM 占用较高** | 🟡 注意 | 290 / 340（86%），主要是 800×480 帧缓存。再加帧缓存前需要重新规划 |
| **闭环未启用** | 🟢 设计选择 | `CLOSED_LOOP = 0`（开环）。PI 代码已就绪，整定步骤见 `MOTOR_DRIVER.md` |
| **LCD 有一条 hold 违规** | 🟢 已知假报警 | PLL 派生时钟在 SDC 解析阶段还不存在，无法写进 group。详见 `fpga_project.sdc` 注释 |

---

## 开发环境

| 项 | 版本 |
|---|---|
| Gowin EDA | V1.9.12.03 |
| Python | 3.9（系统自带，含 tkinter + pyserial）|
| 操作系统 | macOS（APFS）|

> **Python 注意**：`/usr/bin/python3` 是系统自带的 3.9，同时有 tkinter 和 pyserial；
> miniforge 等环境的 `python3` 往往缺 pyserial。上位机请用前者。

---

## 许可

未指定许可证。如需使用请联系仓库所有者。

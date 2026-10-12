# 4K 舵机固件（可直接烧录）

## 1. 文件

| 文件 | 说明 |
|---|---|
| **`servo_serial.fs`** | **主文件。Gowin Programmer 加载这个。** |
| `servo_serial.bin` | 同一比特流的二进制版，个别工具用得上；Programmer 用不到 |

功能：**串口微调 4 路舵机**。上电 4 路恒定停在中位 90 度等你发指令，
每收到一条 4 位数字就动一路并回一行；每秒发一行心跳，开机发一次横幅。

---

## 2. ⚠️ 怎么让它一直待在板子里 —— 烧 External Flash

### 2.0 ⚠️ 实测更正（2026-10-11，用 programmer_cli 真机验证）

**下面 2.1 节原来的推断是错的，这里记录实测结果。**

我之前从"MSPI 引脚存在 + `device.cfg` 里 MSPI 保留"推断这块板用外部 SPI Flash。
用命令行实测后：

| 操作 | 实测结果 |
|---|---|
| `exFlash Erase,Program,Verify`（按配置模式）| ❌ `spi flash not found` |
| `exFlash ... in bscan`（**不限配置模式**）| ❌ `Spi flash not found` |
| `embFlash Erase,Program,Verify` | ⚠️ 擦除+写入跑到校验，**`Verify Failed at 106`** |
| `SRAM Program` | ✅ **成功**（`User Code` = `.fs` 的 CheckSum）|

**结论：这块板子上没有外部 SPI Flash。** 片内 Flash 能擦能写，但校验失败 ——
断电重上电后 `User Code = 0x00000000`，证实**没有真正写进去**。

另一个重要现象：**每次 Flash 操作失败后，器件会锁在 POR 状态**
（之后连 `SRAM Program` 都报 `POR Down!`），**必须物理断电才能恢复**。

**目前可用做法**：只烧 SRAM。板子能用，但**掉电后要重烧**。
持久化还没解决，见文末"待解决"。

> 如果你要重新尝试片内 Flash，**务必先给板子断电重上电**，
> 而且注意：只有断电后的**第一次** Flash 操作能进去。

### 2.1 结论：选 `External Flash Mode`

GW1NSR-4C 支持 7 种 Access Mode，能"掉电保留"的有两个，**这颗芯片要用外部的那个**：

| Access Mode | Operation（选这个） | 说明 |
|---|---|---|
| SRAM Mode | `SRAM Program` | ❌ 掉电就没 |
| **External Flash Mode** | **`exFlash Erase,Program,Verify`** | ✅ **要的就是这个** |
| Embedded Flash Mode | `embFlash Erase,Program,Verify` | 备选（下面解释为什么大概率不需要）|

**为什么是 External 而不是 Embedded**：

1. **GW1NSR 是 SIP 封装 = FPGA 裸片 + PSRAM 裸片**，Sipeed 和 Gowin 的资料都没提
   Flash。片内 256Kb 用户 Flash 只存在于 **GW1NS-4**（不带 C），带 C 的版本
   `device_info.csv` 里那一列是空的
2. 器件有**专用 MSPI 引脚**（1=MCLK、2=MCS_N、47=MI、48=MO），且
   `impl/pnr/device.cfg` 里是 `set MSPI regular_io = false`，接口保留着
3. Programmer 扫描时报告：
   `Name: GW1NSER-4C GW1NS-4C GW1NSR-4C (One of them)`
   —— 它连这三个型号都分不清，那个 Embedded Flash Mode 只是**按系列泛列**

> 万一 `exFlash` 报错说找不到 Flash，再试 `Embedded Flash Mode`
> → `embFlash Erase,Program,Verify`。看到新固件的开机横幅就说明选对了。

### 2.2 GUI 操作步骤

```text
Gowin Programmer
  1. 扫描/选中器件（应该显示 Gowin USB Cable(FT2CH) + ID 0x0100981B）
  2. 选中那一行 -> 双击，或菜单 Edit -> Configure Device
  3. Access Mode : External Flash Mode
     Operation   : exFlash Erase,Program,Verify
     File        : 选本目录的 servo_serial.fs
     Address     : 0x000000（默认，别改）
  4. 点 Program / 右侧的烧录按钮
  5. 等它跑完（erase + program + verify，比烧 SRAM 慢）
```

### 2.3 命令行一条搞定（等价于上面）

Gowin 自带 `programmer_cli`，`--operation_index` 就是模式表里的编号：

```bash
PROG="/Applications/GowinIDE.app/Contents/Resources/Gowin_EDA/Programmer"
IDE="/Applications/GowinIDE.app/Contents/Resources/Gowin_EDA/IDE"

env DYLD_LIBRARY_PATH="$PROG/lib:$IDE/lib" \
    DYLD_FRAMEWORK_PATH="$PROG/lib:$IDE/lib" \
  "$PROG/bin/programmer_cli" \
    -d GW1NSR-4C \
    --operation_index 9 \
    --fsFile /Volumes/YBOYX/FPGA2027/4k/fs/servo_serial.fs
```

常用 operation_index：

| index | 作用 |
|---:|---|
| 2 | SRAM Program（掉电就没）|
| **9** | **exFlash Erase,Program,Verify** ← 要这个 |
| 6 | embFlash Erase,Program,Verify（备选）|
| 11 | exFlash Verify（只读校验，不改内容）|
| 1 | **Reprogram —— 从 Flash 重新加载到 SRAM，不用断电** |

### 2.4 烧完怎么验证（不用拔插电源）

烧完 Flash 后，SRAM 里跑的还是旧内容。用 **`SRAM Mode → Reprogram`**
（命令行 `--operation_index 1`）让它**从 Flash 重新加载一遍**：

```bash
env DYLD_LIBRARY_PATH="$PROG/lib:$IDE/lib" \
    DYLD_FRAMEWORK_PATH="$PROG/lib:$IDE/lib" \
  "$PROG/bin/programmer_cli" -d GW1NSR-4C --operation_index 1
```

然后跑串口监听，应该看到开机横幅 + 每秒心跳，舵机稳定停在中位。

想彻底确认就**拔电重上电**再听一次。

---

### 2.5 ⚠️ 顺带一个坑：引脚 8 不要当普通 IO 用

openFPGALoader 的文档里有一条警告：

> Gowin 的 FPGA 在 **JTAGSEL_N** 被当普通 IO 用时，**可能检测不到 JTAG**。
> 要恢复得在**上电前**把这个脚拉低。

GW1NSR-4C 的 **引脚 8 = IOT4B = JTAGSEL_N**。本工程没有用它，所以 JTAG 一直是好的
（刚才扫描正常就是这个原因）。

**以后加外设时别碰引脚 8**，否则可能把板子搞到连不上 JTAG。真踩了就把引脚 8
在上电前对地短路一下再上电。



---

## 3. 烧进去之后应该看到什么

| 现象 | 说明 |
|---|---|
| 板上 LED **每秒闪一次** | 27MHz 时钟和固件都正常 |
| 串口收到开机横幅 | 约上电后 0.5 秒 |
| 串口**每秒一行** `A 090 090 090 090` | 心跳 |
| 4 路舵机**恒定停在 90 度不动** | ★ 如果它们在摆，就是烧错位置了，看第 2 节 |

开机横幅内容：

```text
=== Tang Nano 4K Servo Ready ===
Send 4 digits, e.g. 2090
```

---

## 4. 引脚

| 信号 | FPGA 引脚 | 电平 | 接到 |
|---|---|---|---|
| `servo[0]` | 43 | 3.3V | 舵机 0 信号线 |
| `servo[1]` | 42 | 3.3V | 舵机 1 信号线 |
| `servo[2]` | 41 | 3.3V | 舵机 2 信号线 |
| `servo[3]` | 40 | 3.3V | 舵机 3 信号线 |
| `uart_tx` | 44 | 3.3V | 串口 TX → 电脑 |
| `uart_rx` | 46 | 3.3V | 串口 RX ← 电脑 |

> ⚠️ **舵机电源必须单独供**（独立 5V ≥3A），只把 GND 和 FPGA 共起来。
> 从板子取电会把 FPGA 拉垮。

---

## 5. 版本信息与验证数据

| 项 | 值 |
|---|---|
| 器件 | `GW1NSR-4C` / `GW1NSR-LV4CQN48PC6/I5`（QFN48P）|
| 工具 | Gowin EDA **V1.9.12.03** |
| 构建时间 | Sun Oct 11 02:16:02 2026 |
| **比特流 CheckSum** | **`0x17E3`** |
| sha256（当前这个文件） | `a8108e7cfbf7e3cffdd4fafcb1ecedff922e7672f7044cf7b67aa801c85f4689` |
| 时序违规 | setup 0 / hold 0，**TNS = 0.000** |
| **实测 Fmax** | **71.741 MHz**（约束 27 MHz，余量 2.66×）|
| Logic | 1083 / 4608（24%）|
| Register | 531 / 3573（15%）|
| CLS | 674 / 2304（30%）|
| DSP | 4.25 / 8（54%）|

### 角度标定：`SCALE_360 = 0`

固件里 `SCALE_360 = 0`，也就是**协议里的值直接就是舵机角度**（0–180）：

```text
2090  ->  2 号舵机 转到 90 度
2180  ->  2 号舵机 转到 180 度
2361  ->  ERR（361 超过 180）
```

所以上位机也要用同一个设置：

```bash
/usr/bin/python3 ../tools/servo_gui.py --scale360 0     # 已经是默认值
```

两边不一致的话，滑块拖到 90 度舵机会跑到 45 度（或反过来 180）。
GUI 的标题栏会显示当前用的是哪个。

### 关于校验和

`.fs` 是「**文本头 + 二进制比特流**」。文本头里带构建时间戳，所以
**同一份源码重新编译两次，sha256 会不一样** —— 实测两次构建
**只差 1 个字节**（在文本头的 `Created Time` 秒数上），
其余 1,167,805 字节完全相同，比特流是确定性的。

所以：

- 想确认「烧的是不是这一份」→ 比对文件头里的 **`CheckSum: 0x17E3`**
- sha256 只能用来标识**这一个具体文件**，不能用来判断两次编译是否一致

## 6. 怎么重新编译

```bash
cd 4k
./tools/build_fs.sh                    # 生成 fs/servo_serial.fs
./tools/build_fs.sh servo_test2        # 换个名字，保留旧版
```

脚本会：

1. **从 `fpga_project/fpga_project.gprj` 读器件和文件列表**
   —— 所以你在 IDE 里加了新 `.v`，脚本会自动跟上，不会出现
   「IDE 编的是新代码、脚本编的是老代码」
2. 调 Gowin 命令行 `gw_sh` 跑完整流程（综合 → 布局 → 布线 → 比特流）
3. 把 `.fs` / `.bin` 拷进本目录并改成可写（Gowin 产出的原件是只读的）
4. 打印资源占用和 sha256

**注意**：脚本用的是 `fpga_project/src/` 下的源文件，和 IDE 用的是同一份。
如果 IDE 里正开着这个工程，改完记得让它重新读盘。

---

## 待解决：掉电保留

截至 2026-10-11 用命令行尝试过的都失败了：

| 尝试 | 结果 |
|---|---|
| `exFlash`（两种模式）| 板上无外部 Flash |
| `embFlash Erase,Program,Verify` | 校验失败 |
| `embFlash Erase,Program`（去掉校验）| `POR Down` |
| `embFlash 2nd` | `POR Down` |
| 关掉 bitstream 的 `SecurityBit` 后重试 | 仍 `POR Down` |

**还没试的**：Gowin IDE 的 Programmer 图形界面（命令行和 GUI 走的可能是
不同的流程）。用 GUI 时注意：**先完全退出其它占用 USB 的程序**
（我们就是被 IDE 自己占着，导致 `Cable open failed`）。

**另外发现**：板载 USB 桥的串口通道（`/dev/cu.usbserial-11200/11201`）
读不到任何数据，怀疑 **44/46 并没有连到板载桥**。若属实，
串口要用外接 USB-TTL 模块接 44/46/GND。

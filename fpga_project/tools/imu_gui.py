#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =====================================================================
# ATK-MS901M / ATK-IMU901 角度传感器 上位机
#
# 协议来源：从正点原子官方例程 atk_ms901m.c / .h 反解得到
#           （厂商资料本身不入库，见 ../../.gitignore 的说明）
#           协议整理见同目录 IMU_ATK_MS901M_PROTOCOL.md
#
# 用法：
#   /usr/bin/python3 imu_gui.py
#   /usr/bin/python3 imu_gui.py --port /dev/cu.usbserial-XXXX --baud 115200
#
# 依赖：只用 Python 标准库 + pyserial（tkinter 画面全部手绘，不需要
#       matplotlib/numpy）。系统自带的 /usr/bin/python3 就够。
#
# ---------------------------------------------------------------------
# 通信帧（8 位校验和）
#
#   [0x55] [帧头高位] [ID] [LEN] [DATA...] [CHECKSUM]
#     帧头高位 = 0x55 -> 模块主动上传帧
#     帧头高位 = 0xAF -> 应答帧
#     校验和 = 从第 0 字节累加到最后一个 DATA 字节，取低 8 位
#
#   读寄存器： 55 AF (ID|0x80) 01 00 SUM
#   写寄存器： 55 AF ID LEN DATA... SUM
#
# ---------------------------------------------------------------------
# 界面分 4 个页签
#   姿态   —— 人工地平仪 + 罗盘 + 全部数值
#   波形   —— 加速度/陀螺/磁力 三轴实时曲线
#   配置   —— 所有可读写寄存器 + 校准/保存/恢复
#   数据   —— CSV 记录 + 原始帧监视
# =====================================================================
import sys
import os
import csv
import time
import glob
import queue
import struct
import argparse
import threading

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("需要 pyserial：/usr/bin/python3 -m pip install pyserial")

try:
    import tkinter as tk
    from tkinter import ttk, filedialog, messagebox
except ImportError:
    sys.exit("需要 tkinter（请用系统自带的 /usr/bin/python3）")


# =====================================================================
# 一、协议层（纯函数 / 纯逻辑，不依赖 GUI，可单独测试）
# =====================================================================
FRAME_HEAD_L      = 0x55
FRAME_HEAD_UPLOAD = 0x55      # 主动上传帧的帧头高位
FRAME_HEAD_ACK    = 0xAF      # 应答帧的帧头高位
FRAME_DAT_MAX     = 28

# ---- 主动上传帧 ID ----
UP_ATTITUDE   = 0x01    # 姿态角  roll/pitch/yaw
UP_QUAT       = 0x02    # 四元数  q0..q3
UP_GYRO_ACCE  = 0x03    # 加速度计 + 陀螺仪
UP_MAG        = 0x04    # 磁力计
UP_BARO       = 0x05    # 气压计
UP_PORT       = 0x06    # 端口 D0~D3

UPLOAD_NAMES = {
    UP_ATTITUDE:  "姿态角",
    UP_QUAT:      "四元数",
    UP_GYRO_ACCE: "加速度/陀螺",
    UP_MAG:       "磁力计",
    UP_BARO:      "气压计",
    UP_PORT:      "端口",
}

# ---- 寄存器（应答帧）ID，来源 atk_ms901m.h ----
REG_SAVE      = 0x00    # (W)   保存配置到 Flash
REG_SENCAL    = 0x01    # (W)   传感器校准
REG_SENSTA    = 0x02    # (R)   读取校准状态
REG_GYROFSR   = 0x03    # (R/W) 陀螺仪量程
REG_ACCFSR    = 0x04    # (R/W) 加速度计量程
REG_GYROBW    = 0x05    # (R/W) 陀螺仪带宽
REG_ACCBW     = 0x06    # (R/W) 加速度计带宽
REG_BAUD      = 0x07    # (R/W) 波特率
REG_RETURNSET = 0x08    # (R/W) 回传内容
REG_RETURNSET2= 0x09    # (R/W) 回传内容2（保留）
REG_RETURNRATE= 0x0A    # (R/W) 回传速率
REG_ALG       = 0x0B    # (R/W) 算法
REG_ASM       = 0x0C    # (R/W) 安装方向
REG_GAUCAL    = 0x0D    # (R/W) 陀螺仪自校准开关
REG_BAUCAL    = 0x0E    # (R/W) 气压计自校准开关
REG_LEDOFF    = 0x0F    # (R/W) LED 开关
REG_D0MODE    = 0x10    # (R/W) 端口 D0 模式
REG_D1MODE    = 0x11
REG_D2MODE    = 0x12
REG_D3MODE    = 0x13
REG_D1PULSE   = 0x16    # (R/W) D1 PWM 高电平脉宽
REG_D3PULSE   = 0x1A
REG_D1PERIOD  = 0x1F    # (R/W) D1 PWM 周期
REG_D3PERIOD  = 0x23
REG_RESET     = 0x7F    # (W)   恢复默认设置

REG_NAMES = {
    REG_SAVE: "保存到Flash", REG_SENCAL: "传感器校准", REG_SENSTA: "校准状态",
    REG_GYROFSR: "陀螺仪量程", REG_ACCFSR: "加速度计量程",
    REG_GYROBW: "陀螺仪带宽", REG_ACCBW: "加速度计带宽",
    REG_BAUD: "波特率", REG_RETURNSET: "回传内容", REG_RETURNSET2: "回传内容2",
    REG_RETURNRATE: "回传速率", REG_ALG: "算法", REG_ASM: "安装方向",
    REG_GAUCAL: "陀螺仪自校准", REG_BAUCAL: "气压计自校准", REG_LEDOFF: "LED",
    REG_D0MODE: "端口D0模式", REG_D1MODE: "端口D1模式",
    REG_D2MODE: "端口D2模式", REG_D3MODE: "端口D3模式",
    REG_D1PULSE: "D1 PWM脉宽", REG_D3PULSE: "D3 PWM脉宽",
    REG_D1PERIOD: "D1 PWM周期", REG_D3PERIOD: "D3 PWM周期",
    REG_RESET: "恢复默认",
}

# ---- 满量程表（来源 atk_ms901m.c）----
GYRO_FSR_DPS = [250, 500, 1000, 2000]     # 索引 = 寄存器取值 0..3
ACC_FSR_G    = [2, 4, 8, 16]


def checksum(buf):
    """帧校验和：从第 0 字节累加到最后一个数据字节，取低 8 位"""
    return sum(buf) & 0xFF


def build_read(reg_id):
    """读寄存器指令：55 AF (ID|0x80) 01 00 SUM"""
    b = [FRAME_HEAD_L, FRAME_HEAD_ACK, reg_id | 0x80, 1, 0]
    b.append(checksum(b))
    return bytes(b)


def build_write(reg_id, data):
    """写寄存器指令：55 AF ID LEN DATA... SUM（data 为 bytes/list）"""
    if isinstance(data, int):
        data = [data]
    b = [FRAME_HEAD_L, FRAME_HEAD_ACK, reg_id, len(data)] + list(data)
    b.append(checksum(b))
    return bytes(b)


class FrameParser:
    """逐字节喂入，吐出完整合法帧。

    状态机与官方例程 atk_ms901m.c 的 get_frame_by_id 完全一致：
      等 0x55 -> 等帧头高位(0x55 或 0xAF) -> 等 ID -> 等 LEN -> 等 DATA -> 校验
    任何一步不对就退回"等 0x55"，所以被截断/串扰的帧不会被误认。
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.st = 0            # 0=等0x55 1=等head_h 2=等id 3=等len 4=等dat 5=等sum
        self.head_h = 0
        self.id = 0
        self.len = 0
        self.dat = []
        self.idx = 0
        self.s = 0

    def feed(self, data):
        """返回本次解析出的帧列表 [(head_h, id, bytes(dat)), ...]"""
        out = []
        for d in data:
            if self.st == 0:
                if d == FRAME_HEAD_L:
                    self.s = d
                    self.st = 1
            elif self.st == 1:
                if d in (FRAME_HEAD_UPLOAD, FRAME_HEAD_ACK):
                    self.head_h = d
                    self.s += d
                    self.st = 2
                else:
                    self.reset() if d != FRAME_HEAD_L else None
            elif self.st == 2:
                self.id = d
                self.s += d
                self.st = 3
            elif self.st == 3:
                if d > FRAME_DAT_MAX:
                    self.reset()
                else:
                    self.len = d
                    self.s += d
                    self.dat = []
                    self.idx = 0
                    self.st = 5 if d == 0 else 4
            elif self.st == 4:
                self.dat.append(d)
                self.s += d
                self.idx += 1
                if self.idx >= self.len:
                    self.st = 5
            else:                                   # st == 5，等校验和
                if d == (self.s & 0xFF):
                    out.append((self.head_h, self.id, bytes(self.dat)))
                self.reset()
        return out


def _i16(dat, i):
    """小端有符号 16 位"""
    return struct.unpack_from("<h", dat, i)[0]


def _i32(dat, i):
    """小端有符号 32 位"""
    return struct.unpack_from("<i", dat, i)[0]


def parse_upload(fid, dat, gyro_fsr_idx=2, acc_fsr_idx=0):
    """把主动上传帧的原始数据解析成带物理单位的字典。

    布局来源 atk_ms901m.c 的 get_xxx 函数，全部小端。
    返回 None 表示长度不够（帧残缺）。
    """
    n = len(dat)
    try:
        if fid == UP_ATTITUDE:
            if n < 6:
                return None
            return {
                "type": "attitude",
                "roll":  _i16(dat, 0) / 32768 * 180,
                "pitch": _i16(dat, 2) / 32768 * 180,
                "yaw":   _i16(dat, 4) / 32768 * 180,
            }

        if fid == UP_QUAT:
            if n < 8:
                return None
            return {
                "type": "quat",
                "q0": _i16(dat, 0) / 32768, "q1": _i16(dat, 2) / 32768,
                "q2": _i16(dat, 4) / 32768, "q3": _i16(dat, 6) / 32768,
            }

        if fid == UP_GYRO_ACCE:
            if n < 12:
                return None
            gfs = GYRO_FSR_DPS[max(0, min(3, gyro_fsr_idx))]
            afs = ACC_FSR_G[max(0, min(3, acc_fsr_idx))]
            ax, ay, az = _i16(dat, 0), _i16(dat, 2), _i16(dat, 4)
            gx, gy, gz = _i16(dat, 6), _i16(dat, 8), _i16(dat, 10)
            return {
                "type": "imu",
                "acc_raw": (ax, ay, az), "gyro_raw": (gx, gy, gz),
                "ax": ax / 32768 * afs, "ay": ay / 32768 * afs, "az": az / 32768 * afs,
                "gx": gx / 32768 * gfs, "gy": gy / 32768 * gfs, "gz": gz / 32768 * gfs,
            }

        if fid == UP_MAG:
            if n < 8:
                return None
            return {
                "type": "mag",
                "mx": _i16(dat, 0), "my": _i16(dat, 2), "mz": _i16(dat, 4),
                "mtemp": _i16(dat, 6) / 100,
            }

        if fid == UP_BARO:
            if n < 10:
                return None
            return {
                "type": "baro",
                "pressure": _i32(dat, 0),        # Pa
                "altitude": _i32(dat, 4),        # cm
                "btemp":    _i16(dat, 8) / 100,  # ℃
            }

        if fid == UP_PORT:
            if n < 8:
                return None
            return {
                "type": "port",
                "d0": struct.unpack_from("<H", dat, 0)[0],
                "d1": struct.unpack_from("<H", dat, 2)[0],
                "d2": struct.unpack_from("<H", dat, 4)[0],
                "d3": struct.unpack_from("<H", dat, 6)[0],
            }
    except struct.error:
        return None
    return None


def c_to_f(c):
    return c * 9.0 / 5.0 + 32.0


def pressure_to_altitude_m(pa):
    """国际标准大气公式估算海拔（米）。模块自己也报海拔，这里用于交叉验证。"""
    if pa <= 0:
        return 0.0
    return 44330.0 * (1.0 - (pa / 101325.0) ** 0.190295)


# =====================================================================
# 二、串口读取线程
# =====================================================================
class Link:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.05, write_timeout=0.3)
        self.q = queue.Queue()          # 解析好的帧
        self.raw_q = queue.Queue()      # 原始字节统计
        self.alive = True
        self.bytes_rx = 0
        self.frames_rx = 0
        self.t = threading.Thread(target=self._reader, daemon=True)
        self.t.start()

    def _reader(self):
        p = FrameParser()
        while self.alive:
            try:
                chunk = self.ser.read(512)
            except Exception:
                break
            if not chunk:
                continue
            self.bytes_rx += len(chunk)
            try:
                for fr in p.feed(chunk):
                    self.frames_rx += 1
                    self.q.put(fr)
            except Exception:
                p.reset()
        self.alive = False

    def send(self, b):
        try:
            self.ser.write(b)
            return True
        except Exception:
            self.alive = False
            return False

    def close(self):
        self.alive = False
        try:
            self.ser.close()
        except Exception:
            pass


def find_ports():
    ports = [p.device for p in list_ports.comports()]
    for pat in ("/dev/cu.usbserial*", "/dev/cu.usbmodem*", "/dev/cu.SLAB*",
                "/dev/cu.wchusbserial*"):
        for p in sorted(glob.glob(pat)):
            if p not in ports:
                ports.append(p)
    return ports


# =====================================================================
# 三、界面
# =====================================================================
PAD = 10
WAVE_N = 400          # 波形保留多少个点


class App:
    def __init__(self, root, port=None, baud=115200):
        self.root = root
        self.link = None

        # 最新一帧的数据（各类型分开存）
        self.d_att = None
        self.d_imu = None
        self.d_mag = None
        self.d_baro = None
        self.d_port = None
        self.d_quat = None

        # 量程索引（陀螺/加计），初始化时从模块读回来
        self.gyro_fsr_idx = 2      # 默认 1000dps
        self.acc_fsr_idx = 0       # 默认 2g

        # 波形缓冲
        self.wave = {k: [] for k in ("ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz")}
        self.wave_mode = "acc"     # acc / gyro / mag

        # ---- 陀螺仪零偏校准 ----
        #   零偏 = 静止时陀螺仪的输出均值。理想是 0，实际上都有几十 LSB 的
        #   固定偏差，会随温度漂。软件扣掉它，短时间的角度积分才不会乱跑。
        self.gyro_bias = [0.0, 0.0, 0.0]      # 扣掉的零偏（dps）
        self.calibrating = False
        self.cal_t0 = 0.0
        self.cal_dur = 3.0
        self.cal_buf = []                      # [(gx,gy,gz), ...] 原始 dps
        self.cal_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     "imu_gyro_cal.json")

        # 记录
        self.recording = False
        self.rec_file = None
        self.rec_writer = None
        self.rec_rows = 0

        # 统计
        self.frames_shown = 0
        self.t_last_stat = time.time()
        self.rate = 0.0

        root.title("ATK-MS901M 角度传感器 上位机")
        root.minsize(880, 700)
        root.configure(bg="#1b1b1f")

        self._build_top(port, baud)
        self._build_tabs()

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.after(50, self._tick)          # 20Hz 刷新界面
        if port:
            root.after(200, self.connect)

    # ---------------- 顶部串口栏 ----------------
    def _build_top(self, port, baud):
        f = ttk.Frame(self.root, padding=(PAD, 8, PAD, 4))
        f.pack(fill="x")

        ttk.Label(f, text="串口").pack(side="left")
        self.port_var = tk.StringVar(value=port or "")
        self.port_box = ttk.Combobox(f, textvariable=self.port_var, width=26,
                                     state="readonly", takefocus=False)
        self.port_box.pack(side="left", padx=4)

        ttk.Label(f, text="波特率").pack(side="left", padx=(6, 0))
        self.baud_var = tk.StringVar(value=str(baud))
        ttk.Combobox(f, textvariable=self.baud_var, width=8, takefocus=False,
                     values=["9600", "19200", "38400", "57600", "115200",
                             "230400", "460800", "921600"]).pack(side="left", padx=4)

        ttk.Button(f, text="刷新", width=5, takefocus=False,
                   command=self.refresh_ports).pack(side="left")
        self.btn = ttk.Button(f, text="连接", width=7, takefocus=False,
                              command=self.toggle)
        self.btn.pack(side="left", padx=4)

        self.stat = ttk.Label(f, text="● 未连接", foreground="#c66")
        self.stat.pack(side="right")
        self.rate_lab = ttk.Label(f, text="", foreground="#888")
        self.rate_lab.pack(side="right", padx=(0, 12))

        self.refresh_ports()

    def refresh_ports(self):
        p = find_ports()
        self.port_box["values"] = p
        if p and not self.port_var.get():
            self.port_var.set(p[0])

    def toggle(self):
        if self.link and self.link.alive:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        pt = self.port_var.get()
        if not pt:
            self._log("先选一个串口", "err"); return
        try:
            self.link = Link(pt, int(self.baud_var.get()))
        except Exception as e:
            self._log("打开 %s 失败：%s" % (pt, e), "err"); return
        self.btn.config(text="断开")
        self.stat.config(text="● 已连接 " + os.path.basename(pt), foreground="#3a3")
        self._log("已连接 %s @%s" % (pt, self.baud_var.get()), "sys")
        # 连上后自动读一次量程和配置，后面显示才准
        self.root.after(300, self.read_all)

    def disconnect(self):
        if self.calibrating:
            self.calibrating = False
            self.btn_cal.config(text="开始校准")
            self.cal_bar["value"] = 0
            self._cal_ui("校准中断（串口断开）", "#ff5f56")
        if self.link:
            try:
                self.link.close()
            except Exception:
                pass                      # 链路对象异常也要保证 UI 能回到"未连接"
            self.link = None
        self.btn.config(text="连接")
        self.stat.config(text="● 未连接", foreground="#c66")

    # ---------------- 页签 ----------------
    def _build_tabs(self):
        nb = ttk.Notebook(self.root)
        nb.pack(fill="both", expand=True, padx=PAD, pady=(0, PAD))
        self.nb = nb

        self.t_att = ttk.Frame(nb); nb.add(self.t_att, text="  姿态  ")
        self.t_wave = ttk.Frame(nb); nb.add(self.t_wave, text="  波形  ")
        self.t_cfg = ttk.Frame(nb); nb.add(self.t_cfg, text="  配置  ")
        self.t_data = ttk.Frame(nb); nb.add(self.t_data, text="  数据  ")

        self._build_attitude()
        self._build_wave()
        self._build_cfg()
        self._build_data()

    # ---------------- 页签 1：姿态 ----------------
    def _build_attitude(self):
        f = self.t_att
        left = ttk.Frame(f); left.pack(side="left", fill="y", padx=(8, 4), pady=8)

        ttk.Label(left, text="人工地平仪（Roll / Pitch）").pack()
        self.cv_ah = tk.Canvas(left, width=260, height=260, bg="#111116",
                               highlightthickness=0)
        self.cv_ah.pack(pady=(2, 10))

        ttk.Label(left, text="罗盘（Yaw）").pack()
        self.cv_cp = tk.Canvas(left, width=260, height=120, bg="#111116",
                               highlightthickness=0)
        self.cv_cp.pack()

        # ---- 陀螺仪零偏校准面板 ----
        cal = ttk.LabelFrame(left, text="陀螺仪零偏校准", padding=(8, 4))
        cal.pack(fill="x", pady=(8, 0))
        r0 = ttk.Frame(cal); r0.pack(fill="x")
        ttk.Label(r0, text="采样").pack(side="left")
        self.cal_dur_var = tk.StringVar(value="3.0")
        ttk.Combobox(r0, textvariable=self.cal_dur_var, width=5, takefocus=False,
                     values=["1.0", "2.0", "3.0", "5.0", "10.0"]).pack(side="left", padx=2)
        ttk.Label(r0, text="秒").pack(side="left")
        self.btn_cal = ttk.Button(r0, text="开始校准", width=9, takefocus=False,
                                  command=self.toggle_cal)
        self.btn_cal.pack(side="left", padx=4)

        r1 = ttk.Frame(cal); r1.pack(fill="x", pady=(2, 0))
        ttk.Button(r1, text="清除", width=6, takefocus=False,
                   command=self.clear_cal).pack(side="left")
        ttk.Button(r1, text="存盘", width=6, takefocus=False,
                   command=self.save_cal).pack(side="left", padx=2)
        ttk.Button(r1, text="载入", width=6, takefocus=False,
                   command=self.load_cal).pack(side="left")
        ttk.Button(r1, text="写回模块", width=10, takefocus=False,
                   command=self.send_module_cal).pack(side="left", padx=4)

        self.cal_bar = ttk.Progressbar(cal, mode="determinate", maximum=100)
        self.cal_bar.pack(fill="x", pady=(4, 2))
        self.cal_state = ttk.Label(cal, text="未校准", foreground="#888",
                                   wraplength=240, justify="left")
        self.cal_state.pack(anchor="w")
        self.cal_bias_lab = ttk.Label(cal, text="零偏  X —  Y —  Z —",
                                      font=("Menlo", 9), foreground="#888")
        self.cal_bias_lab.pack(anchor="w")
        self.cal_noise_lab = ttk.Label(cal, text="噪声  X —  Y —  Z —",
                                       font=("Menlo", 9), foreground="#888")
        self.cal_noise_lab.pack(anchor="w")

        right = ttk.Frame(f); right.pack(side="left", fill="both", expand=True,
                                         padx=(8, 8), pady=8)
        self.vals = {}
        rows = [
            ("姿态角", None, None),
            ("Roll",  "roll",  "°"), ("Pitch", "pitch", "°"), ("Yaw", "yaw", "°"),
            ("加速度计", None, None),
            ("Acc X", "ax", " g"), ("Acc Y", "ay", " g"), ("Acc Z", "az", " g"),
            ("陀螺仪", None, None),
            ("Gyro X", "gx", " dps"), ("Gyro Y", "gy", " dps"), ("Gyro Z", "gz", " dps"),
            ("磁力计", None, None),
            ("Mag X", "mx", ""), ("Mag Y", "my", ""), ("Mag Z", "mz", ""),
            ("气压计", None, None),
            ("气压", "pressure", " Pa"), ("海拔", "altitude", " cm"),
            ("温度", "temp", " ℃"),
            ("四元数", None, None),
            ("Q0", "q0", ""), ("Q1", "q1", ""), ("Q2", "q2", ""), ("Q3", "q3", ""),
        ]
        for name, key, unit in rows:
            r = ttk.Frame(right); r.pack(fill="x", pady=1)
            if key is None:
                ttk.Label(r, text=name, foreground="#7aa2f7",
                          font=("", 11, "bold")).pack(anchor="w", pady=(6, 0))
                continue
            ttk.Label(r, text=name, width=9).pack(side="left")
            lab = ttk.Label(r, text="—", font=("Menlo", 12))
            lab.pack(side="left")
            ttk.Label(r, text=unit, foreground="#888").pack(side="left")
            self.vals[key] = lab

    # ---------------- 页签 2：波形 ----------------
    def _build_wave(self):
        f = self.t_wave
        bar = ttk.Frame(f); bar.pack(fill="x", padx=8, pady=(8, 4))
        ttk.Label(bar, text="显示：").pack(side="left")
        self.wave_var = tk.StringVar(value="acc")
        for key, name in (("acc", "加速度 (g)"), ("gyro", "陀螺仪 (dps)"),
                          ("mag", "磁力计 (原始)")):
            ttk.Radiobutton(bar, text=name, value=key, variable=self.wave_var,
                            command=self._redraw_wave, takefocus=False
                            ).pack(side="left", padx=4)
        self.wave_info = ttk.Label(bar, text="", foreground="#888")
        self.wave_info.pack(side="right")

        self.cv_wave = tk.Canvas(f, bg="#111116", highlightthickness=0)
        self.cv_wave.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self.cv_wave.bind("<Configure>", lambda e: self._redraw_wave())

    # ---------------- 页签 3：配置 ----------------
    def _build_cfg(self):
        f = self.t_cfg
        bar = ttk.Frame(f); bar.pack(fill="x", padx=8, pady=8)
        for txt, cmd in (("读取全部配置", self.read_all),
                         ("保存到 Flash", lambda: self.write_reg(REG_SAVE)),
                         ("传感器校准", lambda: self.write_reg(REG_SENCAL)),
                         ("恢复默认", self.do_reset)):
            ttk.Button(bar, text=txt, takefocus=False,
                       command=cmd).pack(side="left", padx=3)

        # ---- 常用配置 ----
        g = ttk.LabelFrame(f, text="常用配置", padding=(10, 6))
        g.pack(fill="x", padx=8, pady=(0, 8))

        def combo(parent, label, reg, values, width=16, row=None):
            r = ttk.Frame(parent)
            r.pack(fill="x", pady=2)
            ttk.Label(r, text=label, width=16).pack(side="left")
            var = tk.StringVar(value=values[0])
            cb = ttk.Combobox(r, textvariable=var, values=values, width=width,
                              state="readonly", takefocus=False)
            cb.pack(side="left", padx=4)
            ttk.Button(r, text="写入", width=6, takefocus=False,
                       command=lambda: self.write_reg(reg, var.get())
                       ).pack(side="left")
            ttk.Button(r, text="读取", width=6, takefocus=False,
                       command=lambda: self._read_one(reg)).pack(side="left", padx=3)
            return var, cb

        self.cfg_baud = combo(g, "波特率", REG_BAUD,
                              ["9600", "19200", "38400", "57600", "115200",
                               "230400", "460800", "921600"])
        self.cfg_rate = combo(g, "回传速率", REG_RETURNRATE,
                              ["0.2Hz", "0.5Hz", "1Hz", "2Hz", "5Hz", "10Hz",
                               "20Hz", "50Hz", "100Hz", "200Hz", "500Hz"])
        self.cfg_gyro = combo(g, "陀螺仪量程", REG_GYROFSR,
                              ["250dps", "500dps", "1000dps", "2000dps"])
        self.cfg_acc = combo(g, "加速度计量程", REG_ACCFSR,
                             ["2g", "4g", "8g", "16g"])
        self.cfg_alg = combo(g, "算法", REG_ALG,
                             ["9轴(磁力辅助)", "6轴(无磁力)"])
        self.cfg_asm = combo(g, "安装方向", REG_ASM,
                             ["水平(默认)", "竖直", "倒置", "侧装"])
        self.cfg_led = combo(g, "LED", REG_LEDOFF, ["打开", "关闭"])

        # ---- 回传内容（位掩码）----
        g2 = ttk.LabelFrame(f, text="回传内容（勾选后写入）", padding=(10, 6))
        g2.pack(fill="x", padx=8, pady=(0, 8))
        self.ret_vars = {}
        names = [(UP_ATTITUDE, "姿态角"), (UP_QUAT, "四元数"),
                 (UP_GYRO_ACCE, "加速度/陀螺"), (UP_MAG, "磁力计"),
                 (UP_BARO, "气压计"), (UP_PORT, "端口")]
        row = ttk.Frame(g2); row.pack(fill="x")
        for fid, nm in names:
            v = tk.BooleanVar(value=(fid in (UP_ATTITUDE, UP_GYRO_ACCE)))
            ttk.Checkbutton(row, text=nm, variable=v, takefocus=False
                            ).pack(side="left", padx=4)
            self.ret_vars[fid] = v
        ttk.Button(g2, text="写入回传内容", takefocus=False,
                   command=self.write_returns).pack(anchor="w", pady=(6, 0))

        # ---- 通用寄存器读写 ----
        g3 = ttk.LabelFrame(f, text="通用寄存器读写", padding=(10, 6))
        g3.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        r = ttk.Frame(g3); r.pack(fill="x")
        ttk.Label(r, text="寄存器").pack(side="left")
        self.reg_var = tk.StringVar()
        self.reg_box = ttk.Combobox(
            r, textvariable=self.reg_var, width=34, state="readonly", takefocus=False,
            values=["0x%02X  %s" % (k, v) for k, v in sorted(REG_NAMES.items())])
        self.reg_box.pack(side="left", padx=4)
        self.reg_box.current(0)
        ttk.Button(r, text="读取", width=6, takefocus=False,
                   command=self.read_sel_reg).pack(side="left", padx=2)
        ttk.Label(r, text="  值(十进制或0x..)").pack(side="left")
        self.reg_val = tk.StringVar(value="0")
        ttk.Entry(r, textvariable=self.reg_val, width=10).pack(side="left", padx=3)
        ttk.Button(r, text="写入", width=6, takefocus=False,
                   command=self.write_sel_reg).pack(side="left")

        self.cfg_log = tk.Text(g3, height=6, bg="#111116", fg="#ccc",
                               font=("Menlo", 10), wrap="none")
        self.cfg_log.pack(fill="both", expand=True, pady=(6, 0))

    # ---------------- 页签 4：数据 ----------------
    def _build_data(self):
        f = self.t_data
        bar = ttk.Frame(f); bar.pack(fill="x", padx=8, pady=8)
        self.btn_rec = ttk.Button(bar, text="● 开始记录 CSV", width=18,
                                  takefocus=False, command=self.toggle_rec)
        self.btn_rec.pack(side="left")
        self.rec_lab = ttk.Label(bar, text="未记录", foreground="#888")
        self.rec_lab.pack(side="left", padx=10)
        ttk.Button(bar, text="清空原始帧", width=12, takefocus=False,
                   command=lambda: self.raw_text.delete("1.0", "end")).pack(side="right")

        ttk.Label(f, text="原始帧监视（调试用：能看到模块实际发了哪些帧）"
                  ).pack(anchor="w", padx=8)
        self.raw_text = tk.Text(f, height=14, bg="#111116", fg="#ccc",
                                font=("Menlo", 10), wrap="none")
        self.raw_text.pack(fill="both", expand=True, padx=8, pady=(2, 8))
        for tag, col in (("sys", "#7aa2f7"), ("err", "#ff5f56"),
                         ("up", "#63e06a"), ("ack", "#c9a227")):
            self.raw_text.tag_config(tag, foreground=col)

    # =================================================================
    # 四、串口收发
    # =================================================================
    def _send(self, b):
        if not (self.link and self.link.alive):
            self._log("未连接串口", "err"); return False
        return self.link.send(b)

    def _read_one(self, reg):
        if self._send(build_read(reg)):
            self._log("读 %s (0x%02X)" % (REG_NAMES.get(reg, "?"), reg), "sys")

    def read_sel_reg(self):
        try:
            reg = int(self.reg_var.get().split()[0], 16)
        except Exception:
            self._log("寄存器选择无效", "err"); return
        self._read_one(reg)

    def write_sel_reg(self):
        try:
            reg = int(self.reg_var.get().split()[0], 16)
            raw = self.reg_val.get().strip()
            val = int(raw, 16) if raw.lower().startswith("0x") else int(raw)
        except Exception:
            self._log("值必须是十进制或 0x 开头的十六进制", "err"); return
        self.write_reg(reg, val)

    def write_reg(self, reg, value=0):
        """写寄存器。

        注意：几个"选项类"寄存器（波特率/回传速率/量程/算法/安装方向/LED）
        在界面上是下拉框，显示的是文字，要换算成寄存器值。
        """
        if isinstance(value, str):
            value = self._label_to_value(reg, value)
        if value is None:
            self._log("无法识别的取值", "err"); return
        d = [value] if not isinstance(value, (list, tuple)) else list(value)
        if self._send(build_write(reg, d)):
            self._log("写 %s (0x%02X) = %s" % (REG_NAMES.get(reg, "?"), reg, value), "sys")

    @staticmethod
    def _label_to_value(reg, label):
        """下拉框文字 -> 寄存器数值"""
        def idx(lst, s, default=0):
            try:
                return lst.index(s)
            except ValueError:
                return default
        if reg == REG_BAUD:
            return idx(["9600", "19200", "38400", "57600", "115200",
                        "230400", "460800", "921600"], label, 4)
        if reg == REG_RETURNRATE:
            return idx(["0.2Hz", "0.5Hz", "1Hz", "2Hz", "5Hz", "10Hz",
                        "20Hz", "50Hz", "100Hz", "200Hz", "500Hz"], label, 8)
        if reg == REG_GYROFSR:
            return idx(["250dps", "500dps", "1000dps", "2000dps"], label, 2)
        if reg == REG_ACCFSR:
            return idx(["2g", "4g", "8g", "16g"], label, 0)
        if reg == REG_ALG:
            return idx(["9轴(磁力辅助)", "6轴(无磁力)"], label, 0)
        if reg == REG_ASM:
            return idx(["水平(默认)", "竖直", "倒置", "侧装"], label, 0)
        if reg == REG_LEDOFF:
            return idx(["打开", "关闭"], label, 0)
        try:
            return int(label)
        except Exception:
            return None

    def write_returns(self):
        """回传内容是位掩码：bit(n-1) 对应帧 ID n"""
        v = 0
        for fid, var in self.ret_vars.items():
            if var.get():
                v |= (1 << (fid - 1))
        self.write_reg(REG_RETURNSET, v)

    def read_all(self):
        """连上后把关键配置读一遍（回传内容用位掩码，读了也不太能反推，就不读了）"""
        for reg in (REG_GYROFSR, REG_ACCFSR, REG_BAUD, REG_RETURNRATE,
                    REG_ALG, REG_ASM, REG_LEDOFF, REG_SENSTA):
            self._send(build_read(reg))
        self._log("已请求读取全部配置", "sys")

    def do_reset(self):
        if messagebox.askyesno("恢复默认", "确定把模块恢复出厂默认设置？\n"
                                           "（当前配置会丢失，需重新保存才会写入 Flash）"):
            self.write_reg(REG_RESET)

    # =================================================================
    # 五、主循环：收帧 + 刷新
    # =================================================================
    def _tick(self):
        if self.link:
            try:
                for _ in range(500):        # 单次最多处理 500 帧，防止界面卡死
                    head, fid, dat = self.link.q.get_nowait()
                    self._handle(head, fid, dat)
            except queue.Empty:
                pass

            # 帧率统计（每秒刷新一次）
            now = time.time()
            if now - self.t_last_stat >= 1.0:
                dt = now - self.t_last_stat
                self.rate = (self.link.frames_rx -
                             getattr(self, "_last_frames", 0)) / dt
                self._last_frames = self.link.frames_rx
                self.t_last_stat = now
                self.rate_lab.config(text="%.0f 帧/秒" % self.rate)
            if not self.link.alive:
                self._log("串口连接已断开", "err")
                self.disconnect()

        self._refresh()
        self.root.after(50, self._tick)

    def _handle(self, head, fid, dat):
        if head == FRAME_HEAD_UPLOAD:
            d = parse_upload(fid, dat, self.gyro_fsr_idx, self.acc_fsr_idx)
            if d is None:
                return
            t = d["type"]
            if t == "attitude":
                self.d_att = d
            elif t == "imu":
                # ---- 校准采样：存的是**未扣零偏**的原始 dps ----
                if self.calibrating:
                    self.cal_buf.append((d["gx"], d["gy"], d["gz"]))
                # ---- 扣零偏：GX/GY/GZ 从此是"修正后"的值 ----
                d["gx_raw"], d["gy_raw"], d["gz_raw"] = d["gx"], d["gy"], d["gz"]
                d["ax_raw"], d["ay_raw"], d["az_raw"] = d["ax"], d["ay"], d["az"]
                d["gx"] -= self.gyro_bias[0]
                d["gy"] -= self.gyro_bias[1]
                d["gz"] -= self.gyro_bias[2]
                self.d_imu = d
                # 波形缓冲（用修正后的值，曲线才在 0 附近）
                for k in ("ax", "ay", "az", "gx", "gy", "gz"):
                    buf = self.wave[k]; buf.append(d[k])
                    if len(buf) > WAVE_N:
                        del buf[0]
            elif t == "mag":
                self.d_mag = d
                for k, src in (("mx", "mx"), ("my", "my"), ("mz", "mz")):
                    buf = self.wave[k]; buf.append(d[src])
                    if len(buf) > WAVE_N:
                        del buf[0]
            elif t == "baro":
                self.d_baro = d
            elif t == "port":
                self.d_port = d
            elif t == "quat":
                self.d_quat = d

            if self.raw_text is not None:
                self._raw_line("UP  id=0x%02X %-10s len=%2d  %s"
                               % (fid, UPLOAD_NAMES.get(fid, "?"), len(dat),
                                  dat.hex(" ")), "up")
            if self.recording:
                self._record_row()
        else:
            self._raw_line("ACK id=0x%02X %-16s len=%2d  %s"
                           % (fid, REG_NAMES.get(fid & 0x7F, "?"), len(dat),
                              dat.hex(" ")), "ack")
            self._handle_ack(fid & 0x7F, dat)

    def _handle_ack(self, reg, dat):
        """把读回来的配置同步到界面控件上"""
        if not dat:
            return
        v = dat[0]
        self._cfg_log("读回 %-16s = %d (0x%02X)" % (REG_NAMES.get(reg, "?"), v, v))
        try:
            if reg == REG_GYROFSR and v < 4:
                self.gyro_fsr_idx = v
                self.cfg_gyro[0].set(["250dps", "500dps", "1000dps", "2000dps"][v])
            elif reg == REG_ACCFSR and v < 4:
                self.acc_fsr_idx = v
                self.cfg_acc[0].set(["2g", "4g", "8g", "16g"][v])
            elif reg == REG_BAUD and v < 8:
                self.cfg_baud[0].set(["9600", "19200", "38400", "57600", "115200",
                                      "230400", "460800", "921600"][v])
            elif reg == REG_RETURNRATE and v < 11:
                self.cfg_rate[0].set(["0.2Hz", "0.5Hz", "1Hz", "2Hz", "5Hz", "10Hz",
                                      "20Hz", "50Hz", "100Hz", "200Hz", "500Hz"][v])
            elif reg == REG_ALG and v < 2:
                self.cfg_alg[0].set(["9轴(磁力辅助)", "6轴(无磁力)"][v])
            elif reg == REG_ASM and v < 4:
                self.cfg_asm[0].set(["水平(默认)", "竖直", "倒置", "侧装"][v])
            elif reg == REG_LEDOFF and v < 2:
                self.cfg_led[0].set(["打开", "关闭"][v])
        except Exception:
            pass

    # =================================================================
    # 六、界面刷新
    # =================================================================
    def _refresh(self):
        # ---- 校准进度（到点自动收尾）----
        if self.calibrating:
            el = time.time() - self.cal_t0
            self.cal_bar["value"] = min(100.0, el / self.cal_dur * 100.0)
            if el >= self.cal_dur:
                self._finish_cal()
            else:
                self._cal_ui("采样中… %.1f / %.1f 秒（已收 %d 个样本）"
                             % (el, self.cal_dur, len(self.cal_buf)), "#c9a227")

        # ---- 数值面板 ----
        def setv(key, val, fmt="%.2f"):
            lab = self.vals.get(key)
            if lab is None:
                return
            lab.config(text="—" if val is None else (fmt % val))

        a = self.d_att or {}
        setv("roll", a.get("roll")); setv("pitch", a.get("pitch"))
        setv("yaw", a.get("yaw"))

        i = self.d_imu or {}
        for k in ("ax", "ay", "az"):
            setv(k, i.get(k), "%+.3f")
        for k in ("gx", "gy", "gz"):
            setv(k, i.get(k), "%+.1f")

        m = self.d_mag or {}
        for k in ("mx", "my", "mz"):
            setv(k, m.get(k), "%+d")
        if m.get("mtemp") is not None:
            self.vals["temp"].config(text="%+.1f" % m["mtemp"])

        b = self.d_baro or {}
        if b.get("pressure") is not None:
            self.vals["pressure"].config(text="%d" % b["pressure"])
        if b.get("altitude") is not None:
            self.vals["altitude"].config(text="%d" % b["altitude"])
        if b.get("btemp") is not None:
            self.vals["temp"].config(text="%+.1f" % b["btemp"])

        q = self.d_quat or {}
        for k in ("q0", "q1", "q2", "q3"):
            setv(k, q.get(k), "%+.3f")

        # ---- 姿态图 ----
        self._draw_ah(a.get("roll", 0.0), a.get("pitch", 0.0))
        self._draw_compass(a.get("yaw", 0.0))

        # ---- 波形 ----
        self.wave_mode = self.wave_var.get()
        if int(time.time() * 20) % 2 == 0:      # 约 10Hz 重画，省 CPU
            self._redraw_wave()

        # ---- 记录状态 ----
        if self.recording:
            self.rec_lab.config(text="记录中  %d 行" % self.rec_rows,
                                foreground="#3a3")

    # ---------------- 人工地平仪 ----------------
    def _draw_ah(self, roll, pitch):
        c = self.cv_ah
        c.delete("all")
        W = H = 260
        cx, cy = W / 2, H / 2
        R = 96

        # 用"旋转 + 平移"的天地球来表现姿态：
        #   先把天/地两块画在一个足够大的矩形里，再整体旋转 roll、平移 pitch
        #   （tkinter 没有裁剪，所以用圆环遮罩的近似做法：只画圆内部分）
        import math
        rr = math.radians(roll)
        pp = pitch * 9.0                      # 每度俯仰对应的像素位移
        # 天地分界线在旋转后的坐标系里的位置
        # 画一个覆盖全画布的"地平线以下"区域
        def rot(x, y):
            return (cx + x * math.cos(rr) - y * math.sin(rr),
                    cy + x * math.sin(rr) + y * math.cos(rr) + pp)

        # 天空（上）
        sky = [rot(-400, -400), rot(400, -400), rot(400, 0), rot(-400, 0)]
        c.create_polygon([v for p in sky for v in p], fill="#1d3a6e", outline="")
        # 地面（下）
        gnd = [rot(-400, 0), rot(400, 0), rot(400, 400), rot(-400, 400)]
        c.create_polygon([v for p in gnd for v in p], fill="#6b4a1f", outline="")
        # 地平线
        p1, p2 = rot(-400, 0), rot(400, 0)
        c.create_line(p1[0], p1[1], p2[0], p2[1], fill="#fff", width=2)

        # 俯仰刻度（每 10 度一条短线）
        for d in range(-90, 91, 10):
            if d == 0:
                continue
            y = d * -9.0
            a1, a2 = rot(-26, y), rot(26, y)
            c.create_line(a1[0], a1[1], a2[0], a2[1], fill="#9ab", width=1)

        # 圆环遮罩：把圆外涂掉（用背景色画四条粗弧的近似做法 -> 直接画圆环）
        c.create_oval(cx - R - 12, cy - R - 12, cx + R + 12, cy + R + 12,
                      outline="#000", width=26)
        c.create_oval(cx - R, cy - R, cx + R, cy + R, outline="#444", width=2)

        # 飞机符号（固定不动）
        c.create_line(cx - 46, cy, cx - 14, cy, fill="#ffcc00", width=3)
        c.create_line(cx + 14, cy, cx + 46, cy, fill="#ffcc00", width=3)
        c.create_line(cx - 14, cy, cx, cy + 8, fill="#ffcc00", width=3)
        c.create_line(cx + 14, cy, cx, cy + 8, fill="#ffcc00", width=3)
        c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill="#ffcc00", outline="")

        c.create_text(cx, H - 12, text="Roll %+.1f°   Pitch %+.1f°" % (roll, pitch),
                      fill="#ddd", font=("Menlo", 11))

    def _draw_compass(self, yaw):
        import math
        c = self.cv_cp
        c.delete("all")
        W, H = 260, 120
        cx, cy = W / 2, 62
        R = 46
        # 表盘：跟着 yaw 转（yaw 增大 -> 表盘反向转）
        for d in range(0, 360, 15):
            ang = math.radians(d - yaw - 90)
            r1 = R if d % 45 == 0 else R - 6
            x1, y1 = cx + r1 * math.cos(ang), cy + r1 * math.sin(ang)
            x2, y2 = cx + R * math.cos(ang), cy + R * math.sin(ang)
            c.create_line(x1, y1, x2, y2, fill="#888", width=2 if d % 45 == 0 else 1)
        for d, txt in ((0, "N"), (90, "E"), (180, "S"), (270, "W")):
            ang = math.radians(d - yaw - 90)
            x, y = cx + (R + 12) * math.cos(ang), cy + (R + 12) * math.sin(ang)
            c.create_text(x, y, text=txt, fill="#ff5f56" if txt == "N" else "#aaa",
                          font=("Menlo", 11, "bold"))
        # 指针（固定朝上）
        c.create_polygon(cx, cy - R + 4, cx - 6, cy + 8, cx + 6, cy + 8,
                         fill="#63e06a", outline="")
        c.create_text(cx, H - 10, text="Yaw %+.1f°" % yaw, fill="#ddd",
                      font=("Menlo", 11))

    # ---------------- 波形 ----------------
    def _redraw_wave(self):
        c = self.cv_wave
        if not c.winfo_exists():
            return
        c.delete("all")
        W = max(c.winfo_width(), 200)
        H = max(c.winfo_height(), 120)

        mode = self.wave_mode
        if mode == "acc":
            keys, cols, unit = (("ax", "ay", "az"),
                                ("#ff5f56", "#63e06a", "#7aa2f7"), "g")
        elif mode == "gyro":
            keys, cols, unit = (("gx", "gy", "gz"),
                                ("#ff5f56", "#63e06a", "#7aa2f7"), "dps")
        else:
            keys, cols, unit = (("mx", "my", "mz"),
                                ("#ff5f56", "#63e06a", "#7aa2f7"), "raw")

        # 自动量程
        peak = 1e-6
        for k in keys:
            for v in self.wave[k]:
                peak = max(peak, abs(v))
        # 往上取整到好看的值
        import math
        mag = 10 ** math.floor(math.log10(peak))
        peak = math.ceil(peak / mag) * mag
        peak = max(peak, 1.0 if mode == "acc" else 10.0)

        # 网格
        for i in range(5):
            y = 10 + (H - 30) * i / 4
            c.create_line(44, y, W - 8, y, fill="#2a2a31")
        c.create_line(44, H / 2, W - 8, H / 2, fill="#444")
        # 纵轴刻度
        for i in range(5):
            y = 10 + (H - 30) * i / 4
            val = peak * (1 - i / 2.0)
            c.create_text(38, y, text=("%+.2f" % val) if peak < 10 else ("%+.0f" % val),
                          fill="#888", font=("Menlo", 8), anchor="e")
        c.create_text(8, 12, text=unit, fill="#888", font=("Menlo", 9), anchor="w")

        n = WAVE_N
        plot_w = W - 52

        def draw(k, col):
            buf = self.wave[k]
            if len(buf) < 2:
                return
            pts = []
            m = len(buf)
            for i, v in enumerate(buf):
                x = 44 + plot_w * (i + (n - m)) / (n - 1)
                y = (H - 20) / 2 + 5 - v / peak * ((H - 30) / 2)
                pts.extend((x, y))
            c.create_line(*pts, fill=col, width=2)

        for k, col in zip(keys, cols):
            draw(k, col)

        # 图例
        x = 52
        for k, col in zip(keys, cols):
            c.create_line(x, 12, x + 16, 12, fill=col, width=3)
            c.create_text(x + 20, 12, text=k.upper(), fill="#ccc",
                          font=("Menlo", 9), anchor="w")
            x += 62
        self.wave_info.config(text="量程 ±%.2f %s   点数 %d"
                                   % (peak, unit, len(self.wave[keys[0]])))

    # =================================================================
    # 七、记录与日志
    # =================================================================
    def toggle_rec(self):
        if self.recording:
            self.recording = False
            if self.rec_writer:
                try:
                    self.rec_file.close()
                except Exception:
                    pass
            self.btn_rec.config(text="● 开始记录 CSV")
            self._log("停止记录，共 %d 行 -> %s"
                      % (self.rec_rows, os.path.basename(self.rec_path)), "sys")
            self.rec_writer = None
        else:
            p = filedialog.asksaveasfilename(
                title="保存 CSV", defaultextension=".csv",
                initialfile="imu_%s.csv" % time.strftime("%m%d_%H%M%S"),
                filetypes=[("CSV", "*.csv"), ("所有文件", "*.*")])
            if not p:
                return
            try:
                self.rec_file = open(p, "w", newline="", encoding="utf-8")
                self.rec_writer = csv.writer(self.rec_file)
                # 陀螺/加计同时记录"修正后"和"原始"两组：
                #   修正后用来直接用，原始用来事后分析零偏漂移
                self.rec_writer.writerow(
                    ["t", "roll", "pitch", "yaw",
                     "ax", "ay", "az", "gx", "gy", "gz",
                     "ax_raw", "ay_raw", "az_raw", "gx_raw", "gy_raw", "gz_raw",
                     "mx", "my", "mz", "pressure", "altitude", "temp",
                     "q0", "q1", "q2", "q3"])
                self.rec_path = p
            except Exception as e:
                self._log("打开文件失败：%s" % e, "err"); return
            self.rec_rows = 0
            self.recording = True
            self.btn_rec.config(text="■ 停止记录")
            self._log("开始记录 -> %s" % os.path.basename(p), "sys")

    def _record_row(self):
        try:
            a = self.d_att or {}; i = self.d_imu or {}
            m = self.d_mag or {}; b = self.d_baro or {}; q = self.d_quat or {}
            self.rec_writer.writerow([
                "%.3f" % time.time(),
                a.get("roll", ""), a.get("pitch", ""), a.get("yaw", ""),
                i.get("ax", ""), i.get("ay", ""), i.get("az", ""),
                i.get("gx", ""), i.get("gy", ""), i.get("gz", ""),
                i.get("ax_raw", ""), i.get("ay_raw", ""), i.get("az_raw", ""),
                i.get("gx_raw", ""), i.get("gy_raw", ""), i.get("gz_raw", ""),
                m.get("mx", ""), m.get("my", ""), m.get("mz", ""),
                b.get("pressure", ""), b.get("altitude", ""),
                m.get("mtemp", b.get("btemp", "")),
                q.get("q0", ""), q.get("q1", ""), q.get("q2", ""), q.get("q3", ""),
            ])
            self.rec_rows += 1
            if self.rec_rows % 50 == 0:
                self.rec_file.flush()
        except Exception:
            pass

    def _raw_line(self, text, tag="up"):
        t = self.raw_text
        if t is None:
            return
        t.insert("end", "%s  %s\n" % (time.strftime("%H:%M:%S"), text), tag)
        # 只保留最后 400 行，防止内存膨胀
        if int(t.index("end-1c").split(".")[0]) > 400:
            t.delete("1.0", "60.0")
        t.see("end")

    def _cfg_log(self, text):
        t = getattr(self, "cfg_log", None)
        if t is None:
            return
        t.insert("end", "%s  %s\n" % (time.strftime("%H:%M:%S"), text))
        t.see("end")

    def _log(self, text, tag="sys"):
        self._cfg_log(text)
        self._raw_line(text, tag)

    # =================================================================
    # 八、陀螺仪零偏校准
    #
    # 原理：陀螺仪静止时输出应该恒为 0，实际上每轴都有一个固定偏差
    #   （几十 LSB，还会随温度漂）。这个偏差如果不去掉，对角速度做积分
    #   算角度时会**越积越歪** —— 所以必须先测出来扣掉。
    #
    # 做法：让用户保持静止，采 N 秒，每轴取平均就是零偏。
    #
    # 关键点：**必须自动判断采样期间设备有没有动**。
    #   如果边晃边采，算出来的"零偏"里混进了真实转动，越校越歪。
    #   所以同时算标准差 —— 标准差大就说明在动，直接判失败并拒绝写入。
    # =================================================================
    # 阈值怎么定：手拿着晃是几十~几百 dps，放桌上静止通常 < 1 dps，
    # 中间有很大的空档，所以 3.0 既不误杀也不会放过真实的晃动。
    NOISE_MAX = 3.0      # 每轴标准差超过这个值(dps)就认为"采样期间在动"
    BIAS_MAX  = 50.0     # 零偏绝对值超过这个值(dps)就认为数据异常
    CAL_MIN_N = 30       # 至少要有这么多个样本才算数

    def toggle_cal(self):
        if self.calibrating:
            self._finish_cal()          # 手动提前结束
            return
        if not (self.link and self.link.alive):
            self._log("先连接串口再校准", "err"); return
        if self.d_imu is None:
            self._log("还没收到加速度/陀螺帧，无法校准", "err"); return
        try:
            self.cal_dur = max(0.5, float(self.cal_dur_var.get()))
        except Exception:
            self.cal_dur = 3.0
        self.cal_buf = []
        self.calibrating = True
        self.cal_t0 = time.time()
        self.btn_cal.config(text="■ 停止")
        self._cal_ui("采样中… 请保持模块完全静止", "#c9a227")
        self._log("开始陀螺仪校准：采样 %.1f 秒，请保持静止" % self.cal_dur, "sys")

    def _finish_cal(self):
        self.calibrating = False
        self.btn_cal.config(text="开始校准")
        self.cal_bar["value"] = 0
        buf = self.cal_buf
        n = len(buf)
        if n < self.CAL_MIN_N:
            self._cal_ui("样本太少（%d 个），校准失败" % n, "#ff5f56")
            self._log("校准失败：只收到 %d 个样本" % n, "err"); return

        # 每轴：均值 = 零偏，标准差 = 噪声
        bias, std = [], []
        for i in range(3):
            col = [v[i] for v in buf]
            m = sum(col) / n
            var = sum((x - m) ** 2 for x in col) / n
            bias.append(m); std.append(var ** 0.5)

        nmax = max(std)
        bmax = max(abs(x) for x in bias)

        # ---- 自动判定 ----
        if nmax > self.NOISE_MAX:
            self._cal_ui("❌ 采样期间模块在动（噪声 %.1f dps，阈值 %.1f）\n"
                         "请放稳后重试 —— 边晃边校会把转动当成零偏"
                         % (nmax, self.NOISE_MAX), "#ff5f56")
            self._log("校准失败：噪声 %.2f dps 超阈值 %.1f，模块没静止"
                      % (nmax, self.NOISE_MAX), "err")
            self._show_cal_nums(bias, std)
            return
        if bmax > self.BIAS_MAX:
            self._cal_ui("❌ 零偏异常大（%.1f dps，阈值 %.1f）\n"
                         "可能是量程设置不对，或模块有问题"
                         % (bmax, self.BIAS_MAX), "#ff5f56")
            self._log("校准失败：零偏 %.2f dps 超阈值 %.1f" % (bmax, self.BIAS_MAX), "err")
            self._show_cal_nums(bias, std)
            return

        # ---- 通过 ----
        self.gyro_bias = bias
        self._show_cal_nums(bias, std)
        self._cal_ui("✅ 校准完成（%d 个样本，%.1f 秒）\n已生效：显示/波形/记录都会扣掉零偏"
                     % (n, self.cal_dur), "#3a3")
        self._log("✅ 陀螺仪校准完成：零偏 X=%+.3f Y=%+.3f Z=%+.3f dps，"
                  "噪声 %.3f/%.3f/%.3f dps，样本 %d"
                  % (bias[0], bias[1], bias[2], std[0], std[1], std[2], n), "sys")
        self.save_cal(quiet=True)

    def _show_cal_nums(self, bias, std):
        self.cal_bias_lab.config(
            text="零偏  X %+.3f  Y %+.3f  Z %+.3f  dps" % tuple(bias),
            foreground="#ddd")
        self.cal_noise_lab.config(
            text="噪声  X %.3f  Y %.3f  Z %.3f  dps" % tuple(std),
            foreground="#63e06a" if max(std) <= self.NOISE_MAX else "#ff5f56")

    def _cal_ui(self, text, color):
        self.cal_state.config(text=text, foreground=color)

    def clear_cal(self):
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.cal_buf = []
        self.calibrating = False
        self.btn_cal.config(text="开始校准")
        self.cal_bar["value"] = 0
        self._cal_ui("未校准（零偏已清零）", "#888")
        self.cal_bias_lab.config(text="零偏  X —  Y —  Z —", foreground="#888")
        self.cal_noise_lab.config(text="噪声  X —  Y —  Z —", foreground="#888")
        self._log("已清除陀螺仪零偏校正", "sys")

    def save_cal(self, quiet=False):
        import json
        if not any(self.gyro_bias):
            if not quiet:
                self._log("零偏全是 0，没什么可存的", "err")
            return
        try:
            with open(self.cal_path, "w", encoding="utf-8") as fp:
                json.dump({"bias": self.gyro_bias,
                           "saved": time.strftime("%Y-%m-%d %H:%M:%S")}, fp, indent=1)
            if not quiet:
                self._log("零偏已存到 %s" % os.path.basename(self.cal_path), "sys")
        except Exception as e:
            self._log("存盘失败：%s" % e, "err")

    def load_cal(self):
        import json
        if not os.path.exists(self.cal_path):
            self._log("还没有存过零偏文件", "err"); return
        try:
            with open(self.cal_path, encoding="utf-8") as fp:
                d = json.load(fp)
            b = [float(x) for x in d["bias"]]
            assert len(b) == 3
        except Exception as e:
            self._log("载入失败：%s" % e, "err"); return
        self.gyro_bias = b
        self._show_cal_nums(b, [0, 0, 0])
        self.cal_noise_lab.config(text="噪声  —（来自存档，非本次实测）", foreground="#888")
        self._cal_ui("已载入存档零偏（%s）" % d.get("saved", "?"), "#7aa2f7")
        self._log("已载入零偏 X=%+.3f Y=%+.3f Z=%+.3f dps" % tuple(b), "sys")

    def send_module_cal(self):
        """让模块自己做一次内部校准（写 0x01 寄存器）。

        和上面的软件零偏是两回事：
          软件零偏 —— 只影响本上位机显示，不动模块
          模块校准 —— 模块内部重新标定，会改变它之后上传的数据，
                      想让结果持久还要再点「保存到 Flash」
        """
        if not (self.link and self.link.alive):
            self._log("未连接串口", "err"); return
        if not messagebox.askyesno(
                "模块内部校准",
                "这会通知模块自己做一次传感器校准。\n\n"
                "· 校准时模块必须保持静止\n"
                "· 校准只影响模块，和本软件的零偏是两回事\n"
                "· 想断电保留，校准后还要点「保存到 Flash」\n\n"
                "继续吗？"):
            return
        self.write_reg(REG_SENCAL)
        self._log("已发送模块内部校准指令（静止几秒后再保存到 Flash）", "sys")

    # =================================================================
    def on_close(self):
        self.recording = False
        if getattr(self, "rec_writer", None):
            try:
                self.rec_file.close()
            except Exception:
                pass
        self.disconnect()
        self.root.destroy()


# =====================================================================
def main():
    ap = argparse.ArgumentParser(
        description="ATK-MS901M / ATK-IMU901 IMU 上位机",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="协议说明见 IMU_ATK_MS901M_PROTOCOL.md")
    ap.add_argument("--port", help="串口设备，如 /dev/cu.usbserial-1120")
    ap.add_argument("--baud", type=int, default=115200, help="波特率（默认 115200）")
    args = ap.parse_args()

    root = tk.Tk()
    App(root, port=args.port, baud=args.baud)
    root.mainloop()


if __name__ == "__main__":
    main()

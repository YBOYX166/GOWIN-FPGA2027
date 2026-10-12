#!/usr/bin/env python3
# =====================================================================
# servo_gui.py —— Tang Nano 4K 舵机微调 GUI（4 个滑块）
#
# 只依赖 tkinter（Python 自带）+ pyserial。
#
#   ⚠️ 这台机器上有两个 python，别用错：
#        python3              -> miniforge3 的 3.13，tkinter 有、pyserial 没有 ❌
#        /usr/bin/python3     -> 系统自带 3.9，tkinter + pyserial 都有 ✅
#      所以直接这样跑，不用装任何东西：
#        /usr/bin/python3 servo_gui.py
#      非要用 miniforge 那个，就先 /Users/yboyx/miniforge3/bin/pip3 install pyserial
#
#   /usr/bin/python3 servo_gui.py                       # 自动找串口
#   /usr/bin/python3 servo_gui.py -p /dev/cu.usbserial-1120
#   /usr/bin/python3 servo_gui.py --scale360 0          # FPGA 侧 SCALE_360=0 时
#
# ---- 录制 / 回放 ----
#   ● 开始录制：把**每一次真正发到 FPGA 的指令**连时间戳记下来
#                （记在 _sender 里，所以记的就是串口上真实发生的事）
#   保存/打开 ：JSON 文件，能存档、能发给别人复现问题
#   ▶ 回放     ：按原时间戳重发；支持 0.25x~4x 变速和循环
#                回放时滑块会跟着动，但被禁用（免得手动和回放打架）
#
#   为什么回放要绕开 pending 直发：pending 是"同一路只留最新值"的合并机制，
#   回放里连续的同路指令会被合并掉，时间轴就不是原样了。
#
# ---- 时间轴编辑（让四路同时动）----
#   录制时一次只能拖一个滑块，所以时间戳天然是错开的。要让四个舵机同时动，
#   就得能事后编辑：面板上 4 行 = 4 路，横轴是时间。
#
#     单击圆点   = 选中          Shift+单击 = 加选
#     空白处单击 = 取消选中
#     按住拖动   = 改时间（按「吸附」栅格对齐）
#     滚轮       = 缩放          双击圆点 = 改角度
#
#   最常用的三步：
#     ① 四路各自"到位"的那一条 Shift 多选起来
#     ② 点「对齐选中」 -> 它们落到同一时刻
#     ③ 点「▶ 回放」   -> 四个舵机同时动
#
#   「吸附全部」是把所有时间量化到栅格（比如 100ms），把"差一点点"的
#   动作自动并成一组。"整条通道平移"用来调整某一路的整体提前/滞后。
#
# ---- 起始位（保证回放可复现）----
#   录制开始时**自动记下当时的四路位置**作为起始位。回放时先发这四条指令
#   把机械臂送回同一姿势，等设定时间（默认 600ms）让舵机到位，再开始走时间轴。
#
#   为什么必须有：录制只能从"当时那个姿势"开始。如果回放时不先回位，
#   夹子可能正夹着东西、手臂可能停在半路，第一段动作就完全走样了，
#   每次回放结果都不一样。
#
#   面板上还能手动「以当前位置为起始」重新采一个，或「回到起始位」只回位
#   不回放。起始位会一起存进 JSON。
#
# ---- 两个关键的工程问题，以及这里的解法 ----
#
# ① 拖动滑块会产生「事件风暴」
#    滑块每移动 1 像素就回调一次，一秒能出几百次。而 115200 的串口发一条
#    4 位指令要 ~350us，FPGA 回复一条要 ~1.1ms，直接发必然淹没、回复全丢。
#    解法：不在回调里发送。回调只把「目标值」写进 pending 字典（同一路
#    后写的覆盖先写的），另有一个 60ms 的定时器把 pending 里的**最新值**
#    发出去。所以拖得再快，串口上也只有 16 条/秒，而且发的永远是当前位置。
#    （实测：300 次滑块回调最终只发 1 条，就是最后那个值。）
#
# ② 串口读取不能放在主线程
#    主线程被 tkinter 占着，阻塞读会让界面卡死。
#    解法：一个后台线程专门 readline()，把整行丢进 queue.Queue；
#    主线程用 root.after() 定时取队列刷新界面。线程之间只有队列交互。
#
# ---- 与 FPGA 的协议（见 ../fpga_project/src/servo_cmd.v）----
#   发：<舵机号 1-4><角度百位><十位><个位>      例 2090 = 2 号，值 090
#   收：S2=090 A045   （回执）   ERR（非法）   A 090 090 090 090（每秒心跳）
#
#   ⚠️ 滑块显示的是**舵机实际角度 0-180 度**，不是协议里的原始值。
#      两者关系由 FPGA 的 SCALE_360 决定，用 --scale360 告诉本程序：
#         SCALE_360=0（默认）：raw = 度          （0..180 直接就是角度）
#         SCALE_360=1        ：raw = 度 * 2      （0..360 映射 0..180）
#      舵机是 180 度的，所以两边都默认 0。不一致的话滑块拖到 90 度
#      舵机会跑到 45 度（或反过来）。窗口标题栏会显示当前值。
# =====================================================================
import sys
import os
import json
import glob
import queue
import threading
import time
import argparse

try:
    import serial
except ImportError:
    sys.exit("缺少 pyserial，请先安装：  pip3 install pyserial")

try:
    import tkinter as tk
    from tkinter import ttk, filedialog, simpledialog
except ImportError:
    sys.exit("这个 Python 没带 tkinter。macOS 上可以试试 /usr/bin/python3，"
             "或者 brew install python-tk")

BAUD = 115200
N_SERVO = 4
DEG_MIN, DEG_MAX = 0, 180
SEND_MS = 60          # 发送节流周期
POLL_MS = 50          # 界面刷新周期


# =====================================================================
# 纯逻辑部分（不依赖 tkinter，可单独测试）
# =====================================================================
def find_port():
    """返回候选串口列表。优先用 pyserial 自带的枚举（跨平台最可靠）。"""
    ports = []
    try:
        from serial.tools import list_ports
        ports = [p.device for p in list_ports.comports()]
    except Exception:
        ports = []
    if ports:
        return ports
    found = []
    for pat in ("/dev/tty.usbserial*", "/dev/tty.usbmodem*", "/dev/tty.SLAB*",
                "/dev/tty.wchusbserial*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        found += sorted(glob.glob(pat))
    return found


def fmt_cmd(servo_idx, deg, scale360):
    """舵机序号 0..3 + 角度 0..180  ->  4 位协议字符串"""
    deg = max(DEG_MIN, min(DEG_MAX, int(round(deg))))
    raw = deg * 2 if scale360 else deg
    raw = max(0, min(999, raw))
    return f"{servo_idx + 1}{raw:03d}"


def parse_hb(line):
    """'A 090 090 090 090' -> [90, 90, 90, 90]，不是心跳就返回 None"""
    parts = line.split()
    if len(parts) == 5 and parts[0] == "A" and all(p.isdigit() for p in parts[1:]):
        return [int(p) for p in parts[1:]]
    return None


def classify(line):
    """把 FPGA 发来的一行归类，用于日志着色"""
    if line.startswith("ERR"):
        return "err"
    if len(line) > 5 and line[0] == "S" and line[1] in "1234" and line[2] == "=":
        return "reply"
    if parse_hb(line):
        return "hb"
    return "banner"


# =====================================================================
# 动作记录的编辑模型（不依赖 tkinter，可以单独测）
#
# 为什么需要它：
#   录制的时候**一次只能拖一个滑块**，所以记下来的时间戳天然是错开的，
#   回放时四个舵机永远是"你一下我一下"。想让它们**同时动**，就必须能
#   事后改时间轴 —— 把几条动作对齐到同一时刻、整条通道平移、按栅格吸附。
#
#   每条动作有稳定的 id，选中状态用 id 表示，这样排序/增删之后选中不会错位。
#   每个改动前都会压一份快照，所以可以撤销。
# =====================================================================
class Track:
    def __init__(self):
        self.items = []            # [{'id':int,'t':float,'servo':0..3,'deg':0..180}]
        self.start_pose = None     # [d0,d1,d2,d3] 录制开始时的四路位置
        self._next = 1
        self._undo = []

    # ---------------- 基本 ----------------
    def clear(self):
        self.push_undo()
        self.items = []
        self._next = 1

    def add(self, t, servo, deg):
        self.items.append({'id': self._next, 't': float(t),
                           'servo': int(servo), 'deg': int(deg)})
        self._next += 1

    def __len__(self):
        return len(self.items)

    def duration(self):
        return max((e['t'] for e in self.items), default=0.0)

    def tuples(self):
        """排好序的 (t, servo, deg)，回放/保存用"""
        return [(e['t'], e['servo'], e['deg'])
                for e in sorted(self.items, key=lambda x: (x['t'], x['servo']))]

    def by_id(self, i):
        for e in self.items:
            if e['id'] == i:
                return e
        return None

    # ---------------- 撤销 ----------------
    def push_undo(self):
        import copy
        self._undo.append(copy.deepcopy(self.items))
        if len(self._undo) > 80:
            self._undo.pop(0)

    def undo(self):
        if not self._undo:
            return False
        self.items = self._undo.pop()
        return True

    # ---------------- 编辑 ----------------
    def snap(self, grid):
        """所有时间吸附到 grid 秒的整数倍（这就是"让它们同时"的主力手段）"""
        if grid <= 0:
            return 0
        self.push_undo()
        n = 0
        for e in self.items:
            nt = round(e['t'] / grid) * grid
            if abs(nt - e['t']) > 1e-9:
                n += 1
            e['t'] = round(nt, 6)
        return n

    def align(self, ids, target=None, mode='earliest'):
        """把选中的几条对齐到同一时刻 —— 这就是让四路同时动的关键操作"""
        sel = [e for e in self.items if e['id'] in ids]
        if len(sel) < 2:
            return 0
        if target is None:
            ts = [e['t'] for e in sel]
            target = min(ts) if mode == 'earliest' else (
                max(ts) if mode == 'latest' else sum(ts) / len(ts))
        self.push_undo()
        for e in sel:
            e['t'] = round(float(target), 6)
        return len(sel)

    def set_times(self, pairs, push=True):
        """{id: 新时间}，拖动时用；拖动过程 push=False，按下时压一次快照"""
        if push:
            self.push_undo()
        for e in self.items:
            if e['id'] in pairs:
                e['t'] = max(0.0, round(pairs[e['id']], 6))

    def shift_servo(self, servo, dt):
        """整条通道平移"""
        if abs(dt) < 1e-9:
            return 0
        self.push_undo()
        n = 0
        for e in self.items:
            if e['servo'] == servo:
                e['t'] = max(0.0, round(e['t'] + dt, 6))
                n += 1
        return n

    def delete(self, ids):
        if not ids:
            return 0
        self.push_undo()
        before = len(self.items)
        self.items = [e for e in self.items if e['id'] not in ids]
        return before - len(self.items)

    def compact(self):
        """同一路连续的相同角度只留第一条（清理拖动产生的冗余）"""
        self.push_undo()
        out, last = [], {}
        for e in sorted(self.items, key=lambda x: (x['t'], x['servo'])):
            k = e['servo']
            if k in last and last[k]['deg'] == e['deg']:
                continue
            out.append(e)
            last[k] = e
        n = len(self.items) - len(out)
        self.items = out
        return n

    def set_deg(self, i, deg):
        e = self.by_id(i)
        if not e:
            return
        self.push_undo()
        e['deg'] = max(0, min(180, int(deg)))

    # ---------------- 存取 ----------------
    def dump(self):
        return [{'t': round(e['t'], 3), 'servo': e['servo'] + 1, 'deg': e['deg']}
                for e in sorted(self.items, key=lambda x: (x['t'], x['servo']))]

    def load(self, ev):
        self.push_undo()
        self.items = []
        self._next = 1
        for t, s, d in sorted(ev, key=lambda x: x[0]):
            self.add(t, s, d)


# =====================================================================
# 串口收发（后台线程）
# =====================================================================
class Link:
    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD, timeout=0.2)
        self.rxq = queue.Queue()
        self.stop = False
        self.alive = True
        self.port = port
        self.t = threading.Thread(target=self._reader, daemon=True)
        self.t.start()

    def _reader(self):
        buf = b""
        while not self.stop:
            try:
                chunk = self.ser.read(64)
            except Exception:
                self.alive = False
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.rxq.put(line.decode("ascii", errors="replace").rstrip("\r"))

    def send(self, s):
        try:
            self.ser.write(s.encode("ascii"))
            return True
        except Exception:
            self.alive = False
            return False

    def close(self):
        self.stop = True
        time.sleep(0.05)
        try:
            self.ser.close()
        except Exception:
            pass


# =====================================================================
# 界面
# =====================================================================
class App:
    def __init__(self, root, scale360, initial_port):
        self.root = root
        self.scale360 = scale360
        self.link = None
        self.pending = {}          # 待发送：{舵机序号: 角度}
        self.lock = threading.Lock()
        self.suppress = False      # 程序改滑块时抑制回调
        self.last_sent = "—"
        self.hb_ok = False

        # ---- 录制 / 回放 / 编辑 ----
        self.recording = False
        self.track = Track()       # 动作记录（可编辑）
        self.rec_t0 = 0.0
        self.playing = False
        self.rp_i = 0
        self.rp_wall0 = 0.0
        self.rp_file = None
        self.rp_after = None
        self._play_list = []       # 回放开始时对记录做的快照（编辑不会影响正在播的）

        # ---- 时间轴编辑器状态 ----
        self.sel = set()           # 选中的动作 id
        self.view_t0 = 0.0         # 视图左边缘的时间
        self.view_sps = 80.0       # 每秒多少像素
        self._drag = None

        root.title(f"Tang Nano 4K 舵机微调   [SCALE_360={scale360}]")
        root.minsize(640, 860)

        self._build_top(initial_port)
        self._build_sliders()
        self._build_buttons()
        self._build_recorder()
        self._build_editor()
        self._build_readback()
        self._build_log()

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.after(SEND_MS, self._sender)
        root.after(POLL_MS, self._poller)
        if initial_port:
            root.after(200, self.connect)

    # ---------------- 顶部：串口选择 ----------------
    def _build_top(self, initial_port):
        f = ttk.Frame(self.root, padding=(10, 8, 10, 4))
        f.pack(fill="x")
        ttk.Label(f, text="串口").pack(side="left")

        self.port_var = tk.StringVar(value=initial_port or "")
        self.combo = ttk.Combobox(f, textvariable=self.port_var, width=30, values=find_port())
        self.combo.pack(side="left", padx=6)
        if not initial_port:
            ports = find_port()
            if ports:
                self.combo.set(ports[0])

        ttk.Button(f, text="刷新", width=5, command=self.refresh_ports).pack(side="left")
        ttk.Button(f, text="连接", width=5, command=self.connect).pack(side="left", padx=4)
        ttk.Button(f, text="断开", width=5, command=self.disconnect).pack(side="left")

        self.status = ttk.Label(f, text="● 未连接", foreground="#b00")
        self.status.pack(side="right")

    def refresh_ports(self):
        self.combo["values"] = find_port()

    # ---------------- 四个滑块 ----------------
    def _build_sliders(self):
        f = ttk.LabelFrame(self.root, text=f"角度（舵机实际度数 0-{DEG_MAX}）",
                           padding=(10, 6, 10, 10))
        f.pack(fill="x", padx=10, pady=6)

        self.scales = []
        self.val_labels = []
        for i in range(N_SERVO):
            row = ttk.Frame(f)
            row.pack(fill="x", pady=3)
            ttk.Label(row, text=f"舵机 {i+1}", width=7).pack(side="left")

            s = tk.Scale(row, from_=DEG_MIN, to=DEG_MAX, orient="horizontal",
                         resolution=1, showvalue=False, length=330,
                         command=lambda v, n=i: self._on_slide(n, v))
            s.set(90)
            s.pack(side="left")
            self.scales.append(s)

            lab = ttk.Label(row, text="90°  raw 180", width=15)
            lab.pack(side="left", padx=8)
            self.val_labels.append(lab)

    def _on_slide(self, idx, value):
        """回调只记录目标值，绝不在这里直接发串口（见文件头 ①）"""
        deg = int(float(value))
        self.val_labels[idx].config(text=f"{deg}°  raw {deg*2 if self.scale360 else deg}")
        if self.suppress:
            return
        with self.lock:
            self.pending[idx] = deg

    # ---------------- 按钮 ----------------
    def _build_buttons(self):
        f = ttk.Frame(self.root, padding=(10, 0))
        f.pack(fill="x")
        ttk.Button(f, text="全部回中位 90°", command=lambda: self.set_all(90)).pack(side="left")
        ttk.Button(f, text="全部 0°", command=lambda: self.set_all(0)).pack(side="left", padx=4)
        ttk.Button(f, text="全部 180°", command=lambda: self.set_all(180)).pack(side="left")
        ttk.Button(f, text="清空日志", command=lambda: self.log.delete("1.0", "end")).pack(side="right")

    def set_all(self, deg):
        self.suppress = True
        for i, s in enumerate(self.scales):
            s.set(deg)
            self.val_labels[i].config(
                text=f"{deg}°  raw {deg*2 if self.scale360 else deg}")
        self.suppress = False
        with self.lock:
            for i in range(N_SERVO):
                self.pending[i] = deg

    # =================================================================
    # 录制 / 回放
    #
    # 录什么：**每一帧实际发到 FPGA 的指令**，带相对时间戳。
    #         挂在 _sender 里记，所以记下来的就是串口上真实发生的事。
    # 回放：  按记录的时间戳，到点就直接把那条指令发出去，同时把对应滑块
    #         跟着移动（suppress 住回调，避免和直发重复发一遍）。
    #         之所以不让回放走 pending 那条路：pending 是"只留最新值"的
    #         合并机制，回放里连续的同路指令会被合并掉，时间就不是原样了。
    # =================================================================
    def _build_recorder(self):
        f = ttk.LabelFrame(self.root, text="录制 / 回放", padding=(10, 4, 10, 8))
        f.pack(fill="x", padx=10, pady=(6, 0))

        r1 = ttk.Frame(f); r1.pack(fill="x")
        self.btn_rec = ttk.Button(r1, text="● 开始录制", width=12, command=self.toggle_record)
        self.btn_rec.pack(side="left")
        ttk.Button(r1, text="保存…", width=7, command=self.save_events).pack(side="left", padx=(6, 0))
        ttk.Button(r1, text="打开…", width=7, command=self.load_events).pack(side="left", padx=(4, 0))
        ttk.Button(r1, text="清空", width=5, command=self.clear_events).pack(side="left", padx=(4, 0))
        self.rec_label = ttk.Label(r1, text="未录制", foreground="#666")
        self.rec_label.pack(side="right")

        r2 = ttk.Frame(f); r2.pack(fill="x", pady=(6, 0))
        self.btn_play = ttk.Button(r2, text="▶ 回放", width=12, command=self.toggle_play)
        self.btn_play.pack(side="left")
        ttk.Label(r2, text="速度").pack(side="left", padx=(10, 2))
        self.speed_var = tk.StringVar(value="1.0x")
        ttk.Combobox(r2, textvariable=self.speed_var, width=6, state="readonly",
                     values=["0.25x", "0.5x", "1.0x", "2.0x", "4.0x"]).pack(side="left")
        self.loop_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(r2, text="循环", variable=self.loop_var).pack(side="left", padx=(10, 0))
        self.play_label = ttk.Label(r2, text="", foreground="#666")
        self.play_label.pack(side="right")

        r3 = ttk.Frame(f); r3.pack(fill="x", pady=(6, 0))
        ttk.Label(r3, text="起始位").pack(side="left")
        self.pose_label = ttk.Label(r3, text="未记录", foreground="#666",
                                    font=("Menlo", 10))
        self.pose_label.pack(side="left", padx=(4, 8))
        ttk.Button(r3, text="以当前位置为起始", width=16,
                   command=self.ed_set_start).pack(side="left")
        ttk.Button(r3, text="回到起始位", width=11,
                   command=self.ed_go_home).pack(side="left", padx=3)
        self.home_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(r3, text="回放前先回位",
                        variable=self.home_var).pack(side="left", padx=(10, 0))
        self.home_ms = tk.StringVar(value="600")
        ttk.Entry(r3, textvariable=self.home_ms, width=6).pack(side="left", padx=(6, 2))
        ttk.Label(r3, text="ms 等待").pack(side="left")

    def _update_rec_label(self):
        if self.recording:
            self.rec_label.config(
                text="● 录制中 %d 条 / %.1f 秒" % (len(self.track), time.time() - self.rec_t0))
        elif len(self.track):
            self.rec_label.config(
                text="已录 %d 条 / %.1f 秒" % (len(self.track), self.track.duration()))
        else:
            self.rec_label.config(text="未录制")

    def toggle_record(self):
        if self.recording:
            self.recording = False
            self.btn_rec.config(text="● 开始录制")
            n = len(self.track)
            dur = self.track.duration()
            self._append("停止录制：共 %d 条，时长 %.1f 秒" % (n, dur), "sys")
        else:
            if self.playing:
                self.stop_play()
            self.track.clear()
            self.sel = set()
            self.redraw()
            # 起始位：录制一开始就记下当时的四路位置。
            # 这样回放前能先把机械臂送回同一个姿势，回放才是确定性的。
            self.track.start_pose = self.current_pose()
            self._update_pose_label()
            self.rec_t0 = time.time()
            self.recording = True
            self.btn_rec.config(text="■ 停止录制")
            self._append("开始录制…起始位记录为 %s" % self._pose_str(), "sys")
            self._append("（每次真正发到 FPGA 的指令都会被记下）", "sys")
        self._update_rec_label()

    def clear_events(self):
        self.track.clear()
        self.sel = set()
        self.rp_file = None
        self._update_rec_label()
        self.redraw()
        self._append("记录已清空", "sys")

    def save_events(self):
        if not len(self.track):
            self._append("没有可保存的记录", "sys"); return
        path = filedialog.asksaveasfilename(
            title="保存操作记录", defaultextension=".json",
            initialfile="servo_%s.json" % time.strftime("%m%d_%H%M%S"),
            filetypes=[("JSON 记录", "*.json"), ("所有文件", "*.*")])
        if not path:
            return
        data = {
            "version": 1,
            "scale360": self.scale360,
            "saved": time.strftime("%Y-%m-%d %H:%M:%S"),
            "duration": self.track.duration(),
            "count": len(self.track),
            "start_pose": self.track.start_pose,
            "events": self.track.dump(),
        }
        try:
            with open(path, "w", encoding="utf-8") as fp:
                json.dump(data, fp, ensure_ascii=False, indent=1)
            self.rp_file = path
            self._append("已保存 %d 条 -> %s" % (len(self.track), os.path.basename(path)), "sys")
        except Exception as e:
            self._append("保存失败：%s" % e, "err")

    def load_events(self):
        path = filedialog.askopenfilename(
            title="打开操作记录",
            filetypes=[("JSON 记录", "*.json"), ("所有文件", "*.*")])
        if not path:
            return
        try:
            with open(path, encoding="utf-8") as fp:
                data = json.load(fp)
            ev = [(float(e["t"]), int(e["servo"]) - 1, int(e["deg"]))
                  for e in data["events"]]
            ev.sort(key=lambda x: x[0])
        except Exception as e:
            self._append("打开失败：%s" % e, "err"); return
        self.track.load(ev)
        sp = data.get("start_pose")
        self.track.start_pose = [int(x) for x in sp] if sp and len(sp) == N_SERVO else None
        self.sel = set()
        self.rp_file = path
        self._update_rec_label()
        self._update_pose_label()
        self.redraw()
        self._append("已载入 %d 条（%s），时长 %.1f 秒"
                     % (len(self.track), os.path.basename(path), self.track.duration()), "sys")
        if data.get("scale360") is not None and data["scale360"] != self.scale360:
            self._append("⚠️ 记录里是 SCALE_360=%s，当前是 %s —— 回放角度会差一倍"
                         % (data["scale360"], self.scale360), "err")

    # =================================================================
    # 时间轴编辑器 —— "让四路同时动" 就靠这个
    #
    # 录制时你一次只能拖一个滑块，时间戳必然错开，所以必须能事后编辑：
    #   单击圆点    = 选中            Shift+单击 = 加选
    #   空白处单击  = 取消选中
    #   按住拖动    = 改时间（按「吸附」栅格对齐）
    #   滚轮        = 缩放            双击圆点 = 改角度
    #
    # 典型用法：把四路各自"到位"的那一条 Shift 多选起来，点「对齐选中」，
    # 它们就落到同一时刻，回放时四个舵机同时动。
    # =================================================================
    ED_ROW = 32
    ED_L = 48
    ED_T = 22

    def _build_editor(self):
        f = ttk.LabelFrame(self.root, text="动作编辑（时间轴）",
                           padding=(8, 4, 8, 6))
        f.pack(fill="x", padx=10, pady=(6, 0))

        bar = ttk.Frame(f); bar.pack(fill="x")
        ttk.Label(bar, text="吸附").pack(side="left")
        self.snap_var = tk.StringVar(value="50ms")
        ttk.Combobox(bar, textvariable=self.snap_var, width=7, state="readonly",
                     values=["关", "10ms", "20ms", "50ms", "100ms", "200ms", "500ms"]
                     ).pack(side="left", padx=(2, 8))
        ttk.Button(bar, text="吸附全部", width=9, command=self.ed_snap).pack(side="left")
        ttk.Button(bar, text="对齐选中", width=9, command=self.ed_align).pack(side="left", padx=3)
        ttk.Button(bar, text="删除选中", width=9, command=self.ed_delete).pack(side="left")
        ttk.Button(bar, text="合并重叠", width=9, command=self.ed_compact).pack(side="left", padx=3)
        ttk.Button(bar, text="撤销", width=6, command=self.ed_undo).pack(side="left")
        ttk.Button(bar, text="全选", width=6, command=self.ed_all).pack(side="left", padx=3)

        bar2 = ttk.Frame(f); bar2.pack(fill="x", pady=(4, 0))
        ttk.Label(bar2, text="整条通道平移：舵机").pack(side="left")
        self.sh_servo = tk.StringVar(value="1")
        ttk.Combobox(bar2, textvariable=self.sh_servo, width=3, state="readonly",
                     values=["1", "2", "3", "4"]).pack(side="left", padx=2)
        self.sh_ms = tk.StringVar(value="100")
        ttk.Entry(bar2, textvariable=self.sh_ms, width=7).pack(side="left")
        ttk.Label(bar2, text="ms").pack(side="left", padx=(2, 4))
        ttk.Button(bar2, text="← 提前", width=8,
                   command=lambda: self.ed_shift(-1)).pack(side="left")
        ttk.Button(bar2, text="推后 →", width=8,
                   command=lambda: self.ed_shift(+1)).pack(side="left", padx=3)
        ttk.Button(bar2, text="视图复位", width=9,
                   command=self.ed_reset_view).pack(side="left", padx=3)
        self.ed_info = ttk.Label(bar2, text="未选中", foreground="#666")
        self.ed_info.pack(side="right")

        self.canvas = tk.Canvas(f, height=4 * self.ED_ROW + self.ED_T + 8,
                                background="#1b1b1f", highlightthickness=0)
        self.canvas.pack(fill="x", pady=(6, 0))
        self.canvas.bind("<Button-1>", self._cv_press)
        self.canvas.bind("<B1-Motion>", self._cv_drag)
        self.canvas.bind("<ButtonRelease-1>", self._cv_release)
        self.canvas.bind("<Configure>", lambda e: self.redraw())
        self.canvas.bind("<MouseWheel>", self._cv_wheel)
        self.canvas.bind("<Double-Button-1>", self._cv_double)
        self._upd_info()

    # ---------------- 画布绘制 ----------------
    def _grid(self):
        v = self.snap_var.get()
        if v == "关":
            return 0.0
        try:
            return float(v.replace("ms", "")) / 1000.0
        except Exception:
            return 0.05

    def ed_reset_view(self):
        self.view_t0 = 0.0
        dur = max(2.0, self.track.duration() + 0.5)
        w = max(self.canvas.winfo_width() - self.ED_L, 200)
        self.view_sps = max(8.0, w / dur)
        self.redraw()

    def _playhead(self):
        if not self.playing:
            return None
        return (time.time() - self.rp_wall0) * self._speed()

    def redraw(self):
        c = self.canvas
        if not c.winfo_exists():
            return
        c.delete("all")
        w = max(c.winfo_width(), 240)
        h = max(c.winfo_height(), 100)
        t0, sps = self.view_t0, self.view_sps
        tmax = t0 + (w - self.ED_L) / sps

        step = 0.05
        while step * sps < 52 and step < 120:
            step *= 2
        k = 0
        while k * step <= tmax + step:
            t = k * step
            x = self.ED_L + (t - t0) * sps
            if x >= self.ED_L - 1:
                c.create_line(x, self.ED_T - 8, x, h - 2, fill="#2c2c33")
                c.create_text(x, self.ED_T - 14,
                              text=("%.1fs" % t) if step < 1 else ("%gs" % t),
                              fill="#7a7a88", font=("Menlo", 8))
            k += 1

        for i in range(N_SERVO):
            y = self.ED_T + i * self.ED_ROW + self.ED_ROW / 2
            c.create_line(self.ED_L, y, w - 2, y, fill="#2a2a31")
            c.create_text(self.ED_L - 26, y, text="舵机%d" % (i + 1),
                          fill="#9a9aa8", font=("Menlo", 9))

        for e in self.track.items:
            x = self.ED_L + (e['t'] - t0) * sps
            if x < self.ED_L - 8 or x > w + 8:
                continue
            y = self.ED_T + e['servo'] * self.ED_ROW + self.ED_ROW / 2
            on = e['id'] in self.sel
            r = 5 if on else 3
            c.create_oval(x - r, y - r, x + r, y + r,
                          fill=("#ff5f56" if on else "#7aa2f7"), outline="")
            if on:
                c.create_text(x, y - 13, text="%d°" % e['deg'],
                              fill="#ff9a94", font=("Menlo", 8))

        ph = self._playhead()
        if ph is not None:
            px = self.ED_L + (ph - t0) * sps
            if self.ED_L <= px <= w:
                c.create_line(px, self.ED_T - 8, px, h - 2, fill="#63e06a", width=2)

    # ---------------- 鼠标 ----------------
    def _hit(self, ex, ey, tol=11):
        row = int((ey - self.ED_T) // self.ED_ROW)
        if row < 0 or row >= N_SERVO:
            return None
        best, bd = None, tol
        for e in self.track.items:
            if e['servo'] != row:
                continue
            d = abs(self.ED_L + (e['t'] - self.view_t0) * self.view_sps - ex)
            if d < bd:
                bd, best = d, e
        return best

    def _cv_press(self, ev):
        if self.playing:
            self._append("回放中不能编辑，先停止回放", "sys"); return
        hit = self._hit(ev.x, ev.y)
        shift = bool(ev.state & 0x0001)
        if hit is None:
            if not shift:
                self.sel = set()
            self._drag = None
        else:
            if shift:
                self.sel.add(hit['id'])
            elif hit['id'] not in self.sel:
                self.sel = {hit['id']}
            self.track.push_undo()     # 拖动开始前压一次快照，整段拖动算一步
            self._drag = (ev.x, {e['id']: e['t'] for e in self.track.items
                                 if e['id'] in self.sel})
        self.redraw(); self._upd_info()

    def _cv_drag(self, ev):
        if not self._drag or self.playing:
            return
        x0, t0s = self._drag
        dt = (ev.x - x0) / self.view_sps
        g = self._grid()
        pairs = {}
        for i, t in t0s.items():
            nt = max(0.0, t + dt)
            if g:
                nt = round(nt / g) * g
            pairs[i] = nt
        self.track.set_times(pairs, push=False)
        self.redraw(); self._upd_info()

    def _cv_release(self, ev):
        self._drag = None

    def _cv_wheel(self, ev):
        self.view_sps = max(4.0, min(3000.0,
                                     self.view_sps * (1.25 if ev.delta > 0 else 0.8)))
        self.redraw()

    def _cv_double(self, ev):
        if self.playing:
            return
        hit = self._hit(ev.x, ev.y)
        if not hit:
            return
        new = simpledialog.askinteger(
            "修改角度", "舵机%d 的角度（0-180）：" % (hit['servo'] + 1),
            initialvalue=hit['deg'], minvalue=0, maxvalue=180, parent=self.root)
        if new is None:
            return
        self.track.set_deg(hit['id'], new)
        self.redraw(); self._upd_info()

    # ---------------- 工具栏 ----------------
    def _upd_info(self):
        n = len(self.sel)
        if not n:
            self.ed_info.config(text="未选中"); return
        sel = [e for e in self.track.items if e['id'] in self.sel]
        if n == 1:
            e = sel[0]
            self.ed_info.config(text="选中 1 条： t=%.2fs 舵机%d %d°"
                                     % (e['t'], e['servo'] + 1, e['deg']))
        else:
            ts = [e['t'] for e in sel]
            self.ed_info.config(text="选中 %d 条： t=%.2f~%.2fs（跨度 %.0fms）"
                                     % (n, min(ts), max(ts), (max(ts) - min(ts)) * 1000))

    def ed_all(self):
        self.sel = {e['id'] for e in self.track.items}
        self.redraw(); self._upd_info()

    def ed_snap(self):
        g = self._grid()
        if not g:
            self._append("先把「吸附」设成具体栅格（现在是「关」）", "sys"); return
        n = self.track.snap(g)
        self.redraw(); self._upd_info()
        self._append("按 %s 吸附：动到新位置的 %d 条" % (self.snap_var.get(), n), "sys")

    def ed_align(self):
        if len(self.sel) < 2:
            self._append("至少选中 2 条才能对齐（Shift+单击加选）", "sys"); return
        n = self.track.align(self.sel)
        self.redraw(); self._upd_info()
        self._append("已把 %d 条动作对齐到同一时刻 —— 这几路会同时动" % n, "sys")

    def ed_delete(self):
        if not self.sel:
            self._append("还没选中任何动作", "sys"); return
        n = self.track.delete(self.sel)
        self.sel = set()
        self.redraw(); self._upd_info()
        self._append("已删除 %d 条" % n, "sys")

    def ed_compact(self):
        n = self.track.compact()
        self.sel = set()
        self.redraw(); self._upd_info()
        self._append("合并连续重复动作：去掉 %d 条" % n, "sys")

    def ed_undo(self):
        if self.track.undo():
            self.sel = set()
            self.redraw(); self._upd_info()
            self._append("已撤销一步", "sys")
        else:
            self._append("没有可撤销的了", "sys")

    def ed_shift(self, sign):
        try:
            ms = float(self.sh_ms.get())
        except Exception:
            ms = 100.0
        servo = int(self.sh_servo.get()) - 1
        n = self.track.shift_servo(servo, sign * ms / 1000.0)
        self.redraw(); self._upd_info()
        self._append("舵机%d 整条%s %.0f ms（%d 条）"
                     % (servo + 1, "提前" if sign < 0 else "推后", ms, n), "sys")

    # ---------------- 起始位 ----------------
    def current_pose(self):
        """当前四路滑块的度数（也就是我们最后命令 FPGA 的值）"""
        return [int(round(sc.get())) for sc in self.scales]

    def _pose_str(self):
        p = self.track.start_pose
        return " ".join("%d°" % d for d in p) if p else "未记录"

    def _update_pose_label(self):
        p = self.track.start_pose
        if p:
            self.pose_label.config(text=self._pose_str(), foreground="#2a7")
        else:
            self.pose_label.config(text="未记录", foreground="#666")

    def ed_set_start(self):
        self.track.start_pose = self.current_pose()
        self._update_pose_label()
        self._append("已把当前位置记为起始位： %s" % self._pose_str(), "sys")

    def _home_wait(self):
        try:
            return max(0, int(float(self.home_ms.get())))
        except Exception:
            return 600

    def _send_pose(self, pose):
        """把四路送到指定位置（发指令 + 同步界面）"""
        for i, d in enumerate(pose):
            if self.link and self.link.alive:
                cmd = fmt_cmd(i, d, self.scale360)
                if self.link.send(cmd):
                    self.last_sent = cmd
                    self.sent_label.config(text="最后发送： %s" % cmd)
            self.suppress = True
            self.scales[i].set(d)
            self.suppress = False
            self.val_labels[i].config(
                text="%d°  raw %d" % (d, d * 2 if self.scale360 else d))

    def _go_home(self, pose, then_play=False):
        self._send_pose(pose)
        self._append("回到起始位： %s" % self._pose_str(), "sys")
        if then_play:
            w = self._home_wait()
            self._append("等 %d ms 让舵机到位，然后开始回放" % w, "sys")
            self.rp_after = self.root.after(w, self._begin_timeline)

    def ed_go_home(self):
        if not self.track.start_pose:
            self._append("还没有起始位（先点「以当前位置为起始」）", "sys"); return
        if not (self.link and self.link.alive):
            self._append("未连接串口", "err"); return
        self._go_home(self.track.start_pose, then_play=False)

    def _begin_timeline(self):
        """起始位已到位，正式开始走时间轴"""
        if not self.playing:
            return
        self.rp_i = 0
        self.rp_wall0 = time.time()
        self._replay_next()

    def toggle_play(self):
        if self.playing:
            self._append("已停止回放", "sys")
            self.stop_play(); return
        if not len(self.track):
            self._append("没有可回放的记录", "sys"); return
        if not (self.link and self.link.alive):
            self._append("回放前请先连接串口", "err"); return
        if self.recording:
            self.toggle_record()          # 回放时停掉录制，免得互相干扰
        # 回放开始时对记录做一次快照：这样回放当中你去编辑时间轴也不会
        # 把正在播的东西改乱（编辑只影响下一次回放）。
        self._play_list = self.track.tuples()
        self.playing = True
        self.btn_play.config(text="■ 停止回放")
        for s in self.scales:
            s.config(state="disabled")    # 回放期间禁掉滑块，免得手动和回放打架
        self._append("开始回放 %d 条，速度 %s"
                     % (len(self._play_list), self.speed_var.get()), "sys")
        # 先回起始位再放，否则第一段动作会从"当前姿势"开始，每次都不同
        if self.track.start_pose and self.home_var.get():
            self._go_home(self.track.start_pose, then_play=True)
        else:
            if self.track.start_pose is None:
                self._append("⚠️ 没记录起始位，回放结果会依赖当前姿势"
                             "（可用「以当前位置为起始」补一个）", "err")
            self._begin_timeline()

    def stop_play(self):
        self.playing = False
        if self.rp_after is not None:
            try:
                self.root.after_cancel(self.rp_after)
            except Exception:
                pass
            self.rp_after = None
        self.btn_play.config(text="▶ 回放")
        for s in self.scales:
            s.config(state="normal")
        self.play_label.config(text="")

    def _speed(self):
        try:
            v = float(self.speed_var.get().rstrip("x"))
            return v if v > 0 else 1.0
        except Exception:
            return 1.0

    def _replay_next(self):
        if not self.playing:
            return
        if self.rp_i >= len(self._play_list):
            if self.loop_var.get():
                self.rp_i = 0
                self.rp_wall0 = time.time()
            else:
                self._append("回放结束", "sys")
                self.stop_play(); return
        t = self._play_list[self.rp_i][0]
        delay = t / self._speed() - (time.time() - self.rp_wall0)
        self.rp_after = self.root.after(max(0, int(delay * 1000)), self._replay_fire)

    def _replay_fire(self):
        if not self.playing:
            return
        t, servo, deg = self._play_list[self.rp_i]
        if self.link and self.link.alive:
            cmd = fmt_cmd(servo, deg, self.scale360)
            if self.link.send(cmd):
                self.last_sent = cmd
                self.sent_label.config(text=f"最后发送： {cmd}")
        # 界面跟着动，但不让它再往 pending 里塞一条（否则会重复发出去）
        self.suppress = True
        self.scales[servo].set(deg)
        self.suppress = False
        self.val_labels[servo].config(
            text=f"{deg}°  raw {deg*2 if self.scale360 else deg}")
        self.play_label.config(text="%d/%d  t=%.1fs" % (self.rp_i + 1, len(self._play_list), t))
        self.rp_i += 1
        self._replay_next()

    # ---------------- 回读（来自心跳）----------------
    def _build_readback(self):
        f = ttk.LabelFrame(self.root, text="FPGA 回读（心跳，每秒一次）", padding=(10, 4, 10, 6))
        f.pack(fill="x", padx=10, pady=6)
        row = ttk.Frame(f)
        row.pack(fill="x")
        self.hb_labels = []
        for i in range(N_SERVO):
            lab = ttk.Label(row, text=f"舵机{i+1} ----", font=("Menlo", 12))
            lab.pack(side="left", expand=True)
            self.hb_labels.append(lab)

        self.sent_label = ttk.Label(f, text="最后发送： —", foreground="#666")
        self.sent_label.pack(anchor="w", pady=(4, 0))

    # ---------------- 日志 ----------------
    def _build_log(self):
        f = ttk.LabelFrame(self.root, text="串口日志（FPGA → 电脑）", padding=(6, 4, 6, 6))
        f.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        self.log = tk.Text(f, height=12, wrap="none", font=("Menlo", 11),
                           background="#1e1e1e", foreground="#ddd", insertbackground="#ddd")
        sb = ttk.Scrollbar(f, command=self.log.yview)
        self.log.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.log.pack(side="left", fill="both", expand=True)

        self.log.tag_config("reply", foreground="#6ee06e")   # 回执
        self.log.tag_config("err", foreground="#ff6b6b")     # 错误
        self.log.tag_config("hb", foreground="#7aa2f7")      # 心跳
        self.log.tag_config("banner", foreground="#e0c060")  # 横幅/其它
        self.log.tag_config("sys", foreground="#999")        # 本程序提示

    def _append(self, text, tag):
        self.log.insert("end", text + "\n", tag)
        # 日志太长就砍掉前面的，避免越跑越慢
        if int(self.log.index("end-1c").split(".")[0]) > 500:
            self.log.delete("1.0", "100.0")
        self.log.see("end")

    # ---------------- 连接管理 ----------------
    def connect(self):
        if self.link:
            return
        port = self.port_var.get().strip()
        if not port:
            self._append("请先选一个串口。macOS 上可以跑： ls /dev/tty.usb* /dev/tty.wchusbserial*", "sys")
            return
        try:
            self.link = Link(port)
        except Exception as e:
            self._append(f"打不开 {port}：{e}", "err")
            self.status.config(text="● 打开失败", foreground="#b00")
            return
        self.status.config(text=f"● 已连接 {port}", foreground="#080")
        self._append(f"已连接 {port} @ {BAUD} 8N1", "sys")
        self._append("提示：如果迟迟没有横幅，按一下板上复位键", "sys")

    def disconnect(self):
        if self.link:
            self.link.close()
            self.link = None
        self.status.config(text="● 未连接", foreground="#b00")
        self._append("已断开", "sys")

    # ---------------- 发送节流（见文件头 ①）----------------
    def _sender(self):
        if self.link and self.link.alive:
            with self.lock:
                batch = dict(self.pending)
                self.pending.clear()
            for idx, deg in batch.items():
                cmd = fmt_cmd(idx, deg, self.scale360)
                if self.link.send(cmd):
                    self.last_sent = cmd
                    self.sent_label.config(text=f"最后发送： {cmd}")
                    # 录制点：记的就是真正写进串口的那一条，带相对时间戳
                    if self.recording:
                        self.track.add(time.time() - self.rec_t0, idx, deg)
                else:
                    self._append("串口写失败，已断开", "err")
                    self.disconnect()
                    break
        self.root.after(SEND_MS, self._sender)

    # ---------------- 收：刷新界面 ----------------
    def _poller(self):
        if self.link:
            try:
                while True:
                    line = self.link.rxq.get_nowait()
                    self._handle_line(line)
            except queue.Empty:
                pass
            if not self.link.alive:
                self._append("串口连接已断开", "err")
                self.disconnect()
        if self.recording:
            self._update_rec_label()
        if self.playing:
            self.redraw()          # 让时间轴上的播放头动起来
        self.root.after(POLL_MS, self._poller)

    def _handle_line(self, line):
        if not line:
            return
        kind = classify(line)
        if kind == "hb":
            vals = parse_hb(line)
            # 心跳报的是 FPGA 里的**原始值**，换算成度数显示
            for i, v in enumerate(vals):
                deg = v / 2 if self.scale360 else v
                self.hb_labels[i].config(text=f"舵机{i+1} {deg:g}°")
            self.hb_ok = True
            return                      # 心跳不写日志，否则一秒一行刷屏
        self._append(line, kind)

    def on_close(self):
        self.playing = False
        self.disconnect()
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser(description="Tang Nano 4K 舵机微调 GUI")
    ap.add_argument("-p", "--port", help="串口设备；不给就自动猜")
    ap.add_argument("--scale360", type=int, choices=[0, 1], default=0,
                    help="要和 FPGA 里 servo_cmd 的 SCALE_360 一致（默认 0）")
    args = ap.parse_args()

    root = tk.Tk()
    try:
        ttk.Style().theme_use("clam")
    except Exception:
        pass
    App(root, args.scale360, args.port)
    root.mainloop()


if __name__ == "__main__":
    main()

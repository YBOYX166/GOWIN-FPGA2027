#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =====================================================================
# car_gui —— GW5AT-138 麦轮小车 键盘遥控上位机
#
# 用法：
#   /usr/bin/python3 car_gui.py
#   /usr/bin/python3 car_gui.py --port /dev/cu.usbserial-XXXX
#
# ---------------------------------------------------------------------
# 按键映射（同时按自动合成方向）
#
#        ↑ / W            前进          vx = +V
#        ↓ / S            后退          vx = -V
#        ← / A            左横移        vy = +V      （麦轮独有）
#        → / D            右横移        vy = -V
#        Q                原地左转      w  = +V
#        E                原地右转      w  = -V
#        空格             急停（全零，并且松开所有键的锁定）
#
#   方向键是**平移**不是转向 —— 所以 ↑+← 就是"左前方"，和你说的一致。
#   转向用 Q/E，可以和方向键叠加（比如 ↑+Q = 边前进边左转 = 画弧）。
#
#   ⚠️ 按键组合是**严格校验**的，不合法直接整车置零（不做任何抵消）：
#        0 个键          -> 停
#        1 个键          -> 该方向
#        2 个键，不同轴  -> 斜向 / 边走边转（合法）
#        2 个键，同一轴  -> 置零（↑+↓、←+→、↑+W、Q+E 都算）
#        3 个及以上      -> 置零
#      这样按错键只会停车，不会跑出一个谁也没预期的方向。
#
#   斜向时默认按 1/√2 降速（可关）：不然 ↑+← 的合速度是 V√2，会比直行快 41%。
#
# ---------------------------------------------------------------------
# 串口协议（和 FPGA 侧 src/cmd_rx.v 必须一致）
#
#   PC -> FPGA   '@' <svx><ddd> <svy><ddd> <sw><ddd> '\n'      共 14 字节
#                <s> = '+' 或 '-'，<ddd> = 000..999
#                例 "@+600+000+000\n" 前进 600
#
#   FPGA -> PC   每 200ms 一行（来自 enc_report.v）
#                S0=+02200 S1=+02195 S2=-00004 S3=+02210\r\n
#                单位 = 计数/秒（counts/s，带符号）
#                RPM = cps * 60 / CPR    例 CPR=1320 -> cps 2200 = 100 RPM
#
# ---------------------------------------------------------------------
# 两个安全设计
#
#   ① 20Hz 持续发帧（哪怕按键没变）。FPGA 侧有 500ms 掉线保护，
#      所以本程序一崩/USB 一掉/线一松，车 0.5 秒内自己停。
#   ② 窗口失去焦点时（点了别的程序）自动清空所有按键并停车，
#      不然你切走之后车会一直往前冲。
# =====================================================================
import sys
import os
import re
import glob
import time
import queue
import argparse
import threading

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("需要 pyserial：/usr/bin/python3 -m pip install pyserial")

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError:
    sys.exit("需要 tkinter（用系统自带的 /usr/bin/python3）")


# ---------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------
SEND_HZ       = 20        # 发帧频率
ENC_TIMEOUT   = 1.0       # 超过这么久没收到编码器行就认为链路异常
KEY_STALE     = 0.30      # 按键超过这么久没再收到 KeyPress 就当作已松开
                          # （对付 macOS 长按自动重复：重复的 KeyPress 会刷新它）
V_MIN, V_MAX  = 0, 999    # 协议限幅（cmd_rx 的 VMAX 默认 1000，留一点余量）
CPR_DEFAULT   = 1320      # 编码器线数，用来算 RPM（只影响显示）

# ⚠️ 这三个必须和 FPGA 侧 src/mecanum_mixer.v 的例化参数一致（top.v 里）
#    K_GAIN/2^GAIN_SHIFT = 1024/1024 = 1.0  ->  kw 就等于 w
MIX_K_GAIN    = 1024
MIX_GAIN_SHIFT = 10
MIX_OUT_MAX   = 1000

# 键 -> (轴, 符号)
#   轴 X = 前后(vx)  轴 Y = 左右(vy)  轴 W = 自转(w)
AXIS = {
    "Up":    ("X", +1),   "w": ("X", +1),
    "Down":  ("X", -1),   "s": ("X", -1),
    "Left":  ("Y", +1),   "a": ("Y", +1),
    "Right": ("Y", -1),   "d": ("Y", -1),
    "q":     ("W", +1),
    "e":     ("W", -1),
}
VALID_KEYS = frozenset(AXIS)


# =====================================================================
# 纯函数：协议编解码（不依赖 tkinter/串口，可单独测）
# =====================================================================
def fmt_part(v):
    """一个分量 -> '+ddd' / '-ddd'（4 字符），自动限幅到 ±999"""
    v = max(-999, min(999, int(round(v))))
    return ("+" if v >= 0 else "-") + "%03d" % abs(v)


def fmt_frame(vx, vy, w):
    """(vx,vy,w) -> 14 字节的指令帧"""
    return "@%s%s%s\n" % (fmt_part(vx), fmt_part(vy), fmt_part(w))


ENC_RE = re.compile(r"S0=([+-]?\d+)\s+S1=([+-]?\d+)\s+S2=([+-]?\d+)\s+S3=([+-]?\d+)")


def parse_enc(line):
    """解析编码器上报行 -> [轮0,轮1,轮2,轮3]（计数/秒），不是这格式就返回 None"""
    m = ENC_RE.search(line or "")
    if not m:
        return None
    return [int(m.group(i)) for i in (1, 2, 3, 4)]


def cps_to_rpm(cps, cpr=CPR_DEFAULT):
    """计数/秒 -> RPM"""
    return cps * 60.0 / cpr


def keys_check(held):
    """检查按键组合合不合法 —— 不合法就整车置零，**不做任何抵消**。

    返回 (ok, 原因)。规则：

      0 个键            -> 合法（就是停车）
      1 个键             -> 合法
      2 个键，不同轴     -> 合法（斜向 / 边走边转）
      2 个键，同一轴     -> **不合法**（非相邻：如 ↑+↓、←+→、↑+W、Q+E）
      3 个及以上         -> **不合法**

    "同一轴"的判定：X=前后 Y=左右 W=自转。所以 ↑+↓ 是同轴（冲突），
    而 ↑+← 是不同轴（合法的左前斜向）。
    注意 ↑+W 也算同轴不合法 —— 它俩指的是同一个方向，不是"斜向"。
    """
    keys = [k for k in held if k in AXIS]
    n = len(keys)
    if n >= 3:
        return False, "按了 %d 个键 -> 停车" % n
    if n == 2:
        axes = {AXIS[k][0] for k in keys}
        if len(axes) != 2:
            return False, "同轴按键冲突（%s）-> 停车" % "+".join(sorted(keys))
    return True, ""


def keys_to_cmd(held, speed, normalize=True, rot_scale=1.0):
    """把"当前按住的键"换算成 (vx, vy, w)

    held : set，元素是 AXIS 的键名
    speed: 单轴基准幅值
    normalize: 斜向时按 1/sqrt(2) 降速，保持合速度不变

    组合不合法时（见 keys_check）**直接返回全零**，不做部分抵消 ——
    否则按错键会得到一个谁也没预期的运动方向。
    """
    if not keys_check(held)[0]:
        return 0, 0, 0
    vx = vy = w = 0
    for k in held:
        ax, sg = AXIS.get(k, (None, 0))
        if   ax == "X": vx += sg
        elif ax == "Y": vy += sg
        elif ax == "W": w  += sg
    scale = float(speed)
    if normalize and vx and vy:
        scale *= 0.7071067811865476      # 1/√2
    return (int(round(vx * scale)),
            int(round(vy * scale)),
            int(round(w * scale * rot_scale)))


def mix_wheels(vx, vy, w):
    """按 FPGA 的公式算出四轮速度（顺序 FL, FR, RL, RR）"""
    kw = (MIX_K_GAIN * w) >> MIX_GAIN_SHIFT        # Python 的 >> 对负数是算术右移，和 Verilog >>> 一致
    return (vx - vy - kw, vx + vy + kw, vx + vy - kw, vx - vy + kw)


def prescale(vx, vy, w):
    """预缩放：如果四轮里有人会超出限幅，就**整体等比缩小**，方向不变。

    为什么要在上位机做这件事：
      FPGA 的 clamp 是**逐轮**夹的（mecanum_mixer 的 clamp 函数）。一旦有一轮
      先到顶，四轮的比例就破坏了 —— 本来"原地左转"会变成"边走边转"。
      这里整体乘同一个系数，运动方向完全不变，只是整体慢一点，
      这才是"限速"应该有的行为。
    """
    wheels = mix_wheels(vx, vy, w)
    peak = max(abs(x) for x in wheels)
    if peak <= MIX_OUT_MAX:
        return vx, vy, w
    k = MIX_OUT_MAX / float(peak)
    return int(vx * k), int(vy * k), int(w * k)


def describe_dir(vx, vy, w):
    """给界面显示当前方向的中文名"""
    if w and not vx and not vy:
        return "原地左转" if w > 0 else "原地右转"
    parts = []
    if vx > 0:   parts.append("前")
    elif vx < 0: parts.append("后")
    if vy > 0:   parts.append("左")
    elif vy < 0: parts.append("右")
    if not vx and vy:
        name = "左横移" if vy > 0 else "右横移"   # 纯横移单独说，避免只显示"左"
    else:
        name = "".join(parts)
    if name in ("前左", "左前"): name = "左前"
    elif name in ("前右", "右前"): name = "右前"
    elif name in ("后左", "左后"): name = "左后"
    elif name in ("后右", "右后"): name = "右后"
    if w > 0:   name += " + 左转"
    elif w < 0: name += " + 右转"
    return name or "停"


# =====================================================================
# 串口（后台线程读，主线程发）
# =====================================================================
class Link:
    def __init__(self, port, baud=115200):
        self.port = port
        self.ser = serial.Serial(port, baud, timeout=0.1, write_timeout=0.2)
        self.rxq = queue.Queue()
        self.alive = True
        self.t = threading.Thread(target=self._reader, daemon=True)
        self.t.start()

    def _reader(self):
        buf = b""
        while self.alive:
            try:
                chunk = self.ser.read(256)
            except Exception:
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.rxq.put(line.decode("ascii", "replace").strip())
            if len(buf) > 4096:
                buf = b""
        self.alive = False

    def send(self, s):
        try:
            self.ser.write(s.encode("ascii"))
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


# =====================================================================
# 界面
# =====================================================================
class App:
    def __init__(self, root, port=None, speed=600):
        self.root = root
        self.link = None
        self.held = {}             # 键名 -> 最后 KeyPress 的时刻
        self.last_enc = None
        self.last_enc_t = 0.0
        self.last_sent = ""
        self.speed = tk.IntVar(value=speed)
        self.normalize = tk.BooleanVar(value=True)
        self.show_rpm = tk.BooleanVar(value=False)

        root.title("GW5AT-138 麦轮小车 —— 键盘遥控")
        root.minsize(660, 620)
        root.configure(bg="#1b1b1f")

        self._build_top(port)
        self._build_control()
        self._build_encoder()
        self._build_log()
        self._bind_keys()

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.bind("<FocusOut>", self._on_focus_out)
        root.after(int(1000 / SEND_HZ), self._tick)
        if port:
            root.after(150, self.connect)
        root.after(300, root.focus_force)

    # ---------------- 串口栏 ----------------
    def _build_top(self, port):
        f = ttk.Frame(self.root, padding=(10, 8, 10, 4))
        f.pack(fill="x")
        ttk.Label(f, text="串口").pack(side="left")
        self.port_var = tk.StringVar(value=port or "")
        self.port_box = ttk.Combobox(f, textvariable=self.port_var, width=30,
                                     state="readonly", takefocus=False)
        self.port_box.pack(side="left", padx=(4, 4))
        self.port_box.bind("<<ComboboxSelected>>", lambda e: self.root.focus_force())
        self.port_box.bind("<FocusOut>", lambda e: self.root.focus_force())
        ttk.Button(f, text="刷新", width=6, takefocus=False,
                   command=self.refresh_ports).pack(side="left")
        self.btn_conn = ttk.Button(f, text="连接", width=8, takefocus=False,
                                   command=self.toggle_conn)
        self.btn_conn.pack(side="left", padx=4)
        self.conn_label = ttk.Label(f, text="● 未连接", foreground="#c66")
        self.conn_label.pack(side="right")
        self.focus_hint = ttk.Label(f, text="", foreground="#888")
        self.focus_hint.pack(side="right", padx=(0, 10))
        self.refresh_ports()

    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        ports += [p for p in sorted(glob.glob("/dev/cu.usbserial*"))
                  if p not in ports]
        ports += [p for p in sorted(glob.glob("/dev/cu.usbmodem*"))
                  if p not in ports]
        self.port_box["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def toggle_conn(self):
        if self.link and self.link.alive:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        p = self.port_var.get()
        if not p:
            self._append("先选一个串口", "err"); return
        try:
            self.link = Link(p)
        except Exception as e:
            self._append("打开 %s 失败：%s" % (p, e), "err"); return
        self.btn_conn.config(text="断开")
        self.conn_label.config(text="● 已连接 " + os.path.basename(p), foreground="#3a3")
        self._append("已连接 %s @115200" % p, "sys")

    def disconnect(self):
        if self.link:
            self.link.close()
            self.link = None
        self.btn_conn.config(text="连接")
        self.conn_label.config(text="● 未连接", foreground="#c66")

    # ---------------- 控制区 ----------------
    def _build_control(self):
        f = ttk.LabelFrame(self.root, text="控制", padding=(10, 6, 10, 8))
        f.pack(fill="x", padx=10, pady=(6, 0))

        # 速度
        r = ttk.Frame(f); r.pack(fill="x")
        ttk.Label(r, text="速度", width=5).pack(side="left")
        self.scale = tk.Scale(r, from_=V_MIN, to=V_MAX, orient="horizontal",
                              resolution=10, showvalue=False, length=330,
                              takefocus=False, command=self._on_speed)
        self.scale.set(self.speed.get())
        self.scale.pack(side="left")
        self.speed_label = ttk.Label(r, text=str(self.speed.get()), width=5)
        self.speed_label.pack(side="left", padx=6)
        ttk.Checkbutton(r, text="斜向降速", variable=self.normalize,
                        takefocus=False).pack(side="left", padx=(8, 0))

        # 转向力度：w 相对 vx/vy 的倍数。1.0 = 转 600 和走 600 一样快
        r1b = ttk.Frame(f); r1b.pack(fill="x", pady=(2, 0))
        ttk.Label(r1b, text="转向力度", width=5).pack(side="left")
        self.rot_scale = tk.DoubleVar(value=1.0)
        rs = tk.Scale(r1b, from_=0.1, to=1.5, orient="horizontal", resolution=0.05,
                      showvalue=False, length=330, takefocus=False,
                      command=self._on_rot)
        rs.set(1.0)
        rs.pack(side="left")
        self.rot_label = ttk.Label(r1b, text="100%", width=5)
        self.rot_label.pack(side="left", padx=6)
        ttk.Label(r1b, text="（调小 -> 转向比直行慢）",
                  foreground="#888").pack(side="left")

        # 方向盘 + 状态
        r2 = ttk.Frame(f); r2.pack(fill="x", pady=(8, 0))
        self.pad = tk.Canvas(r2, width=150, height=150, bg="#111116",
                             highlightthickness=0)
        self.pad.pack(side="left")
        self.pad.bind("<Button-1>", lambda e: self.root.focus_force())

        info = ttk.Frame(r2); info.pack(side="left", fill="both", expand=True, padx=(12, 0))
        self.dir_label = ttk.Label(info, text="停", font=("Menlo", 20))
        self.dir_label.pack(anchor="w", pady=(6, 8))
        self.cmd_label = ttk.Label(info, text="vx=+000  vy=+000  w=+000",
                                   font=("Menlo", 11))
        self.cmd_label.pack(anchor="w")
        self.frame_label = ttk.Label(info, text="帧: @+000+000+000",
                                     font=("Menlo", 10), foreground="#888")
        self.frame_label.pack(anchor="w", pady=(4, 0))
        self.key_label = ttk.Label(info, text="按住: —", foreground="#888")
        self.key_label.pack(anchor="w", pady=(8, 0))
        self.link_label = ttk.Label(info, text="FPGA 回包: 无", foreground="#888")
        self.link_label.pack(anchor="w")
        self.wheel_label = ttk.Label(info, text="轮速 FL/FR/RL/RR: 0 0 0 0",
                                     font=("Menlo", 10), foreground="#7aa2f7")
        self.wheel_label.pack(anchor="w", pady=(4, 0))

        # 调试：手动发一帧
        r3 = ttk.Frame(f); r3.pack(fill="x", pady=(8, 0))
        ttk.Label(r3, text="手动发帧").pack(side="left")
        self.manual = tk.StringVar(value="@+000+000+000\n")
        ttk.Entry(r3, textvariable=self.manual, width=26,
                  font=("Menlo", 11)).pack(side="left", padx=4)
        ttk.Button(r3, text="发送", width=6, takefocus=False,
                   command=self._send_manual).pack(side="left")
        ttk.Label(r3, text="（可用串口助手手打同样的帧调试）",
                  foreground="#888").pack(side="left", padx=6)

    def _on_speed(self, v):
        self.speed.set(int(float(v)))
        self.speed_label.config(text=str(self.speed.get()))

    def _on_rot(self, v):
        self.rot_scale.set(round(float(v), 2))
        self.rot_label.config(text="%d%%" % round(self.rot_scale.get() * 100))

    # ---------------- 编码器 ----------------
    def _build_encoder(self):
        f = ttk.LabelFrame(self.root, text="编码器（每 200ms 一行，计数/秒）",
                           padding=(10, 4, 10, 8))
        f.pack(fill="x", padx=10, pady=(6, 0))
        r = ttk.Frame(f); r.pack(fill="x")
        self.enc_labels = []
        names = ["轮0 左前", "轮1 右前", "轮2 左后", "轮3 右后"]
        for i, nm in enumerate(names):
            col = ttk.Frame(r); col.pack(side="left", expand=True, fill="x")
            ttk.Label(col, text=nm, foreground="#888").pack()
            lab = ttk.Label(col, text="----", font=("Menlo", 15))
            lab.pack()
            self.enc_labels.append(lab)
        r2 = ttk.Frame(f); r2.pack(fill="x", pady=(6, 0))
        ttk.Checkbutton(r2, text="同时显示 RPM（按 CPR=%d 换算）" % CPR_DEFAULT,
                        variable=self.show_rpm, takefocus=False).pack(side="left")
        self.show_rpm.trace_add("write", lambda *a: self._render_enc())
        self.enc_time = ttk.Label(r2, text="", foreground="#888")
        self.enc_time.pack(side="right")

    # ---------------- 日志 ----------------
    def _build_log(self):
        f = ttk.LabelFrame(self.root, text="日志", padding=(8, 4, 8, 6))
        f.pack(fill="both", expand=True, padx=10, pady=(6, 10))
        self.log = tk.Text(f, height=7, wrap="none", bg="#111116", fg="#ccc",
                           font=("Menlo", 10), insertbackground="#ccc")
        self.log.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(f, command=self.log.yview)
        sb.pack(side="right", fill="y")
        self.log.config(yscrollcommand=sb.set)
        for tag, col in (("sys", "#7aa2f7"), ("err", "#ff5f56"),
                         ("enc", "#63e06a"), ("send", "#c9a227")):
            self.log.tag_config(tag, foreground=col)
        self.log.config(state="disabled")

    def _append(self, text, tag="sys"):
        self.log.config(state="normal")
        self.log.insert("end", "%s  %s\n" % (time.strftime("%H:%M:%S"), text), tag)
        if int(self.log.index("end-1c").split(".")[0]) > 500:
            self.log.delete("1.0", "100.0")
        self.log.see("end")
        self.log.config(state="disabled")

    # ---------------- 键盘 ----------------
    # ⚠️ 踩过的坑：Tk 的事件是先给**获得焦点的那个控件**处理，再冒泡。
    #    所以只在 root 上绑是不够的 —— 只要焦点在某个控件上，该控件的
    #    **类绑定**会先跑：
    #      · Scale      的方向键 = 改自己的值
    #      · Combobox   的方向键 = 切换下拉选项  <-- 就是"选串口后方向键被吃掉"
    #      · Button     的空格     = 按下按钮
    #    唯一可靠的做法：**递归给整棵控件树都绑上**，并且返回 "break"
    #    掐断默认行为。
    #    注意 ttk.Combobox 是 ttk.Entry 的子类，所以判断"文本控件"时必须
    #    先把 Combobox 排除掉，否则它会被一起跳过（这个也踩过）。
    def _bind_keys(self):
        seqs = ["<Up>", "<Down>", "<Left>", "<Right>", "<space>"]
        for c in "wasdqeWASDQE":
            seqs.append("<KeyPress-%s>" % c)
        self._bind_recursive(self.root, seqs)
        self.root.bind("<KeyRelease>", self._on_key_up)
        self.root.bind("<Escape>", self._on_escape)

    def _bind_recursive(self, w, seqs):
        # 真正的文本输入控件（Entry/Text）要留着自己的方向键来移动光标，
        # 但 Combobox 不算 —— 它的方向键是切选项，必须掐掉。
        is_text = isinstance(w, (tk.Entry, tk.Text)) and not isinstance(w, ttk.Combobox)
        if is_text:
            w.bind("<Escape>", self._on_escape)
            w.bind("<Return>", self._on_escape)
        else:
            for sq in seqs:
                w.bind(sq, self._on_key)
            w.bind("<Escape>", self._on_escape)
        for c in w.winfo_children():
            self._bind_recursive(c, seqs)

    def _on_escape(self, ev=None):
        """任何时候按 Esc：把焦点交还给主窗口，恢复键盘遥控"""
        try:
            self.root.focus_force()
        except Exception:
            pass
        self._update_focus_hint()
        return "break"

    def _update_focus_hint(self):
        """如果焦点落在会抢键盘的控件上，明确提示用户按 Esc"""
        try:
            f = self.root.focus_get()
        except Exception:
            f = None
        blocked = isinstance(f, (tk.Entry, tk.Text)) and not isinstance(f, ttk.Combobox)
        if blocked:
            self.focus_hint.config(
                text="⚠️ 键盘被输入框占用，方向键无效 —— 按 Esc 交还控制权",
                foreground="#ff5f56")
        else:
            self.focus_hint.config(text="", foreground="#888")

    def _key_name(self, ev):
        k = ev.keysym
        if k == "space":
            return "space"
        if len(k) == 1:
            return k.lower()
        return k

    def _on_key(self, ev):
        k = self._key_name(ev)
        if k == "space":
            self.held.clear()
            self._append("急停（清空所有按键）", "err")
        elif k in VALID_KEYS:
            self.held[k] = time.time()
        else:
            return None
        return "break"          # 阻止 Tkinter 默认处理（方向键改滑块/移动焦点）

    def _on_key_up(self, ev):
        self.held.pop(self._key_name(ev), None)
        return None

    def _on_focus_out(self, ev):
        if self.held:
            self.held.clear()
            self._append("窗口失去焦点 -> 已停车（防止切走后车一直跑）", "err")

    def _purge_stale(self):
        now = time.time()
        for k in list(self.held):
            if now - self.held[k] > KEY_STALE:
                del self.held[k]

    # ---------------- 主循环 ----------------
    def _tick(self):
        self._purge_stale()
        self._update_focus_hint()
        vx, vy, w = keys_to_cmd(self.held, self.speed.get(),
                                self.normalize.get(), self.rot_scale.get())
        # 预缩放：保证四轮都不超限，这样 FPGA 的逐轮 clamp 永远不触发，
        # 运动方向不会被夹变形。显示用的是缩放后的值，和真正发出去的一致。
        vx, vy, w = prescale(vx, vy, w)
        frame = fmt_frame(vx, vy, w)

        if self.link and self.link.alive:
            if self.link.send(frame):
                if frame != self.last_sent:
                    self._append("发 " + frame.rstrip("\n"), "send")
                self.last_sent = frame
            else:
                self._append("串口写失败，已断开", "err")
                self.disconnect()

        self._update_pad(vx, vy)
        ok, why = keys_check(self.held)
        if not ok:
            # 按键组合不合法 -> 已置零。把原因显出来，不然按错了会莫名其妙
            self.dir_label.config(text="⛔ 停车", foreground="#ff5f56")
            self.key_label.config(text="按住: " + " ".join(sorted(self.held))
                                       + "   " + why, foreground="#ff5f56")
        else:
            self.dir_label.config(text=describe_dir(vx, vy, w), foreground="#000")
            self.key_label.config(
                text="按住: " + (" ".join(sorted(self.held)) if self.held else "—"),
                foreground="#888")
        self.cmd_label.config(text="vx=%+04d  vy=%+04d  w=%+04d" % (vx, vy, w))
        self.frame_label.config(text="帧: " + frame.rstrip("\n"))
        wl = mix_wheels(vx, vy, w)
        self.wheel_label.config(text="轮速 FL/FR/RL/RR: " + " ".join("%+d" % x for x in wl))

        # 收编码器
        if self.link:
            try:
                while True:
                    line = self.link.rxq.get_nowait()
                    if line:
                        self._handle_line(line)
            except queue.Empty:
                pass
            if not self.link.alive:
                self._append("串口连接已断开", "err")
                self.disconnect()

        # 回包超时提示
        if self.last_enc_t and time.time() - self.last_enc_t > ENC_TIMEOUT:
            self.link_label.config(text="FPGA 回包: 超时（%0.1fs 没收到）"
                                        % (time.time() - self.last_enc_t),
                                   foreground="#c66")
        self.root.after(int(1000 / SEND_HZ), self._tick)

    def _render_enc(self):
        """按当前 show_rpm 重画四路编码器（勾选框变化时也要调）"""
        if not self.last_enc:
            return
        for i, cps in enumerate(self.last_enc):
            if self.show_rpm.get():
                txt = "%+d  %.1f RPM" % (cps, cps_to_rpm(cps))
            else:
                txt = "%+d" % cps
            self.enc_labels[i].config(text=txt,
                                      foreground="#63e06a" if cps else "#888")

    def _handle_line(self, line):
        v = parse_enc(line)
        if v:
            self.last_enc = v
            self.last_enc_t = time.time()
            self._render_enc()
            self.link_label.config(text="FPGA 回包: 正常", foreground="#3a3")
            self.enc_time.config(text=time.strftime("%H:%M:%S"))
        else:
            self._append("FPGA: " + line, "enc")

    def _send_manual(self):
        if not (self.link and self.link.alive):
            self._append("未连接串口", "err"); return
        s = self.manual.get()
        if not s.endswith("\n"):
            s += "\n"
        if self.link.send(s):
            self._append("手动发 " + s.rstrip("\n"), "send")
        else:
            self._append("发送失败", "err")

    # ---------------- 方向盘绘制 ----------------
    def _update_pad(self, vx, vy):
        c = self.pad
        c.delete("all")
        W = H = 150
        cx, cy = W / 2, H / 2
        c.create_line(10, cy, W - 10, cy, fill="#333")
        c.create_line(cx, 10, cx, H - 10, fill="#333")
        c.create_text(cx, 16, text="前", fill="#666", font=("Menlo", 9))
        c.create_text(cx, H - 14, text="后", fill="#666", font=("Menlo", 9))
        c.create_text(16, cy, text="左", fill="#666", font=("Menlo", 9))
        c.create_text(W - 16, cy, text="右", fill="#666", font=("Menlo", 9))
        R = 52
        # vy 正 = 左 -> 屏幕 x 减小；按当前速度归一化，满速时到外圈
        base = max(1.0, float(self.speed.get() or 1))
        x = cx - (vy / base) * R
        y = cy - (vx / base) * R
        x = max(12, min(W - 12, x))
        y = max(12, min(H - 12, y))
        if vx or vy:
            c.create_line(cx, cy, x, y, fill="#7aa2f7", width=2)
            c.create_oval(x - 8, y - 8, x + 8, y + 8, fill="#7aa2f7", outline="")
        else:
            c.create_oval(cx - 6, cy - 6, cx + 6, cy + 6, fill="#444", outline="")

    # ---------------- 退出 ----------------
    def on_close(self):
        # 退出前务必发一帧全零，别让车带着最后的速度继续跑
        if self.link and self.link.alive:
            try:
                self.link.send(fmt_frame(0, 0, 0))
                time.sleep(0.05)
            except Exception:
                pass
        self.disconnect()
        self.root.destroy()


# =====================================================================
def main():
    ap = argparse.ArgumentParser(
        description="GW5AT-138 麦轮小车 键盘遥控上位机",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="按键：↑↓←→/WASD 平移    Q/E 转向    空格 急停")
    ap.add_argument("--port", help="串口设备，如 /dev/cu.usbserial-1120")
    ap.add_argument("--speed", type=int, default=600,
                    help="初始速度幅值，0-999（默认 600）")
    args = ap.parse_args()

    root = tk.Tk()
    App(root, port=args.port, speed=max(0, min(V_MAX, args.speed)))
    root.mainloop()


if __name__ == "__main__":
    main()

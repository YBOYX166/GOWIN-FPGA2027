#!/usr/bin/env python3
"""Encoder-only serial monitor. Run: /usr/bin/python3 tools/encoder_debug_gui.py"""
import argparse
import csv
import math
import queue
import re
import threading
import time
from collections import deque
from datetime import datetime
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import serial
from serial.tools import list_ports

FRAME = re.compile(rb'S0=([+-][0-9]{5}) S1=([+-][0-9]{5}) S2=([+-][0-9]{5}) S3=([+-][0-9]{5}) *')


def parse_frame(raw):
    """Validate the complete ASCII payload; never accept a partial number."""
    match = FRAME.fullmatch(raw)
    if not match:
        raise ValueError('格式错误：要求完整 S0～S3，各含符号和 5 位数字')
    values = tuple(int(v) for v in match.groups())
    if any(abs(v) > 32767 for v in values):
        raise ValueError('数值超出 FPGA 协议范围 ±32767')
    return values


class Framer:
    def __init__(self):
        self.buf = bytearray()
        self.discard = False

    def feed(self, chunk):
        out = []
        for byte in chunk:
            if byte == 10:
                if not self.discard:
                    out.append((bytes(self.buf).removesuffix(b'\r'), None))
                self.buf.clear()
                self.discard = False
            elif not self.discard:
                self.buf.append(byte)
                if len(self.buf) > 512:
                    out.append((bytes(self.buf), '单行超过 512 字节，丢弃至下一换行'))
                    self.buf.clear()
                    self.discard = True
        return out


def reader(port, events, stop):
    framer = Framer()
    try:
        while not stop.is_set():
            chunk = port.read(max(1, min(port.in_waiting, 4096)))
            for raw, error in framer.feed(chunk):
                events.put((time.time(), time.monotonic(), raw, error))
    except Exception as exc:
        if not stop.is_set():
            events.put((time.time(), time.monotonic(), None, str(exc)))


class App:
    COLORS = ('#176bca', '#d06a00', '#148353', '#9349bd')

    def __init__(self, root, initial_port=''):
        self.root = root
        root.title('编码器调试 · 原始串口 / 四路反馈')
        root.geometry('1120x840')
        root.minsize(900, 680)
        self.port = None
        self.stop = None
        self.worker = None
        self.events = queue.Queue()
        self.samples = deque(maxlen=5000)
        self.last = None
        self.last_t = None
        self.good = self.bad = 0
        self.record = self.writer = None
        self.cpr = tk.StringVar(value='1320')
        self.port_name = tk.StringVar(value=initial_port)
        self.status = tk.StringVar(value='未连接')
        self.stats = tk.StringVar(value='有效帧 0    坏帧 0')
        self.note = tk.StringVar(value='等待数据；本程序只接收，不发送电机控制指令。')
        self.record_label = tk.StringVar(value='未录制')
        self.visible = [tk.BooleanVar(value=True) for _ in range(4)]
        self.build()
        self.refresh_ports()
        self.cpr.trace_add('write', lambda *_: self.render_values())
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.after(50, self.poll)

    def build(self):
        bar = ttk.Frame(self.root, padding=10)
        bar.pack(fill='x')
        ttk.Label(bar, text='串口').pack(side='left')
        self.ports = ttk.Combobox(bar, textvariable=self.port_name, width=30)
        self.ports.pack(side='left', padx=6)
        ttk.Button(bar, text='刷新', command=self.refresh_ports).pack(side='left')
        self.connect_button = ttk.Button(bar, text='连接', command=self.toggle)
        self.connect_button.pack(side='left', padx=6)
        ttk.Label(bar, text='115200 · 8N1 · 只接收').pack(side='left', padx=8)
        ttk.Label(bar, textvariable=self.status).pack(side='right')
        settings = ttk.Frame(self.root, padding=(10, 0, 10, 8))
        settings.pack(fill='x')
        ttk.Label(settings, text='轮轴每转四倍频计数 CPR：').pack(side='left')
        ttk.Entry(settings, textvariable=self.cpr, width=9).pack(side='left')
        ttk.Label(settings, text='  RPM = cps × 60 / CPR；CPR 只影响 RPM 显示').pack(side='left')
        ttk.Button(settings, text='清空显示', command=self.clear).pack(side='right')
        panel = ttk.Frame(self.root, padding=(10, 0))
        panel.pack(fill='x')
        self.cps_labels, self.rpm_labels = [], []
        for i in range(4):
            box = ttk.LabelFrame(panel, text='S%d · 通道 %d' % (i, i), padding=10)
            box.pack(side='left', fill='both', expand=True, padx=3)
            label = ttk.Label(box, text='— cps', font=('Menlo', 20), foreground=self.COLORS[i])
            label.pack()
            rpm = ttk.Label(box, text='— RPM')
            rpm.pack()
            self.cps_labels.append(label)
            self.rpm_labels.append(rpm)
        ttk.Label(self.root, textvariable=self.note, padding=(12, 6)).pack(anchor='w')
        ttk.Label(self.root, textvariable=self.stats, padding=(12, 0)).pack(anchor='w')
        graphbar = ttk.Frame(self.root, padding=(12, 5))
        graphbar.pack(fill='x')
        ttk.Label(graphbar, text='最近 20 秒 · 计数/秒 · 自动纵轴').pack(side='left', padx=(0, 15))
        for i in range(4):
            ttk.Checkbutton(graphbar, text='S%d' % i, variable=self.visible[i], command=self.draw).pack(side='left')
        self.canvas = tk.Canvas(self.root, background='#fafbfc', height=225, highlightthickness=1, highlightbackground='#ccd2da')
        self.canvas.pack(fill='both', expand=True, padx=12)
        self.canvas.bind('<Configure>', lambda _: self.draw())
        rawbar = ttk.Frame(self.root, padding=(12, 8))
        rawbar.pack(fill='x')
        ttk.Label(rawbar, text='原始接收记录（坏帧显示原因及十六进制，不更新读数）').pack(side='left')
        ttk.Button(rawbar, text='录制 / 停止 CSV', command=self.toggle_record).pack(side='right')
        ttk.Label(rawbar, textvariable=self.record_label).pack(side='right', padx=10)
        logbox = ttk.Frame(self.root)
        logbox.pack(fill='both', expand=True, padx=12, pady=(0, 10))
        self.log = tk.Text(logbox, height=10, wrap='none', font=('Menlo', 11), state='disabled')
        scroll = ttk.Scrollbar(logbox, command=self.log.yview)
        self.log.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right', fill='y')
        self.log.pack(fill='both', expand=True)
        self.log.tag_configure('bad', foreground='#b3261e')

    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.ports['values'] = ports
        if not self.port_name.get() and ports:
            self.port_name.set(ports[0])

    def toggle(self):
        if self.port:
            self.disconnect()
            return
        try:
            port = serial.Serial(self.port_name.get(), 115200, timeout=0.1)
        except Exception as exc:
            messagebox.showerror('连接失败', str(exc))
            return
        self.clear()
        self.events = queue.Queue()
        self.stop = threading.Event()
        self.port = port
        self.worker = threading.Thread(target=reader, args=(port, self.events, self.stop), daemon=True)
        self.worker.start()
        self.connect_button.config(text='断开')
        self.status.set('已连接 · 等待有效帧')

    def disconnect(self):
        if self.stop:
            self.stop.set()
        if self.worker:
            self.worker.join(timeout=0.4)
        if self.port:
            self.port.close()
        self.port = None
        self.connect_button.config(text='连接')
        self.status.set('未连接')
        self.render_values()
        self.draw()

    def clear(self):
        self.samples.clear()
        self.last = self.last_t = None
        self.good = self.bad = 0
        self.log.config(state='normal')
        self.log.delete('1.0', 'end')
        self.log.config(state='disabled')
        self.render_values()
        self.draw()

    def append(self, text, bad=False):
        self.log.config(state='normal')
        self.log.insert('end', text + '\n', 'bad' if bad else '')
        if int(self.log.index('end-1c').split('.')[0]) > 600:
            self.log.delete('1.0', '101.0')
        self.log.see('end')
        self.log.config(state='disabled')

    def handle(self, wall, mono, raw, error):
        stamp = datetime.fromtimestamp(wall).isoformat(timespec='milliseconds')
        if raw is None:
            self.append(stamp + ' 串口错误：' + error, True)
            self.disconnect()
            return
        values = None
        if not error:
            try:
                values = parse_frame(raw)
            except ValueError as exc:
                error = str(exc)
        if error:
            self.bad += 1
            self.append('%s [坏帧] %r | %s | HEX %s' % (stamp, raw, error, raw.hex(' ')), True)
        else:
            self.good += 1
            self.last, self.last_t = values, mono
            self.samples.append((mono, values))
            self.append(stamp + ' [有效] ' + raw.decode('ascii'))
        if self.writer:
            try:
                self.writer.writerow([stamp, 'bad' if error else 'ok', * (values or ('',)*4), self.cpr.get(), error or '', repr(raw), raw.hex(' ')])
                self.record.flush()
            except OSError as exc:
                self.stop_record()
                messagebox.showerror('录制失败', str(exc))

    def render_values(self):
        age = time.monotonic() - self.last_t if self.last_t is not None else None
        fresh = self.port is not None and age is not None and age <= 1.0
        try:
            cpr = float(self.cpr.get())
            if not math.isfinite(cpr) or cpr <= 0:
                raise ValueError
        except ValueError:
            cpr = None
        for i in range(4):
            value = self.last[i] if self.last is not None else None
            self.cps_labels[i].config(text=('%+d cps' % value) if fresh else '— cps')
            self.rpm_labels[i].config(text=('%.2f RPM' % (value * 60 / cpr)) if fresh and cpr else ('CPR 必须为正数' if not cpr else '— RPM'))
        if age is None:
            self.note.set('等待有效报文；通道号对应 FPGA S0～S3，不预设实际轮位。')
        elif not fresh:
            self.note.set('数据已过期（%.1f 秒）；最后有效值：%s cps' % (age, self.last))
        else:
            self.note.set('收到反馈 %.2f 秒前；数字为 FPGA 原始 cps，非 PWM 指令。' % age)
        if self.port:
            self.status.set('已连接 · ' + ('接收正常' if fresh else '等待 / 超时'))
        self.stats.set('有效帧 %d    坏帧 %d' % (self.good, self.bad))

    def draw(self):
        c = self.canvas
        c.delete('all')
        w, h = c.winfo_width(), c.winfo_height()
        if w < 100 or h < 60:
            return
        now = time.monotonic()
        pts = [(t, v) for t, v in self.samples if now-20 <= t <= now]
        channels = [i for i in range(4) if self.visible[i].get()]
        limit = max([10] + [abs(v[i]) for _, v in pts for i in channels]) * 1.1
        left, right, top, bottom = 70, w-15, 15, h-30
        y = lambda val: (top+bottom)/2 - val / limit * (bottom-top)/2
        for val in (-limit, 0, limit):
            c.create_line(left, y(val), right, y(val), fill='#dce1e8')
            c.create_text(left-8, y(val), text='%.0f' % val, anchor='e', fill='#59636f')
        for sec in (-20, -15, -10, -5, 0):
            x = left + (sec+20)/20*(right-left)
            c.create_text(x, bottom+15, text='%ds' % sec, fill='#59636f')
        for i in channels:
            previous = None
            for t, values in pts:
                x = left + (t-now+20)/20*(right-left)
                yy = y(values[i])
                if previous and t-previous[0] <= 1.0:
                    c.create_line(previous[1], previous[2], x, yy, fill=self.COLORS[i], width=2)
                else:
                    c.create_oval(x-2, yy-2, x+2, yy+2, fill=self.COLORS[i], outline='')
                previous = (t, x, yy)

    def toggle_record(self):
        if self.record:
            self.stop_record()
            return
        path = filedialog.asksaveasfilename(defaultextension='.csv', initialfile='encoder_%s.csv' % datetime.now().strftime('%Y%m%d_%H%M%S'), filetypes=[('CSV', '*.csv')])
        if not path:
            return
        try:
            self.record = open(path, 'w', newline='', encoding='utf-8-sig')
            self.writer = csv.writer(self.record)
            self.writer.writerow(['timestamp', 'status', 'S0_cps', 'S1_cps', 'S2_cps', 'S3_cps', 'CPR', 'error', 'raw_bytes', 'raw_hex'])
            self.record.flush()
        except OSError as exc:
            self.stop_record()
            messagebox.showerror('录制失败', str(exc))
            return
        self.record_label.set('录制中')

    def stop_record(self):
        if self.record:
            self.record.close()
        self.record = self.writer = None
        self.record_label.set('未录制')

    def poll(self):
        for _ in range(500):
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            self.handle(*event)
        self.render_values()
        self.draw()
        self.root.after(100, self.poll)

    def close(self):
        self.disconnect()
        self.stop_record()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='', help='预选串口（不自动连接）')
    args = parser.parse_args()
    root = tk.Tk()
    App(root, args.port)
    root.mainloop()


if __name__ == '__main__':
    main()

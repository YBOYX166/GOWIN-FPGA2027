#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
验收仿真：对应 src/DVP_Capture.v + top.v 写口。

DVP_Capture 在 PCLK 上升沿打一拍 VSYNC/HREF/DATA，偶拍拼高字节、奇拍出像素。
前 10 帧被丢掉。top.v 只在 DataValid 期间写帧缓冲，写满 384000 后停写。
"""
import random
import sys
STEP = 509
IMG_W, IMG_H, IMG_PIXEL = 800, 480, 384000
LINE_PCLK = 1700
VBLANK_LINES = 10
FRAME_PCLK = (VBLANK_LINES + IMG_H) * LINE_PCLK

class Camera:
    def __init__(self):
        self.t = 0
        self.line = -VBLANK_LINES
        self.hb = 0
        self.pix = 0
        self.vsync = 1
        self.href = 0
        self.din = 0

    def advance(self):
        self.t += 10
        if self.t % 20 == 10:
            self.vsync = 0 if -3 <= self.line < 0 else 1
            self.href = 1 if (0 <= self.line < IMG_H and 100 <= self.hb < 100 + 1600) else 0
            if self.href:
                v = (self.line * IMG_W + self.pix) * STEP % 65536
                self.din = ((v >> 8) & 0xFF) if (self.hb - 100) % 2 == 0 else (v & 0xFF)
            else:
                self.din = 0
            if self.href and (self.hb - 100) % 2 == 1:
                self.pix += 1
            self.hb += 1
            if self.hb >= LINE_PCLK:
                self.hb = 0
                self.pix = 0
                self.line += 1
                if self.line >= IMG_H:
                    self.line = -VBLANK_LINES
        return self.vsync, self.href, self.din

class DVPCapture:
    """src/DVP_Capture.v"""
    def __init__(self):
        self.rst_n = 0
        self.r_vsync = 0
        self.r_href = 0
        self.r_data = 0
        self.hcount = 0
        self.r_pixel = 0
        self.r_valid = 0
        self.r_vs = 0
        self.frame_cnt = 0
        self.dump_frame = 0

    def data_valid(self):
        return 1 if (self.r_valid and self.dump_frame) else 0
    def data_pixel(self):
        return self.r_pixel
    def data_vs(self):
        return 1 if (self.r_vs and self.dump_frame) else 0

    def posedge(self, vsync, href, din):
        if not self.rst_n:
            self.r_vsync = 0
            self.r_href = 0
            self.r_data = 0
            self.hcount = 0
            self.r_pixel = 0
            self.r_valid = 0
            self.r_vs = 0
            self.frame_cnt = 0
            self.dump_frame = 0
            return
        old_vs, old_href, old_h = self.r_vsync, self.r_href, self.hcount
        old_pixel, old_cnt = self.r_pixel, self.frame_cnt
        self.r_vsync = vsync
        self.r_href = href
        self.r_data = din
        self.hcount = (old_h + 1) if old_href else 0
        if (old_h & 1) == 0:
            self.r_pixel = ((din << 8) & 0xFF00) | (old_pixel & 0x00FF)
        else:
            self.r_pixel = (old_pixel & 0xFF00) | (din & 0xFF)
        self.r_valid = 1 if ((old_h & 1) and old_href) else 0
        self.r_vs = 0 if old_vs else 1
        if {old_vs: vsync} and (old_vs, vsync) == (0, 1):
            self.frame_cnt = 10 if old_cnt >= 10 else old_cnt + 1
        self.dump_frame = 1 if self.frame_cnt >= 10 else 0

class Writer:
    def __init__(self):
        self.rst_n = 0
        self.cap_en = 1
        self.wr_pix = 0
        self.mem = {}
        self.full = False
        self.ready = False
    def posedge(self, valid, data, dvp_vs):
        if not self.rst_n:
            self.wr_pix = 0
            self.full = False
            self.ready = False
            return
        if not self.cap_en:
            self.wr_pix = 0
            self.full = False
            return
        if not dvp_vs:
            self.wr_pix = 0
            self.full = False
        elif valid and not self.full:
            self.mem[self.wr_pix] = data & 0xFFFF
            if self.wr_pix == IMG_PIXEL - 1:
                self.full = True
                self.ready = True
            else:
                self.wr_pix += 1

H_TOTAL, V_TOTAL, H_ACTIVE, V_ACTIVE = 1056, 525, 800, 480
class Lcd:
    def __init__(self):
        self.hcnt = 0; self.vcnt = 0
        self.de = False
        self.rd_addr = 0
        self.samples = []
    def posedge(self, mem):
        rd = mem.get(self.rd_addr, 0)
        if self.de:
            self.samples.append((self.hcnt, self.vcnt, rd))
        if self.hcnt == H_TOTAL - 1:
            self.hcnt = 0
            self.vcnt = 0 if self.vcnt == V_TOTAL - 1 else self.vcnt + 1
        else:
            self.hcnt += 1
        self.de = (self.hcnt < H_ACTIVE and self.vcnt < V_ACTIVE)
        self.rd_addr = (self.vcnt * 800 + self.hcnt) if self.de else 0

def v565(idx):
    return idx * STEP % 65536
def p565(v):
    return v & 0xFFFF

def run(release_at_pclk, label):
    cam = Camera()
    cap = DVPCapture()
    wr = Writer()
    lcd = Lcd()
    t = 0; pclk = 0; lcdph = 0; released = False
    stop_t = release_at_pclk * 20 + 12 * FRAME_PCLK * 20 + int(1.3 * H_TOTAL * V_TOTAL * 30)
    while t < stop_t:
        if t % 10 == 0:
            vs, hr, dn = cam.advance()
            if t % 20 == 0:
                pclk += 1
                if not released and pclk >= release_at_pclk:
                    cap.rst_n = 1; wr.rst_n = 1; released = True
                v, d, dvs = cap.data_valid(), cap.data_pixel(), cap.data_vs()
                cap.posedge(vs, hr, dn)
                wr.posedge(v, d, dvs)
        if t % 15 == 0:
            lcdph ^= 1
            if lcdph:
                lcd.posedge(wr.mem)
        t += 5
    if not wr.ready:
        print(f"{label}: 未写满一帧")
        return False
    s = lcd.samples[-IMG_PIXEL:]
    if len(s) != IMG_PIXEL:
        print(f"{label}: LCD 采样不足 {len(s)}")
        return False
    bad = sum(1 for (x, y, g) in s if g != p565(v565(y * 800 + x)))
    status = "OK" if bad == 0 else f"FAIL {bad}"
    line, pix = release_at_pclk // LINE_PCLK - VBLANK_LINES, (release_at_pclk % LINE_PCLK - 100) // 2
    print(f"{label}: line={line:>4} col={pix:>3} | {IMG_PIXEL - bad}/{IMG_PIXEL} {status}")
    return bad == 0

if __name__ == "__main__":
    writer = Writer()
    writer.rst_n = 1
    for _ in range(IMG_PIXEL - 1):
        writer.posedge(True, 0, 1)
    assert not writer.ready
    writer.posedge(False, 0, 0)
    assert writer.wr_pix == 0
    for _ in range(IMG_PIXEL):
        writer.posedge(True, 0xffff, 1)
    assert writer.ready and writer.full
    writer.posedge(True, 0, 1)
    assert writer.mem[IMG_PIXEL - 1] == 0xffff
    writer.rst_n = 0
    writer.posedge(False, 0, 1)
    assert not writer.ready
    random.seed(2027)
    print("== DVP_Capture + framebuffer ==")
    ok = True
    ok &= run(100, "1 V-blank")
    ok &= run(9 * LINE_PCLK + 500, "2 V-blank end")
    base = (VBLANK_LINES + 123) * LINE_PCLK + 100
    ok &= run(base + 2 * 400, "3 active even")
    ok &= run(base + 2 * 400 + 1, "4 active odd")
    for i in range(6):
        ok &= run(random.randrange(FRAME_PCLK), f"5 rand {i+1}")
    print("result:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)

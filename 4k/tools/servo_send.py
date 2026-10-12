#!/usr/bin/env python3
# =====================================================================
# servo_send.py —— Tang Nano 4K 舵机微调上位机
#
# FPGA 侧协议（见 ../fpga_project/src/servo_cmd.v）：
#   发：4 个 ASCII 数字  <舵机号 1-4><角度百位><十位><个位>
#       带不带回车都行 —— FPGA 收满 4 位就执行，非数字字符会清空计数。
#
#   FPGA 会主动发三种行：
#     === Tang Nano 4K Servo Ready ===   <- 开机横幅（复位后约 0.5 秒）
#     Send 4 digits, e.g. 2090
#     A 090 090 090 090                  <- 心跳，每秒一行，四路当前角度
#     S2=090 A045                        <- 动作回复（号=原值 A实际角度）
#     ERR                                <- 编号不是 1-4，或角度超范围
#
# 用法：
#   python3 servo_send.py -m              # 监听模式：只看 FPGA 发什么（排查第一步）
#   python3 servo_send.py                 # 交互模式
#   python3 servo_send.py 2090            # 发一条就退出
#   python3 servo_send.py 1090 2180 3270  # 连发多条
#   python3 servo_send.py -p /dev/tty.usbserial-XXXX 2090
#
# 依赖：pip3 install pyserial
# =====================================================================
import sys
import re
import time
import glob
import argparse

try:
    import serial
except ImportError:
    sys.exit("缺少 pyserial，请先安装：  pip3 install pyserial")

# 只有这两种才算是「对指令的回复」，横幅和心跳都不算
RE_REPLY = re.compile(r"^S[1-4]=\d{3} A\d{3}$")
RE_ERR = re.compile(r"^ERR$")
RE_HB = re.compile(r"^A( \d{3}){4}$")


def find_port():
    pats = ["/dev/tty.usbserial*", "/dev/tty.usbmodem*", "/dev/tty.SLAB*",
            "/dev/tty.wchusbserial*", "/dev/ttyACM*", "/dev/ttyUSB*"]
    found = []
    for p in pats:
        found += sorted(glob.glob(p))
    return found[0] if found else None


def open_port(port, baud):
    try:
        return serial.Serial(port, baud, timeout=0.2)
    except serial.SerialException as e:
        sys.exit(f"打不开 {port}: {e}\n"
                 f"  macOS 上先看一眼有哪些口：  ls /dev/tty.usb* /dev/tty.wchusbserial*")


def read_reply(ser, timeout=2.0):
    """读到一条回复为止，途中的心跳/横幅打印出来但不当作回复。"""
    t0 = time.time()
    seen = []
    while time.time() - t0 < timeout:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("ascii", errors="replace").strip()
        if not line:
            continue
        if RE_REPLY.match(line) or RE_ERR.match(line):
            return line, seen
        seen.append(line)
    return None, seen


def send_one(ser, cmd, quiet=False):
    ser.reset_input_buffer()
    ser.write(cmd.encode("ascii"))
    ser.flush()
    line, seen = read_reply(ser)
    if not quiet:
        for s in seen:
            print(f"  (顺带收到) {s}")
        if line is None:
            print(f"  {cmd}  ->  ⚠️  2 秒内没收到回复")
        else:
            mark = "❌" if line.startswith("ERR") else "✅"
            print(f"  {cmd}  ->  {mark} {line}")
    return line


def monitor(ser):
    print("监听模式：只读不写。Ctrl-C 退出。")
    print("  应该先看到开机横幅（如果没有，按一下板上复位键）")
    print("  然后每秒一行心跳 A 090 090 090 090\n")
    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").rstrip("\r\n")
            if not line:
                continue
            t = time.strftime("%H:%M:%S")
            if RE_HB.match(line):
                print(f"[{t}] 心跳  {line}")
            elif RE_REPLY.match(line):
                print(f"[{t}] 回复  {line}")
            elif RE_ERR.match(line):
                print(f"[{t}] 错误  {line}")
            else:
                print(f"[{t}] 其它  {line}")
    except KeyboardInterrupt:
        print()


def main():
    ap = argparse.ArgumentParser(
        description="Tang Nano 4K 舵机微调上位机",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="指令格式：<舵机号 1-4><角度百位><十位><个位>   例 2090 = 2 号转到 90")
    ap.add_argument("cmds", nargs="*", help="要发送的 4 位数字，可给多个")
    ap.add_argument("-p", "--port", help="串口设备，不给就自动猜")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("-m", "--monitor", action="store_true",
                    help="监听模式：不发送，只打印 FPGA 发来的内容（排查第一步）")
    ap.add_argument("-q", "--quiet", action="store_true")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        sys.exit("找不到串口，请用 -p 指定，例如 -p /dev/tty.usbserial-1420\n"
                 "  macOS 上先看一眼：  ls /dev/tty.usb* /dev/tty.wchusbserial*")
    if not args.quiet:
        print(f"串口 {port} @ {args.baud} 8N1\n")

    ser = open_port(port, args.baud)
    time.sleep(0.1)

    if args.monitor:
        monitor(ser)
        ser.close()
        return

    if args.cmds:
        for c in args.cmds:
            if len(c) != 4 or not c.isdigit():
                print(f"  {c}  ->  ⚠️  跳过（必须是 4 位数字）")
                continue
            send_one(ser, c, args.quiet)
        ser.close()
        return

    print("输入 4 位数字并回车即可，例：2090 = 2 号舵机转到 90")
    print("  心跳 A xxx xxx xxx xxx 会每秒刷一行，属正常现象")
    print("  输入 m 切到监听模式，q 退出，h 看帮助\n")
    try:
        while True:
            try:
                s = input("指令> ").strip()
            except EOFError:
                break
            if not s:
                continue
            if s.lower() in ("q", "quit", "exit"):
                break
            if s.lower() in ("h", "help", "?"):
                print("  格式：<舵机号 1-4><角度百位><十位><个位>")
                print("    1090 -> 1 号舵机到 90")
                print("    4360 -> 4 号舵机到 360")
                print("  注意：舵机号也占 1 位，所以总数一定是 4 位\n")
                continue
            if s.lower() == "m":
                monitor(ser)
                continue
            if len(s) != 4 or not s.isdigit():
                print("  ⚠️  必须是 4 位数字，例 2090\n")
                continue
            send_one(ser, s)
    except KeyboardInterrupt:
        print()
    finally:
        ser.close()
        print("已关闭串口")


if __name__ == "__main__":
    main()

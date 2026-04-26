"""
tools/imu_plot.py
=================

PC 端实时绘图，配合 KneeExo 固件 profile = imu_only：
  - 固件每 10ms 通过主串口（UART0/USB-CDC）打印一行 CSV：
      $IMU,t_us,ax,ay,az,gx,gy,gz,roll,pitch,yaw,T
  - 本脚本订阅串口、过滤 $IMU, 开头的行，画 4 张实时滚动曲线：
      1) accelerometer (g)
      2) gyroscope     (deg/s)
      3) euler         (deg)
      4) temperature   (°C)

依赖：
    pip install pyserial matplotlib

用法（先 idf.py monitor 退出，把串口让出来）：
    python tools/imu_plot.py --port COM3

按 Ctrl+C 退出。
"""

from __future__ import annotations

import argparse
import collections
import re
import sys
import threading
import time

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import serial

LINE_RE = re.compile(
    r"^\$IMU,"
    r"(?P<t>-?\d+),"
    r"(?P<ax>-?[\d.]+),(?P<ay>-?[\d.]+),(?P<az>-?[\d.]+),"
    r"(?P<gx>-?[\d.]+),(?P<gy>-?[\d.]+),(?P<gz>-?[\d.]+),"
    r"(?P<roll>-?[\d.]+),(?P<pitch>-?[\d.]+),(?P<yaw>-?[\d.]+),"
    r"(?P<T>-?[\d.]+)\s*$"
)


class IMURingBuffer:
    """线程安全的环形缓冲，固定最近 N 帧。"""

    def __init__(self, capacity: int = 800):
        self.cap = capacity
        self.lock = threading.Lock()
        self.t = collections.deque(maxlen=capacity)
        self.ax = collections.deque(maxlen=capacity)
        self.ay = collections.deque(maxlen=capacity)
        self.az = collections.deque(maxlen=capacity)
        self.gx = collections.deque(maxlen=capacity)
        self.gy = collections.deque(maxlen=capacity)
        self.gz = collections.deque(maxlen=capacity)
        self.roll = collections.deque(maxlen=capacity)
        self.pitch = collections.deque(maxlen=capacity)
        self.yaw = collections.deque(maxlen=capacity)
        self.T = collections.deque(maxlen=capacity)
        self.frames = 0

    def push(self, fields: dict):
        with self.lock:
            self.t.append(int(fields["t"]) / 1e6)  # us -> s
            self.ax.append(float(fields["ax"]))
            self.ay.append(float(fields["ay"]))
            self.az.append(float(fields["az"]))
            self.gx.append(float(fields["gx"]))
            self.gy.append(float(fields["gy"]))
            self.gz.append(float(fields["gz"]))
            self.roll.append(float(fields["roll"]))
            self.pitch.append(float(fields["pitch"]))
            self.yaw.append(float(fields["yaw"]))
            self.T.append(float(fields["T"]))
            self.frames += 1


def reader_thread(ser: serial.Serial, buf: IMURingBuffer, stop_evt: threading.Event):
    while not stop_evt.is_set():
        try:
            raw = ser.readline()
        except serial.SerialException as e:
            print(f"[reader] serial error: {e}", file=sys.stderr)
            break
        if not raw:
            continue
        try:
            line = raw.decode("utf-8", errors="ignore").strip()
        except Exception:
            continue
        m = LINE_RE.match(line)
        if not m:
            continue
        buf.push(m.groupdict())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="串口，如 COM3 / /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")
    ap.add_argument("--window", type=int, default=400,
                    help="显示最近多少帧（默认 400 帧 ≈ 4 秒 @100Hz）")
    args = ap.parse_args()

    print(f"open serial {args.port} @ {args.baud}")
    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    buf = IMURingBuffer(capacity=args.window * 2)

    stop_evt = threading.Event()
    th = threading.Thread(target=reader_thread, args=(ser, buf, stop_evt), daemon=True)
    th.start()

    fig, axes = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
    fig.suptitle("KneeExo IMU live plot — Ctrl+C to quit")
    ax_acc, ax_gyr, ax_eul, ax_T = axes

    def get_x(d: collections.deque):
        if not d:
            return []
        t0 = d[0]
        return [v - t0 for v in d]

    def update(_frame):
        with buf.lock:
            tx = get_x(buf.t)
            ax = list(buf.ax); ay = list(buf.ay); az = list(buf.az)
            gx = list(buf.gx); gy = list(buf.gy); gz = list(buf.gz)
            ro = list(buf.roll); pi = list(buf.pitch); ya = list(buf.yaw)
            T = list(buf.T)
            n = len(tx)
        if n < 2:
            return []

        ax_acc.cla(); ax_gyr.cla(); ax_eul.cla(); ax_T.cla()

        ax_acc.plot(tx, ax, label="ax"); ax_acc.plot(tx, ay, label="ay"); ax_acc.plot(tx, az, label="az")
        ax_acc.set_ylabel("acc (g)"); ax_acc.legend(loc="upper right"); ax_acc.grid(True)

        ax_gyr.plot(tx, gx, label="gx"); ax_gyr.plot(tx, gy, label="gy"); ax_gyr.plot(tx, gz, label="gz")
        ax_gyr.set_ylabel("gyro (deg/s)"); ax_gyr.legend(loc="upper right"); ax_gyr.grid(True)

        ax_eul.plot(tx, ro, label="roll")
        ax_eul.plot(tx, pi, label="pitch")
        ax_eul.plot(tx, ya, label="yaw")
        ax_eul.set_ylabel("euler (deg)"); ax_eul.legend(loc="upper right"); ax_eul.grid(True)

        ax_T.plot(tx, T, color="tab:red")
        ax_T.set_ylabel("temp (°C)"); ax_T.set_xlabel("t (s, since first frame)"); ax_T.grid(True)

        ax_T.set_title(f"frames received: {buf.frames}", fontsize=9)
        return []

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        stop_evt.set()
        ser.close()
        th.join(timeout=1.0)
        print("bye.")


if __name__ == "__main__":
    main()

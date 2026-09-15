"""
KneeExo live console.

Reads firmware telemetry lines:
  $EXO,t_us,state,shank_deg,shank_rate_dps,acc_g,knee_rad,knee_vel,
       tau_g,tau_imp,tau_cmd,tau_fb,k,b,theta_eq,landing,flex_peak,shank_cross,
       bat_v,motor_age_ms,motor_ok,impact_g

Legacy 17-, 18- and 20-field frames remain supported.

Example:
  python tools/exo_console.py --port COM3 --csv logs/run.csv
"""

from __future__ import annotations

import argparse
import collections
import csv
import re
import sys
import threading
from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import serial


FIELDS = [
    "t_us",
    "state",
    "shank_deg",
    "shank_rate_dps",
    "acc_g",
    "knee_rad",
    "knee_vel",
    "tau_g",
    "tau_imp",
    "tau_cmd",
    "tau_fb",
    "k",
    "b",
    "theta_eq",
    "landing",
    "flex_peak",
    "shank_cross",
    "bat_v",
    "motor_age_ms",
    "motor_ok",
    "impact_g",
]

LINE_RE = re.compile(r"^\$EXO,(?P<body>.+)\s*$")


class Ring:
    def __init__(self, capacity: int):
        self.lock = threading.Lock()
        self.data = {name: collections.deque(maxlen=capacity) for name in FIELDS}
        self.frames = 0
        self.last_state = "NA"

    def push(self, row: dict[str, object]) -> None:
        with self.lock:
            for name in FIELDS:
                self.data[name].append(row[name])
            self.frames += 1
            self.last_state = str(row["state"])

    def snapshot(self) -> dict[str, list]:
        with self.lock:
            return {name: list(values) for name, values in self.data.items()}


def parse_exo(line: str) -> dict[str, object] | None:
    m = LINE_RE.match(line)
    if not m:
        return None
    parts = m.group("body").split(",")
    if len(parts) not in (17, 18, 20, 21):
        return None

    row: dict[str, object] = {}
    try:
        row["t_us"] = int(parts[0])
        row["state"] = parts[1]
        for name, value in zip(FIELDS[2:14], parts[2:14]):
            row[name] = float(value)
        for name, value in zip(FIELDS[14:17], parts[14:17]):
            row[name] = int(value)
        row["bat_v"] = float(parts[17]) if len(parts) >= 18 else float("nan")
        row["motor_age_ms"] = float(parts[18]) if len(parts) >= 20 else float("nan")
        row["motor_ok"] = int(float(parts[19])) if len(parts) >= 20 else 0
        row["impact_g"] = float(parts[20]) if len(parts) >= 21 else float("nan")
    except (ValueError, OverflowError):
        # A damaged telemetry line must not terminate the serial reader thread.
        return None
    return row


def reader(ser: serial.Serial, ring: Ring, stop: threading.Event, csv_writer) -> None:
    while not stop.is_set():
        try:
            raw = ser.readline()
        except serial.SerialException as exc:
            print(f"serial error: {exc}", file=sys.stderr)
            break
        if not raw:
            continue
        line = raw.decode("utf-8", errors="ignore").strip()
        row = parse_exo(line)
        if row is None:
            continue
        ring.push(row)
        if csv_writer is not None:
            csv_writer.writerow(row)


def rel_time_s(t_us: list[int]) -> list[float]:
    if not t_us:
        return []
    t0 = t_us[0]
    return [(t - t0) / 1_000_000.0 for t in t_us]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="Serial port, e.g. COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--window", type=int, default=600, help="frames kept in plots")
    ap.add_argument("--csv", default="", help="optional CSV output path")
    args = ap.parse_args()

    csv_file = None
    csv_writer = None
    if args.csv:
        path = Path(args.csv)
        path.parent.mkdir(parents=True, exist_ok=True)
        csv_file = path.open("w", newline="", encoding="utf-8")
        csv_writer = csv.DictWriter(csv_file, fieldnames=FIELDS)
        csv_writer.writeheader()

    print(f"open {args.port} @ {args.baud}")
    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    ring = Ring(args.window)
    stop = threading.Event()
    th = threading.Thread(target=reader, args=(ser, ring, stop, csv_writer), daemon=True)
    th.start()

    fig, axes = plt.subplots(4, 1, figsize=(11, 8), sharex=True)
    ax_state, ax_motion, ax_torque, ax_imp = axes

    def update(_):
        snap = ring.snapshot()
        t = rel_time_s(snap["t_us"])
        if len(t) < 2:
            return []

        ax_state.cla()
        ax_motion.cla()
        ax_torque.cla()
        ax_imp.cla()

        state_ids = []
        mapping: dict[str, int] = {}
        for s in snap["state"]:
            if s not in mapping:
                mapping[s] = len(mapping)
            state_ids.append(mapping[s])
        ax_state.step(t, state_ids, where="post")
        ax_state.set_yticks(list(mapping.values()), list(mapping.keys()))
        ax_state.set_ylabel("state")
        ax_state.grid(True)

        ax_motion.plot(t, snap["shank_deg"], label="shank deg")
        ax_motion.plot(t, [v * 57.2958 for v in snap["knee_rad"]], label="knee deg")
        ax_motion.plot(t, snap["acc_g"], label="acc g")
        ax_motion.set_ylabel("motion")
        ax_motion.legend(loc="upper right")
        ax_motion.grid(True)

        ax_torque.plot(t, snap["tau_g"], label="gravity ff")
        ax_torque.plot(t, snap["tau_imp"], label="impedance")
        ax_torque.plot(t, snap["tau_cmd"], label="cmd")
        ax_torque.plot(t, snap["tau_fb"], label="feedback")
        ax_torque.set_ylabel("torque Nm")
        ax_torque.legend(loc="upper right")
        ax_torque.grid(True)

        ax_imp.plot(t, snap["k"], label="K Nm/rad")
        ax_imp.plot(t, snap["b"], label="B Nm*s/rad")
        ax_imp.plot(t, [v * 57.2958 for v in snap["theta_eq"]], label="theta eq deg")
        ax_imp.set_ylabel("params")
        ax_imp.set_xlabel("time s")
        ax_imp.legend(loc="upper right")
        ax_imp.grid(True)

        fig.suptitle(f"KneeExo live console | frames={ring.frames} state={ring.last_state}")
        return []

    ani = animation.FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
    try:
        plt.tight_layout()
        plt.show()
    finally:
        stop.set()
        ser.close()
        th.join(timeout=1.0)
        if csv_file is not None:
            csv_file.close()


if __name__ == "__main__":
    main()

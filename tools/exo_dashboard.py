"""
KneeExo unified dashboard.

It reads both firmware telemetry formats:
  $IMU,t_us,ax,ay,az,gx,gy,gz,roll,pitch,yaw,T
  $EXO,t_us,state,shank_deg,shank_rate_dps,acc_g,knee_rad,knee_vel,
       tau_g,tau_imp,tau_cmd,tau_fb,k,b,theta_eq,landing,flex_peak,shank_cross

Hotkeys:
  z zero current IMU shank angle
  1 static
  2 shank_forward
  3 shank_backward
  4 knee_flex
  5 landing
  m custom marker
  c clear markers
  q close

Example:
  python tools/exo_dashboard.py --port COM5 --csv logs/run.csv
"""

from __future__ import annotations

import argparse
import collections
import csv
import math
import re
import sys
import threading
import time
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from matplotlib.widgets import CheckButtons
from matplotlib.widgets import SpanSelector
from matplotlib.widgets import TextBox
import serial
import serial.tools.list_ports


IMU_FIELDS = ["ax", "ay", "az", "gx", "gy", "gz", "roll", "pitch", "yaw", "temp"]
THIGH_FIELDS = [f"thigh_{name}" for name in IMU_FIELDS]
SHANK_RAW_FIELDS = [f"shank_raw_{name}" for name in IMU_FIELDS]
EXO_FIELDS = [
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
]

IMU_RE = re.compile(r"^\$IMU(?P<idx>[12]?),(?P<body>.+)\s*$")
EXO_RE = re.compile(r"^\$EXO,(?P<body>.+)\s*$")
ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
NUM_RE = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)"

STATE_STRATEGY = {
    "SAFE": "Safe passive: gravity ff + zero/low impedance, no gait assist",
    "IDS": "Initial double support: impact buffering, stable knee",
    "SINGLE": "Single support: higher support stiffness/damping",
    "ISWING": "Initial swing: allow/assist flexion, low resistance",
    "MSWING": "Mid swing: transparent following, minimal impedance",
    "TSWING": "Terminal swing: extension damping before landing",
    "NA": "No EXO state yet: run normal/gait profile",
}


def solve_3x3(a: list[list[float]], b: list[float]) -> list[float]:
    m = [a[0] + [b[0]], a[1] + [b[1]], a[2] + [b[2]]]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-9:
            raise RuntimeError("singular matrix")
        if pivot != col:
            m[col], m[pivot] = m[pivot], m[col]
        div = m[col][col]
        for j in range(col, 4):
            m[col][j] /= div
        for r in range(3):
            if r == col:
                continue
            scale = m[r][col]
            for j in range(col, 4):
                m[r][j] -= scale * m[col][j]
    return [m[i][3] for i in range(3)]


class TelemetryBuffer:
    def __init__(self, capacity: int):
        self.lock = threading.Lock()
        self.t = collections.deque(maxlen=capacity)
        all_fields = IMU_FIELDS + THIGH_FIELDS + SHANK_RAW_FIELDS + EXO_FIELDS
        self.v = {name: collections.deque(maxlen=capacity) for name in all_fields}
        self.state = collections.deque(maxlen=capacity)
        self.markers: list[tuple[float, str]] = []
        self.gravity_samples: list[tuple[float, float]] = []
        self.latest_grav_fit: tuple[float, float, float, float] | None = None
        self.selection: tuple[float, float, str] | None = None
        self.latest_thresh_fit: tuple[float, float] | None = None
        self.control_line = "commands: ready"
        self.frames = 0
        self.last_line = ""

    def _ensure_time(self, t_us: int) -> None:
        if self.t and self.t[-1] == t_us:
            return
        self.t.append(t_us / 1_000_000.0)
        for values in self.v.values():
            values.append(values[-1] if values else float("nan"))
        self.state.append(self.state[-1] if self.state else "")

    def push_imu(self, row: dict[str, float]) -> None:
        with self.lock:
            self._ensure_time(int(row["t_us"]))
            idx = int(row.get("idx", 0))
            if idx == 1:
                prefixes = ["thigh_"]
            elif idx == 2:
                prefixes = ["", "shank_raw_"]
            else:
                prefixes = [""]
            for prefix in prefixes:
                for name in IMU_FIELDS:
                    self.v[prefix + name][-1] = float(row[name])
            self.frames += 1
            self.last_line = f"IMU{idx}" if idx else "IMU"

    def push_exo(self, row: dict[str, object]) -> None:
        with self.lock:
            self._ensure_time(int(row["t_us"]))
            self.state[-1] = str(row["state"])
            for name in EXO_FIELDS:
                if name == "state":
                    continue
                self.v[name][-1] = float(row[name])
            self.frames += 1
            self.last_line = "EXO"

    def mark(self, label: str) -> None:
        with self.lock:
            if not self.t:
                return
            self.markers.append((self.t[-1], label))

    def clear_markers(self) -> None:
        with self.lock:
            self.markers.clear()

    def set_control_line(self, line: str) -> None:
        with self.lock:
            self.control_line = line[-180:]

    def set_selection(self, t0: float, t1: float, label: str = "selected") -> None:
        if t1 < t0:
            t0, t1 = t1, t0
        with self.lock:
            self.selection = (t0, t1, label)

    def fit_thresholds_from_selection(self) -> tuple[float, float, dict[str, float]] | None:
        with self.lock:
            if self.selection is None or not self.t:
                return None
            t0_rel, t1_rel, _label = self.selection
            base = self.t[0]
            rows = []
            for i, abs_t in enumerate(self.t):
                tr = abs_t - base
                if t0_rel <= tr <= t1_rel:
                    rows.append(i)
            acc = [self.v["acc_g"][i] for i in rows if self.v["acc_g"][i] == self.v["acc_g"][i]]
            shank_rate = [self.v["shank_rate_dps"][i] for i in rows if self.v["shank_rate_dps"][i] == self.v["shank_rate_dps"][i]]
        if len(acc) < 2:
            return None
        acc_sorted = sorted(acc)
        baseline = acc_sorted[len(acc_sorted) // 2]
        peak = max(acc)
        impact = max(0.02, peak - baseline)
        impact_thresh = max(0.05, 0.55 * impact)
        if shank_rate:
            swing_abs = sorted(abs(x) for x in shank_rate)
            rate_peak = swing_abs[-1]
            swing_thresh = min(250.0, max(5.0, 0.45 * rate_peak))
        else:
            rate_peak = float("nan")
            swing_thresh = 35.0
        impact_thresh = min(2.0, impact_thresh)
        with self.lock:
            self.latest_thresh_fit = (impact_thresh, swing_thresh)
        return impact_thresh, swing_thresh, {
            "acc_baseline": baseline,
            "acc_peak": peak,
            "impact_peak": impact,
            "rate_peak": rate_peak,
        }

    def latest_value(self, name: str) -> float:
        with self.lock:
            vals = self.v.get(name, [])
            for v in reversed(vals):
                if v == v:
                    return v
        return float("nan")

    def add_gravity_sample(self) -> tuple[float, float, int] | None:
        shank = self.latest_value("shank_deg")
        if shank != shank:
            shank = self.latest_value("imu_shank_deg")
        tau = self.latest_value("tau_fb")
        if shank != shank or tau != tau:
            return None
        with self.lock:
            self.gravity_samples.append((shank, tau))
            return shank, tau, len(self.gravity_samples)

    def clear_gravity_samples(self) -> None:
        with self.lock:
            self.gravity_samples.clear()
            self.latest_grav_fit = None

    def fit_gravity(self) -> tuple[float, float, float, float] | None:
        with self.lock:
            samples = list(self.gravity_samples)
        if len(samples) < 3:
            return None
        ata = [[0.0] * 3 for _ in range(3)]
        atb = [0.0] * 3
        for shank_deg, tau in samples:
            r = math.radians(shank_deg)
            x = [math.sin(r), math.cos(r), 1.0]
            for i in range(3):
                atb[i] += x[i] * tau
                for j in range(3):
                    ata[i][j] += x[i] * x[j]
        try:
            A, B, C = solve_3x3(ata, atb)
        except RuntimeError:
            return None
        G = math.sqrt(A * A + B * B)
        phi = math.atan2(B, A)
        bias = C
        rmse = math.sqrt(sum(
            (tau - (A * math.sin(math.radians(sh)) + B * math.cos(math.radians(sh)) + C)) ** 2
            for sh, tau in samples
        ) / len(samples))
        with self.lock:
            self.latest_grav_fit = (G, phi, bias, rmse)
        return G, phi, bias, rmse

    def current_metrics(self) -> dict[str, float] | None:
        with self.lock:
            tau = [v for v in self.v.get("tau_fb", []) if v == v]
            cmd = [v for v in self.v.get("tau_cmd", []) if v == v]
            vel = [v for v in self.v.get("knee_vel", []) if v == v]
        n = min(len(tau), len(cmd), len(vel))
        if n < 5:
            return None
        tau = tau[-n:]
        cmd = cmd[-n:]
        vel = vel[-n:]
        def rms(xs: list[float]) -> float:
            return math.sqrt(sum(x * x for x in xs) / len(xs))
        tm = sum(tau) / n
        vm = sum(vel) / n
        cov = sum((a - tm) * (b - vm) for a, b in zip(tau, vel))
        vt = sum((a - tm) ** 2 for a in tau)
        vv = sum((b - vm) ** 2 for b in vel)
        corr = cov / math.sqrt(vt * vv) if vt > 1e-12 and vv > 1e-12 else float("nan")
        return {
            "samples": float(n),
            "tau_fb_rms": rms(tau),
            "tau_fb_peak": max(abs(x) for x in tau),
            "tau_cmd_rms": rms(cmd),
            "tau_vel_corr": corr,
        }

    def snapshot(self):
        with self.lock:
            return (
                list(self.t),
                {name: list(values) for name, values in self.v.items()},
                list(self.state),
                list(self.markers),
                self.selection,
                self.control_line,
                self.frames,
                self.last_line,
            )


def parse_imu(line: str) -> dict[str, float] | None:
    line = ANSI_RE.sub("", line)
    m = IMU_RE.match(line)
    if not m:
        return None
    nums = re.findall(NUM_RE, m.group("body"))
    if len(nums) < 11:
        return None
    names = ["t_us"] + IMU_FIELDS
    row = {name: float(value) for name, value in zip(names, nums[:11])}
    row["idx"] = int(m.group("idx") or 0)
    return row


def parse_exo(line: str) -> dict[str, object] | None:
    line = ANSI_RE.sub("", line)
    m = EXO_RE.match(line)
    if not m:
        return None
    p = m.group("body").split(",")
    if len(p) != 17:
        return None
    row: dict[str, object] = {"t_us": int(p[0]), "state": p[1]}
    for name, value in zip(EXO_FIELDS[1:], p[2:]):
        row[name] = float(value)
    return row


def reader(ser: serial.Serial, buf: TelemetryBuffer, stop: threading.Event, writer) -> None:
    while not stop.is_set():
        try:
            raw = ser.readline()
        except serial.SerialException as exc:
            print(f"serial error: {exc}", file=sys.stderr)
            break
        if not raw:
            continue
        line = raw.decode("utf-8", errors="ignore").strip()
        imu = parse_imu(line)
        if imu is not None:
            buf.push_imu(imu)
            if writer:
                writer.writerow({"type": "IMU", **imu})
            continue
        exo = parse_exo(line)
        if exo is not None:
            buf.push_exo(exo)
            if writer:
                writer.writerow({"type": "EXO", **exo})
            continue
        if line.startswith("$ACK") or line.startswith("$ERR"):
            buf.set_control_line(line)


def rel_time(t: list[float]) -> list[float]:
    if not t:
        return []
    t0 = t[0]
    return [x - t0 for x in t]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--window", type=int, default=300)
    ap.add_argument("--plot-points", type=int, default=180,
                    help="maximum points drawn per signal; lower is faster")
    ap.add_argument("--csv", default="")
    ap.add_argument("--interval", type=int, default=120, help="plot refresh interval ms")
    ap.add_argument("--smooth", type=int, default=5,
                    help="moving-average samples for plotted curves; latest text remains raw")
    ap.add_argument("--shank-axis", choices=["roll", "pitch", "yaw"], default="pitch",
                    help="Euler angle used as shank pitch when only $IMU is available")
    ap.add_argument("--shank-sign", type=float, default=1.0,
                    help="Sign for IMU-derived shank angle; forward swing should be positive")
    args = ap.parse_args()

    csv_file = None
    writer = None
    if args.csv:
        path = Path(args.csv)
        path.parent.mkdir(parents=True, exist_ok=True)
        csv_file = path.open("w", newline="", encoding="utf-8")
        fieldnames = ["type", "t_us", "idx"] + IMU_FIELDS + EXO_FIELDS
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.5)
    except serial.SerialException as exc:
        print(f"\n[error] failed to open serial port {args.port}: {exc}", file=sys.stderr)
        print("\nAvailable serial ports:", file=sys.stderr)
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            print("  (none found)", file=sys.stderr)
        for p in ports:
            print(f"  {p.device}: {p.description}  hwid={p.hwid}", file=sys.stderr)
        print(
            "\nHints:\n"
            "  1) Close idf.py monitor / imu_plot.py / another dashboard using the same COM port.\n"
            "  2) Re-check the actual ESP32 port with: python -m serial.tools.list_ports -v\n"
            "  3) If you just flashed, press RESET or unplug/replug the ESP32.\n"
            "  4) Use one command line, e.g. python tools\\exo_dashboard.py --port COM5 --csv logs\\run.csv\n",
            file=sys.stderr,
        )
        raise SystemExit(2)
    buf = TelemetryBuffer(args.window)
    stop = threading.Event()
    th = threading.Thread(target=reader, args=(ser, buf, stop, writer), daemon=True)
    th.start()

    plt.rcParams.update({"font.size": 9})
    fig = plt.figure(figsize=(14.5, 9.2))
    fig.subplots_adjust(bottom=0.18, top=0.92, right=0.68)
    gs = fig.add_gridspec(4, 2, width_ratios=[5.2, 1.45], hspace=0.35)
    axes = [fig.add_subplot(gs[i, 0]) for i in range(4)]
    panel = fig.add_subplot(gs[:, 1])
    panel.set_title("Signals")
    panel.axis("off")

    signal_groups = {
        "acc": ["ax", "ay", "az", "thigh_ax", "thigh_ay", "thigh_az", "acc_g"],
        "gyro/euler": ["gx", "gy", "gz", "roll", "pitch", "yaw", "thigh_pitch", "imu_shank_deg", "shank_deg", "shank_rate_dps"],
        "knee": ["knee_rad", "knee_vel"],
        "torque/params": ["tau_g", "tau_imp", "tau_cmd", "tau_fb", "k", "b", "theta_eq"],
    }
    selected = {
        "ax": True, "ay": True, "az": True,
        "thigh_ax": False, "thigh_ay": False, "thigh_az": False,
        "gx": False, "gy": False, "gz": False,
        "roll": False, "pitch": True, "yaw": False, "thigh_pitch": True,
        "imu_shank_deg": True, "shank_deg": True, "shank_rate_dps": False,
        "acc_g": True, "knee_rad": True, "knee_vel": True,
        "tau_g": True, "tau_imp": True, "tau_cmd": True, "tau_fb": True,
        "k": False, "b": False, "theta_eq": False,
    }
    imu_zero = {"offset": 0.0, "set": False}
    labels = list(selected.keys())
    checks_ax = fig.add_axes([0.865, 0.16, 0.12, 0.72])
    checks = CheckButtons(checks_ax, labels, [selected[x] for x in labels])

    def toggle(label: str) -> None:
        selected[label] = not selected[label]

    checks.on_clicked(toggle)

    cmd_status = {"text": "commands: ready"}

    def send_cmd(cmd: str) -> None:
        try:
            ser.write((cmd + "\n").encode("ascii"))
            cmd_status["text"] = f"sent {cmd}"
        except serial.SerialException as exc:
            cmd_status["text"] = f"send failed: {exc}"

    def btn_zero_imu(_event) -> None:
        send_cmd("$CMD,ZERO_IMU")

    def btn_zero_motor(_event) -> None:
        send_cmd("$CMD,ZERO_MOTOR")

    def btn_safe_on(_event) -> None:
        send_cmd("$CMD,SAFE,1")

    def btn_safe_off(_event) -> None:
        send_cmd("$CMD,SAFE,0")

    def btn_state_on(_event) -> None:
        send_cmd("$CMD,STATE_CTRL,1")

    def btn_state_off(_event) -> None:
        send_cmd("$CMD,STATE_CTRL,0")

    def lock_state(name: str):
        def _cb(_event) -> None:
            send_cmd(f"$CMD,STATE_LOCK,{name}")
        return _cb

    def btn_grav_sample(_event) -> None:
        sample = buf.add_gravity_sample()
        if sample is None:
            cmd_status["text"] = "sample failed: need normal EXO with shank/tau_fb"
        else:
            shank, tau, count = sample
            buf.mark(f"G{count}")
            cmd_status["text"] = f"G sample {count}: shank={shank:+.1f} deg tau={tau:+.3f} Nm"

    def btn_grav_fit(_event) -> None:
        fit = buf.fit_gravity()
        if fit is None:
            cmd_status["text"] = "fit failed: need >=3 samples with angle spread"
        else:
            G, phi, bias, rmse = fit
            cmd_status["text"] = f"fit G={G:.3f} phi={phi:.3f} bias={bias:.3f} rmse={rmse:.3f}"

    def btn_grav_send(_event) -> None:
        fit = buf.latest_grav_fit or buf.fit_gravity()
        if fit is None:
            cmd_status["text"] = "send failed: fit first"
            return
        G, phi, bias, _rmse = fit
        send_cmd(f"$CMD,SET_GRAV,{G:.6f},{phi:.6f},{bias:.6f}")
        grav_G_box.set_val(f"{G:.4f}")
        grav_phi_box.set_val(f"{phi:.4f}")
        grav_bias_box.set_val(f"{bias:.4f}")

    def btn_save_params(_event) -> None:
        send_cmd("$CMD,SAVE_PARAMS")

    def btn_grav_on(_event) -> None:
        send_cmd("$CMD,GRAV_ENABLE,1")

    def btn_grav_off(_event) -> None:
        send_cmd("$CMD,GRAV_ENABLE,0")

    def btn_grav_apply_manual(_event) -> None:
        try:
            G = float(grav_G_box.text)
            phi = float(grav_phi_box.text)
            bias = float(grav_bias_box.text)
        except ValueError:
            cmd_status["text"] = "manual gravity parse failed"
            return
        send_cmd(f"$CMD,SET_GRAV,{G:.6f},{phi:.6f},{bias:.6f}")

    def btn_export_report(_event) -> None:
        metrics = buf.current_metrics()
        if metrics is None:
            cmd_status["text"] = "report failed: need EXO torque samples"
            return
        out = Path("logs") / f"transparency_live_{int(time.time())}.txt"
        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open("w", encoding="utf-8") as f:
            f.write("KneeExo live transparency report\n")
            f.write("===============================\n")
            for k, v in metrics.items():
                f.write(f"{k}: {v:.6f}\n")
        cmd_status["text"] = f"report saved: {out}"

    def btn_grav_clear(_event) -> None:
        buf.clear_gravity_samples()
        cmd_status["text"] = "gravity samples cleared"

    def btn_thresh_fit(_event) -> None:
        fit = buf.fit_thresholds_from_selection()
        if fit is None:
            cmd_status["text"] = "threshold fit failed: drag-select impact/swing window first"
            return
        impact, swing, stats = fit
        impact_box.set_val(f"{impact:.3f}")
        swing_box.set_val(f"{swing:.1f}")
        cmd_status["text"] = (
            f"T fit impact={impact:.3f}g swing={swing:.1f}dps "
            f"(acc_peak={stats['acc_peak']:.2f}g, base={stats['acc_baseline']:.2f}g)"
        )

    def btn_thresh_send(_event) -> None:
        try:
            impact = float(impact_box.text)
            swing = float(swing_box.text)
        except ValueError:
            cmd_status["text"] = "threshold parse failed"
            return
        send_cmd(f"$CMD,SET_THRESH,{impact:.6f},{swing:.3f}")

    button_defs = [
        ("Zero IMU", btn_zero_imu),
        ("Zero Motor", btn_zero_motor),
        ("SAFE on", btn_safe_on),
        ("SAFE off", btn_safe_off),
        ("State on", btn_state_on),
        ("State off", btn_state_off),
        ("Auto FSM", lock_state("OFF")),
        ("Lock SAFE", lock_state("SAFE")),
        ("Lock IDS", lock_state("IDS")),
        ("Lock SINGLE", lock_state("SINGLE")),
        ("Lock ISW", lock_state("ISWING")),
        ("Lock MSW", lock_state("MSWING")),
        ("Lock TSW", lock_state("TSWING")),
        ("G sample", btn_grav_sample),
        ("G fit", btn_grav_fit),
        ("G send", btn_grav_send),
        ("G on", btn_grav_on),
        ("G off", btn_grav_off),
        ("G apply", btn_grav_apply_manual),
        ("Save params", btn_save_params),
        ("Report", btn_export_report),
        ("T fit", btn_thresh_fit),
        ("T send", btn_thresh_send),
        ("G clear", btn_grav_clear),
    ]
    buttons = []
    for i, (label, cb) in enumerate(button_defs):
        col = i % 2
        row = i // 2
        axb = fig.add_axes([0.695 + col * 0.079, 0.885 - row * 0.037, 0.072, 0.028])
        b = Button(axb, label)
        b.on_clicked(cb)
        buttons.append(b)

    grav_G_ax = fig.add_axes([0.695, 0.135, 0.075, 0.026])
    grav_phi_ax = fig.add_axes([0.695, 0.102, 0.075, 0.026])
    grav_bias_ax = fig.add_axes([0.695, 0.069, 0.075, 0.026])
    grav_G_box = TextBox(grav_G_ax, "G", initial="0.0")
    grav_phi_box = TextBox(grav_phi_ax, "phi", initial="0.0")
    grav_bias_box = TextBox(grav_bias_ax, "bias", initial="0.0")
    impact_ax = fig.add_axes([0.785, 0.135, 0.075, 0.026])
    swing_ax = fig.add_axes([0.785, 0.102, 0.075, 0.026])
    impact_box = TextBox(impact_ax, "impact", initial="0.18")
    swing_box = TextBox(swing_ax, "swing", initial="35.0")

    def on_key(event) -> None:
        mapping = {
            "1": "static",
            "2": "shank_forward",
            "3": "shank_backward",
            "4": "knee_flex",
            "5": "landing",
            "m": "marker",
        }
        if event.key in mapping:
            buf.mark(mapping[event.key])
        elif event.key == "z":
            t_raw, values, _states, _markers, _selection, _control_line, _frames, _last_line = buf.snapshot()
            vals = values.get(args.shank_axis, [])
            current = next((v for v in reversed(vals) if v == v), None)
            if current is not None:
                imu_zero["offset"] = current
                imu_zero["set"] = True
                buf.mark("imu_zero")
        elif event.key == "c":
            buf.clear_markers()
        elif event.key == "q":
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)

    def on_select(xmin: float, xmax: float) -> None:
        buf.set_selection(xmin, xmax, "selected")
        cmd_status["text"] = f"selected {xmin:.2f}-{xmax:.2f}s; click T fit to estimate thresholds"

    span_selectors = [
        SpanSelector(ax, on_select, "horizontal", useblit=True,
                     props=dict(alpha=0.18, facecolor="tab:orange"),
                     interactive=True)
        for ax in axes
    ]
    _keep_span_selectors_alive = span_selectors

    latest_text = fig.text(0.02, 0.125, "", fontsize=9, family="monospace")
    command_text = fig.text(0.02, 0.095, "", fontsize=9, family="monospace", color="tab:blue")
    state_text = fig.text(0.02, 0.058, "", fontsize=8.5, family="monospace", color="tab:purple")

    def update(_):
        t_raw, values, states, markers, selection, control_line, frames, last_line = buf.snapshot()
        t = rel_time(t_raw)
        if len(t) < 2:
            return []

        stride = max(1, len(t) // args.plot_points)
        t_plot = t[::stride]
        values_plot = {name: vals[::stride] for name, vals in values.items()}
        imu_shank = [
            args.shank_sign * (v - imu_zero["offset"])
            if v == v else float("nan")
            for v in values[args.shank_axis]
        ]
        values["imu_shank_deg"] = imu_shank
        values_plot["imu_shank_deg"] = imu_shank[::stride]

        for ax in axes:
            ax.clear()
            ax.grid(True, alpha=0.35)
            ax.axhline(0.0, color="0.35", linewidth=0.8, alpha=0.45)
            ax.set_xlim(t_plot[0], t_plot[-1])
            if selection is not None:
                sx0, sx1, _slabel = selection
                if sx1 >= t_plot[0] and sx0 <= t_plot[-1]:
                    ax.axvspan(max(sx0, t_plot[0]), min(sx1, t_plot[-1]),
                               color="tab:orange", alpha=0.12)

        for ax, (title, names) in zip(axes, signal_groups.items()):
            for name in names:
                if selected.get(name, False):
                    y = values_plot[name]
                    if name in ("knee_rad", "theta_eq"):
                        y = [v * 57.2958 for v in y]
                        label = f"{name} deg"
                    else:
                        label = name
                    if args.smooth > 1 and len(y) >= args.smooth:
                        smoothed = []
                        q = collections.deque(maxlen=args.smooth)
                        for v in y:
                            if v == v:
                                q.append(v)
                            smoothed.append(sum(q) / len(q) if q else v)
                        y = smoothed
                    ax.plot(t_plot, y, label=label, linewidth=1.2)
            for mt, label in markers:
                x = mt - t_raw[0]
                if x < t_plot[0] or x > t_plot[-1]:
                    continue
                ax.axvline(x, color="tab:red", alpha=0.35, linewidth=1.0)
                ax.text(x, 0.98, label, transform=ax.get_xaxis_transform(),
                        rotation=90, va="top", ha="right", fontsize=8, color="tab:red")
            ax.set_title(title, loc="left", fontsize=10)
            if ax.lines:
                ax.legend(loc="upper right", fontsize=8, ncol=2)

        if states:
            state = next((s for s in reversed(states) if s), "NA")
        else:
            state = "NA"
        def latest(name: str) -> float:
            vals = values.get(name, [])
            for v in reversed(vals):
                if v == v:
                    return v
            return float("nan")

        shank_latest = latest("shank_deg")
        if shank_latest != shank_latest:
            shank_latest = latest("imu_shank_deg")
        latest_text.set_text(
            "latest | "
            f"ax={latest('ax'):+.2f} ay={latest('ay'):+.2f} az={latest('az'):+.2f} g | "
            f"thigh={latest('thigh_pitch'):+.1f} pitch={latest('pitch'):+.1f} yaw={latest('yaw'):+.1f} shank={shank_latest:+.1f} deg | "
            f"knee={latest('knee_rad')*57.2958:+.1f} deg vel={latest('knee_vel'):+.2f} rad/s | "
            f"tau_cmd={latest('tau_cmd'):+.2f} tau_fb={latest('tau_fb'):+.2f} Nm"
        )
        if control_line != "commands: ready":
            cmd_status["text"] = control_line
        command_text.set_text(cmd_status["text"])
        state_text.set_text(
            f"state={state} | {STATE_STRATEGY.get(state, 'unknown')}\n"
            f"K={latest('k'):.2f} Nm/rad  B={latest('b'):.2f} Nm*s/rad  "
            f"theta_eq={latest('theta_eq')*57.2958:.1f} deg  "
            f"tau_g={latest('tau_g'):+.2f} tau_imp={latest('tau_imp'):+.2f} tau_cmd={latest('tau_cmd'):+.2f}"
        )
        fig.suptitle(
            f"KneeExo Unified Dashboard | frames={frames} last={last_line} state={state} "
            f"| z zero({args.shank_axis}, {'set' if imu_zero['set'] else 'raw'}), "
            f"drag select -> T fit/T send, keys: 1 static, 2 forward, 3 backward, 4 flex, 5 landing, c clear",
            fontsize=11,
        )
        axes[-1].set_xlabel("time (s)")
        return []

    ani = FuncAnimation(fig, update, interval=args.interval, blit=False, cache_frame_data=False)
    try:
        plt.show()
    finally:
        stop.set()
        ser.close()
        th.join(timeout=1.0)
        if csv_file is not None:
            csv_file.close()


if __name__ == "__main__":
    main()

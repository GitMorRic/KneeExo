"""
Fast PyQtGraph dashboard for KneeExo.

Example:
  python tools/exo_dashboard_qt.py --port COM8 --csv logs/normal_qt.csv --smooth 5

Install deps if needed:
  python -m pip install PyQt6 pyqtgraph pyserial
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

import serial
import serial.tools.list_ports

try:
    from PyQt6 import QtCore, QtWidgets
    import pyqtgraph as pg
except ImportError as exc:
    print(
        "[error] Qt dashboard needs PyQt6 + pyqtgraph.\n"
        "Install with:\n"
        "  python -m pip install PyQt6 pyqtgraph pyserial\n"
        f"\nImport error: {exc}",
        file=sys.stderr,
    )
    raise SystemExit(2)


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

PLOT_GROUPS = {
    "acc": ["ax", "ay", "az", "acc_g"],
    "gyro/euler": ["pitch", "thigh_pitch", "imu_shank_deg", "shank_deg", "shank_rate_dps"],
    "knee": ["knee_rad", "knee_vel"],
    "torque/params": ["tau_g", "tau_imp", "tau_cmd", "tau_fb", "k", "b", "theta_eq"],
}

DEFAULT_SELECTED = {
    "ax", "ay", "az", "acc_g",
    "pitch", "thigh_pitch", "imu_shank_deg", "shank_deg",
    "knee_rad", "knee_vel",
    "tau_g", "tau_imp", "tau_cmd", "tau_fb",
}

COLORS = [
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
    "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf",
]


def parse_imu(line: str) -> dict[str, float] | None:
    line = ANSI_RE.sub("", line)
    m = IMU_RE.match(line)
    if not m:
        return None
    nums = re.findall(NUM_RE, m.group("body"))
    if len(nums) < 11:
        return None
    row = {name: float(value) for name, value in zip(["t_us"] + IMU_FIELDS, nums[:11])}
    row["idx"] = int(m.group("idx") or 0)
    return row


def parse_exo(line: str) -> dict[str, object] | None:
    line = ANSI_RE.sub("", line)
    m = EXO_RE.match(line)
    if not m:
        return None
    parts = m.group("body").split(",")
    if len(parts) != 17:
        return None
    row: dict[str, object] = {"t_us": int(parts[0]), "state": parts[1]}
    for name, value in zip(EXO_FIELDS[1:], parts[2:]):
        row[name] = float(value)
    return row


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
        fields = IMU_FIELDS + THIGH_FIELDS + SHANK_RAW_FIELDS + EXO_FIELDS
        self.v = {name: collections.deque(maxlen=capacity) for name in fields}
        self.state = collections.deque(maxlen=capacity)
        self.frames = 0
        self.last_line = ""
        self.control_line = "ready"
        self.gravity_status = "G: unknown"
        self.gravity_samples: list[tuple[float, float]] = []
        self.latest_grav_fit: tuple[float, float, float, float] | None = None

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
            prefixes = ["thigh_"] if idx == 1 else (["", "shank_raw_"] if idx == 2 else [""])
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
                if name != "state":
                    self.v[name][-1] = float(row[name])
            self.frames += 1
            self.last_line = "EXO"

    def set_control_line(self, line: str) -> None:
        with self.lock:
            self.control_line = line[-220:]
            if line.startswith("$ACK,GRAV_ENABLE,"):
                parts = line.split(",", 3)
                if len(parts) >= 3:
                    self.gravity_status = "G: ON" if parts[2] == "1" else "G: OFF"
                    if len(parts) == 4:
                        self.gravity_status += f" | {parts[3]}"
            elif line.startswith("$ACK,SET_GRAV,"):
                self.gravity_status = "G params applied: " + line.removeprefix("$ACK,SET_GRAV,")
            elif line.startswith("$ACK,PARAMS,"):
                self.gravity_status = line.removeprefix("$ACK,PARAMS,")[-140:]

    def latest(self, name: str) -> float:
        with self.lock:
            vals = self.v.get(name, [])
            for v in reversed(vals):
                if v == v:
                    return v
        return float("nan")

    def snapshot(self):
        with self.lock:
            return (
                list(self.t),
                {name: list(values) for name, values in self.v.items()},
                list(self.state),
                self.frames,
                self.last_line,
                self.control_line,
                self.gravity_status,
            )

    def add_gravity_sample(self) -> tuple[float, float, int] | None:
        shank = self.latest("shank_deg")
        if shank != shank:
            shank = self.latest("imu_shank_deg")
        tau = self.latest("tau_fb")
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
        ss = 0.0
        s1 = 0.0
        st = 0.0
        tt = 0.0
        n = float(len(samples))
        for shank_deg, tau in samples:
            s = math.sin(math.radians(shank_deg))
            ss += s * s
            s1 += s
            st += s * tau
            tt += tau
        det = ss * n - s1 * s1
        if abs(det) < 1e-9:
            return None
        G = (st * n - s1 * tt) / det
        phi = 0.0
        C = (ss * tt - s1 * st) / det
        rmse = math.sqrt(sum(
            (tau - (G * math.sin(math.radians(sh)) + C)) ** 2
            for sh, tau in samples
        ) / len(samples))
        with self.lock:
            self.latest_grav_fit = (G, phi, C, rmse)
        return G, phi, C, rmse


class SerialReader(threading.Thread):
    def __init__(self, ser: serial.Serial, buf: TelemetryBuffer, writer):
        super().__init__(daemon=True)
        self.ser = ser
        self.buf = buf
        self.writer = writer
        self.stop_event = threading.Event()

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                raw = self.ser.readline()
            except serial.SerialException as exc:
                self.buf.set_control_line(f"serial error: {exc}")
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").strip()
            imu = parse_imu(line)
            if imu is not None:
                self.buf.push_imu(imu)
                if self.writer:
                    self.writer.writerow({"type": "IMU", **imu})
                continue
            exo = parse_exo(line)
            if exo is not None:
                self.buf.push_exo(exo)
                if self.writer:
                    self.writer.writerow({"type": "EXO", **exo})
                continue
            if line.startswith("$ACK") or line.startswith("$ERR"):
                self.buf.set_control_line(line)


class Dashboard(QtWidgets.QMainWindow):
    def __init__(self, args, ser: serial.Serial, buf: TelemetryBuffer):
        super().__init__()
        self.args = args
        self.ser = ser
        self.buf = buf
        self.imu_zero_offset = 0.0
        self.imu_zero_set = False
        self.selected = {name: name in DEFAULT_SELECTED for group in PLOT_GROUPS.values() for name in group}
        self.curves: dict[str, pg.PlotDataItem] = {}
        self.last_values: dict[str, float] = {}

        pg.setConfigOptions(antialias=False, foreground="k", background="w")
        self.setWindowTitle("KneeExo Fast Dashboard")
        self.resize(1500, 920)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        left = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        self.title = QtWidgets.QLabel("KneeExo Fast Dashboard")
        self.title.setStyleSheet("font-size: 15px; font-weight: 600;")
        left_layout.addWidget(self.title)

        self.plots: dict[str, pg.PlotWidget] = {}
        for group in PLOT_GROUPS:
            plot = pg.PlotWidget(title=group)
            plot.showGrid(x=True, y=True, alpha=0.25)
            plot.addLine(y=0.0, pen=pg.mkPen((120, 120, 120), width=1))
            self.plots[group] = plot
            left_layout.addWidget(plot, stretch=1)
        root.addWidget(left, stretch=6)

        control = QtWidgets.QWidget()
        control_layout = QtWidgets.QVBoxLayout(control)
        control_layout.setContentsMargins(4, 0, 4, 0)
        control_layout.setSpacing(6)
        root.addWidget(control, stretch=2)

        button_grid = QtWidgets.QGridLayout()
        control_layout.addLayout(button_grid)
        buttons = [
            ("Zero IMU", lambda: self.send("$CMD,ZERO_IMU")),
            ("Zero Motor", lambda: self.send("$CMD,ZERO_MOTOR")),
            ("SAFE on", lambda: self.send("$CMD,SAFE,1")),
            ("SAFE off", lambda: self.send("$CMD,SAFE,0")),
            ("State on", lambda: self.send("$CMD,STATE_CTRL,1")),
            ("State off", lambda: self.send("$CMD,STATE_CTRL,0")),
            ("Auto FSM", lambda: self.send("$CMD,STATE_LOCK,OFF")),
            ("Lock SAFE", lambda: self.send("$CMD,STATE_LOCK,SAFE")),
            ("Lock IDS", lambda: self.send("$CMD,STATE_LOCK,IDS")),
            ("Lock SINGLE", lambda: self.send("$CMD,STATE_LOCK,SINGLE")),
            ("Lock ISW", lambda: self.send("$CMD,STATE_LOCK,ISWING")),
            ("Lock MSW", lambda: self.send("$CMD,STATE_LOCK,MSWING")),
            ("Lock TSW", lambda: self.send("$CMD,STATE_LOCK,TSWING")),
            ("G sample", self.gravity_sample),
            ("G fit", self.gravity_fit),
            ("G send", self.gravity_send),
            ("G on", self.gravity_on),
            ("G off", self.gravity_off),
            ("G apply", self.gravity_apply),
            ("Save params", lambda: self.send("$CMD,SAVE_PARAMS")),
            ("T fit", self.threshold_fit),
            ("T send", self.threshold_send),
            ("Report", self.report),
            ("G clear", self.gravity_clear),
        ]
        for i, (text, callback) in enumerate(buttons):
            btn = QtWidgets.QPushButton(text)
            btn.clicked.connect(callback)
            button_grid.addWidget(btn, i // 2, i % 2)

        form = QtWidgets.QFormLayout()
        self.g_edit = QtWidgets.QLineEdit("0.0")
        self.phi_edit = QtWidgets.QLineEdit("0.0")
        self.bias_edit = QtWidgets.QLineEdit("0.0")
        self.impact_edit = QtWidgets.QLineEdit("0.18")
        self.swing_edit = QtWidgets.QLineEdit("35.0")
        for edit in [self.g_edit, self.phi_edit, self.bias_edit, self.impact_edit, self.swing_edit]:
            edit.setMaximumWidth(120)
        form.addRow("G", self.g_edit)
        form.addRow("phi", self.phi_edit)
        form.addRow("bias", self.bias_edit)
        form.addRow("impact", self.impact_edit)
        form.addRow("swing", self.swing_edit)
        control_layout.addLayout(form)

        self.range_region = pg.LinearRegionItem([0.0, 1.0], movable=True)
        self.range_region.setZValue(10)
        self.plots["acc"].addItem(self.range_region)
        hint = QtWidgets.QLabel("Drag orange range on acc plot, then T fit/T send.")
        hint.setWordWrap(True)
        control_layout.addWidget(hint)

        self.latest_label = QtWidgets.QLabel()
        self.latest_label.setWordWrap(True)
        self.latest_label.setStyleSheet("font-family: Consolas, monospace;")
        self.command_label = QtWidgets.QLabel("commands: ready")
        self.command_label.setWordWrap(True)
        self.command_label.setStyleSheet("color: #1266b0; font-family: Consolas, monospace;")
        self.state_label = QtWidgets.QLabel()
        self.state_label.setWordWrap(True)
        self.state_label.setStyleSheet("color: #7b3fb2; font-family: Consolas, monospace;")
        self.gravity_label = QtWidgets.QLabel("G: unknown")
        self.gravity_label.setWordWrap(True)
        self.gravity_label.setStyleSheet("color: #0b7a38; font-family: Consolas, monospace;")
        control_layout.addWidget(self.latest_label)
        control_layout.addWidget(self.command_label)
        control_layout.addWidget(self.gravity_label)
        control_layout.addWidget(self.state_label)
        control_layout.addStretch(1)

        signal_panel = QtWidgets.QScrollArea()
        signal_panel.setWidgetResizable(True)
        signal_body = QtWidgets.QWidget()
        signal_layout = QtWidgets.QVBoxLayout(signal_body)
        signal_layout.setContentsMargins(8, 8, 8, 8)
        signal_layout.addWidget(QtWidgets.QLabel("Signals"))
        for name in sorted(self.selected.keys()):
            cb = QtWidgets.QCheckBox(name)
            cb.setChecked(self.selected[name])
            cb.toggled.connect(lambda checked, n=name: self.toggle_signal(n, checked))
            signal_layout.addWidget(cb)
        signal_layout.addStretch(1)
        signal_panel.setWidget(signal_body)
        root.addWidget(signal_panel, stretch=1)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(args.interval)

    def keyPressEvent(self, event) -> None:
        key = event.key()
        if key == QtCore.Qt.Key.Key_Z:
            current = self.last_values.get(self.args.shank_axis, float("nan"))
            if current == current:
                self.imu_zero_offset = current
                self.imu_zero_set = True
        elif key == QtCore.Qt.Key.Key_Q:
            self.close()
        else:
            super().keyPressEvent(event)

    def send(self, cmd: str) -> None:
        try:
            self.ser.write((cmd + "\n").encode("ascii"))
            self.command_label.setText(f"sent {cmd}")
        except serial.SerialException as exc:
            self.command_label.setText(f"send failed: {exc}")

    def toggle_signal(self, name: str, checked: bool) -> None:
        self.selected[name] = checked
        if name in self.curves:
            self.curves[name].setVisible(checked)

    def gravity_sample(self) -> None:
        sample = self.buf.add_gravity_sample()
        if sample is None:
            self.command_label.setText("G sample failed: need shank_deg and tau_fb")
            return
        shank, tau, count = sample
        self.command_label.setText(f"G sample {count}: shank={shank:+.1f} deg tau={tau:+.3f} Nm")

    def gravity_fit(self) -> None:
        fit = self.buf.fit_gravity()
        if fit is None:
            self.command_label.setText("G fit failed: need >=3 samples")
            return
        G, phi, bias, rmse = fit
        self.g_edit.setText(f"{G:.4f}")
        self.phi_edit.setText(f"{phi:.4f}")
        self.bias_edit.setText(f"{bias:.4f}")
        self.command_label.setText(f"G fit G={G:.3f} phi={phi:.3f} bias={bias:.3f} rmse={rmse:.3f}")

    def gravity_send(self) -> None:
        fit = self.buf.latest_grav_fit or self.buf.fit_gravity()
        if fit is None:
            self.command_label.setText("G send failed: fit first")
            return
        G, phi, bias, _rmse = fit
        self.send(f"$CMD,SET_GRAV,{G:.6f},{phi:.6f},{bias:.6f}")

    def gravity_apply(self) -> None:
        try:
            G = float(self.g_edit.text())
            phi = float(self.phi_edit.text())
            bias = float(self.bias_edit.text())
        except ValueError:
            self.command_label.setText("manual gravity parse failed")
            return False
        self.send(f"$CMD,SET_GRAV,{G:.6f},{phi:.6f},{bias:.6f}")
        return True

    def gravity_on(self) -> None:
        if self.gravity_apply():
            QtCore.QTimer.singleShot(80, lambda: self.send("$CMD,GRAV_ENABLE,1"))

    def gravity_off(self) -> None:
        self.send("$CMD,GRAV_ENABLE,0")

    def gravity_clear(self) -> None:
        self.buf.clear_gravity_samples()
        self.command_label.setText("gravity samples cleared")

    def threshold_fit(self) -> None:
        t, values, _states, _frames, _last_line, _control, _gravity_status = self.buf.snapshot()
        if len(t) < 2:
            self.command_label.setText("T fit failed: no data")
            return
        t0, t1 = self.range_region.getRegion()
        rel = [x - t[0] for x in t]
        idx = [i for i, tr in enumerate(rel) if t0 <= tr <= t1]
        acc = [values["acc_g"][i] for i in idx if values["acc_g"][i] == values["acc_g"][i]]
        rate = [values["shank_rate_dps"][i] for i in idx if values["shank_rate_dps"][i] == values["shank_rate_dps"][i]]
        if len(acc) < 2:
            self.command_label.setText("T fit failed: select impact window with acc_g data")
            return
        acc_sorted = sorted(acc)
        baseline = acc_sorted[len(acc_sorted) // 2]
        peak = max(acc)
        impact = min(2.0, max(0.05, 0.55 * max(0.02, peak - baseline)))
        if rate:
            swing = min(250.0, max(5.0, 0.45 * max(abs(x) for x in rate)))
        else:
            swing = float(self.swing_edit.text() or "35.0")
        self.impact_edit.setText(f"{impact:.3f}")
        self.swing_edit.setText(f"{swing:.1f}")
        self.command_label.setText(
            f"T fit impact={impact:.3f}g swing={swing:.1f}dps "
            f"(acc_peak={peak:.2f}g base={baseline:.2f}g)"
        )

    def threshold_send(self) -> None:
        try:
            impact = float(self.impact_edit.text())
            swing = float(self.swing_edit.text())
        except ValueError:
            self.command_label.setText("threshold parse failed")
            return
        self.send(f"$CMD,SET_THRESH,{impact:.6f},{swing:.3f}")

    def report(self) -> None:
        t, values, _states, _frames, _last_line, _control, _gravity_status = self.buf.snapshot()
        tau = [x for x in values.get("tau_fb", []) if x == x]
        cmd = [x for x in values.get("tau_cmd", []) if x == x]
        if len(tau) < 5:
            self.command_label.setText("report failed: need torque samples")
            return
        def rms(xs):
            return math.sqrt(sum(x * x for x in xs) / len(xs))
        out = Path("logs") / f"transparency_qt_{int(time.time())}.txt"
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(
            "KneeExo live transparency report\n"
            f"samples: {len(tau)}\n"
            f"tau_fb_rms: {rms(tau):.6f}\n"
            f"tau_fb_peak: {max(abs(x) for x in tau):.6f}\n"
            f"tau_cmd_rms: {rms(cmd) if cmd else float('nan'):.6f}\n",
            encoding="utf-8",
        )
        self.command_label.setText(f"report saved: {out}")

    def refresh(self) -> None:
        t, values, states, frames, last_line, control_line, gravity_status = self.buf.snapshot()
        if len(t) < 2:
            return
        rel = [x - t[0] for x in t]
        stride = max(1, len(rel) // self.args.plot_points)
        x = rel[::stride]

        imu_raw = values.get(self.args.shank_axis, [])
        imu_shank = [
            self.args.shank_sign * (v - self.imu_zero_offset) if v == v else float("nan")
            for v in imu_raw
        ]
        values["imu_shank_deg"] = imu_shank

        state = next((s for s in reversed(states) if s), "NA") if states else "NA"
        for group, names in PLOT_GROUPS.items():
            plot = self.plots[group]
            for i, name in enumerate(names):
                y = values.get(name, [])
                if not y:
                    continue
                if name in ("knee_rad", "theta_eq"):
                    y_plot = [v * 57.2958 if v == v else v for v in y[::stride]]
                    label = f"{name} deg"
                else:
                    y_plot = y[::stride]
                    label = name
                if self.args.smooth > 1 and len(y_plot) >= self.args.smooth:
                    y_plot = self.smooth(y_plot, self.args.smooth)
                if name not in self.curves:
                    self.curves[name] = plot.plot(
                        x, y_plot,
                        pen=pg.mkPen(COLORS[i % len(COLORS)], width=1.5),
                        name=label,
                    )
                    self.curves[name].setVisible(self.selected.get(name, False))
                else:
                    self.curves[name].setData(x, y_plot)
                self.curves[name].setVisible(self.selected.get(name, False))
        for plot in self.plots.values():
            plot.setXRange(max(0.0, x[-1] - self.args.view), x[-1], padding=0.0)

        def latest(name: str) -> float:
            vals = values.get(name, [])
            for v in reversed(vals):
                if v == v:
                    return v
            return float("nan")

        self.last_values = {name: latest(name) for name in values}
        shank_latest = latest("shank_deg")
        if shank_latest != shank_latest:
            shank_latest = latest("imu_shank_deg")
        self.title.setText(
            f"KneeExo Fast Dashboard | frames={frames} last={last_line} state={state} "
            f"| z zero({self.args.shank_axis}, {'set' if self.imu_zero_set else 'raw'})"
        )
        self.latest_label.setText(
            f"ax={latest('ax'):+.2f} ay={latest('ay'):+.2f} az={latest('az'):+.2f} g\n"
            f"thigh={latest('thigh_pitch'):+.1f} pitch={latest('pitch'):+.1f} "
            f"shank={shank_latest:+.1f} deg\n"
            f"knee={latest('knee_rad')*57.2958:+.1f} deg vel={latest('knee_vel'):+.2f} rad/s\n"
            f"tau_cmd={latest('tau_cmd'):+.2f} tau_fb={latest('tau_fb'):+.2f} Nm"
        )
        if control_line != "ready":
            self.command_label.setText(control_line)
        self.gravity_label.setText(gravity_status)
        self.state_label.setText(
            f"state={state} | {STATE_STRATEGY.get(state, 'unknown')}\n"
            f"K={latest('k'):.2f} B={latest('b'):.2f} "
            f"theta_eq={latest('theta_eq')*57.2958:.1f} deg\n"
            f"tau_g={latest('tau_g'):+.2f} tau_imp={latest('tau_imp'):+.2f} "
            f"tau_cmd={latest('tau_cmd'):+.2f}"
        )

    @staticmethod
    def smooth(values: list[float], n: int) -> list[float]:
        out = []
        q = collections.deque(maxlen=n)
        for v in values:
            if v == v:
                q.append(v)
            out.append(sum(q) / len(q) if q else v)
        return out


def open_serial(port: str, baud: int) -> serial.Serial:
    try:
        return serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as exc:
        print(f"[error] failed to open {port}: {exc}", file=sys.stderr)
        print("Available ports:", file=sys.stderr)
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}: {p.description}  {p.hwid}", file=sys.stderr)
        raise SystemExit(2)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--window", type=int, default=1200)
    ap.add_argument("--view", type=float, default=10.0, help="visible seconds")
    ap.add_argument("--plot-points", type=int, default=500)
    ap.add_argument("--interval", type=int, default=50)
    ap.add_argument("--smooth", type=int, default=3)
    ap.add_argument("--csv", default="")
    ap.add_argument("--shank-axis", choices=["roll", "pitch", "yaw"], default="pitch")
    ap.add_argument("--shank-sign", type=float, default=1.0)
    args = ap.parse_args()

    csv_file = None
    writer = None
    if args.csv:
        path = Path(args.csv)
        path.parent.mkdir(parents=True, exist_ok=True)
        csv_file = path.open("w", newline="", encoding="utf-8")
        writer = csv.DictWriter(csv_file, fieldnames=["type", "t_us", "idx"] + IMU_FIELDS + EXO_FIELDS,
                                extrasaction="ignore")
        writer.writeheader()

    ser = open_serial(args.port, args.baud)
    buf = TelemetryBuffer(args.window)
    reader = SerialReader(ser, buf, writer)
    reader.start()

    app = QtWidgets.QApplication(sys.argv)
    win = Dashboard(args, ser, buf)
    win.show()
    code = app.exec()

    reader.stop_event.set()
    ser.close()
    reader.join(timeout=1.0)
    if csv_file is not None:
        csv_file.close()
    raise SystemExit(code)


if __name__ == "__main__":
    main()

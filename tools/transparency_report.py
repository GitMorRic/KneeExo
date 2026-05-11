"""
Compare transparency metrics from one or two dashboard CSV files.

Examples:
  python tools/transparency_report.py --csv logs/gravity_off.csv
  python tools/transparency_report.py --before logs/gravity_off.csv --after logs/gravity_on.csv
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def fnum(row: dict[str, str], name: str) -> float | None:
    try:
        x = float(row.get(name, ""))
    except ValueError:
        return None
    return x if x == x else None


def load_metrics(path: str) -> dict[str, float]:
    tau = []
    vel = []
    cmd = []
    with Path(path).open("r", encoding="utf-8", newline="") as f:
        for row in csv.DictReader(f):
            if row.get("type") != "EXO":
                continue
            tfb = fnum(row, "tau_fb")
            kv = fnum(row, "knee_vel")
            tc = fnum(row, "tau_cmd")
            if tfb is None or kv is None or tc is None:
                continue
            tau.append(tfb)
            vel.append(kv)
            cmd.append(tc)

    if not tau:
        raise SystemExit(f"no EXO torque samples in {path}; run normal profile")

    def rms(xs: list[float]) -> float:
        return math.sqrt(sum(x * x for x in xs) / len(xs))

    tau_mean = sum(tau) / len(tau)
    vel_mean = sum(vel) / len(vel)
    cov = sum((a - tau_mean) * (b - vel_mean) for a, b in zip(tau, vel))
    var_tau = sum((a - tau_mean) ** 2 for a in tau)
    var_vel = sum((b - vel_mean) ** 2 for b in vel)
    corr = cov / math.sqrt(var_tau * var_vel) if var_tau > 1e-12 and var_vel > 1e-12 else float("nan")

    return {
        "samples": float(len(tau)),
        "tau_fb_rms": rms(tau),
        "tau_fb_peak": max(abs(x) for x in tau),
        "tau_cmd_rms": rms(cmd),
        "tau_vel_corr": corr,
    }


def print_metrics(name: str, m: dict[str, float]) -> None:
    print(name)
    print("-" * len(name))
    print(f"samples      : {m['samples']:.0f}")
    print(f"tau_fb RMS   : {m['tau_fb_rms']:.5f} Nm")
    print(f"tau_fb peak  : {m['tau_fb_peak']:.5f} Nm")
    print(f"tau_cmd RMS  : {m['tau_cmd_rms']:.5f} Nm")
    print(f"tau-vel corr : {m['tau_vel_corr']:.5f}")
    print()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="")
    ap.add_argument("--before", default="")
    ap.add_argument("--after", default="")
    args = ap.parse_args()

    if args.csv:
        print_metrics(args.csv, load_metrics(args.csv))
        return
    if not args.before or not args.after:
        raise SystemExit("provide --csv or both --before and --after")

    b = load_metrics(args.before)
    a = load_metrics(args.after)
    print_metrics("before", b)
    print_metrics("after", a)
    if b["tau_fb_rms"] > 1e-9:
        imp = (b["tau_fb_rms"] - a["tau_fb_rms"]) / b["tau_fb_rms"] * 100.0
        print(f"RMS reduction: {imp:+.1f}%")


if __name__ == "__main__":
    main()

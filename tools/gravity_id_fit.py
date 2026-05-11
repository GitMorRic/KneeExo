"""
Fit shank-link gravity compensation parameters from exo_dashboard CSV.

Model:
  tau_link = G * sin(shank_rad + phi) + bias
           = A * sin(shank_rad) + B * cos(shank_rad) + C

Usage:
  python tools/gravity_id_fit.py --csv logs/gravity_id_raw.csv
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def fnum(row: dict[str, str], name: str) -> float | None:
    value = row.get(name, "")
    if value is None or value == "":
        return None
    try:
        x = float(value)
    except ValueError:
        return None
    if math.isnan(x):
        return None
    return x


def solve_3x3(a: list[list[float]], b: list[float]) -> list[float]:
    m = [a[0] + [b[0]], a[1] + [b[1]], a[2] + [b[2]]]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-9:
            raise RuntimeError("singular fit matrix; collect wider angle range")
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


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--vel-max", type=float, default=0.08,
                    help="keep samples with |knee_vel| below this rad/s")
    ap.add_argument("--torque-field", default="tau_fb",
                    choices=["tau_fb", "tau_cmd"],
                    help="measured torque field to fit")
    ap.add_argument("--min-samples", type=int, default=30)
    args = ap.parse_args()

    rows: list[tuple[float, float]] = []
    path = Path(args.csv)
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("type") != "EXO":
                continue
            shank_deg = fnum(row, "shank_deg")
            knee_vel = fnum(row, "knee_vel")
            tau = fnum(row, args.torque_field)
            if shank_deg is None or knee_vel is None or tau is None:
                continue
            if abs(knee_vel) > args.vel_max:
                continue
            rows.append((math.radians(shank_deg), tau))

    if len(rows) < args.min_samples:
        raise SystemExit(
            f"not enough static EXO samples: {len(rows)} < {args.min_samples}. "
            "Run normal profile, hold several shank angles for 1-2s each."
        )

    # Normal equations for [A, B, C].
    ata = [[0.0] * 3 for _ in range(3)]
    atb = [0.0] * 3
    for shank_rad, tau in rows:
        x = [math.sin(shank_rad), math.cos(shank_rad), 1.0]
        for i in range(3):
            atb[i] += x[i] * tau
            for j in range(3):
                ata[i][j] += x[i] * x[j]

    A, B, C = solve_3x3(ata, atb)
    G = math.sqrt(A * A + B * B)
    phi = math.atan2(B, A)
    bias = C

    err2 = 0.0
    for shank_rad, tau in rows:
        pred = A * math.sin(shank_rad) + B * math.cos(shank_rad) + C
        err2 += (tau - pred) ** 2
    rmse = math.sqrt(err2 / len(rows))

    shank_min = min(math.degrees(r[0]) for r in rows)
    shank_max = max(math.degrees(r[0]) for r in rows)
    print("Gravity fit result")
    print("==================")
    print(f"samples        : {len(rows)}")
    print(f"shank range    : {shank_min:+.1f} .. {shank_max:+.1f} deg")
    print(f"model          : tau = G * sin(shank + phi) + bias")
    print(f"G              : {G:.5f} Nm")
    print(f"phi            : {phi:.5f} rad ({math.degrees(phi):+.2f} deg)")
    print(f"bias           : {bias:.5f} Nm")
    print(f"rmse           : {rmse:.5f} Nm")
    print()
    print("Paste into components/config/include/config.h after checking sign:")
    print(f"constexpr float   SHANK_GRAVITY_G_NM      = {G:.5f}f;")
    print(f"constexpr float   SHANK_GRAVITY_PHI_RAD   = {phi:.5f}f;")
    print(f"constexpr float   SHANK_GRAVITY_BIAS_NM   = {bias:.5f}f;")
    print()
    print("If compensation makes the link feel heavier, flip the fitted sign by negating G or bias convention.")


if __name__ == "__main__":
    main()

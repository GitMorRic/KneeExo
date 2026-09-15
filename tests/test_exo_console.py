"""Offline telemetry regression tests; no serial port or GUI packages required.

Run from the repository root: python -m unittest discover -s tests -p test_exo_console.py
"""

from __future__ import annotations

import ast
import csv
import io
import math
from pathlib import Path
import re
import unittest


def load_parser() -> dict[str, object]:
    """Execute the real parser and schema without importing serial/matplotlib."""
    source = Path(__file__).resolve().parents[1] / "tools" / "exo_console.py"
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    nodes = [
        node for node in tree.body
        if (isinstance(node, ast.Assign)
            and any(isinstance(target, ast.Name) and target.id in {"FIELDS", "LINE_RE"}
                    for target in node.targets))
        or (isinstance(node, ast.FunctionDef) and node.name == "parse_exo")
    ]
    namespace: dict[str, object] = {"re": re}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), "exec"), namespace)
    return namespace


PARSER = load_parser()
parse_exo = PARSER["parse_exo"]
FIELDS = PARSER["FIELDS"]

# Current main/control_task.cpp printf wire format (21 fields after $EXO).
# Sensor values are from the first recorded 20260516_195331 fsm_swing CSV row;
# its relative timestamp is represented here as a firmware microsecond timestamp.
CURRENT_FRAME = (
    "$EXO,20000,SAFE,-12.947,0.000,0.998,0.202,0.018,"
    "-0.009,-0.001,-0.010,-0.006,0.000,0.040,0.000,"
    "0,0,0,20.342,8.3,1,-0.002"
)


class ExoConsoleParserTests(unittest.TestCase):
    def test_current_firmware_frame_preserves_all_fields(self) -> None:
        row = parse_exo(CURRENT_FRAME + "\r\n")
        self.assertIsNotNone(row)
        self.assertEqual(set(row), set(FIELDS))
        self.assertEqual(row["t_us"], 20000)
        self.assertEqual(row["state"], "SAFE")
        self.assertAlmostEqual(row["shank_deg"], -12.947)
        self.assertAlmostEqual(row["tau_cmd"], -0.010)
        self.assertAlmostEqual(row["theta_eq"], 0.0)
        self.assertAlmostEqual(row["bat_v"], 20.342)
        self.assertAlmostEqual(row["motor_age_ms"], 8.3)
        self.assertEqual(row["motor_ok"], 1)
        self.assertAlmostEqual(row["impact_g"], -0.002)

        output = io.StringIO()
        writer = csv.DictWriter(output, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerow(row)
        recorded = next(csv.DictReader(io.StringIO(output.getvalue())))
        self.assertEqual(recorded["impact_g"], "-0.002")

    def test_legacy_frames_supply_missing_optional_fields(self) -> None:
        parts = CURRENT_FRAME.split(",")[1:]
        for count in (17, 18, 20):
            with self.subTest(fields=count):
                row = parse_exo("$EXO," + ",".join(parts[:count]))
                self.assertIsNotNone(row)
                self.assertEqual(set(row), set(FIELDS))
                self.assertEqual(row["state"], "SAFE")
                self.assertAlmostEqual(row["tau_cmd"], -0.010)
                self.assertEqual(row["landing"], 0)
                self.assertTrue(math.isnan(row["impact_g"]))
                if count >= 18:
                    self.assertAlmostEqual(row["bat_v"], 20.342)
                else:
                    self.assertTrue(math.isnan(row["bat_v"]))
                if count == 20:
                    self.assertAlmostEqual(row["motor_age_ms"], 8.3)
                    self.assertEqual(row["motor_ok"], 1)
                else:
                    self.assertTrue(math.isnan(row["motor_age_ms"]))
                    self.assertEqual(row["motor_ok"], 0)

    def test_feedback_unavailable_nan_is_valid_telemetry(self) -> None:
        parts = CURRENT_FRAME.split(",")
        parts[11] = "nan"  # tau_fb
        parts[19] = "nan"  # motor_age_ms
        parts[20] = "0"    # motor_ok
        row = parse_exo(",".join(parts))
        self.assertIsNotNone(row)
        self.assertTrue(math.isnan(row["tau_fb"]))
        self.assertTrue(math.isnan(row["motor_age_ms"]))
        self.assertEqual(row["motor_ok"], 0)

    def test_non_telemetry_and_incomplete_frames_are_ignored(self) -> None:
        parts = CURRENT_FRAME.split(",")[1:]
        for line in ("", "$ACK,SAVE_PARAMS", "I (100) control: state=SAFE",
                     "$EXO," + ",".join(parts[:16]),
                     "$EXO," + ",".join(parts[:19]), CURRENT_FRAME + ",extra"):
            with self.subTest(line=line):
                self.assertIsNone(parse_exo(line))

    def test_invalid_numeric_fields_do_not_crash_reader(self) -> None:
        for index, value in ((1, "bad-time"), (3, "bad-angle"),
                             (15, "bad-event"), (20, "inf"), (21, "bad-impact")):
            with self.subTest(index=index, value=value):
                parts = CURRENT_FRAME.split(",")
                parts[index] = value
                self.assertIsNone(parse_exo(",".join(parts)))


if __name__ == "__main__":
    unittest.main()

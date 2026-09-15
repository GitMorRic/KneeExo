"""Offline IMU wire-format tests; no GUI packages or connected hardware required."""

from __future__ import annotations

import ast
from pathlib import Path
import re
import unittest


def load_parser():
    source = Path(__file__).resolve().parents[1] / "tools" / "imu_plot.py"
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    nodes = [
        node for node in tree.body
        if (isinstance(node, ast.Assign)
            and any(isinstance(target, ast.Name) and target.id in {"FIELDS", "LINE_RE"}
                    for target in node.targets))
        or (isinstance(node, ast.FunctionDef) and node.name == "parse_imu")
    ]
    namespace = {"re": re}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), "exec"), namespace)
    return namespace["parse_imu"]


parse_imu = load_parser()
# Current firmware's IMU printf format using values from a recorded shank sample.
BODY = "20000,0.1812,-0.9814,0.0107,0.00,0.98,0.12,-89.02,-10.70,51.81,28.5"


class ImuPlotParserTests(unittest.TestCase):
    def test_default_selects_current_imu2_frame(self) -> None:
        row = parse_imu("$IMU2," + BODY + "\r\n")
        self.assertIsNotNone(row)
        self.assertEqual(row["t"], 20000)
        self.assertAlmostEqual(row["ax"], 0.1812)
        self.assertAlmostEqual(row["gy"], 0.98)
        self.assertAlmostEqual(row["pitch"], -10.70)
        self.assertAlmostEqual(row["T"], 28.5)
        self.assertIsNone(parse_imu("$IMU1," + BODY))

    def test_explicit_channel_one_excludes_channel_two(self) -> None:
        self.assertIsNotNone(parse_imu("$IMU1," + BODY, channel=1))
        self.assertIsNone(parse_imu("$IMU2," + BODY, channel=1))

    def test_legacy_untagged_frame_is_still_supported(self) -> None:
        for channel in (1, 2):
            with self.subTest(channel=channel):
                row = parse_imu("$IMU," + BODY, channel=channel)
                self.assertEqual(row["t"], 20000)
                self.assertAlmostEqual(row["yaw"], 51.81)

    def test_interleaved_sensor_stream_does_not_mix_channels(self) -> None:
        lines = [
            "$IMU1," + BODY.replace("-10.70", "65.00"),
            "$IMU2," + BODY,
            "$IMU1," + BODY.replace("-10.70", "66.00"),
            "$IMU2," + BODY.replace("-10.70", "-11.20"),
        ]
        rows = [row for line in lines if (row := parse_imu(line)) is not None]
        self.assertEqual(len(rows), 2)
        self.assertEqual([row["pitch"] for row in rows], [-10.70, -11.20])

    def test_malformed_and_non_imu_lines_are_ignored(self) -> None:
        for line in ("", "$ACK,SAVE_PARAMS", "$IMU3," + BODY,
                     "$IMU2," + BODY + ",extra", "$IMU2," + BODY.rsplit(",", 1)[0],
                     "$IMU2," + BODY.replace("20000", "bad-time"),
                     "$IMU2," + BODY.replace("0.1812", "..1")):
            with self.subTest(line=line):
                self.assertIsNone(parse_imu(line))


if __name__ == "__main__":
    unittest.main()

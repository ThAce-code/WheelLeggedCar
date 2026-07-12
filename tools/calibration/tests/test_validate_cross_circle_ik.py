from __future__ import annotations

import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

CALIBRATION_DIR = Path(__file__).resolve().parents[1]
if str(CALIBRATION_DIR) not in sys.path:
    sys.path.insert(0, str(CALIBRATION_DIR))

from validate_measurement import (  # noqa: E402
    PositionValidationReport,
    load_cross_circle_ik_csv,
    main,
)


def write_inputs(tmp, rows, positions):
    csv_path = Path(tmp) / "ik.csv"
    truth_path = Path(tmp) / "truth.json"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=["label", "measured_x_mm", "measured_y_mm"])
        writer.writeheader()
        writer.writerows(rows)
    truth_path.write_text(json.dumps({"positions": positions}), encoding="utf-8")
    return csv_path, truth_path


def repeats(label, x, y, count=5):
    return [{"label": label, "measured_x_mm": x,
             "measured_y_mm": y} for _ in range(count)]


class TestValidateCrossCircleIk(unittest.TestCase):
    def test_rejects_truth_position_missing_from_csv(self):
        with tempfile.TemporaryDirectory() as tmp:
            csv_path, truth_path = write_inputs(
                tmp, repeats("p0", 0, 50),
                {"p0": {"x_mm": 0, "y_mm": 50},
                 "p1": {"x_mm": 50, "y_mm": 50}})
            with self.assertRaisesRegex(ValueError, "missing truth positions.*p1"):
                load_cross_circle_ik_csv(csv_path, truth_path)

    def test_rejects_csv_label_absent_from_truth(self):
        with tempfile.TemporaryDirectory() as tmp:
            csv_path, truth_path = write_inputs(
                tmp, repeats("unknown", 0, 50),
                {"p0": {"x_mm": 0, "y_mm": 50}})
            with self.assertRaisesRegex(ValueError, "unknown CSV labels.*unknown"):
                load_cross_circle_ik_csv(csv_path, truth_path)

    def test_rejects_fewer_than_five_repeats_per_position(self):
        with tempfile.TemporaryDirectory() as tmp:
            csv_path, truth_path = write_inputs(
                tmp, repeats("p0", 0, 50, count=4),
                {"p0": {"x_mm": 0, "y_mm": 50}})
            with self.assertRaisesRegex(ValueError, "at least 5 repeats.*p0"):
                load_cross_circle_ik_csv(csv_path, truth_path)

    def test_report_preserves_signed_axis_bias_and_systematic_span(self):
        with tempfile.TemporaryDirectory() as tmp:
            rows = repeats("left", -48, 50) + repeats("right", 48, 50)
            csv_path, truth_path = write_inputs(tmp, rows, {
                "left": {"x_mm": -50, "y_mm": 50},
                "right": {"x_mm": 50, "y_mm": 50},
            })
            report = PositionValidationReport(load_cross_circle_ik_csv(
                csv_path, truth_path))
            report.compute()
        self.assertEqual(report.positions[0].mean_dx_mm, 2.0)
        self.assertEqual(report.positions[1].mean_dx_mm, -2.0)
        self.assertEqual(report.x_bias_span_mm, 4.0)
        self.assertFalse(report.no_systematic_error)

    def test_cli_writes_axis_biases_and_returns_nonzero_on_failed_gate(self):
        with tempfile.TemporaryDirectory() as tmp:
            csv_path, truth_path = write_inputs(
                tmp, repeats("p0", 4, 50),
                {"p0": {"x_mm": 0, "y_mm": 50}})
            output = Path(tmp) / "report.json"
            with mock.patch.object(sys, "argv", [
                    "validate_measurement.py", "--ik-csv", str(csv_path),
                    "--position-truth", str(truth_path),
                    "--output", str(output)]):
                exit_code = main()
            payload = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual(exit_code, 2)
        self.assertFalse(payload["passed"])
        self.assertIn("mean_dx_mm", payload["positions"][0])
        self.assertIn("no_systematic_error", payload["gates"])


if __name__ == "__main__":
    unittest.main()

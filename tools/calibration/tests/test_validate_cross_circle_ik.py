from __future__ import annotations

import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path

CALIBRATION_DIR = Path(__file__).resolve().parents[1]
if str(CALIBRATION_DIR) not in sys.path:
    sys.path.insert(0, str(CALIBRATION_DIR))

from validate_measurement import (  # noqa: E402
    ValidationReport,
    load_cross_circle_ik_csv,
)


class TestValidateCrossCircleIk(unittest.TestCase):
    def test_groups_xy_errors_by_truth_label(self):
        with tempfile.TemporaryDirectory() as tmp:
            csv_path = Path(tmp) / "ik.csv"
            truth_path = Path(tmp) / "truth.json"
            with csv_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(
                    stream, fieldnames=["label", "measured_x_mm", "measured_y_mm"])
                writer.writeheader()
                writer.writerows([
                    {"label": "x0_y50", "measured_x_mm": "0.2", "measured_y_mm": "50.1"},
                    {"label": "x0_y50", "measured_x_mm": "-0.1", "measured_y_mm": "49.8"},
                ])
            truth_path.write_text(json.dumps({
                "positions": {"x0_y50": {"x_mm": 0.0, "y_mm": 50.0}}
            }), encoding="utf-8")
            pairs = load_cross_circle_ik_csv(csv_path, truth_path)
        self.assertEqual([p.pair_id for p in pairs], ["x0_y50"])
        self.assertEqual(len(pairs[0].measurements), 2)
        self.assertAlmostEqual(pairs[0].measurements[0], 0.2236068, places=6)

    def test_repeatability_uses_xy_spread_not_error_magnitude_spread(self):
        with tempfile.TemporaryDirectory() as tmp:
            csv_path = Path(tmp) / "ik.csv"
            truth_path = Path(tmp) / "truth.json"
            csv_path.write_text(
                "label,measured_x_mm,measured_y_mm\n"
                "p,1.0,0.0\n"
                "p,-1.0,0.0\n", encoding="utf-8")
            truth_path.write_text(json.dumps({
                "positions": {"p": {"x_mm": 0.0, "y_mm": 0.0}}
            }), encoding="utf-8")
            report = ValidationReport(load_cross_circle_ik_csv(
                csv_path, truth_path))
            report.compute()
        self.assertGreater(report.repeatability_std_mm, 1.0)


if __name__ == "__main__":
    unittest.main()

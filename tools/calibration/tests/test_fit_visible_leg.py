from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[2]
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import fit_leg_ik_calibration as fit  # noqa: E402


VISIBLE_LEG_ROWS = [
    ("ref_start", 90, 90, -21.45, 48.09),
    ("differential_80_100", 80, 100, -20.72, 38.79),
    ("differential_70_110", 70, 110, -20.38, 32.17),
    ("ref_mid", 90, 90, -20.41, 46.59),
    ("differential_100_80", 100, 80, -21.03, 59.07),
    ("differential_110_70", 110, 70, -21.40, 73.27),
    ("differential_120_60", 120, 60, -22.03, 88.49),
    ("asymmetric_120_80", 120, 80, -31.42, 74.12),
    ("asymmetric_120_100", 120, 100, -37.94, 59.34),
    ("asymmetric_120_110", 120, 110, -39.58, 53.01),
    ("common_120", 120, 120, -40.62, 47.37),
    ("asymmetric_110_120", 110, 120, -35.96, 43.96),
    ("asymmetric_100_120", 100, 120, -30.91, 39.63),
    ("asymmetric_100_110", 100, 110, -29.63, 43.06),
    ("common_110", 110, 110, -33.08, 46.70),
    ("asymmetric_110_100", 110, 100, -31.53, 52.01),
    ("common_100", 100, 100, -26.82, 47.21),
    ("asymmetric_90_100", 90, 100, -23.86, 43.26),
    ("common_80", 80, 80, -15.04, 47.60),
    ("ref_end", 90, 90, -20.44, 47.39),
]


def sample(label: str, servo, x_mm: float = 0.0, y_mm: float = 55.0):
    return fit.Sample(
        sample_id=label,
        label=label,
        servo=servo,
        measured_x_mm=x_mm,
        measured_y_mm=y_mm,
    )


def visible_leg_samples():
    return [
        sample(label, (q0, 90.0, q2, 90.0), x_mm, y_mm)
        for label, q0, q2, x_mm, y_mm in VISIBLE_LEG_ROWS
    ]


def point_in_convex_polygon(point, polygon):
    for first, second in zip(polygon, polygon[1:] + polygon[:1]):
        cross = ((second[0] - first[0]) * (point[1] - first[1]) -
                 (second[1] - first[1]) * (point[0] - first[0]))
        if cross < -1e-9:
            return False
    return True


class TestFitVisibleLeg(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.samples = visible_leg_samples()
        cls.cfg = fit.KinematicsConfig()
        cls.reference = fit.reference_physical_point(cls.samples)
        cls.candidate = fit.fit_similarity_candidate(
            cls.samples, cls.cfg, cls.reference)

    def test_reference_is_mean_of_all_three_90_degree_captures(self):
        reference = fit.reference_physical_point(self.samples)
        self.assertAlmostEqual(reference[0], -20.7666666667, places=6)
        self.assertAlmostEqual(reference[1], 47.3566666667, places=6)

    def test_reference_requires_three_named_90_degree_captures(self):
        error = fit.validate_reference_samples(self.samples[:-1], self.cfg)
        self.assertIn("exactly one ref_start, ref_mid, and ref_end", error)

    def test_reference_rejects_non_neutral_command(self):
        broken = list(self.samples)
        broken[0] = sample("ref_start", (89.0, 90.0, 90.0, 90.0),
                           -21.45, 48.09)
        self.assertIn("reference commands",
                      fit.validate_reference_samples(broken, self.cfg))

    def test_fit_recovers_anchored_low_error_similarity(self):
        candidate = self.candidate
        self.assertEqual((self.cfg.servo_a.direction,
                          self.cfg.servo_b.direction), (-1.0, -1.0))
        self.assertEqual(candidate.determinant, -1)
        self.assertAlmostEqual(candidate.reference_physical_x_mm,
                               -20.7666666667, places=6)
        self.assertAlmostEqual(candidate.reference_physical_y_mm,
                               47.3566666667, places=6)
        metrics = fit.evaluate_candidate(candidate, self.samples, self.cfg)
        self.assertLess(metrics.radial_rmse_mm, 1.0)
        self.assertLess(metrics.max_radial_error_mm, 2.0)
        self.assertGreaterEqual(candidate.scale, 0.8)
        self.assertLessEqual(candidate.scale, 1.2)

    def test_similarity_forward_inverse_round_trip(self):
        candidate = self.candidate
        local = (20.0, 70.0)
        physical = fit.model_to_physical(candidate, *local)
        physical_delta_x = ((physical[0] -
                             candidate.reference_physical_x_mm) /
                            candidate.scale)
        physical_delta_y = ((physical[1] -
                             candidate.reference_physical_y_mm) /
                            candidate.scale)
        m00, m01, m10, m11 = candidate.matrix
        recovered = (
            candidate.reference_model_x_mm +
            m00 * physical_delta_x + m10 * physical_delta_y,
            candidate.reference_model_y_mm +
            m01 * physical_delta_x + m11 * physical_delta_y,
        )
        self.assertAlmostEqual(recovered[0], local[0], places=6)
        self.assertAlmostEqual(recovered[1], local[1], places=6)

    def test_prediction_uses_servo0_and_servo2_only(self):
        first = sample("first", (100.0, 70.0, 110.0, 80.0))
        second = sample("second", (100.0, 120.0, 110.0, 130.0))
        self.assertEqual(fit.predict_candidate(self.candidate, first, self.cfg),
                         fit.predict_candidate(self.candidate, second, self.cfg))

    def test_reference_drift_gate_rejects_large_start_end_shift(self):
        error = fit.validate_reference_drift([
            sample("ref_start", (90.0, 90.0, 90.0, 90.0), 10.0, 50.0),
            sample("ref_end", (90.0, 90.0, 90.0, 90.0), 13.0, 54.0),
        ], 2.0)
        self.assertIn("reference drift", error)

    def test_convex_hull_contains_all_samples(self):
        hull = fit.convex_hull([
            (item.measured_x_mm, item.measured_y_mm) for item in self.samples
        ])
        self.assertEqual(len(hull), 8)
        for item in self.samples:
            self.assertTrue(point_in_convex_polygon(
                (item.measured_x_mm, item.measured_y_mm), hull))

    def test_hull_rejects_physical_x_zero(self):
        hull = fit.convex_hull([
            (item.measured_x_mm, item.measured_y_mm) for item in self.samples
        ])
        self.assertFalse(point_in_convex_polygon((0.0, 55.0), hull))

    def test_candidate_matrix_is_orthogonal(self):
        m00, m01, m10, m11 = self.candidate.matrix
        self.assertAlmostEqual(m00 * m00 + m10 * m10, 1.0, places=6)
        self.assertAlmostEqual(m01 * m01 + m11 * m11, 1.0, places=6)
        self.assertAlmostEqual(m00 * m01 + m10 * m11, 0.0, places=6)
        self.assertAlmostEqual(m00 * m11 - m01 * m10, -1.0, places=6)


if __name__ == "__main__":
    unittest.main()

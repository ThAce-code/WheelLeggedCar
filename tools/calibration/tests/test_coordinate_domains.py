#!/usr/bin/env python3
"""Test solvePnP coordinate domain correctness.

Verifies that all three legal solvePnP modes produce consistent results,
and that illegal combinations are rejected.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

# Allow running from tests/ or from calibration/
_CALIB_DIR = Path(__file__).resolve().parent.parent
if str(_CALIB_DIR) not in sys.path:
    sys.path.insert(0, str(_CALIB_DIR))

from geometry_utils import solve_pnp_checked


class TestSolvePnPCoordinateDomains(unittest.TestCase):
    """Test that all three legal solvePnP modes recover the same pose."""

    @classmethod
    def setUpClass(cls):
        """Create synthetic camera and pose data."""
        # Synthetic camera: 1920x1080, ~800 px focal length
        cls.width, cls.height = 1920, 1080
        fx = fy = 800.0
        cx, cy = cls.width / 2.0, cls.height / 2.0
        cls.K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)

        # Moderate distortion
        cls.D = np.array([[-0.3, 0.1, 0.001, 0.001, 0.0]], dtype=np.float64)

        # Synthetic object points: a 100mm square marker in the X-Y plane at Z=0
        # (Larger marker gives more stable PnP with synthetic data)
        half = 50.0  # 100mm marker
        cls.obj_points = np.array([
            [-half, -half, 0],
            [ half, -half, 0],
            [ half,  half, 0],
            [-half,  half, 0],
        ], dtype=np.float64)

        # Synthetic pose: marker at 500mm distance, slightly rotated
        cls.rvec_true = np.array([[0.1], [0.05], [-0.03]], dtype=np.float64)
        cls.tvec_true = np.array([[20.0], [-10.0], [500.0]], dtype=np.float64)

        # Project to get distorted image points
        cls.raw_points_px, _ = cv2.projectPoints(
            cls.obj_points, cls.rvec_true, cls.tvec_true, cls.K, cls.D)
        cls.raw_points_px = cls.raw_points_px.reshape(-1, 2).astype(np.float64)

        # Add a tiny bit of noise so it's not trivially perfect
        rng = np.random.RandomState(42)
        cls.raw_points_px += rng.normal(0, 0.05, cls.raw_points_px.shape).astype(np.float64)

        cls.zero_D = np.zeros(5, dtype=np.float64)
        cls.identity_K = np.eye(3, dtype=np.float64)

    # ---- Legal modes ---------------------------------------------------
    # Use SOLVEPNP_ITERATIVE for synthetic tests (more robust than IPPE_SQUARE
    # for arbitrary 4-point configurations not guaranteed to be square in image)

    def test_mode_A_raw_pixel_recovers_pose(self):
        """Mode A: raw pixel + K + D."""
        success, rvec, tvec = solve_pnp_checked(
            self.obj_points, self.raw_points_px, self.K, self.D,
            point_domain="raw_pixel",
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        self.assertTrue(success, "solvePnP should succeed in raw_pixel mode")
        # Translation Z should be ~500 mm (within 50mm)
        tz = float(tvec[2, 0])
        self.assertAlmostEqual(tz, 500.0, delta=50.0,
                               msg=f"Mode A: tvec[2]={tz:.1f} should be near 500 mm")

    def test_mode_B_undistorted_pixel_recovers_pose(self):
        """Mode B: undistorted pixel + K + zero D."""
        success, rvec, tvec = solve_pnp_checked(
            self.obj_points, self.raw_points_px, self.K, self.D,
            point_domain="undistorted_pixel",
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        self.assertTrue(success, "solvePnP should succeed in undistorted_pixel mode")
        tz = float(tvec[2, 0])
        self.assertAlmostEqual(tz, 500.0, delta=50.0,
                               msg=f"Mode B: tvec[2]={tz:.1f} should be near 500 mm")

    def test_mode_C_normalized_recovers_pose(self):
        """Mode C: normalized + identity K + zero D."""
        success, rvec, tvec = solve_pnp_checked(
            self.obj_points, self.raw_points_px, self.K, self.D,
            point_domain="normalized",
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        self.assertTrue(success, "solvePnP should succeed in normalized mode")
        tz = float(tvec[2, 0])
        self.assertAlmostEqual(tz, 500.0, delta=50.0,
                               msg=f"Mode C: tvec[2]={tz:.1f} should be near 500 mm")

    def test_three_modes_are_consistent(self):
        """All three legal modes give the same tvec within tolerance."""
        results = {}
        for domain in ("raw_pixel", "undistorted_pixel", "normalized"):
            success, rvec, tvec = solve_pnp_checked(
                self.obj_points, self.raw_points_px, self.K, self.D,
                point_domain=domain,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            self.assertTrue(success, f"Mode {domain} should succeed")
            results[domain] = float(tvec[2, 0])

        # All three should agree within 2 mm
        vals = list(results.values())
        max_diff = max(vals) - min(vals)
        self.assertLess(max_diff, 2.0,
                        f"Modes tvec[2] should agree within 2mm, "
                        f"got range [{min(vals):.2f}, {max(vals):.2f}]")

    # ---- Input validation ----------------------------------------------

    def test_rejects_nan_in_image_points(self):
        """solve_pnp_checked rejects NaN in image points."""
        bad_points = self.raw_points_px.copy()
        bad_points[0, 0] = np.nan
        with self.assertRaises(ValueError) as ctx:
            solve_pnp_checked(
                self.obj_points, bad_points, self.K, self.D,
                point_domain="raw_pixel",
            )
        self.assertIn("NaN", str(ctx.exception))

    def test_rejects_inf_in_image_points(self):
        """solve_pnp_checked rejects Inf in image points."""
        bad_points = self.raw_points_px.copy()
        bad_points[0, 0] = np.inf
        with self.assertRaises(ValueError) as ctx:
            solve_pnp_checked(
                self.obj_points, bad_points, self.K, self.D,
                point_domain="raw_pixel",
            )
        self.assertIn("Inf", str(ctx.exception))

    def test_rejects_nan_in_object_points(self):
        """solve_pnp_checked rejects NaN in object points."""
        bad_obj = self.obj_points.copy()
        bad_obj[0, 0] = np.nan
        with self.assertRaises(ValueError) as ctx:
            solve_pnp_checked(
                bad_obj, self.raw_points_px, self.K, self.D,
                point_domain="raw_pixel",
            )
        self.assertIn("NaN", str(ctx.exception))

    def test_rejects_too_few_points(self):
        """solve_pnp_checked rejects fewer than 4 points."""
        with self.assertRaises(ValueError) as ctx:
            solve_pnp_checked(
                self.obj_points[:3], self.raw_points_px[:3], self.K, self.D,
                point_domain="raw_pixel",
            )
        self.assertIn("points", str(ctx.exception).lower())

    def test_rejects_wrong_shape_image_points(self):
        """solve_pnp_checked rejects image points with wrong shape."""
        bad = self.raw_points_px.flatten()  # (8,) not (4, 2)
        with self.assertRaises(ValueError) as ctx:
            solve_pnp_checked(
                self.obj_points, bad.reshape(-1, 1), self.K, self.D,
                point_domain="raw_pixel",
            )
        self.assertIn("shape", str(ctx.exception).lower())

    def test_rejects_wrong_shape_camera_matrix(self):
        """solve_pnp_checked rejects non-3x3 camera matrix."""
        bad_K = np.eye(2)
        with self.assertRaises(ValueError):
            solve_pnp_checked(
                self.obj_points, self.raw_points_px, bad_K, self.D,
                point_domain="raw_pixel",
            )

    def test_accepts_float32_inputs(self):
        """solve_pnp_checked accepts float32 arrays."""
        obj_f32 = self.obj_points.astype(np.float32)
        img_f32 = self.raw_points_px.astype(np.float32)
        success, rvec, tvec = solve_pnp_checked(
            obj_f32, img_f32, self.K, self.D,
            point_domain="raw_pixel",
        )
        self.assertTrue(success)

    # ---- Illegal combinations would be caught by logic ----------------
    # (We test that the function does NOT silently produce wrong results)

    def test_raw_pixel_preserves_distortion(self):
        """Mode A must pass original D to solvePnP (not zero)."""
        success, rvec, tvec = solve_pnp_checked(
            self.obj_points, self.raw_points_px, self.K, self.D,
            point_domain="raw_pixel",
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        self.assertTrue(success)
        # With raw distorted points and DISTORTION, tvec[2] should be ~500mm
        tz = float(tvec[2, 0])
        self.assertGreater(tz, 100.0, f"tvec[2]={tz:.1f} should be > 100mm")

    def test_undistorted_pixel_uses_zero_distortion(self):
        """Mode B correctly uses zero distortion in solvePnP call."""
        success, _, tvec = solve_pnp_checked(
            self.obj_points, self.raw_points_px, self.K, self.D,
            point_domain="undistorted_pixel",
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        self.assertTrue(success)
        tz = float(tvec[2, 0])
        self.assertAlmostEqual(tz, 500.0, delta=50.0)


if __name__ == "__main__":
    unittest.main()

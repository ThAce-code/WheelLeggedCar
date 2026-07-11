#!/usr/bin/env python3
"""Test homography coordinate domain consistency.

Verifies that homography computation and application use consistent
coordinate domains, and that domain mismatches are detected.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

_CALIB_DIR = Path(__file__).resolve().parent.parent
if str(_CALIB_DIR) not in sys.path:
    sys.path.insert(0, str(_CALIB_DIR))

from camera_utils import compute_homography, apply_homography, undistort_points
from geometry_utils import (
    HomographyData,
    save_homography,
    load_homography,
    validate_homography_match,
)


class TestHomographyDomainConsistency(unittest.TestCase):
    """Test homography coordinate domain correctness."""

    @classmethod
    def setUpClass(cls):
        """Create synthetic data for homography testing."""
        cls.width, cls.height = 1920, 1080
        fx = fy = 800.0
        cx, cy = cls.width / 2.0, cls.height / 2.0
        cls.K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        cls.D = np.array([[-0.3, 0.1, 0.001, 0.001, 0.0]], dtype=np.float64)

        # Define world points (known positions on a plane at Z=0)
        cls.plane_points_mm = np.array([
            [0.0, 0.0],
            [100.0, 0.0],
            [100.0, 80.0],
            [0.0, 80.0],
        ], dtype=np.float64)

        # Project to distorted image points using known extrinsics
        rvec = np.array([[0.05], [-0.02], [0.01]], dtype=np.float64)
        tvec = np.array([[10.0], [5.0], [600.0]], dtype=np.float64)
        raw_px, _ = cv2.projectPoints(
            np.hstack([cls.plane_points_mm, np.zeros((4, 1))]),
            rvec, tvec, cls.K, cls.D)
        cls.raw_points_px = raw_px.reshape(-1, 2).astype(np.float64)

        # Undistort points for homography computation (consistent pipeline)
        cls.undistorted_points_px = undistort_points(
            cls.raw_points_px, cls.K, cls.D)

    def test_compute_and_apply_with_same_domain(self):
        """Homography computed from undistorted points, applied to undistorted points."""
        H = compute_homography(self.undistorted_points_px, self.plane_points_mm)
        self.assertIsNotNone(H)

        # Apply to the same points → should recover plane points
        recovered = apply_homography(H, self.undistorted_points_px)
        for i in range(4):
            self.assertAlmostEqual(
                recovered[i, 0], self.plane_points_mm[i, 0], delta=2.0,
                msg=f"Point {i} x: recovered={recovered[i,0]:.2f} vs true={self.plane_points_mm[i,0]:.2f}")
            self.assertAlmostEqual(
                recovered[i, 1], self.plane_points_mm[i, 1], delta=2.0,
                msg=f"Point {i} y: recovered={recovered[i,1]:.2f} vs true={self.plane_points_mm[i,1]:.2f}")

    def test_raw_points_give_wrong_result_with_undistorted_homography(self):
        """Using raw distorted points with a homography fit to undistorted points should differ."""
        H = compute_homography(self.undistorted_points_px, self.plane_points_mm)
        # Apply to raw (distorted) points — this is the wrong usage pattern
        wrong_result = apply_homography(H, self.raw_points_px)
        correct_result = apply_homography(H, self.undistorted_points_px)
        diff = np.max(np.abs(wrong_result - correct_result))
        # Should differ by a measurable amount due to distortion
        self.assertGreater(diff, 0.01,
                           msg="Raw vs undistorted points should give different "
                               "homography results when distortion is present")

    def test_save_and_load_homography(self):
        """HomographyData round-trips through NPZ correctly."""
        H = compute_homography(self.undistorted_points_px, self.plane_points_mm)
        data = HomographyData(
            H=H,
            point_domain="undistorted_pixel",
            image_width=self.width,
            image_height=self.height,
            camera_index=0,
            backend="MSMF",
            camera_matrix=self.K,
            dist_coeffs=self.D,
            calib_path="camera_calib.npz",
        )

        import tempfile
        with tempfile.NamedTemporaryFile(suffix=".npz", delete=False) as f:
            tmp_path = Path(f.name)

        try:
            save_homography(tmp_path, data)
            loaded = load_homography(tmp_path)

            self.assertEqual(loaded.point_domain, "undistorted_pixel")
            self.assertEqual(loaded.image_width, self.width)
            self.assertEqual(loaded.image_height, self.height)
            self.assertEqual(loaded.camera_index, 0)
            self.assertEqual(loaded.backend, "MSMF")
            np.testing.assert_array_almost_equal(loaded.H, H)
            np.testing.assert_array_almost_equal(loaded.camera_matrix, self.K)
            np.testing.assert_array_almost_equal(loaded.dist_coeffs, self.D)
        finally:
            tmp_path.unlink(missing_ok=True)
            # Also clean up the JSON sidecar
            json_path = tmp_path.with_suffix(".json")
            json_path.unlink(missing_ok=True)

    def test_validate_homography_match_passes_for_same_params(self):
        """validate_homography_match succeeds when camera params match."""
        H = compute_homography(self.undistorted_points_px, self.plane_points_mm)
        data = HomographyData(
            H=H,
            point_domain="undistorted_pixel",
            image_width=self.width,
            image_height=self.height,
            camera_index=0,
            backend="MSMF",
            camera_matrix=self.K.copy(),
            dist_coeffs=self.D.copy(),
            calib_path="camera_calib.npz",
        )
        # Should NOT raise
        validate_homography_match(
            data, self.K, self.D, (self.width, self.height))

    def test_validate_homography_match_detects_resolution_mismatch(self):
        """validate_homography_match raises on resolution mismatch."""
        H = compute_homography(self.undistorted_points_px, self.plane_points_mm)
        data = HomographyData(
            H=H,
            point_domain="undistorted_pixel",
            image_width=self.width,
            image_height=self.height,
            camera_index=0,
            backend="MSMF",
            camera_matrix=self.K.copy(),
            dist_coeffs=self.D.copy(),
            calib_path="camera_calib.npz",
        )
        with self.assertRaises(RuntimeError) as ctx:
            validate_homography_match(
                data, self.K, self.D, (640, 480))  # Different resolution
        self.assertIn("resolution", str(ctx.exception).lower())

    def test_validate_homography_match_detects_K_mismatch(self):
        """validate_homography_match raises on significant K mismatch."""
        H = compute_homography(self.undistorted_points_px, self.plane_points_mm)
        data = HomographyData(
            H=H,
            point_domain="undistorted_pixel",
            image_width=self.width,
            image_height=self.height,
            camera_index=0,
            backend="MSMF",
            camera_matrix=self.K.copy(),
            dist_coeffs=self.D.copy(),
            calib_path="camera_calib.npz",
        )
        # Different focal length
        different_K = self.K.copy()
        different_K[0, 0] = 400.0  # Half the focal length
        with self.assertRaises(RuntimeError) as ctx:
            validate_homography_match(
                data, different_K, self.D, (self.width, self.height))
        self.assertIn("camera", str(ctx.exception).lower())


if __name__ == "__main__":
    unittest.main()

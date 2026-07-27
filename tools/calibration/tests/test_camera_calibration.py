#!/usr/bin/env python3
"""Regression tests for camera intrinsic calibration calculations."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

_CALIB_DIR = Path(__file__).resolve().parent.parent
if str(_CALIB_DIR) not in sys.path:
    sys.path.insert(0, str(_CALIB_DIR))

from camera_calibrate import calibrate_from_frames


class TestCalibrationPointLayouts(unittest.TestCase):
    def test_sb_corner_layout_computes_per_view_errors(self):
        """OpenCV 5 SB corners shaped (N, 2) must match projected points."""
        objp = np.zeros((9 * 6, 3), dtype=np.float32)
        objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2) * 25.0
        camera_matrix = np.array([
            [1200.0, 0.0, 960.0],
            [0.0, 1180.0, 540.0],
            [0.0, 0.0, 1.0],
        ])

        object_points = []
        image_points = []
        for i in range(5):
            rvec = np.array([0.03 * i, -0.02 * i, 0.01 * i])
            tvec = np.array([-80.0 + 35.0 * i, -50.0 + 20.0 * i,
                             850.0 + 45.0 * i])
            projected, _ = cv2.projectPoints(
                objp, rvec, tvec, camera_matrix, None)
            object_points.append(objp.copy())
            # findChessboardCornersSB on OpenCV 5 returns (N, 2), not
            # projectPoints' (N, 1, 2) channel layout.
            image_points.append(projected.reshape(-1, 2).astype(np.float32))

        mtx, dist, rmse, per_view = calibrate_from_frames(
            object_points, image_points, (1920, 1080))

        self.assertIsNotNone(mtx)
        self.assertIsNotNone(dist)
        self.assertTrue(np.isfinite(rmse))
        self.assertEqual(len(per_view), 5)
        self.assertTrue(all(np.isfinite(error) for error in per_view))


if __name__ == "__main__":
    unittest.main()

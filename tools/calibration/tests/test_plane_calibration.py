from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

CALIBRATION_DIR = Path(__file__).resolve().parents[1]
if str(CALIBRATION_DIR) not in sys.path:
    sys.path.insert(0, str(CALIBRATION_DIR))

from plane_calibration import (  # noqa: E402
    PlaneCalibration,
    compute_plane_calibration,
    load_plane_calibration,
    save_plane_calibration,
    validate_plane_calibration,
)
from plane_calibrate import _detected_corner_count  # noqa: E402


K = np.array([
    [1097.6, 0.0, 950.7],
    [0.0, 1097.6, 492.8],
    [0.0, 0.0, 1.0],
], dtype=np.float64)
D = np.array([-0.0114, 0.00355, 0.0, 0.0], dtype=np.float64)


def make_synthetic_plane_calibration(
    front_direction: str = "left", down_direction: str = "down",
) -> PlaneCalibration:
    src = np.array([
        [300.0, 200.0], [1500.0, 240.0],
        [1460.0, 900.0], [340.0, 860.0],
    ], dtype=np.float64)
    dst = np.array([
        [250.0, 0.0], [0.0, 0.0],
        [0.0, 175.0], [250.0, 175.0],
    ], dtype=np.float64)
    H = cv2.getPerspectiveTransform(src.astype(np.float32), dst.astype(np.float32))
    mapped = cv2.perspectiveTransform(src.reshape(-1, 1, 2), H).reshape(-1, 2)
    rmse = float(np.sqrt(np.mean(np.sum((mapped - dst) ** 2, axis=1))))
    return PlaneCalibration(
        H=H, camera_matrix=K, dist_coeffs=D,
        image_size=(1920, 1080), calib_path="camera_calib.npz",
        backend="ffmpeg-dshow", front_direction=front_direction,
        down_direction=down_direction, board_cols=9, board_rows=6,
        square_size_mm=25.0, rmse_mm=rmse,
        src_points_undistorted_px=src, dst_points_mm=dst,
    )


def synthetic_distorted_corners() -> np.ndarray:
    cols, rows, square = 9, 6, 25.0
    dst = np.array(
        [[(cols - 1 - col) * square, row * square]
         for row in range(rows) for col in range(cols)], dtype=np.float64)
    image_quad = np.array(
        [[300.0, 200.0], [1500.0, 240.0],
         [1460.0, 900.0], [340.0, 860.0]], dtype=np.float32)
    plane_quad = np.array(
        [[200.0, 0.0], [0.0, 0.0], [0.0, 125.0], [200.0, 125.0]],
        dtype=np.float32)
    plane_to_image = cv2.getPerspectiveTransform(plane_quad, image_quad)
    undistorted = cv2.perspectiveTransform(
        dst.reshape(-1, 1, 2), plane_to_image).reshape(-1, 2)
    normalized = np.column_stack((
        (undistorted[:, 0] - K[0, 2]) / K[0, 0],
        (undistorted[:, 1] - K[1, 2]) / K[1, 1],
        np.ones(len(undistorted)),
    ))
    distorted, _ = cv2.projectPoints(
        normalized, np.zeros(3), np.zeros(3), K, D)
    return distorted.reshape(-1, 2)


class TestPlaneCalibration(unittest.TestCase):
    def test_compute_recovers_left_positive_down_positive_plane(self):
        calibration = compute_plane_calibration(
            synthetic_distorted_corners(), K, D, (1920, 1080),
            "camera_calib.npz", "ffmpeg-dshow", "left", "down", 9, 6, 25.0)
        expected = np.array(
            [[(8 - col) * 25.0, row * 25.0]
             for row in range(6) for col in range(9)])
        mapped = calibration.map_undistorted_points(
            calibration.src_points_undistorted_px)
        np.testing.assert_allclose(mapped, expected, atol=0.05)
        self.assertLess(calibration.rmse_mm, 0.05)
        self.assertEqual(calibration.inlier_count, 54)

    def test_missing_detection_reports_zero_corners(self):
        self.assertEqual(_detected_corner_count(None), 0)

    def test_round_trip_preserves_provenance(self):
        calibration = make_synthetic_plane_calibration()
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp) / "plane_homography"
            save_plane_calibration(base, calibration)
            loaded = load_plane_calibration(base)
        self.assertEqual(loaded.front_direction, "left")
        self.assertEqual(loaded.down_direction, "down")
        self.assertEqual(loaded.image_size, (1920, 1080))
        self.assertEqual(loaded.backend, "ffmpeg-dshow")
        self.assertEqual(loaded.inlier_count, calibration.inlier_count)
        np.testing.assert_allclose(loaded.H, calibration.H)

    def test_rejects_camera_matrix_mismatch(self):
        calibration = make_synthetic_plane_calibration()
        wrong_K = calibration.camera_matrix.copy()
        wrong_K[0, 0] += 20.0
        with self.assertRaisesRegex(ValueError, "camera matrix"):
            validate_plane_calibration(
                calibration, wrong_K, calibration.dist_coeffs,
                (1920, 1080), "left", "down")

    def test_rejects_each_other_compatibility_mismatch(self):
        calibration = make_synthetic_plane_calibration()
        cases = [
            (K, D, (1280, 720), "left", "down", "image size"),
            (K, D + 0.01, (1920, 1080), "left", "down", "distortion"),
            (K, D, (1920, 1080), "right", "down", "front direction"),
            (K, D, (1920, 1080), "left", "up", "down direction"),
        ]
        for matrix, dist, size, front, down, message in cases:
            with self.subTest(message=message), self.assertRaisesRegex(ValueError, message):
                validate_plane_calibration(
                    calibration, matrix, dist, size, front, down)


if __name__ == "__main__":
    unittest.main()

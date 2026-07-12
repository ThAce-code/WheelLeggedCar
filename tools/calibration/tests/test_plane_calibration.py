from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

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
from plane_calibrate import (  # noqa: E402
    _detected_corner_count,
    _validate_capture_request,
)


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
        inlier_count=54,
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

    def test_current_mf200_calibration_requires_ffmpeg_1920x1080(self):
        with self.assertRaisesRegex(ValueError, "image size"):
            _validate_capture_request(
                (1920, 1080), "ffmpeg-dshow", (640, 480), True, "AUTO")
        with self.assertRaisesRegex(ValueError, "capture backend"):
            _validate_capture_request(
                (1920, 1080), "ffmpeg-dshow", (1920, 1080), False, "DSHOW")
        _validate_capture_request(
            (1920, 1080), "ffmpeg-dshow", (1920, 1080), True, "AUTO")

    def test_opencv_calibration_requires_matching_backend(self):
        with self.assertRaisesRegex(ValueError, "capture backend"):
            _validate_capture_request(
                (1280, 720), "MSMF", (1280, 720), False, "DSHOW")
        _validate_capture_request(
            (1280, 720), "MSMF", (1280, 720), False, "msmf")

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
                (1920, 1080), "left", "down", "ffmpeg-dshow",
                "camera_calib.npz")

    def test_rejects_each_other_compatibility_mismatch(self):
        calibration = make_synthetic_plane_calibration()
        cases = [
            (K, D, (1280, 720), "left", "down", "ffmpeg-dshow", "camera_calib.npz", "image size"),
            (K, D + 0.01, (1920, 1080), "left", "down", "ffmpeg-dshow", "camera_calib.npz", "distortion"),
            (K, D, (1920, 1080), "right", "down", "ffmpeg-dshow", "camera_calib.npz", "front direction"),
            (K, D, (1920, 1080), "left", "up", "ffmpeg-dshow", "camera_calib.npz", "down direction"),
            (K, D, (1920, 1080), "left", "down", "MSMF", "camera_calib.npz", "backend"),
            (K, D, (1920, 1080), "left", "down", "ffmpeg-dshow", "other_calib.npz", "calibration path"),
        ]
        for matrix, dist, size, front, down, backend, calib_path, message in cases:
            with self.subTest(message=message), self.assertRaisesRegex(ValueError, message):
                validate_plane_calibration(
                    calibration, matrix, dist, size, front, down,
                    backend, calib_path)

    def test_load_rejects_each_corrupt_persisted_category(self):
        cases = [
            ("H", np.full((3, 3), np.nan), None, "finite homography"),
            ("H", np.zeros((3, 3)), None, "singular homography"),
            ("src_points_undistorted_px", np.zeros((4, 3)), None, "source points"),
            ("dst_points_mm", np.zeros((4, 3)), None, "destination points"),
            ("src_points_undistorted_px", np.array([
                [np.nan, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]),
             None, "finite source points"),
            ("dst_points_mm", np.array([
                [0.0, 0.0], [np.inf, 0.0], [1.0, 1.0], [0.0, 1.0]]),
             None, "finite destination points"),
            (None, None, ("board_cols", 0), "board columns"),
            (None, None, ("board_rows", -1), "board rows"),
            (None, None, ("square_size_mm", 0.0), "square size"),
            (None, None, ("rmse_mm", 0.51), "stored RMSE"),
            (None, None, ("inlier_count", 42), "stored RANSAC inliers"),
            (None, None, ("inlier_count", 55), "exceed board points"),
        ]
        for array_key, array_value, scalar_change, message in cases:
            with self.subTest(message=message), tempfile.TemporaryDirectory() as tmp:
                base = Path(tmp) / "plane_homography"
                save_plane_calibration(base, make_synthetic_plane_calibration())
                if array_key is not None:
                    with np.load(base.with_suffix(".npz")) as saved:
                        arrays = {key: saved[key] for key in saved.files}
                    arrays[array_key] = array_value
                    np.savez(base.with_suffix(".npz"), **arrays)
                if scalar_change is not None:
                    import json
                    sidecar = base.with_suffix(".json")
                    values = json.loads(sidecar.read_text(encoding="utf-8"))
                    values[scalar_change[0]] = scalar_change[1]
                    sidecar.write_text(json.dumps(values), encoding="utf-8")
                with self.assertRaisesRegex(ValueError, message):
                    load_plane_calibration(base)

    def test_compute_rejects_low_ransac_inlier_count(self):
        H = np.eye(3)
        mask = np.zeros((54, 1), dtype=np.uint8)
        mask[:42] = 1
        with mock.patch("plane_calibration.cv2.findHomography", return_value=(H, mask)):
            with self.assertRaisesRegex(ValueError, "80%"):
                compute_plane_calibration(
                    synthetic_distorted_corners(), K, D, (1920, 1080),
                    "camera_calib.npz", "ffmpeg-dshow", "left", "down", 9, 6, 25.0)

    def test_compute_rejects_high_inlier_rmse(self):
        H = np.eye(3)
        mask = np.ones((54, 1), dtype=np.uint8)
        with mock.patch("plane_calibration.cv2.findHomography", return_value=(H, mask)):
            with self.assertRaisesRegex(ValueError, "RMSE"):
                compute_plane_calibration(
                    synthetic_distorted_corners(), K, D, (1920, 1080),
                    "camera_calib.npz", "ffmpeg-dshow", "left", "down", 9, 6, 25.0)


if __name__ == "__main__":
    unittest.main()

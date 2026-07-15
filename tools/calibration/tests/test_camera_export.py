#!/usr/bin/env python3
"""Tests for the MT9V03X batch-calibration export contract."""

from __future__ import annotations

import tempfile
import unittest
import inspect
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path

import cv2
import numpy as np

_CALIB_DIR = Path(__file__).resolve().parent.parent
import sys

if str(_CALIB_DIR) not in sys.path:
    sys.path.insert(0, str(_CALIB_DIR))

import camera_calibrate
from camera_utils import undistort_points


class TestMt9v03xCalibrationExport(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.root = Path(self.temp_dir.name)
        self.image_paths = []
        self.corners = []

        object_points = np.zeros((9 * 6, 3), dtype=np.float32)
        object_points[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2) * 25.0
        matrix = np.array([
            [150.0, 0.0, 94.0],
            [0.0, 150.0, 60.0],
            [0.0, 0.0, 1.0],
        ])
        for index in range(3):
            image = np.full((120, 188), 127, dtype=np.uint8)
            path = self.root / f"frame_{index:02d}.bmp"
            self.assertTrue(cv2.imwrite(str(path), image))
            self.image_paths.append(path)
            projected, _ = cv2.projectPoints(
                object_points,
                np.array([0.03 * index, -0.02 * index, 0.01 * index]),
                np.array([-30.0 + 30.0 * index, -20.0 + 12.0 * index,
                          700.0 + 35.0 * index]),
                matrix,
                None,
            )
            self.corners.append(projected.reshape(-1, 2).astype(np.float32))

    def tearDown(self) -> None:
        self.temp_dir.cleanup()

    def _detect_corners(self, _image: np.ndarray, _cols: int, _rows: int) -> np.ndarray:
        return self.corners.pop(0)

    def _require_parameters(self, function, *names: str) -> bool:
        parameters = inspect.signature(function).parameters
        for name in names:
            if name not in parameters:
                self.fail(f"{function.__name__} must accept {name}")
                return False
        return True

    def test_bmp_images_return_their_native_resolution(self) -> None:
        if not self._require_parameters(
                camera_calibrate.calibrate_from_images,
                "expected_size", "detect_fn"):
            return
        result = camera_calibrate.calibrate_from_images(
            self.image_paths,
            9,
            6,
            25.0,
            model="standard",
            expected_size=(188, 120),
            detect_fn=self._detect_corners,
        )

        self.assertEqual(result["image_size"], [188, 120])
        self.assertEqual(result["model"], "standard")

    def test_mixed_resolution_bmp_images_are_rejected(self) -> None:
        if not self._require_parameters(
                camera_calibrate.calibrate_from_images,
                "expected_size", "detect_fn"):
            return
        wrong_size = self.root / "wrong.bmp"
        self.assertTrue(cv2.imwrite(str(wrong_size), np.zeros((121, 188), dtype=np.uint8)))

        with self.assertRaisesRegex(ValueError, "same resolution"):
            camera_calibrate.calibrate_from_images(
                [self.image_paths[0], wrong_size],
                9,
                6,
                25.0,
                expected_size=None,
                detect_fn=lambda *_args: self.corners[0],
            )

    def test_model_selection_requires_five_percent_rms_improvement(self) -> None:
        selector = getattr(camera_calibrate, "select_model", None)
        self.assertTrue(callable(selector), "select_model must be available")
        self.assertEqual(selector(0.80, 0.78), "standard")
        self.assertEqual(selector(0.80, 0.74), "fisheye")

    def test_export_header_contains_resolution_crc_and_fixed_point_coefficients(self) -> None:
        exporter = getattr(camera_calibrate, "export_c_header", None)
        self.assertTrue(callable(exporter), "export_c_header must be available")
        header_path = self.root / "mt9v03x_calibration.h"
        exporter({
            "schema_version": 1,
            "model": "standard",
            "image_size": [188, 120],
            "K": [[150.0, 0.0, 94.0], [0.0, 151.0, 60.0], [0.0, 0.0, 1.0]],
            "D": [0.01, -0.02, 0.0, 0.0, 0.0],
            "rms": 0.42,
            "per_view_rms": [0.4, 0.42, 0.44],
            "board": {"cols": 9, "rows": 6, "square_mm": 25.0},
        }, header_path)

        text = header_path.read_text(encoding="utf-8")
        self.assertIn("CAMERA_CALIBRATION_WIDTH (188U)", text)
        self.assertIn("CAMERA_CALIBRATION_HEIGHT (120U)", text)
        self.assertIn("camera_calibration_crc32", text)
        self.assertIn("CAMERA_CALIBRATION_FX_Q16", text)

    def test_fisheye_undistortion_keeps_zero_distortion_pixel(self) -> None:
        if not self._require_parameters(undistort_points, "model"):
            return
        points = np.array([[94.0, 60.0]], dtype=np.float64)
        matrix = np.array([[150.0, 0.0, 94.0], [0.0, 150.0, 60.0], [0.0, 0.0, 1.0]])
        result = undistort_points(points, matrix, np.zeros(4), model="fisheye")
        np.testing.assert_allclose(result, points, atol=1e-6)

    def test_empty_bmp_glob_returns_a_readable_cli_error(self) -> None:
        original_argv = sys.argv
        output = StringIO()
        try:
            sys.argv = ["camera_calibrate.py", "--images", str(self.root / "*.bmp")]
            with redirect_stdout(output):
                try:
                    exit_code = camera_calibrate.main()
                except ValueError as exc:
                    self.fail(f"main must report calibration input errors: {exc}")
        finally:
            sys.argv = original_argv

        self.assertEqual(exit_code, 2)
        self.assertIn("[ERROR] no valid chessboard frames", output.getvalue())


if __name__ == "__main__":
    unittest.main()

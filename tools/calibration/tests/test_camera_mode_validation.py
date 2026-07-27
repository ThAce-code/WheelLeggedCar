#!/usr/bin/env python3
"""Test camera mode validation logic (offline/synthetic).

These tests use synthetic data to verify the validation report data structures,
sharpness computation, and probe-mode parsing — NOT real hardware.
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

from camera_utils import (
    CameraValidationReport,
    compute_sharpness,
    decode_fourcc,
)


class TestSharpnessComputation(unittest.TestCase):
    """Test the Laplacian variance sharpness metric."""

    def test_uniform_image_zero_sharpness(self):
        """A completely uniform image should have near-zero sharpness."""
        img = np.ones((480, 640, 3), dtype=np.uint8) * 128
        sharp = compute_sharpness(img)
        self.assertAlmostEqual(sharp, 0.0, delta=1.0,
                               msg="Uniform image sharpness should be near 0")

    def test_high_contrast_sharper_than_uniform(self):
        """An image with edges should have higher sharpness than uniform."""
        uniform = np.ones((480, 640, 3), dtype=np.uint8) * 128
        edges = np.zeros((480, 640, 3), dtype=np.uint8)
        # Add a checkerboard pattern
        for i in range(0, 480, 20):
            for j in range(0, 640, 20):
                if (i + j) % 40 == 0:
                    edges[i:i + 20, j:j + 20] = 255

        sharp_uniform = compute_sharpness(uniform)
        sharp_edges = compute_sharpness(edges)
        self.assertGreater(sharp_edges, sharp_uniform,
                           "Edge image should be sharper than uniform")

    def test_grayscale_input(self):
        """compute_sharpness accepts grayscale input."""
        gray = np.random.randint(0, 255, (480, 640), dtype=np.uint8)
        sharp = compute_sharpness(gray)
        self.assertIsInstance(sharp, float)
        self.assertTrue(np.isfinite(sharp))


class TestFOURCCDecoding(unittest.TestCase):
    """Test FOURCC code decoding."""

    def test_mjpeg_fourcc(self):
        """MJPG FOURCC should decode correctly."""
        mjpg = cv2.VideoWriter_fourcc(*"MJPG")
        result = decode_fourcc(mjpg)
        self.assertEqual(result, "MJPG")

    def test_msmf_returns_na(self):
        """MSMF backend's 0x00000016 should show N/A (not garbage)."""
        result = decode_fourcc(0x00000016)
        self.assertEqual(result, "N/A")

    def test_zero_returns_na(self):
        """Zero FOURCC should return N/A."""
        self.assertEqual(decode_fourcc(0), "N/A")


class TestValidationReport(unittest.TestCase):
    """Test CameraValidationReport data structure."""

    def test_to_dict_contains_required_fields(self):
        """Report.to_dict() should contain all required fields."""
        report = CameraValidationReport(
            camera_index=2,
            backend="MSMF",
            requested_width=1920,
            requested_height=1080,
            requested_fps=30.0,
            cap_prop_width=1920.0,
            cap_prop_height=1080.0,
            cap_prop_fps=30.0,
            actual_frame_shape=(1080, 1920, 3),
            measured_fps=29.97,
            fourcc_int=0,
            fourcc_str="N/A",
            frames_read=30,
            frames_ok=30,
            passed=True,
            sharpness_values=[100.0, 110.0, 105.0],
        )
        d = report.to_dict()
        required = [
            "camera_index", "backend", "requested_resolution", "requested_fps",
            "cap_prop_resolution", "cap_prop_fps", "actual_frame_shape",
            "measured_fps", "fourcc", "fourcc_int", "frames_read", "frames_ok",
            "passed", "sharpness_mean", "sharpness_std",
        ]
        for key in required:
            self.assertIn(key, d, f"Report dict missing key: {key}")

    def test_failed_report(self):
        """A failed report should have passed=False and error_message set."""
        report = CameraValidationReport(
            camera_index=2,
            backend="MSMF",
            requested_width=1920,
            requested_height=1080,
            requested_fps=30.0,
            cap_prop_width=0.0,
            cap_prop_height=0.0,
            cap_prop_fps=0.0,
            actual_frame_shape=(0, 0, 0),
            measured_fps=0.0,
            fourcc_int=0,
            fourcc_str="",
            frames_read=0,
            frames_ok=0,
            passed=False,
            error_message="Test failure",
        )
        self.assertFalse(report.passed)
        self.assertEqual(report.error_message, "Test failure")


if __name__ == "__main__":
    unittest.main()

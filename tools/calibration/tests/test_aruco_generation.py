#!/usr/bin/env python3
"""Test ArUco marker generation and detection.

Verifies that generated SVG markers contain the correct IDs and
that the existing markers_0-7.png is detectable.
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


class TestArUcoMarkerDetection(unittest.TestCase):
    """Test that generated ArUco markers are detectable."""

    @classmethod
    def setUpClass(cls):
        cls.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        cls.detector = cv2.aruco.ArucoDetector(cls.dictionary, params)

    def _detect_ids(self, image: np.ndarray) -> set[int]:
        """Detect ArUco marker IDs in an image."""
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return set()
        return {int(i) for i in ids.flatten()}

    def test_generate_and_detect_individual_markers(self):
        """Generate each marker ID 0-7 on a white canvas and verify detection."""
        # Place each marker on a white canvas with ample padding
        # — this matches real-world usage better than a bare marker image
        marker_size = 200
        canvas_size = 400  # 2x the marker, giving generous white border

        for marker_id in range(8):
            marker_img = cv2.aruco.generateImageMarker(
                self.dictionary, marker_id, marker_size, borderBits=1)
            # Place on white canvas centered
            canvas = np.ones((canvas_size, canvas_size), dtype=np.uint8) * 255
            offset = (canvas_size - marker_size) // 2
            canvas[offset:offset + marker_size, offset:offset + marker_size] = marker_img

            detected_ids = self._detect_ids(canvas)
            self.assertIn(marker_id, detected_ids,
                          f"Marker ID {marker_id} should be detected")
            self.assertEqual(len(detected_ids), 1,
                             f"Only one marker should be detected, got {detected_ids}")

    def test_generate_all_eight_on_one_image(self):
        """Generate all 8 markers on one image and verify all detected."""
        # Arrange 8 markers in 2 rows x 4 columns
        marker_size = 150
        spacing = 50
        cols = 4
        rows = 2
        img_w = cols * (marker_size + spacing) + spacing
        img_h = rows * (marker_size + spacing) + spacing
        composite = np.ones((img_h, img_w), dtype=np.uint8) * 255

        for idx in range(8):
            marker_img = cv2.aruco.generateImageMarker(
                self.dictionary, idx, marker_size, borderBits=1)
            row = idx // cols
            col = idx % cols
            x0 = spacing + col * (marker_size + spacing)
            y0 = spacing + row * (marker_size + spacing)
            composite[y0:y0 + marker_size, x0:x0 + marker_size] = marker_img

        detected_ids = self._detect_ids(composite)
        self.assertEqual(detected_ids, set(range(8)),
                         f"Should detect all IDs 0-7, got {sorted(detected_ids)}")

    def test_existing_markers_png(self):
        """Test detection on the existing markers_0-7.png file."""
        png_path = _CALIB_DIR / "markers" / "markers_0-7.png"
        if not png_path.exists():
            self.skipTest(f"File not found: {png_path}")

        img = cv2.imread(str(png_path))
        self.assertIsNotNone(img, f"Failed to read {png_path}")

        detected_ids = self._detect_ids(img)
        # This PNG may or may not have all 8, but should have at least some
        # Note: the existing PNG has no reliable physical print scale
        self.assertTrue(len(detected_ids) > 0,
                        f"Should detect at least one marker in {png_path}")
        # Check all detected IDs are within 0-7
        for mid in detected_ids:
            self.assertIn(mid, range(8),
                          f"Detected ID {mid} not in expected range 0-7")

        print(f"\n  markers_0-7.png: detected IDs {sorted(detected_ids)}")

    def test_no_extra_ids_in_clean_generation(self):
        """Generate clean markers; verify no extra IDs beyond expected range."""
        marker_size = 200
        composite = np.ones((600, 1200), dtype=np.uint8) * 255
        for idx in range(8):
            marker_img = cv2.aruco.generateImageMarker(
                self.dictionary, idx, marker_size, borderBits=1)
            x0 = 50 + (idx % 4) * 280
            y0 = 50 + (idx // 4) * 280
            composite[y0:y0 + marker_size, x0:x0 + marker_size] = marker_img

        detected_ids = self._detect_ids(composite)
        # Should ONLY contain 0-7
        extra = detected_ids - set(range(8))
        self.assertEqual(len(extra), 0,
                         f"No extra IDs expected beyond 0-7, got {sorted(extra)}")

    def test_generated_markers_have_four_corners(self):
        """Each detected marker should have exactly 4 corners."""
        marker_img = cv2.aruco.generateImageMarker(
            self.dictionary, 3, 200, borderBits=1)
        canvas = np.ones((400, 400), dtype=np.uint8) * 255
        offset = (400 - 200) // 2
        canvas[offset:offset + 200, offset:offset + 200] = marker_img

        corners, ids, _ = self.detector.detectMarkers(canvas)
        self.assertIsNotNone(ids, "Marker ID 3 should be detected on white canvas")
        self.assertEqual(len(corners[0][0]), 4,
                         "Each marker should have 4 corners")


if __name__ == "__main__":
    unittest.main()

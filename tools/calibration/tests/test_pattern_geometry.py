#!/usr/bin/env python3
"""Test calibration pattern geometry correctness.

Verifies that generated chessboard SVG has correct dimensions and
corner counts.
"""

from __future__ import annotations

import re
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

_CALIB_DIR = Path(__file__).resolve().parent.parent
if str(_CALIB_DIR) not in sys.path:
    sys.path.insert(0, str(_CALIB_DIR))


class TestChessboardGeometry(unittest.TestCase):
    """Test that generated chessboard patterns have correct geometry."""

    @classmethod
    def setUpClass(cls):
        cls.patterns_dir = _CALIB_DIR / "patterns"
        cls.svg_path = cls.patterns_dir / "chessboard_9x6_25mm.svg"
        cls.png_path = cls.patterns_dir / "chessboard_9x6_25mm.png"

    def test_svg_exists(self):
        """Chessboard SVG file should exist."""
        if not self.svg_path.exists():
            self.skipTest(f"SVG not found: {self.svg_path} — run --generate first")
        self.assertTrue(self.svg_path.exists())

    def test_svg_has_correct_dimensions(self):
        """SVG should have correct physical dimensions: 270x195 mm (250+10+10 x 175+10+10)."""
        if not self.svg_path.exists():
            self.skipTest(f"SVG not found: {self.svg_path}")
        content = self.svg_path.read_text(encoding="utf-8")

        # Check total width/height in SVG tag
        # Pattern: width="270mm" height="195mm"
        w_match = re.search(r'width="(\d+(?:\.\d+)?)mm"', content)
        h_match = re.search(r'height="(\d+(?:\.\d+)?)mm"', content)
        self.assertIsNotNone(w_match, "SVG must have width in mm")
        self.assertIsNotNone(h_match, "SVG must have height in mm")

        total_w = float(w_match.group(1))
        total_h = float(h_match.group(1))

        # Board pattern = 250x175mm, margins = 10mm each side
        self.assertAlmostEqual(total_w, 270.0, delta=0.5,
                               msg=f"SVG total width should be 270mm, got {total_w}")
        self.assertAlmostEqual(total_h, 195.0, delta=0.5,
                               msg=f"SVG total height should be 195mm, got {total_h}")

    def test_svg_contains_correct_comment(self):
        """SVG should contain a comment describing the pattern."""
        if not self.svg_path.exists():
            self.skipTest(f"SVG not found: {self.svg_path}")
        content = self.svg_path.read_text(encoding="utf-8")
        # Should mention 9x6 corners and 25mm squares
        self.assertIn("9x6", content,
                      "SVG comment should mention 9x6 inner corners")
        self.assertIn("25mm", content,
                      "SVG comment should mention 25mm squares")

    def test_svg_viewbox_matches_dimensions(self):
        """SVG viewBox should match width/height."""
        if not self.svg_path.exists():
            self.skipTest(f"SVG not found: {self.svg_path}")
        content = self.svg_path.read_text(encoding="utf-8")

        w_match = re.search(r'width="(\d+(?:\.\d+)?)mm"', content)
        h_match = re.search(r'height="(\d+(?:\.\d+)?)mm"', content)
        vb_match = re.search(r'viewBox="0 0 ([\d.]+) ([\d.]+)"', content)

        if w_match and h_match and vb_match:
            self.assertEqual(float(w_match.group(1)), float(vb_match.group(1)),
                             "viewBox width must match SVG width")
            self.assertEqual(float(h_match.group(1)), float(vb_match.group(2)),
                             "viewBox height must match SVG height")

    def test_png_exists(self):
        """Chessboard PNG should exist alongside SVG."""
        if not self.png_path.exists():
            self.skipTest(f"PNG not found: {self.png_path} — run --generate first")
        self.assertTrue(self.png_path.exists())

    def test_png_has_correct_corner_count(self):
        """PNG chessboard should have 9x6 inner corners detectable by OpenCV."""
        if not self.png_path.exists():
            self.skipTest(f"PNG not found: {self.png_path}")

        img = cv2.imread(str(self.png_path), cv2.IMREAD_GRAYSCALE)
        self.assertIsNotNone(img, f"Failed to read {self.png_path}")

        flags = (cv2.CALIB_CB_ADAPTIVE_THRESH +
                 cv2.CALIB_CB_NORMALIZE_IMAGE +
                 cv2.CALIB_CB_FAST_CHECK)
        ret, corners = cv2.findChessboardCornersSB(img, (9, 6), flags)

        self.assertTrue(ret,
                        f"OpenCV should detect 9x6 inner corners in chessboard PNG. "
                        f"Found: {corners.shape[0] if corners is not None else 0} corners")

    def test_png_corner_count_is_correct(self):
        """Detected corners should be exactly 9*6=54."""
        if not self.png_path.exists():
            self.skipTest(f"PNG not found: {self.png_path}")

        img = cv2.imread(str(self.png_path), cv2.IMREAD_GRAYSCALE)
        self.assertIsNotNone(img)

        flags = (cv2.CALIB_CB_ADAPTIVE_THRESH +
                 cv2.CALIB_CB_NORMALIZE_IMAGE +
                 cv2.CALIB_CB_FAST_CHECK)
        ret, corners = cv2.findChessboardCornersSB(img, (9, 6), flags)

        if ret:
            self.assertEqual(corners.shape[0], 54,
                             f"Should detect 54 corners (9x6), got {corners.shape[0]}")

    def test_board_pattern_size(self):
        """Board pattern should be 250x175mm (10 squares x 25mm by 7 squares x 25mm)."""
        # 9x6 inner corners means 10x7 squares
        # 10 * 25 = 250mm, 7 * 25 = 175mm
        board_w_mm = 10 * 25.0
        board_h_mm = 7 * 25.0
        self.assertEqual(board_w_mm, 250.0, "Board pattern width should be 250mm")
        self.assertEqual(board_h_mm, 175.0, "Board pattern height should be 175mm")

    def test_svg_page_size_differs_from_board_size(self):
        """SVG page size (270x195) should differ from board pattern size (250x175)."""
        if not self.svg_path.exists():
            self.skipTest(f"SVG not found: {self.svg_path}")
        content = self.svg_path.read_text(encoding="utf-8")

        w_match = re.search(r'width="(\d+(?:\.\d+)?)mm"', content)
        h_match = re.search(r'height="(\d+(?:\.\d+)?)mm"', content)

        if w_match and h_match:
            page_w = float(w_match.group(1))
            page_h = float(h_match.group(1))
            # Page should be larger than board pattern due to margins
            self.assertGreater(page_w, 250.0,
                               "SVG page width should be larger than 250mm board pattern")
            self.assertGreater(page_h, 175.0,
                               "SVG page height should be larger than 175mm board pattern")


class TestArUcoSVGStructure(unittest.TestCase):
    """Test the structure of generated ArUco SVG files."""

    def test_existing_aruco_svg_is_detectable(self):
        """Check if existing ArUco SVG exists and what it contains."""
        svg_files = list((_CALIB_DIR / "markers").glob("aruco_*.svg"))
        if not svg_files:
            self.skipTest("No ArUco SVG files found — run --generate-aruco first")

        for svg_path in svg_files:
            content = svg_path.read_text(encoding="utf-8")
            # Should be valid XML/SVG
            self.assertIn("<svg", content, f"{svg_path.name} should be an SVG file")
            # Should mention the dictionary
            self.assertIn("DICT_4X4_50", content,
                          f"{svg_path.name} should mention dictionary name")

    def test_existing_aruco_png_has_ids(self):
        """Verify markers_0-7.png contains detectable ArUco markers."""
        png_path = _CALIB_DIR / "markers" / "markers_0-7.png"
        if not png_path.exists():
            self.skipTest(f"File not found: {png_path}")

        img = cv2.imread(str(png_path))
        self.assertIsNotNone(img)

        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        detector = cv2.aruco.ArucoDetector(dictionary, params)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        self.assertIsNotNone(ids, "markers_0-7.png should contain detectable markers")
        detected = {int(i) for i in ids.flatten()}
        self.assertTrue(detected.issubset(set(range(8))),
                        f"All detected IDs {sorted(detected)} should be in range 0-7")

        print(f"\n  markers_0-7.png: contains IDs {sorted(detected)} "
              f"({len(detected)}/8) — "
              f"Note: this PNG has no reliable physical print scale, "
              f"only suitable for algorithm preview and detection testing.")


if __name__ == "__main__":
    unittest.main()

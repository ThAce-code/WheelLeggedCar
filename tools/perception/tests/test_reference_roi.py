#!/usr/bin/env python3
"""Reference tests for discovery and tracking ROIs."""

from __future__ import annotations

import unittest

import numpy as np

try:
    from tools.perception import reference_geometry
except ImportError:
    reference_geometry = None


class TestReferenceRoi(unittest.TestCase):
    def setUp(self) -> None:
        self.assertIsNotNone(reference_geometry, "reference_geometry must be available")
        self.horizon = np.full(188, 60.0)

    def test_discovery_runs_only_on_even_frames(self) -> None:
        even = reference_geometry.build_roi(20, self.horizon, [])
        odd = reference_geometry.build_roi(21, self.horizon, [])
        self.assertTrue(even.discovery_enabled)
        self.assertFalse(odd.discovery_enabled)

    def test_discovery_begins_four_pixels_below_horizon(self) -> None:
        roi = reference_geometry.build_roi(20, self.horizon, [])
        self.assertTrue(np.all(roi.discovery_top_y == 64))

    def test_tracking_window_is_clipped_above_horizon_and_vehicle_mask(self) -> None:
        roi = reference_geometry.build_roi(
            21, self.horizon, [reference_geometry.TrackWindow(20, 40, 50, 119)])
        self.assertEqual(len(roi.tracking_windows), 1)
        window = roi.tracking_windows[0]
        self.assertEqual(window.top, 64)
        self.assertEqual(window.bottom, 107)

    def test_window_fully_inside_vehicle_mask_is_removed(self) -> None:
        roi = reference_geometry.build_roi(
            21, self.horizon, [reference_geometry.TrackWindow(20, 109, 50, 119)])
        self.assertEqual(roi.tracking_windows, [])


if __name__ == "__main__":
    unittest.main()

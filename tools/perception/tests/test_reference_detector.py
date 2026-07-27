#!/usr/bin/env python3
"""Tests for the bounded grayscale cone detector reference."""

from __future__ import annotations

import unittest

import cv2
import numpy as np

from tools.perception.reference_geometry import TrackWindow, build_roi

try:
    from tools.perception import reference_detector
except ImportError:
    reference_detector = None


def cone_image() -> np.ndarray:
    image = np.full((120, 188), 50, dtype=np.uint8)
    triangle = np.array([[94, 32], [49, 105], [139, 105]], dtype=np.int32)
    cv2.fillConvexPoly(image, triangle, 220)
    cv2.rectangle(image, (70, 62), (118, 70), 255, thickness=-1)
    return image


def rectangle_image() -> np.ndarray:
    image = np.full((120, 188), 50, dtype=np.uint8)
    cv2.rectangle(image, (50, 32), (138, 105), 220, thickness=-1)
    return image


def pole_image() -> np.ndarray:
    image = np.full((120, 188), 50, dtype=np.uint8)
    cv2.rectangle(image, (89, 25), (99, 105), 220, thickness=-1)
    return image


class TestReferenceDetector(unittest.TestCase):
    def setUp(self) -> None:
        self.assertIsNotNone(reference_detector, "reference_detector must be available")
        self.horizon = np.full(188, 20.0)

    def test_tapered_cone_with_white_band_scores_above_acceptance(self) -> None:
        result = reference_detector.detect(
            cone_image(), build_roi(2, self.horizon, []))
        self.assertTrue(result.accepted)
        self.assertGreaterEqual(result.accepted[0].score, 166)
        self.assertEqual(result.accepted[0].slice_count, 5)

    def test_rectangle_and_pole_do_not_pass_cone_score(self) -> None:
        roi = build_roi(2, self.horizon, [])
        rectangle = reference_detector.detect(rectangle_image(), roi)
        pole = reference_detector.detect(pole_image(), roi)
        self.assertFalse(rectangle.accepted)
        self.assertFalse(pole.accepted)

    def test_score_components_are_bounded_and_weighted_to_256(self) -> None:
        candidate = reference_detector.detect(
            cone_image(), build_roi(2, self.horizon, [])).candidates[0]
        self.assertEqual(sum(reference_detector.SCORE_WEIGHTS.values()), 256)
        for component in candidate.components.values():
            self.assertGreaterEqual(component, 0)
            self.assertLessEqual(component, 256)
        expected = (sum(reference_detector.SCORE_WEIGHTS[name] * value
                        for name, value in candidate.components.items()) + 128) >> 8
        self.assertEqual(candidate.score, expected)

    def test_tracking_window_runs_on_odd_frame_when_discovery_is_off(self) -> None:
        roi = build_roi(3, self.horizon, [TrackWindow(40, 25, 145, 107)])
        self.assertFalse(roi.discovery_enabled)
        result = reference_detector.detect(cone_image(), roi)
        self.assertTrue(result.accepted)

    def test_saturated_and_flat_images_reject_mapping_observations(self) -> None:
        roi = build_roi(2, self.horizon, [])
        saturated = reference_detector.detect(np.full((120, 188), 255, dtype=np.uint8), roi)
        dark = reference_detector.detect(np.full((120, 188), 30, dtype=np.uint8), roi)
        self.assertFalse(saturated.quality.map_observation_allowed)
        self.assertFalse(dark.quality.map_observation_allowed)
        self.assertFalse(saturated.accepted)
        self.assertFalse(dark.accepted)


if __name__ == "__main__":
    unittest.main()

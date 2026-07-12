from __future__ import annotations

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

CALIBRATION_DIR = Path(__file__).resolve().parents[1]
if str(CALIBRATION_DIR) not in sys.path:
    sys.path.insert(0, str(CALIBRATION_DIR))

from cross_circle_detector import CrossCircleDetector  # noqa: E402


def draw_cross_circle(image, center, diameter):
    cx, cy = (int(round(center[0])), int(round(center[1])))
    radius = int(round(diameter / 2.0))
    thickness = max(2, int(round(diameter * 0.08)))
    arm = int(round(radius * 0.68))
    cv2.circle(image, (cx, cy), radius, (0, 0, 0), thickness,
               lineType=cv2.LINE_AA)
    cv2.line(image, (cx - arm, cy), (cx + arm, cy),
             (0, 0, 0), thickness, cv2.LINE_AA)
    cv2.line(image, (cx, cy - arm), (cx, cy + arm),
             (0, 0, 0), thickness, cv2.LINE_AA)


def render_pair(origin=(400.0, 280.0), wheel=(720.0, 600.0),
                origin_diameter=70, wheel_diameter=50, angle_deg=4.0,
                blur_sigma=0.8, scale=1.0, perspective=False,
                gradient=False, clutter=False):
    image = np.full((900, 1200, 3), 255, dtype=np.uint8)
    draw_cross_circle(image, origin, origin_diameter)
    draw_cross_circle(image, wheel, wheel_diameter)
    matrix = cv2.getRotationMatrix2D((600, 450), angle_deg, scale)
    image = cv2.warpAffine(
        image, matrix, (1200, 900), borderValue=(255, 255, 255))
    homography = np.vstack((matrix, (0.0, 0.0, 1.0)))
    if perspective:
        src = np.float32([[0, 0], [1199, 0], [1199, 899], [0, 899]])
        dst = np.float32([[18, 12], [1175, 3], [1190, 885], [8, 896]])
        warp = cv2.getPerspectiveTransform(src, dst)
        image = cv2.warpPerspective(
            image, warp, (1200, 900), borderValue=(255, 255, 255))
        homography = warp @ homography
    points = np.array([[*origin, 1.0], [*wheel, 1.0]], dtype=np.float64).T
    mapped = homography @ points
    mapped = (mapped[:2] / mapped[2]).T
    expected = {"origin": mapped[0], "wheel": mapped[1]}
    if gradient:
        gain = np.linspace(0.58, 1.0, image.shape[1], dtype=np.float32)
        image = np.clip(image.astype(np.float32) * gain[None, :, None],
                        0, 255).astype(np.uint8)
    if clutter:
        cv2.rectangle(image, (80, 80), (190, 105), (0, 0, 0), 3)
        cv2.line(image, (870, 100), (1100, 170), (0, 0, 0), 5)
        cv2.circle(image, (980, 700), 28, (0, 0, 0), -1)
    if blur_sigma > 0:
        image = cv2.GaussianBlur(image, (0, 0), blur_sigma)
    return image, expected


def render_one():
    image = np.full((900, 1200, 3), 255, dtype=np.uint8)
    draw_cross_circle(image, (400, 300), 70)
    return image


def render_three():
    image, _ = render_pair(angle_deg=0, blur_sigma=0)
    draw_cross_circle(image, (950, 450), 60)
    return image


class TestCrossCircleDetector(unittest.TestCase):
    def setUp(self):
        self.detector = CrossCircleDetector()

    def assert_centers(self, roles, expected):
        self.assertEqual(roles.status, "VALID")
        np.testing.assert_allclose(roles.origin.center, expected["origin"], atol=0.5)
        np.testing.assert_allclose(roles.wheel.center, expected["wheel"], atol=0.5)

    def test_detects_subpixel_centers_under_rotation_and_blur(self):
        image, expected = render_pair(
            origin=(400.3, 280.6), wheel=(720.4, 600.2))
        self.assert_centers(self.detector.update(image), expected)

    def test_detects_across_scale(self):
        image, expected = render_pair(scale=1.35, angle_deg=-7.0)
        self.assert_centers(self.detector.update(image), expected)

    def test_detects_under_perspective_gradient_blur_and_clutter(self):
        image, expected = render_pair(
            angle_deg=6.0, perspective=True, gradient=True,
            blur_sigma=1.2, clutter=True)
        self.assert_centers(self.detector.update(image), expected)

    def test_assigns_larger_marker_to_origin(self):
        roles = self.detector.update(render_pair()[0])
        self.assertGreater(roles.origin.diameter_px, roles.wheel.diameter_px)

    def test_rejects_invalid_candidate_counts(self):
        self.assertEqual(self.detector.update(render_one()).status, "MISSING")
        self.detector.reset()
        self.assertEqual(self.detector.update(render_three()).status, "AMBIGUOUS")

    def test_rejects_wrong_size_ratio(self):
        image, _ = render_pair(origin_diameter=60, wheel_diameter=55)
        self.assertEqual(self.detector.update(image).status, "AMBIGUOUS")

    def test_lock_does_not_silently_swap_roles(self):
        self.assertEqual(self.detector.update(render_pair()[0]).status, "VALID")
        moved, _ = render_pair(origin=(400, 280), wheel=(720, 600),
                               origin_diameter=50, wheel_diameter=70,
                               angle_deg=4.0)
        roles = self.detector.update(moved)
        self.assertEqual(roles.status, "VALID")
        self.assertLess(roles.origin.diameter_px, roles.wheel.diameter_px)

    def test_ambiguous_frame_does_not_clear_role_lock(self):
        self.assertEqual(self.detector.update(render_pair()[0]).status, "VALID")
        self.assertEqual(self.detector.update(render_three()).status, "AMBIGUOUS")
        self.assertTrue(self.detector.locked)

    def test_reset_clears_role_lock(self):
        self.assertEqual(self.detector.update(render_pair()[0]).status, "VALID")
        self.detector.reset()
        self.assertFalse(self.detector.locked)


if __name__ == "__main__":
    unittest.main()

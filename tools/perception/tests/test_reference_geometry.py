#!/usr/bin/env python3
"""Reference tests for IMU horizon and pixel-to-ground projection."""

from __future__ import annotations

import math
import unittest

import numpy as np

try:
    from tools.perception import reference_geometry
except ImportError:
    reference_geometry = None


class TestReferenceGeometry(unittest.TestCase):
    def setUp(self) -> None:
        self.assertIsNotNone(reference_geometry, "reference_geometry must be available")
        self.calibration = reference_geometry.CameraCalibration(
            width=188,
            height=120,
            matrix=np.array([[150.0, 0.0, 94.0],
                             [0.0, 150.0, 60.0],
                             [0.0, 0.0, 1.0]]),
            distortion=np.zeros(5),
            model="standard",
        )

    @staticmethod
    def pose(**changes: float):
        values = {
            "x_m": 0.0,
            "y_m": 0.0,
            "yaw_rad": 0.0,
            "roll_rad": 0.0,
            "pitch_rad": 0.0,
            "camera_height_m": 0.25,
        }
        values.update(changes)
        return reference_geometry.VehiclePose(**values)

    def test_level_horizon_is_horizontal_at_principal_row(self) -> None:
        curve = reference_geometry.horizon_curve(self.calibration, self.pose())
        self.assertTrue(np.allclose(curve, 60.0, atol=1e-4))

    def test_positive_roll_lowers_the_left_horizon_endpoint(self) -> None:
        curve = reference_geometry.horizon_curve(
            self.calibration, self.pose(roll_rad=math.radians(10.0)))
        self.assertGreater(curve[0], curve[-1])

    def test_nose_up_pitch_moves_horizon_downward(self) -> None:
        level = reference_geometry.horizon_curve(self.calibration, self.pose())
        nose_up = reference_geometry.horizon_curve(
            self.calibration, self.pose(pitch_rad=math.radians(8.0)))
        self.assertGreater(float(np.mean(nose_up)), float(np.mean(level)))

    def test_bottom_pixel_projects_to_ground_in_front_of_vehicle(self) -> None:
        result = reference_geometry.project_pixel_to_ground(
            self.calibration, self.pose(), (94.0, 100.0))
        self.assertTrue(result.valid)
        self.assertGreater(result.x_m, 0.0)
        self.assertAlmostEqual(result.y_m, 0.0, places=5)

    def test_center_horizon_ray_is_rejected_as_near_parallel(self) -> None:
        result = reference_geometry.project_pixel_to_ground(
            self.calibration, self.pose(), (94.0, 60.0))
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "near_parallel")

    def test_dynamic_height_changes_range_without_changing_bearing(self) -> None:
        pixel = (130.0, 100.0)
        low = reference_geometry.project_pixel_to_ground(
            self.calibration, self.pose(camera_height_m=0.20), pixel)
        high = reference_geometry.project_pixel_to_ground(
            self.calibration, self.pose(camera_height_m=0.30), pixel)
        self.assertTrue(low.valid)
        self.assertTrue(high.valid)
        self.assertGreater(high.range_m, low.range_m)
        self.assertAlmostEqual(high.bearing_rad, low.bearing_rad, places=5)


if __name__ == "__main__":
    unittest.main()

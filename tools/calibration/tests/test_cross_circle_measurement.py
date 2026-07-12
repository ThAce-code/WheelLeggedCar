from __future__ import annotations

import sys
import unittest
from collections import deque
from pathlib import Path

import numpy as np

CALIBRATION_DIR = Path(__file__).resolve().parents[1]
if str(CALIBRATION_DIR) not in sys.path:
    sys.path.insert(0, str(CALIBRATION_DIR))

from cross_circle_detector import (  # noqa: E402
    CrossCircleCandidate,
    CrossCircleMeasurementTracker,
    CrossCircleRoles,
)
from plane_calibration import PlaneCalibration  # noqa: E402


class FakeDetector:
    def __init__(self, frames):
        self.frames = deque(frames)
        self.reset_count = 0

    def update(self, frame):
        del frame
        return self.frames.popleft()

    def reset(self):
        self.reset_count += 1


def candidate(center, diameter, confidence=0.9):
    return CrossCircleCandidate(
        center=np.asarray(center, dtype=np.float64),
        diameter_px=float(diameter), ellipse=None, circularity=0.95,
        ring_score=0.9, horizontal_score=0.9, vertical_score=0.9,
        center_error_px=0.1, confidence=confidence)


def roles(origin=(0, 0), wheel=(0, 0), status="VALID",
          origin_confidence=0.9, wheel_confidence=0.9):
    return CrossCircleRoles(
        origin=candidate(origin, 70, origin_confidence),
        wheel=candidate(wheel, 50, wheel_confidence), status=status)


def ambiguous_roles():
    return CrossCircleRoles(origin=None, wheel=None, status="AMBIGUOUS")


def jump_roles():
    return roles(origin=(0, 0), wheel=(1000, 1000))


def blank_frame():
    return np.zeros((8, 8, 3), dtype=np.uint8)


def make_tracker(frames, jump_threshold_mm=20.0):
    plane = PlaneCalibration(
        H=np.diag([-1.0, 1.0, 1.0]),
        camera_matrix=np.eye(3), dist_coeffs=np.zeros(4),
        image_size=(1920, 1080), calib_path="camera_calib.npz",
        backend="test", front_direction="left", down_direction="down",
        board_cols=9, board_rows=6, square_size_mm=25.0, rmse_mm=0.0,
        src_points_undistorted_px=np.zeros((4, 2)),
        dst_points_mm=np.zeros((4, 2)))
    detector = FakeDetector(frames)
    tracker = CrossCircleMeasurementTracker(
        detector, plane, np.eye(3), np.zeros(4), jump_threshold_mm)
    return tracker, detector


class TestCrossCircleMeasurement(unittest.TestCase):
    def test_relative_axes_are_left_positive_and_down_positive(self):
        tracker, _ = make_tracker(
            frames=[roles(origin=(600, 400), wheel=(500, 520))] * 15)
        for _ in range(15):
            tracker.process(blank_frame())
        captured = tracker.capture()
        self.assertAlmostEqual(captured.x_mm, 100.0, places=6)
        self.assertAlmostEqual(captured.y_mm, 120.0, places=6)

    def test_capture_requires_fifteen_valid_frames(self):
        tracker, _ = make_tracker(frames=[roles()] * 14)
        for _ in range(14):
            tracker.process(blank_frame())
        self.assertIsNone(tracker.capture())

    def test_live_value_is_median_of_latest_five(self):
        sequence = [roles(origin=(0, 0), wheel=(-x, 0))
                    for x in [10, 11, 100, 12, 13]]
        tracker, _ = make_tracker(frames=sequence, jump_threshold_mm=200.0)
        for _ in range(5):
            tracker.process(blank_frame())
        self.assertEqual(tracker.live_measurement().x_mm, 12.0)

    def test_invalid_and_jump_frames_do_not_enter_history(self):
        tracker, _ = make_tracker(
            frames=[roles(), ambiguous_roles(), jump_roles()])
        for _ in range(3):
            tracker.process(blank_frame())
        self.assertEqual(tracker.valid_frame_count, 1)

    def test_jump_rejection_uses_each_component(self):
        frames = [roles(origin=(0, 0), wheel=(-10, 10))] * 5
        frames += [roles(origin=(0, 0), wheel=(-31, 10)),
                   roles(origin=(0, 0), wheel=(-10, 31))]
        tracker, _ = make_tracker(frames)
        for _ in frames:
            tracker.process(blank_frame())
        self.assertEqual(tracker.valid_frame_count, 5)

    def test_trace_fields_confidence_and_status_are_preserved(self):
        tracker, _ = make_tracker(frames=[roles(
            origin=(600, 400), wheel=(500, 520),
            origin_confidence=0.82, wheel_confidence=0.74)])
        tracker.process(blank_frame())
        measurement = tracker.live_measurement()
        self.assertEqual(measurement.origin_u_px, 600.0)
        self.assertEqual(measurement.origin_v_px, 400.0)
        self.assertEqual(measurement.wheel_u_px, 500.0)
        self.assertEqual(measurement.wheel_v_px, 520.0)
        self.assertEqual(measurement.confidence, 0.74)
        self.assertEqual(measurement.valid_frames, 1)
        self.assertEqual(measurement.status, "VALID")

    def test_capture_uses_component_medians_for_all_trace_fields(self):
        frames = [roles(
            origin=(600 + i, 400 + i), wheel=(500 + i, 520 + i),
            origin_confidence=0.95, wheel_confidence=0.60 + i * 0.01)
                  for i in range(15)]
        tracker, _ = make_tracker(frames)
        for _ in frames:
            tracker.process(blank_frame())
        measurement = tracker.capture()
        self.assertEqual(measurement.origin_u_px, 607.0)
        self.assertEqual(measurement.wheel_v_px, 527.0)
        self.assertAlmostEqual(measurement.confidence, 0.67)
        self.assertEqual(measurement.valid_frames, 15)

    def test_reset_clears_history_and_resets_detector(self):
        tracker, detector = make_tracker(frames=[roles()])
        tracker.process(blank_frame())
        tracker.reset()
        self.assertEqual(tracker.valid_frame_count, 0)
        self.assertIsNone(tracker.live_measurement())
        self.assertIsNone(tracker.capture())
        self.assertEqual(detector.reset_count, 1)


if __name__ == "__main__":
    unittest.main()

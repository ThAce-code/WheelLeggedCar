from __future__ import annotations

import sys
import tempfile
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np

CALIBRATION_DIR = Path(__file__).resolve().parents[1]
if str(CALIBRATION_DIR) not in sys.path:
    sys.path.insert(0, str(CALIBRATION_DIR))

from cross_circle_detector import CrossCircleMeasurement  # noqa: E402
from calibrate_with_camera import (  # noqa: E402
    DEFAULT_POSES,
    CrossCirclePoseState,
    _build_cross_circle_ik_row,
    _run_cross_circle_ik_loop,
    build_parser,
    manual_measure_cross_circle,
)


def sample_measurement(valid_frames=15):
    return CrossCircleMeasurement(
        x_mm=25.5, y_mm=80.25,
        origin_u_px=900.0, origin_v_px=400.0,
        wheel_u_px=760.0, wheel_v_px=620.0,
        confidence=0.91, valid_frames=valid_frames, status="VALID")


class FakeSource:
    def __init__(self, frames):
        self.frames = deque(frames)
        self.timeouts = []
        self.closed = False

    def read(self, timeout_sec):
        self.timeouts.append(timeout_sec)
        return self.frames.popleft() if self.frames else None

    def close(self):
        self.closed = True


class FakeTracker:
    def __init__(self, valid_frames=15, statuses=None, accepted=None):
        self.valid_frame_count = valid_frames
        self.statuses = deque(statuses or [])
        self.accepted = deque(accepted or [])
        self.current_sample_accepted = False
        self.capture_count = 0
        self.reset_count = 0
        self.clear_history_count = 0
        self.detector = SimpleNamespace(
            current_roles=SimpleNamespace(status="VALID", origin=None, wheel=None))

    def process(self, frame):
        del frame
        if self.statuses:
            self.detector.current_roles.status = self.statuses.popleft()
        self.current_sample_accepted = (
            self.accepted.popleft() if self.accepted else
            self.detector.current_roles.status == "VALID")
        return sample_measurement(self.valid_frame_count)

    def capture(self):
        self.capture_count += 1
        return sample_measurement() if self.valid_frame_count >= 15 else None

    def reset(self):
        self.reset_count += 1

    def clear_history(self):
        self.clear_history_count += 1
        self.valid_frame_count = 0
        self.current_sample_accepted = False


class FakeCv:
    FONT_HERSHEY_SIMPLEX = 0

    def __init__(self, keys):
        self.keys = deque(keys)
        self.destroyed = False
        self.saved = []

    def waitKey(self, delay):
        self.delay = delay
        return self.keys.popleft() if self.keys else ord("q")

    def destroyAllWindows(self):
        self.destroyed = True

    def imshow(self, *args):
        del args

    def ellipse(self, *args):
        del args

    def circle(self, *args):
        del args

    def putText(self, *args):
        del args

    def rectangle(self, *args):
        del args

    def imwrite(self, path, frame):
        del frame
        self.saved.append(path)
        return True


class TestCalibrateWithCrossCircle(unittest.TestCase):
    def test_ik_parser_accepts_cross_circle_ffmpeg(self):
        args = build_parser().parse_args([
            "--marker-type", "cross-circle", "--ffmpeg"])
        self.assertEqual(args.marker_type, "cross-circle")
        self.assertTrue(args.ffmpeg)

    def test_parser_defaults_to_aruco(self):
        self.assertEqual(build_parser().parse_args([]).marker_type, "aruco")

    def test_valid_capture_populates_existing_measurement_columns(self):
        row = _build_cross_circle_ik_row(
            sample_id=0, pose=(90, 90, 90, 90, "mid_center"),
            measurement=sample_measurement())
        self.assertEqual(row["measured_x_mm"], "25.50")
        self.assertEqual(row["measured_y_mm"], "80.25")
        self.assertEqual(row["note"], "camera_measured_cross_circle")
        self.assertEqual(row["valid_frames"], 15)
        self.assertEqual(row["marker_count"], 2)
        self.assertEqual(row["marker_ids"], "origin,wheel")

    def test_missing_capture_does_not_advance_pose(self):
        state = CrossCirclePoseState(DEFAULT_POSES)
        self.assertFalse(state.capture(None))
        self.assertEqual(state.pose_index, 0)

    def test_space_requires_current_valid_frame_and_fifteen_history_frames(self):
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        tracker = FakeTracker(valid_frames=15, statuses=["AMBIGUOUS"])
        rows = _run_cross_circle_ik_loop(
            FakeSource([frame, None]), tracker, CrossCirclePoseState(DEFAULT_POSES),
            Path("snapshots"), cv_module=FakeCv([ord(" "), ord("q")]))
        self.assertEqual(rows, [])
        self.assertEqual(tracker.capture_count, 0)

    def test_jump_rejected_valid_roles_cannot_capture_stale_history(self):
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        tracker = FakeTracker(
            valid_frames=15, statuses=["VALID"], accepted=[False])
        state = CrossCirclePoseState(DEFAULT_POSES)
        rows = _run_cross_circle_ik_loop(
            FakeSource([frame, None]), tracker, state, Path("snapshots"),
            cv_module=FakeCv([ord(" "), ord("q")]))
        self.assertEqual(rows, [])
        self.assertEqual(state.pose_index, 0)
        self.assertEqual(tracker.capture_count, 0)

    def test_space_captures_and_remove_moves_back_without_unlocking_roles(self):
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        tracker = FakeTracker()
        state = CrossCirclePoseState(DEFAULT_POSES)
        rows = _run_cross_circle_ik_loop(
            FakeSource([frame, None, None]), tracker, state, Path("snapshots"),
            cv_module=FakeCv([ord(" "), ord("r"), ord("q")]))
        self.assertEqual(rows, [])
        self.assertEqual(state.pose_index, 0)
        self.assertEqual(tracker.reset_count, 0)

    def test_next_pose_requires_fifteen_new_valid_frames(self):
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        tracker = FakeTracker(valid_frames=15)
        state = CrossCirclePoseState(DEFAULT_POSES)
        rows = _run_cross_circle_ik_loop(
            FakeSource([frame, frame, None]), tracker, state, Path("snapshots"),
            cv_module=FakeCv([ord(" "), ord(" "), ord("q")]))
        self.assertEqual(len(rows), 1)
        self.assertEqual(state.pose_index, 1)
        self.assertEqual(tracker.capture_count, 1)
        self.assertEqual(tracker.clear_history_count, 1)

    def test_l_clears_tracker_lock(self):
        tracker = FakeTracker()
        _run_cross_circle_ik_loop(
            FakeSource([None, None]), tracker, CrossCirclePoseState(DEFAULT_POSES),
            Path("snapshots"), cv_module=FakeCv([ord("l"), ord("q")]))
        self.assertEqual(tracker.reset_count, 1)

    def test_setup_failure_closes_opened_source(self):
        source = FakeSource([])
        args = build_parser().parse_args(["--marker-type", "cross-circle"])
        plane = SimpleNamespace(backend="AUTO")
        with (mock.patch("calibrate_with_camera.CalibrationData.load",
                         return_value=object()),
              mock.patch("calibrate_with_camera.load_plane_calibration",
                         return_value=plane),
              mock.patch("calibrate_with_camera._validate_cross_circle_calibrations"),
              mock.patch("calibrate_with_camera.open_capture_source",
                         return_value=(source, "AUTO")),
              mock.patch("calibrate_with_camera.CrossCircleDetector",
                         side_effect=RuntimeError("setup failed"))):
            with self.assertRaisesRegex(RuntimeError, "setup failed"):
                manual_measure_cross_circle(args)
        self.assertTrue(source.closed)

    def test_ik_integration_rejects_non_design_axis_metadata_before_open(self):
        source = FakeSource([])
        args = build_parser().parse_args([
            "--marker-type", "cross-circle", "--backend", "MSMF"])
        calib = SimpleNamespace(
            camera_matrix=np.eye(3), dist_coeffs=np.zeros(4),
            image_size=(1920, 1080), backend="MSMF")
        plane = SimpleNamespace(
            backend="MSMF", front_direction="right", down_direction="down")
        with (mock.patch("calibrate_with_camera.CalibrationData.load",
                         return_value=calib),
              mock.patch("calibrate_with_camera.load_plane_calibration",
                         return_value=plane),
              mock.patch("calibrate_with_camera.open_capture_source") as opener):
            with self.assertRaisesRegex(ValueError, "direction mismatch"):
                manual_measure_cross_circle(args)
        opener.assert_not_called()


if __name__ == "__main__":
    unittest.main()

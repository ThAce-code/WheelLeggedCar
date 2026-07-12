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
from detect_markers import (  # noqa: E402
    _cross_circle_csv_row,
    _run_cross_circle_loop,
    _save_cross_circle_csv,
    _validate_cross_circle_calibrations,
    build_parser,
    interactive_measure_cross_circle,
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
    def __init__(self, valid_frames=0, statuses=None):
        self.valid_frame_count = valid_frames
        self.process_count = 0
        self.reset_count = 0
        self.capture_count = 0
        self.statuses = deque(statuses or [])
        self.detector = SimpleNamespace(
            current_roles=SimpleNamespace(
                status="VALID", origin=None, wheel=None))

    def process(self, frame):
        del frame
        self.process_count += 1
        if self.statuses:
            self.detector.current_roles.status = self.statuses.popleft()
        return sample_measurement(self.valid_frame_count) if self.valid_frame_count else None

    def capture(self):
        self.capture_count += 1
        return sample_measurement() if self.valid_frame_count >= 15 else None

    def reset(self):
        self.reset_count += 1
        self.valid_frame_count = 0


class FakeCv:
    FONT_HERSHEY_SIMPLEX = 0

    def __init__(self, keys):
        self.keys = deque(keys)
        self.wait_count = 0
        self.destroyed = False

    def waitKey(self, delay):
        self.assert_delay = delay
        self.wait_count += 1
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


class TestDetectMarkersCrossCircle(unittest.TestCase):
    def test_parser_accepts_cross_circle_ffmpeg_mode(self):
        args = build_parser().parse_args([
            "--marker-type", "cross-circle", "--ffmpeg",
            "--ffmpeg-name", "USB Camera"])
        self.assertEqual(args.marker_type, "cross-circle")
        self.assertTrue(args.ffmpeg)
        self.assertEqual(args.ffmpeg_name, "USB Camera")

    def test_parser_defaults_to_aruco(self):
        self.assertEqual(build_parser().parse_args([]).marker_type, "aruco")

    def test_cross_circle_csv_contains_traceability_fields(self):
        row = _cross_circle_csv_row(sample_measurement(), "meas_000")
        self.assertEqual(row["x_mm"], "25.50")
        self.assertIn("origin_u_px", row)
        self.assertIn("wheel_v_px", row)
        self.assertIn("confidence", row)
        self.assertEqual(row["valid_frames"], 15)

    def test_saved_csv_header_has_exact_required_order(self):
        expected = (
            "label,x_mm,y_mm,origin_u_px,origin_v_px,wheel_u_px,wheel_v_px,"
            "confidence,valid_frames,status")
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "measurements.csv"
            _save_cross_circle_csv([
                _cross_circle_csv_row(sample_measurement(), "meas_000")], output)
            header = output.read_text(encoding="utf-8").splitlines()[0]
        self.assertEqual(header, expected)

    def test_live_request_must_match_camera_calibration_resolution(self):
        class Calibration:
            camera_matrix = np.eye(3)
            dist_coeffs = np.zeros(4)
            image_size = (640, 480)

        class Plane:
            camera_matrix = np.eye(3)
            dist_coeffs = np.zeros(4)
            image_size = (640, 480)
            front_direction = "left"
            down_direction = "down"
            backend = "MSMF"
            calib_path = str(Path("camera_calib.npz").resolve())

        args = build_parser().parse_args([
            "--calib", "camera_calib.npz", "--backend", "MSMF",
            "--width", "1920", "--height", "1080"])
        with self.assertRaisesRegex(ValueError, "image size mismatch"):
            _validate_cross_circle_calibrations(Calibration(), Plane(), args)

    def test_no_frame_still_pumps_events_and_uses_short_timeout(self):
        source, tracker, cv = FakeSource([None]), FakeTracker(), FakeCv([ord("q")])
        _run_cross_circle_loop(source, tracker, None, cv_module=cv)
        self.assertEqual(source.timeouts, [0.01])
        self.assertEqual(cv.wait_count, 1)
        self.assertEqual(tracker.process_count, 0)

    def test_space_refuses_capture_until_fifteen_valid_frames(self):
        source = FakeSource([None, None])
        tracker = FakeTracker(valid_frames=14)
        messages = []
        _run_cross_circle_loop(
            source, tracker, None, cv_module=FakeCv([ord(" "), ord("q")]),
            printer=messages.append)
        self.assertEqual(tracker.capture_count, 0)
        self.assertIn("need 1 more valid frames", messages)

    def test_space_captures_at_fifteen_valid_frames(self):
        source = FakeSource([np.zeros((4, 4, 3), dtype=np.uint8), None])
        tracker = FakeTracker(valid_frames=15)
        measurements = _run_cross_circle_loop(
            source, tracker, None, cv_module=FakeCv([ord(" "), ord("q")]))
        self.assertEqual(tracker.capture_count, 1)
        self.assertEqual(measurements[0]["label"], "meas_000")

    def test_space_rejects_ambiguous_current_state_after_fifteen_frames(self):
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        tracker = FakeTracker(valid_frames=15, statuses=["AMBIGUOUS"])
        measurements = _run_cross_circle_loop(
            FakeSource([frame, None]), tracker, None,
            cv_module=FakeCv([ord(" "), ord("q")]))
        self.assertEqual(measurements, [])
        self.assertEqual(tracker.capture_count, 0)

    def test_space_rejects_when_current_iteration_has_no_frame(self):
        tracker = FakeTracker(valid_frames=15)
        measurements = _run_cross_circle_loop(
            FakeSource([None, None]), tracker, None,
            cv_module=FakeCv([ord(" "), ord("q")]))
        self.assertEqual(measurements, [])
        self.assertEqual(tracker.capture_count, 0)

    def test_reset_key_resets_tracker(self):
        tracker = FakeTracker(valid_frames=10)
        _run_cross_circle_loop(
            FakeSource([None, None]), tracker, None,
            cv_module=FakeCv([ord("r"), ord("q")]))
        self.assertEqual(tracker.reset_count, 1)

    def test_quit_always_closes_source_and_windows(self):
        source, cv = FakeSource([None]), FakeCv([ord("q")])
        _run_cross_circle_loop(source, FakeTracker(), None, cv_module=cv)
        self.assertTrue(source.closed)
        self.assertTrue(cv.destroyed)

    def test_camera_backend_mismatch_is_rejected_before_open(self):
        calib = SimpleNamespace(
            camera_matrix=np.eye(3), dist_coeffs=np.zeros(4),
            image_size=(1920, 1080), backend="MSMF")
        plane = SimpleNamespace(backend="DSHOW")
        args = build_parser().parse_args([
            "--marker-type", "cross-circle", "--backend", "DSHOW"])
        with (mock.patch("detect_markers.CalibrationData.load", return_value=calib),
              mock.patch("detect_markers.load_plane_calibration", return_value=plane),
              mock.patch("detect_markers.open_capture_source") as opener):
            with self.assertRaisesRegex(ValueError, "camera calibration backend"):
                interactive_measure_cross_circle(args)
        opener.assert_not_called()

    def test_actual_backend_mismatch_closes_source(self):
        source = FakeSource([])
        args = build_parser().parse_args([
            "--marker-type", "cross-circle", "--backend", "MSMF"])
        plane = SimpleNamespace(backend="MSMF")
        with (mock.patch("detect_markers.CalibrationData.load", return_value=object()),
              mock.patch("detect_markers.load_plane_calibration", return_value=plane),
              mock.patch("detect_markers._validate_cross_circle_calibrations"),
              mock.patch("detect_markers.open_capture_source",
                         return_value=(source, "DSHOW"))):
            with self.assertRaisesRegex(ValueError, "opened DSHOW"):
                interactive_measure_cross_circle(args)
        self.assertTrue(source.closed)

    def test_tracker_setup_exception_closes_source(self):
        source = FakeSource([])
        args = build_parser().parse_args([
            "--marker-type", "cross-circle", "--backend", "MSMF"])
        plane = SimpleNamespace(backend="MSMF")
        with (mock.patch("detect_markers.CalibrationData.load", return_value=object()),
              mock.patch("detect_markers.load_plane_calibration", return_value=plane),
              mock.patch("detect_markers._validate_cross_circle_calibrations"),
              mock.patch("detect_markers.open_capture_source",
                         return_value=(source, "MSMF")),
              mock.patch("detect_markers.CrossCircleDetector",
                         side_effect=RuntimeError("setup failed"))):
            with self.assertRaisesRegex(RuntimeError, "setup failed"):
                interactive_measure_cross_circle(args)
        self.assertTrue(source.closed)


if __name__ == "__main__":
    unittest.main()

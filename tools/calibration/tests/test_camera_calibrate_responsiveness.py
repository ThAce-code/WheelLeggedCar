#!/usr/bin/env python3
"""Regression tests for the interactive calibration GUI event loop."""

from __future__ import annotations

import sys
import threading
import unittest
from pathlib import Path

import numpy as np

_CALIB_DIR = Path(__file__).resolve().parent.parent
if str(_CALIB_DIR) not in sys.path:
    sys.path.insert(0, str(_CALIB_DIR))

from camera_calibrate import AsyncChessboardDetector, poll_ffmpeg_gui


class TestAsyncChessboardDetector(unittest.TestCase):
    def test_slow_detection_does_not_block_gui_thread(self):
        started = threading.Event()
        release = threading.Event()
        expected_corners = np.zeros((54, 1, 2), dtype=np.float32)

        def slow_detect(frame, cols, rows):
            started.set()
            release.wait(timeout=2.0)
            return expected_corners

        detector = AsyncChessboardDetector(9, 6, detect_fn=slow_detect)
        frame = np.zeros((8, 8, 3), dtype=np.uint8)
        try:
            self.assertTrue(detector.submit(frame))
            self.assertTrue(started.wait(timeout=0.5))

            # A busy detector must reject new work immediately so the GUI
            # thread remains free to call cv2.waitKey().
            self.assertFalse(detector.submit(frame))
            self.assertIsNone(detector.poll())

            release.set()
            result = detector.wait_for_result(timeout_sec=0.5)
            self.assertIsNotNone(result)
            detected_frame, corners = result
            self.assertIs(detected_frame, frame)
            self.assertIs(corners, expected_corners)
        finally:
            release.set()
            detector.close()

    def test_camera_poll_always_pumps_window_events(self):
        calls = []

        class EmptyCamera:
            def read(self, timeout_sec):
                calls.append(("read", timeout_sec))
                return None

        def wait_key(delay_ms):
            calls.append(("waitKey", delay_ms))
            return ord("q")

        frame, key = poll_ffmpeg_gui(
            EmptyCamera(), wait_key_fn=wait_key, timeout_sec=0.01)

        self.assertIsNone(frame)
        self.assertEqual(key, ord("q"))
        self.assertEqual(calls, [("read", 0.01), ("waitKey", 1)])


if __name__ == "__main__":
    unittest.main()

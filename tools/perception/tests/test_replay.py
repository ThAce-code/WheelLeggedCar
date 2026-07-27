#!/usr/bin/env python3
"""Tests for deterministic perception BMP replay."""

from __future__ import annotations

import csv
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from tools.perception.reference_geometry import CameraCalibration, VehiclePose

try:
    from tools.perception import replay
except ImportError:
    replay = None


class TestReplay(unittest.TestCase):
    def setUp(self) -> None:
        self.assertIsNotNone(replay, "replay module must be available")
        self.temp_dir = tempfile.TemporaryDirectory()
        self.root = Path(self.temp_dir.name)
        self.frames = []
        for timestamp in (20, 3):
            path = self.root / f"frame_{timestamp}.bmp"
            self.assertTrue(cv2.imwrite(str(path), np.full((120, 188), 40, dtype=np.uint8)))
            self.frames.append(path)
        self.calibration = CameraCalibration(
            188, 120,
            np.array([[150.0, 0.0, 94.0], [0.0, 150.0, 60.0], [0.0, 0.0, 1.0]]),
            np.zeros(5), "standard")
        self.poses = {
            timestamp: VehiclePose(0.0, 0.0, 0.0, 0.0, 0.0, 0.25)
            for timestamp in (3, 20)
        }

    def tearDown(self) -> None:
        self.temp_dir.cleanup()

    def test_replay_sorts_by_numeric_frame_timestamp(self) -> None:
        output = self.root / "result.csv"
        replay.replay_frames(self.frames, self.calibration, self.poses, output)

        with output.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual([int(row["timestamp_us"]) for row in rows], [3, 20])

    def test_replay_rejects_frame_without_pose_timestamp(self) -> None:
        with self.assertRaisesRegex(ValueError, "missing pose"):
            replay.replay_frames(
                self.frames, self.calibration, {3: self.poses[3]}, self.root / "result.csv")

    def test_direct_script_help_resolves_project_imports(self) -> None:
        script = Path(__file__).resolve().parent.parent / "replay.py"
        completed = subprocess.run(
            [sys.executable, str(script), "--help"],
            capture_output=True, text=True, check=False)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("--pose-csv", completed.stdout)


if __name__ == "__main__":
    unittest.main()

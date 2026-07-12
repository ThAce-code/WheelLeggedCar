from __future__ import annotations

import argparse
import sys
import unittest
from pathlib import Path

import numpy as np

CALIB = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(CALIB))

from camera_source import add_capture_source_args, open_capture_source


class FakeCap:
    def __init__(self):
        self.frame = np.zeros((4, 6, 3), dtype=np.uint8)
        self.released = False

    def read(self):
        return True, self.frame

    def release(self):
        self.released = True


class FakeFfmpeg:
    def __init__(self, name, width, height, fps):
        self.args = (name, width, height, fps)
        self.opened = False
        self.closed = False

    def open(self):
        self.opened = True

    def read(self, timeout_sec=0.01):
        return np.ones((4, 6, 3), dtype=np.uint8)

    def close(self):
        self.closed = True


class TestCaptureSource(unittest.TestCase):
    def test_ffmpeg_source_forwards_timeout_and_closes(self):
        created = []
        source, backend = open_capture_source(
            camera_index=0, backend_label="AUTO", width=1920, height=1080,
            fps=30, use_ffmpeg=True, ffmpeg_name="USB Camera",
            ffmpeg_factory=lambda *args: created.append(FakeFfmpeg(*args)) or created[-1])
        self.assertEqual(backend, "ffmpeg-dshow")
        self.assertEqual(source.read(0.01).shape, (4, 6, 3))
        source.close()
        self.assertTrue(created[0].opened)
        self.assertTrue(created[0].closed)

    def test_opencv_source_normalizes_read_contract(self):
        cap = FakeCap()
        source, backend = open_capture_source(
            2, "MSMF", 1920, 1080, 30, False, "USB Camera",
            opencv_opener=lambda *args: (cap, "MSMF"))
        self.assertEqual(backend, "MSMF")
        self.assertIs(source.read(0.01), cap.frame)
        source.close()
        self.assertTrue(cap.released)

    def test_cli_arguments_default_to_opencv(self):
        parser = argparse.ArgumentParser()
        add_capture_source_args(parser)
        args = parser.parse_args([])
        self.assertFalse(args.ffmpeg)
        self.assertEqual(args.ffmpeg_name, "USB Camera")


if __name__ == "__main__":
    unittest.main()

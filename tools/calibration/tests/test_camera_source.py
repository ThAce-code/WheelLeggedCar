from __future__ import annotations

import argparse
import sys
import threading
import time
import unittest
from pathlib import Path

import numpy as np

CALIB = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(CALIB))

from camera_source import add_capture_source_args, open_capture_source
from ffmpeg_camera import FfmpegCamera


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
        self.timeouts = []

    def open(self):
        self.opened = True

    def read(self, timeout_sec=0.01):
        self.timeouts.append(timeout_sec)
        return np.ones((4, 6, 3), dtype=np.uint8)

    def close(self):
        self.closed = True


class FailedCap:
    def __init__(self):
        self.released = False

    def read(self):
        return False, None

    def release(self):
        self.released = True


class StalledCap:
    def __init__(self):
        self.released = threading.Event()

    def read(self):
        self.released.wait()
        return False, None

    def release(self):
        self.released.set()


class BlockingStdout:
    def __init__(self):
        self.closed = threading.Event()

    def read(self, size):
        del size
        self.closed.wait()
        return b""

    def close(self):
        self.closed.set()


class FakeProcess:
    def __init__(self):
        self.stdout = BlockingStdout()
        self.stderr = None
        self.returncode = None

    def terminate(self):
        self.returncode = 0

    def wait(self, timeout=None):
        del timeout
        return self.returncode


class TestCaptureSource(unittest.TestCase):
    def test_ffmpeg_source_forwards_timeout_and_closes(self):
        created = []
        source, backend = open_capture_source(
            camera_index=0, backend_label="AUTO", width=1920, height=1080,
            fps=30, use_ffmpeg=True, ffmpeg_name="USB Camera",
            ffmpeg_factory=lambda *args: created.append(FakeFfmpeg(*args)) or created[-1])
        self.assertEqual(backend, "ffmpeg-dshow")
        self.assertEqual(source.read(0.123).shape, (4, 6, 3))
        source.close()
        self.assertTrue(created[0].opened)
        self.assertTrue(created[0].closed)
        self.assertEqual(created[0].timeouts, [0.123])

    def test_opencv_source_normalizes_read_contract(self):
        cap = FakeCap()
        source, backend = open_capture_source(
            2, "MSMF", 1920, 1080, 30, False, "USB Camera",
            opencv_opener=lambda *args: (cap, "MSMF"))
        self.assertEqual(backend, "MSMF")
        self.assertIs(source.read(0.01), cap.frame)
        source.close()
        self.assertTrue(cap.released)

    def test_opencv_failed_reads_return_none(self):
        cap = FailedCap()
        source, _ = open_capture_source(
            0, "AUTO", 640, 480, 30, False, "USB Camera",
            opencv_opener=lambda *args: (cap, "MSMF"))
        self.assertIsNone(source.read(0.02))
        source.close()
        self.assertTrue(cap.released)

    def test_opencv_read_timeout_is_bounded_while_backend_stalls(self):
        cap = StalledCap()
        source, _ = open_capture_source(
            0, "AUTO", 640, 480, 30, False, "USB Camera",
            opencv_opener=lambda *args: (cap, "MSMF"))
        started = time.perf_counter()
        self.assertIsNone(source.read(0.02))
        self.assertLess(time.perf_counter() - started, 0.2)
        source.close()
        self.assertFalse(source._thread.is_alive())

    def test_ffmpeg_latest_frame_replaces_oldest_when_queue_is_full(self):
        camera = FfmpegCamera("USB Camera", width=1, height=1, max_queue=1)
        old = np.zeros((1, 1, 3), dtype=np.uint8)
        newest = np.ones((1, 1, 3), dtype=np.uint8)
        camera._queue.put_nowait(old)
        camera._put_latest(newest)
        self.assertIs(camera._queue.get_nowait(), newest)

    def test_ffmpeg_shutdown_sentinel_replaces_full_queue(self):
        camera = FfmpegCamera("USB Camera", width=1, height=1, max_queue=1)
        camera._queue.put_nowait(np.zeros((1, 1, 3), dtype=np.uint8))
        camera._put_latest(camera._SENTINEL)
        self.assertIs(camera._queue.get_nowait(), camera._SENTINEL)

    def test_ffmpeg_close_terminates_reader_with_full_queue(self):
        camera = FfmpegCamera("USB Camera", width=1, height=1, max_queue=1)
        camera._proc = FakeProcess()
        camera._queue.put_nowait(np.zeros((1, 1, 3), dtype=np.uint8))
        camera._reader_thread = threading.Thread(target=camera._reader_loop)
        camera._reader_thread.start()
        camera.close()
        self.assertFalse(camera._reader_thread.is_alive())

    def test_cli_arguments_default_to_opencv(self):
        parser = argparse.ArgumentParser()
        add_capture_source_args(parser)
        args = parser.parse_args([])
        self.assertFalse(args.ffmpeg)
        self.assertEqual(args.ffmpeg_name, "USB Camera")


if __name__ == "__main__":
    unittest.main()

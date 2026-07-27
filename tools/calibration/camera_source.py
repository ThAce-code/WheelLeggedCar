from __future__ import annotations

import argparse
import queue
import threading
from typing import Callable, Optional, Protocol, Tuple

import numpy as np

from camera_utils import open_camera_strict


class CaptureSource(Protocol):
    def read(self, timeout_sec: float = 0.01) -> Optional[np.ndarray]: ...
    def close(self) -> None: ...


class OpenCVCaptureSource:
    def __init__(self, cap):
        self.cap = cap
        self._queue = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._reader_loop, daemon=True,
            name="opencv-camera-reader")
        self._thread.start()

    def _reader_loop(self) -> None:
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            if not ok:
                self._stop.wait(0.01)
                continue
            try:
                self._queue.put_nowait(frame)
            except queue.Full:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self._queue.put_nowait(frame)
                except queue.Full:
                    pass
            self._stop.wait(0.001)

    def read(self, timeout_sec: float = 0.01) -> Optional[np.ndarray]:
        try:
            return self._queue.get(timeout=timeout_sec)
        except queue.Empty:
            return None

    def close(self) -> None:
        self._stop.set()
        self.cap.release()
        self._thread.join(timeout=2.0)


class FfmpegCaptureSource:
    def __init__(self, camera):
        self.camera = camera

    def read(self, timeout_sec: float = 0.01) -> Optional[np.ndarray]:
        return self.camera.read(timeout_sec=timeout_sec)

    def close(self) -> None:
        self.camera.close()


def add_capture_source_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--ffmpeg", action="store_true",
                        help="Capture through ffmpeg DirectShow")
    parser.add_argument("--ffmpeg-name", default="USB Camera",
                        help="DirectShow friendly device name")


def open_capture_source(
    camera_index: int,
    backend_label: str,
    width: int,
    height: int,
    fps: float,
    use_ffmpeg: bool,
    ffmpeg_name: str,
    opencv_opener: Callable = open_camera_strict,
    ffmpeg_factory: Optional[Callable] = None,
) -> Tuple[CaptureSource, str]:
    if use_ffmpeg:
        if ffmpeg_factory is None:
            from ffmpeg_camera import FfmpegCamera
            ffmpeg_factory = FfmpegCamera
        camera = ffmpeg_factory(
            ffmpeg_name, width, height, int(fps) if fps > 0 else 30)
        camera.open()
        return FfmpegCaptureSource(camera), "ffmpeg-dshow"
    cap, backend = opencv_opener(
        camera_index, backend_label, width, height, fps)
    return OpenCVCaptureSource(cap), backend

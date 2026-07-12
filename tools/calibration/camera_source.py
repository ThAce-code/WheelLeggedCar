from __future__ import annotations

import argparse
from typing import Callable, Optional, Protocol, Tuple

import numpy as np

from camera_utils import open_camera_strict


class CaptureSource(Protocol):
    def read(self, timeout_sec: float = 0.01) -> Optional[np.ndarray]: ...
    def close(self) -> None: ...


class OpenCVCaptureSource:
    def __init__(self, cap):
        self.cap = cap

    def read(self, timeout_sec: float = 0.01) -> Optional[np.ndarray]:
        del timeout_sec
        ok, frame = self.cap.read()
        return frame if ok else None

    def close(self) -> None:
        self.cap.release()


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

#!/usr/bin/env python3
"""Open a camera via ffmpeg subprocess when OpenCV backends cannot access it.

Useful for USB cameras that DirectShow enumerates correctly but OpenCV's
DSHOW/MSMF backends fail to open by index (common with MJPEG-only UVC devices).

The ffmpeg process decodes the stream to raw BGR24 and writes frame data
prefixed by a 4-byte little-endian frame size to stdout.  This module reads
that pipe and yields numpy frames compatible with the rest of the
calibration toolchain.

Usage (standalone test):
    python tools/calibration/ffmpeg_camera.py --list
    python tools/calibration/ffmpeg_camera.py --name "USB Camera" --width 1920 --height 1080
"""

from __future__ import annotations

import argparse
import queue
import re
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Iterator, List, Optional, Tuple

import cv2
import numpy as np


# =========================================================================
# List cameras via ffmpeg -list_devices
# =========================================================================

def list_ffmpeg_devices() -> List[Tuple[str, str]]:
    """Use ffmpeg -list_devices to enumerate DirectShow video devices.

    Returns list of (friendly_name, dshow_device_name) tuples.
    """
    cmd = [
        "ffmpeg", "-hide_banner", "-f", "dshow", "-list_devices", "true",
        "-i", "dummy",
    ]
    proc = subprocess.run(
        cmd, capture_output=True, timeout=15,
    )
    # ffmpeg writes device list to stderr.  Use latin-1 to survive ANSI
    # escape codes that trip up stricter codecs on Windows.
    output = proc.stderr.decode("latin-1") if proc.stderr else ""

    devices: List[Tuple[str, str]] = []
    for line in output.splitlines():
        # Each video device appears as:  "Device Name" (video)
        m = re.search(r'"([^"]+)"\s+\(video\)', line)
        if m:
            name = m.group(1)
            devices.append((name, name))
    return devices


# =========================================================================
# FfmpegCamera — subprocess-based capture
# =========================================================================

class FfmpegCamera:
    """Read video frames from an ffmpeg subprocess pipe.

    The subprocess runs::

        ffmpeg -f dshow -video_size WxH -framerate FPS
               -vcodec mjpeg -i video="DEVICE NAME"
               -vcodec rawvideo -pix_fmt bgr24
               -f rawvideo -

    On Windows, pipes do not support select(), so a background thread
    continuously reads raw bytes, assembles frames, and pushes them into
    a thread-safe queue.  The main thread dequeues frames with a timeout.
    """

    # Sentinel pushed when the reader thread exits.
    _SENTINEL = object()

    def __init__(
        self,
        device_name: str,
        width: int = 1920,
        height: int = 1080,
        fps: int = 30,
        pixel_format: str = "mjpeg",
        max_queue: int = 4,
    ):
        self.device_name = device_name
        self.width = width
        self.height = height
        self.fps = fps
        self.pixel_format = pixel_format
        self.frame_bytes = width * height * 3
        self._max_queue = max_queue
        self._proc: Optional[subprocess.Popen] = None
        self._frame_count: int = 0
        self._t_start: float = 0.0
        self._queue: queue.Queue = queue.Queue(maxsize=max_queue)
        self._reader_thread: Optional[threading.Thread] = None
        self._reader_error: Optional[str] = None
        self._stop = threading.Event()

    # ------------------------------------------------------------------
    # Background reader thread
    # ------------------------------------------------------------------

    def _put_latest(self, item: object) -> None:
        """Insert without blocking, dropping the oldest queued item if full."""
        try:
            self._queue.put_nowait(item)
            return
        except queue.Full:
            pass
        try:
            self._queue.get_nowait()
        except queue.Empty:
            pass
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            pass

    def _reader_loop(self) -> None:
        """Run in background thread: read raw bytes, assemble frames, enqueue."""
        try:
            stdout = self._proc.stdout  # type: ignore[union-attr]
            buf = bytearray()
            while not self._stop.is_set():
                # Read whatever chunk is available (blocking — but in a
                # dedicated thread this is fine).
                chunk = stdout.read(max(65536, self.frame_bytes - len(buf)))
                if not chunk:
                    break
                buf.extend(chunk)

                # Emit as many complete frames as we have
                while len(buf) >= self.frame_bytes:
                    frame_bytes = bytes(buf[:self.frame_bytes])
                    del buf[:self.frame_bytes]

                    frame = np.frombuffer(frame_bytes, dtype=np.uint8)
                    frame = frame.reshape(self.height, self.width, 3)
                    # Keep only recent frames without blocking the reader.
                    self._put_latest(frame.copy())
        except Exception as exc:
            self._reader_error = str(exc)
        finally:
            self._put_latest(self._SENTINEL)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def open(self) -> None:
        """Start the ffmpeg subprocess and background reader thread."""
        self._stop.clear()
        cmd = [
            "ffmpeg",
            "-hide_banner", "-loglevel", "error",
            "-f", "dshow",
            "-video_size", f"{self.width}x{self.height}",
            "-framerate", str(self.fps),
            "-vcodec", self.pixel_format,
            "-i", f'video={self.device_name}',
            "-vcodec", "rawvideo",
            "-pix_fmt", "bgr24",
            "-f", "rawvideo",
            "pipe:1",
        ]
        self._proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            stdin=subprocess.DEVNULL,
        )
        self._t_start = time.perf_counter()

        # Start the background reader
        self._reader_thread = threading.Thread(
            target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        # Read the first frame to verify the pipeline works
        first = self.read(timeout_sec=10.0)
        if first is None:
            err = self._reader_error or "timeout"
            self.close()
            raise RuntimeError(
                f"ffmpeg: failed to read first frame from '{self.device_name}'. "
                f"Check device name, resolution ({self.width}x{self.height}), "
                f"and that no other app is using the camera. "
                f"({err})"
            )

    def read(self, timeout_sec: float = 2.0) -> Optional[np.ndarray]:
        """Dequeue one BGR frame.

        Returns None on timeout, EOF, or reader error.  Thread-safe —
        callable from any thread.
        """
        try:
            item = self._queue.get(timeout=timeout_sec)
        except queue.Empty:
            return None

        if item is self._SENTINEL:
            return None

        self._frame_count += 1
        return item  # type: ignore[return-value]

    def read_with_timeout(self, timeout_sec: float = 5.0) -> Optional[np.ndarray]:
        """Read a frame, raising RuntimeError if no frame within timeout."""
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < timeout_sec:
            frame = self.read(timeout_sec=min(1.0, timeout_sec))
            if frame is not None:
                return frame
            # Check if process died
            if self._proc is not None and self._proc.poll() is not None:
                stderr_raw = self._proc.stderr.read() if self._proc.stderr else b""
                stderr = stderr_raw.decode(errors="replace")
                raise RuntimeError(
                    f"ffmpeg process exited with code {self._proc.returncode}. "
                    f"stderr: {stderr[:500]}"
                )
            if self._reader_error:
                raise RuntimeError(
                    f"Reader thread error: {self._reader_error}"
                )
            time.sleep(0.01)
        raise RuntimeError(
            f"No frame received from '{self.device_name}' within {timeout_sec}s"
        )

    def close(self) -> None:
        """Terminate the ffmpeg subprocess and join the reader thread."""
        self._stop.set()
        if self._proc is not None:
            try:
                self._proc.stdout.close()
            except Exception:
                pass
            self._proc.terminate()
            try:
                self._proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._proc.kill()
                self._proc.wait()
            self._proc = None
        if self._reader_thread is not None and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=2)

    @property
    def frame_count(self) -> int:
        return self._frame_count

    @property
    def elapsed(self) -> float:
        return time.perf_counter() - self._t_start

    @property
    def measured_fps(self) -> float:
        if self.elapsed <= 0:
            return 0.0
        return self._frame_count / self.elapsed

    def __enter__(self) -> "FfmpegCamera":
        self.open()
        return self

    def __exit__(self, *args) -> None:
        self.close()


# =========================================================================
# Quick validation
# =========================================================================

def validate_ffmpeg_camera(
    device_name: str,
    width: int = 1920,
    height: int = 1080,
    fps: int = 30,
    num_frames: int = 30,
    samples_dir: Optional[Path] = None,
) -> dict:
    """Open the camera via ffmpeg, read N frames, report results.

    Similar in spirit to camera_utils.validate_camera_mode_strict() but
    using the ffmpeg subprocess backend.
    """
    if samples_dir is None:
        samples_dir = Path(__file__).resolve().parent / "samples"
    samples_dir = Path(samples_dir)
    samples_dir.mkdir(parents=True, exist_ok=True)

    print(f"\nValidating '{device_name}' {width}x{height} @ {fps} FPS via ffmpeg...")

    cam = FfmpegCamera(device_name, width, height, fps)

    frames: list[np.ndarray] = []
    try:
        cam.open()
    except RuntimeError as exc:
        return {"passed": False, "error": str(exc), "frames_read": 0}

    t_start = time.perf_counter()
    for i in range(num_frames):
        frame = cam.read_with_timeout(timeout_sec=5.0)
        actual_h, actual_w = frame.shape[:2]
        if actual_w != width or actual_h != height:
            cam.close()
            return {
                "passed": False,
                "error": f"Frame {i+1} size mismatch: "
                         f"got {actual_w}x{actual_h}, expected {width}x{height}",
                "frames_read": i,
            }
        frames.append(frame)

    t_elapsed = time.perf_counter() - t_start
    cam.close()

    measured_fps = num_frames / t_elapsed if t_elapsed > 0 else 0.0
    actual_h, actual_w = frames[0].shape[:2]

    # Save 3 samples
    sample_paths: list[Path] = []
    safe_name = re.sub(r'[^a-zA-Z0-9_-]', '_', device_name)
    for label, idx in [("frame01", 0),
                        (f"frame{num_frames//2+1:02d}", num_frames // 2),
                        (f"frame{num_frames:02d}", num_frames - 1)]:
        sp = samples_dir / f"ffmpeg_{safe_name}_{width}x{height}_{label}.png"
        cv2.imwrite(str(sp), frames[idx])
        sample_paths.append(sp)

    result = {
        "passed": True,
        "device_name": device_name,
        "backend": "ffmpeg-dshow",
        "requested": f"{width}x{height} @ {fps} FPS",
        "actual_frame_shape": [actual_h, actual_w, 3],
        "measured_fps": round(measured_fps, 2),
        "frames_read": num_frames,
        "sample_paths": [str(p) for p in sample_paths],
    }

    return result


# =========================================================================
# Preview / snapshot
# =========================================================================

def preview_ffmpeg(device_name: str, width: int = 1920, height: int = 1080,
                   fps: int = 30) -> None:
    """Live preview window from ffmpeg camera feed."""
    cam = FfmpegCamera(device_name, width, height, fps)
    cam.open()

    snapshot_dir = Path(__file__).resolve().parent / "snapshots"
    snapshot_dir.mkdir(parents=True, exist_ok=True)

    print(f"Preview: '{device_name}' {width}x{height} @ {fps} FPS (ffmpeg)")
    print("  q/ESC=quit  s=snapshot  SPACE=print FPS")

    cv2.namedWindow("FFmpeg Camera", cv2.WINDOW_NORMAL)

    while True:
        frame = cam.read()
        if frame is None:
            time.sleep(0.05)
            continue

        h, w = frame.shape[:2]
        fps_text = f"{cam.measured_fps:.1f}"
        cv2.putText(frame, f"{w}x{h}  FPS:{fps_text}  ffmpeg",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("FFmpeg Camera", frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break
        elif key == ord("s"):
            stamp = time.strftime("%Y%m%d_%H%M%S")
            fname = snapshot_dir / f"ffmpeg_snapshot_{stamp}.png"
            cv2.imwrite(str(fname), frame)
            print(f"Snapshot: {fname}")
        elif key == ord(" "):
            print(f"FPS: {cam.measured_fps:.1f}  frames: {cam.frame_count}")

    cam.close()
    cv2.destroyAllWindows()


# =========================================================================
# Main
# =========================================================================

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Open camera via ffmpeg subprocess when OpenCV backends fail",
    )
    parser.add_argument("--list", action="store_true",
                        help="List DirectShow devices via ffmpeg and exit")
    parser.add_argument("--name", type=str, default="USB Camera",
                        help="DirectShow device friendly name (default: 'USB Camera')")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--validate", action="store_true",
                        help="Run acceptance test (30 frames) and exit")
    parser.add_argument("--preview", action="store_true",
                        help="Live preview window")
    parser.add_argument("--snapshot", type=Path, default=None,
                        help="Capture a single frame and save to this path")
    args = parser.parse_args()

    if args.list:
        devices = list_ffmpeg_devices()
        print("\nDirectShow video devices (via ffmpeg):")
        for name, dshow_name in devices:
            print(f"  '{name}'  (dshow: video={dshow_name})")
        if not devices:
            print("  (none found)")
        return 0

    if args.validate:
        result = validate_ffmpeg_camera(
            args.name, args.width, args.height, args.fps)
        print()
        print("=" * 60)
        print(f"  FFMPEG CAMERA VALIDATION — {'PASS' if result['passed'] else 'FAIL'}")
        print("=" * 60)
        for k, v in result.items():
            if k == "sample_paths":
                print(f"  Samples:")
                for p in v:
                    print(f"    {p}")
            else:
                print(f"  {k}: {v}")
        print("=" * 60)
        return 0 if result["passed"] else 1

    if args.snapshot:
        cam = FfmpegCamera(args.name, args.width, args.height, args.fps)
        cam.open()
        frame = cam.read_with_timeout()
        cv2.imwrite(str(args.snapshot), frame)
        print(f"Saved: {args.snapshot}  ({frame.shape[1]}x{frame.shape[0]})")
        cam.close()
        return 0

    # Default: preview
    preview_ffmpeg(args.name, args.width, args.height, args.fps)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Shared camera utilities for the tools/calibration/ suite.

Provides: unified CLI args, strict camera open with mode validation,
FOURCC decoding, and consistent undistortion coordinate handling.
"""

from __future__ import annotations

import argparse
import json
import os
import struct
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")
os.environ.setdefault("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS", "0")

import cv2
import numpy as np

# -- Backend name -> OpenCV constant --
BACKEND_MAP: dict[str, int] = {
    "MSMF":    cv2.CAP_MSMF,
    "DSHOW":   cv2.CAP_DSHOW,
    "V4L2":    cv2.CAP_V4L2,
    "ANY":     cv2.CAP_ANY,
    "AUTO":    cv2.CAP_ANY,
}


def resolve_backend(label: str) -> int:
    """Resolve a backend name string to an OpenCV constant. Case-insensitive."""
    upper = label.upper()
    if upper in BACKEND_MAP:
        return BACKEND_MAP[upper]
    raise ValueError(
        f"Unknown backend '{label}'. Supported: {', '.join(BACKEND_MAP)}"
    )


def add_camera_args(parser: argparse.ArgumentParser) -> None:
    """Add standard --camera, --backend, --width, --height, --fps args."""
    parser.add_argument("--camera", type=int, default=0,
                        help="Camera device index (default: 0)")
    parser.add_argument("--backend", type=str, default="AUTO",
                        help="Video capture backend: MSMF, DSHOW, AUTO (default: AUTO)")
    parser.add_argument("--width", type=int, default=1920,
                        help="Requested capture width (default: 1920)")
    parser.add_argument("--height", type=int, default=1080,
                        help="Requested capture height (default: 1080)")
    parser.add_argument("--fps", type=float, default=0.0,
                        help="Requested capture FPS (0 = do not set, default: 0)")


def open_camera_strict(
    index: int,
    backend_label: str = "AUTO",
    width: int = 1920,
    height: int = 1080,
    fps: float = 0.0,
) -> Tuple[cv2.VideoCapture, str]:
    """Open camera with specified backend and set resolution/FPS.

    Returns (cap, actual_backend_name).  **Always** raises if:
    - the camera cannot be opened
    - the camera delivers a frame with shape != (height, width, 3)

    The user must explicitly request a different resolution -- we DO NOT silently
    fall back to whatever the camera defaults to.
    """
    backends_to_try: list[tuple[int, str]]
    if backend_label.upper() == "AUTO":
        backends_to_try = [
            (cv2.CAP_MSMF, "MSMF"),
            (cv2.CAP_DSHOW, "DSHOW"),
            (0, "DEFAULT"),
        ]
    else:
        backends_to_try = [(resolve_backend(backend_label), backend_label.upper())]

    last_error = ""
    for backend_id, bname in backends_to_try:
        try:
            cap = (cv2.VideoCapture(index)
                   if backend_id == 0
                   else cv2.VideoCapture(index, backend_id))
        except Exception as exc:
            last_error = str(exc)
            continue

        if not cap.isOpened():
            cap.release()
            continue

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
        if fps > 0:
            cap.set(cv2.CAP_PROP_FPS, fps)

        # Read one frame to verify
        ret, frame = cap.read()
        if not ret:
            cap.release()
            continue

        actual_h, actual_w = frame.shape[:2]
        channels = frame.shape[2] if frame.ndim == 3 else 1

        if actual_w != width or actual_h != height:
            cap.release()
            raise RuntimeError(
                f"Camera {index} [{bname}]: requested {width}x{height}, "
                f"delivered {actual_w}x{actual_h}. "
                f"Specify --width {actual_w} --height {actual_h} to use this mode "
                f"or choose a different camera."
            )

        if channels != 3:
            cap.release()
            raise RuntimeError(
                f"Camera {index} [{bname}]: expected 3-channel BGR, "
                f"got {channels}-channel frame. Not a standard color camera."
            )

        return cap, bname

    raise RuntimeError(
        f"Cannot open camera {index} with backend={backend_label}. "
        f"Tried: {backends_to_try}. Last error: {last_error or 'timeout/no data'}"
    )


@dataclass
class CameraReport:
    """Summary of a single camera's actual operating parameters."""
    index: int
    backend: str
    width: int
    height: int
    channels: int
    fps: float = 0.0
    fourcc_int: int = 0
    fourcc_str: str = ""
    sample_path: Optional[Path] = None
    extra: dict = None  # type: ignore[assignment]

    def __post_init__(self):
        if self.extra is None:
            self.extra = {}


def inspect_camera(
    index: int,
    backend_label: str = "AUTO",
    width: int = 1920,
    height: int = 1080,
    fps: float = 0.0,
    save_sample: bool = True,
    samples_dir: Optional[Path] = None,
) -> CameraReport:
    """Open a camera, read one frame, report its actual parameters, and save a sample.

    Returns a CameraReport. **Always raises on mode mismatch** -- the caller
    gets exactly the mode they requested.
    """
    cap, actual_backend = open_camera_strict(index, backend_label, width, height, fps)

    ret, frame = cap.read()
    if not ret:
        cap.release()
        raise RuntimeError(f"Camera {index}: failed to read frame after open")

    actual_h, actual_w = frame.shape[:2]
    channels = frame.shape[2] if frame.ndim == 3 else 1
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc_int = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc_str = decode_fourcc(fourcc_int)

    sample_path: Optional[Path] = None
    if save_sample:
        if samples_dir is None:
            samples_dir = Path(__file__).resolve().parent / "samples"
        samples_dir.mkdir(parents=True, exist_ok=True)
        sample_path = samples_dir / f"camera{index}_{actual_backend}_{actual_w}x{actual_h}.png"
        cv2.imwrite(str(sample_path), frame)

    cap.release()

    return CameraReport(
        index=index,
        backend=actual_backend,
        width=actual_w,
        height=actual_h,
        channels=channels,
        fps=actual_fps,
        fourcc_int=fourcc_int,
        fourcc_str=fourcc_str,
        sample_path=sample_path,
    )


def decode_fourcc(fourcc_int: int) -> str:
    """Decode an OpenCV FOURCC integer to a 4-character string.

    Returns 'N/A' when the backend doesn't report a valid FOURCC
    (common with MSMF which uses media types instead).
    """
    if fourcc_int <= 0 or fourcc_int < 0x20202020 or fourcc_int > 0xFFFFFFFF:
        return "N/A"
    try:
        raw = struct.pack("<I", fourcc_int)
        # Check if the bytes are all printable ASCII
        if all(0x20 <= b < 0x7F for b in raw.strip(b"\x00")):
            return raw.decode("ascii").strip("\x00")
    except Exception:
        pass
    return f"0x{fourcc_int:08X}"


# =======================================================================
# Consistent undistortion helpers (requirement 8 audit)
# =======================================================================

def undistort_points(
    points_2d: np.ndarray,            # (N, 2) distorted pixel coords
    camera_matrix: np.ndarray,        # (3, 3)
    dist_coeffs: np.ndarray,          # (k1,k2,p1,p2[,k3...])
) -> np.ndarray:
    """Undistort a set of image points using the camera calibration.

    Returns (N, 2) undistorted pixel coordinates.  This is the **only**
    function that should be used to convert distorted pixel coords to
    undistorted coords -- never use cv2.undistort() on the whole image and
    then extract points, because that creates a resampling mismatch.
    """
    pts = points_2d.reshape(-1, 1, 2).astype(np.float64)
    undistorted = cv2.undistortPoints(pts, camera_matrix, dist_coeffs, P=camera_matrix)
    return undistorted.reshape(-1, 2)


def undistort_image_for_display(
    image: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> np.ndarray:
    """Undistort an image for *display only* -- NOT for measurement.

    Measurement coordinates must be computed via undistort_points() on
    the original distorted image's detected corners.
    """
    h, w = image.shape[:2]
    new_mtx, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
    return cv2.undistort(image, camera_matrix, dist_coeffs, None, new_mtx)


def compute_homography(
    src_points: np.ndarray,   # (N, 2) undistorted pixel coords
    dst_points: np.ndarray,   # (N, 2) world coords (mm)
) -> np.ndarray:
    """Compute homography H mapping undistorted pixel coords -> world coords (mm).

    Uses RANSAC for robustness against outlier corners.
    """
    H, mask = cv2.findHomography(src_points, dst_points, cv2.RANSAC, 3.0)
    if H is None:
        raise RuntimeError("Homography computation failed -- not enough inliers")
    return H


def apply_homography(H: np.ndarray, pts_undistorted: np.ndarray) -> np.ndarray:
    """Apply homography to undistorted pixel points, returning world-mm coords."""
    pts = pts_undistorted.reshape(-1, 1, 2).astype(np.float64)
    world = cv2.perspectiveTransform(pts, H)
    return world.reshape(-1, 2)


# =======================================================================
# Image sharpness metric (requirement: relative comparison only)
# =======================================================================

def compute_sharpness(image: np.ndarray) -> float:
    """Compute Laplacian variance as a relative sharpness/focus indicator.

    IMPORTANT: This metric is for RELATIVE comparison between images from
    the same or similar cameras only.  It CANNOT:
      - Determine whether a resolution is "native"
      - Declare one camera "better" than another in absolute terms
      - Replace visual inspection of sample images

    Higher values generally indicate more in-focus / sharper images, but
    the absolute number is meaningless outside a controlled comparison.

    Args:
        image: BGR or grayscale image as numpy array.

    Returns:
        Laplacian variance (float).  Higher = sharper.
    """
    if image.ndim == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    lap = cv2.Laplacian(gray, cv2.CV_64F)
    return float(lap.var())


# =======================================================================
# Strict camera mode acceptance testing (requirement 1)
# =======================================================================

@dataclass
class CameraValidationReport:
    """Result of a strict camera mode acceptance test."""
    camera_index: int
    backend: str
    requested_width: int
    requested_height: int
    requested_fps: float
    cap_prop_width: float
    cap_prop_height: float
    cap_prop_fps: float
    actual_frame_shape: Tuple[int, int, int]  # (H, W, C)
    measured_fps: float
    fourcc_int: int
    fourcc_str: str
    frames_read: int
    frames_ok: int
    passed: bool
    error_message: str = ""
    sharpness_values: List[float] = field(default_factory=list)
    sample_paths: List[Path] = field(default_factory=list)
    report_path: Optional[Path] = None

    def to_dict(self) -> dict:
        return {
            "camera_index": self.camera_index,
            "backend": self.backend,
            "requested_resolution": f"{self.requested_width}x{self.requested_height}",
            "requested_fps": self.requested_fps,
            "cap_prop_resolution": f"{self.cap_prop_width:.0f}x{self.cap_prop_height:.0f}",
            "cap_prop_fps": self.cap_prop_fps,
            "actual_frame_shape": list(self.actual_frame_shape),
            "measured_fps": round(self.measured_fps, 2),
            "fourcc": self.fourcc_str,
            "fourcc_int": f"0x{self.fourcc_int:08X}",
            "frames_read": self.frames_read,
            "frames_ok": self.frames_ok,
            "passed": self.passed,
            "error_message": self.error_message,
            "sharpness_mean": float(np.mean(self.sharpness_values)) if self.sharpness_values else 0.0,
            "sharpness_std": float(np.std(self.sharpness_values)) if self.sharpness_values else 0.0,
            "sample_paths": [str(p) for p in self.sample_paths],
        }


def validate_camera_mode_strict(
    index: int,
    backend_label: str,
    width: int,
    height: int,
    fps: float = 0.0,
    num_frames: int = 30,
    samples_dir: Optional[Path] = None,
) -> CameraValidationReport:
    """Strict camera mode acceptance test.

    Opens the camera with the EXACT backend specified (no fallback),
    sets resolution and FPS, then reads *num_frames* consecutive frames,
    checking EVERY frame's actual shape.

    Args:
        index: Camera device index.
        backend_label: Backend name (MSMF, DSHOW) — no AUTO allowed here.
        width, height: Requested resolution.
        fps: Requested FPS (0 = don't set).
        num_frames: Minimum number of frames to read (default 30).
        samples_dir: Directory for sample images and JSON report.

    Returns:
        CameraValidationReport with full acceptance data.

    Raises:
        RuntimeError: On critical failure (camera cannot be opened at all).
        The report's ``passed`` field will be False for non-critical failures.
    """
    if samples_dir is None:
        samples_dir = Path(__file__).resolve().parent / "samples"
    samples_dir = Path(samples_dir)
    samples_dir.mkdir(parents=True, exist_ok=True)

    # -- Resolve backend strictly (no AUTO fallback) --
    if backend_label.upper() == "AUTO":
        raise ValueError(
            "validate_camera_mode_strict requires an explicit backend "
            "(MSMF or DSHOW), not AUTO.  Use --backend MSMF."
        )
    backend_id = resolve_backend(backend_label)
    bname = backend_label.upper()

    # -- Build report stub for early failures --
    def _fail(msg: str) -> CameraValidationReport:
        return CameraValidationReport(
            camera_index=index,
            backend=bname,
            requested_width=width,
            requested_height=height,
            requested_fps=fps,
            cap_prop_width=0.0,
            cap_prop_height=0.0,
            cap_prop_fps=0.0,
            actual_frame_shape=(0, 0, 0),
            measured_fps=0.0,
            fourcc_int=0,
            fourcc_str="",
            frames_read=0,
            frames_ok=0,
            passed=False,
            error_message=msg,
        )

    # -- Open camera --
    cap = cv2.VideoCapture(index, backend_id)
    if not cap.isOpened():
        cap.release()
        return _fail(f"Cannot open camera {index} with backend {bname}. "
                     f"Check: is another app using the camera?")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
    if fps > 0:
        cap.set(cv2.CAP_PROP_FPS, fps)

    cap_prop_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    cap_prop_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    cap_prop_fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc_int = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc_str = decode_fourcc(fourcc_int)

    # -- Read frames with strict checking --
    frames: list[np.ndarray] = []
    sharpness_vals: list[float] = []
    first_shape: Optional[Tuple[int, int, int]] = None

    t_start = time.perf_counter()
    for i in range(num_frames):
        ret, frame = cap.read()
        if not ret or frame is None:
            cap.release()
            return _fail(
                f"Failed to read frame {i + 1}/{num_frames}. "
                f"Camera {index} [{bname}] {width}x{height}: "
                f"empty frame or read error after {len(frames)} successful frames."
            )

        actual_h, actual_w = frame.shape[:2]
        channels = frame.shape[2] if frame.ndim == 3 else 1
        current_shape = (actual_h, actual_w, channels)

        # Check resolution
        if actual_w != width or actual_h != height:
            cap.release()
            return _fail(
                f"Frame {i + 1}/{num_frames} size mismatch: "
                f"current=({actual_h},{actual_w},{channels}), "
                f"expected=({height},{width},3), "
                f"suggestion=Verify the camera truly supports {width}x{height} "
                f"with backend {bname}."
            )

        # Check shape consistency
        if first_shape is None:
            first_shape = current_shape
        elif current_shape != first_shape:
            cap.release()
            return _fail(
                f"Frame {i + 1}/{num_frames} shape changed: "
                f"current={current_shape}, first={first_shape}, "
                f"suggestion=Check camera stability, try a different USB port or cable."
            )

        frames.append(frame)
        sharpness_vals.append(compute_sharpness(frame))

    t_elapsed = time.perf_counter() - t_start
    measured_fps = num_frames / t_elapsed if t_elapsed > 0 else 0.0

    cap.release()

    # -- Save samples: first, middle, last --
    sample_paths: list[Path] = []
    if len(frames) >= 3:
        for label, idx in [("frame01", 0),
                            (f"frame{num_frames // 2 + 1:02d}", num_frames // 2),
                            (f"frame{num_frames:02d}", num_frames - 1)]:
            sp = samples_dir / f"camera{index}_{bname}_{width}x{height}_{label}.png"
            cv2.imwrite(str(sp), frames[idx])
            sample_paths.append(sp)

    # -- Build report --
    report = CameraValidationReport(
        camera_index=index,
        backend=bname,
        requested_width=width,
        requested_height=height,
        requested_fps=fps,
        cap_prop_width=cap_prop_w,
        cap_prop_height=cap_prop_h,
        cap_prop_fps=cap_prop_fps,
        actual_frame_shape=first_shape if first_shape else (0, 0, 0),
        measured_fps=measured_fps,
        fourcc_int=fourcc_int,
        fourcc_str=fourcc_str,
        frames_read=num_frames,
        frames_ok=num_frames,
        passed=True,
        sharpness_values=sharpness_vals,
        sample_paths=sample_paths,
    )

    # -- Save JSON report --
    report_name = f"camera{index}_{bname}_{width}x{height}_report.json"
    report_path = samples_dir / report_name
    report.report_path = report_path
    report_path.write_text(
        json.dumps(report.to_dict(), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    return report


def probe_camera_modes(
    index: int,
    backend_label: str,
    mode_list: List[Tuple[int, int]],
    fps: float = 0.0,
    num_frames: int = 30,
    samples_dir: Optional[Path] = None,
) -> List[CameraValidationReport]:
    """Test multiple resolution modes on a single camera.

    Each mode is tested with actual frame reads (not just property queries).
    A mode that fails does not prevent testing the remaining modes.

    Args:
        index: Camera device index.
        backend_label: Backend (MSMF or DSHOW).
        mode_list: List of (width, height) tuples to test.
        fps: Requested FPS for all modes.
        num_frames: Frames to read per mode.
        samples_dir: Directory for reports and samples.

    Returns:
        List of CameraValidationReport, one per attempted mode.
    """
    if samples_dir is None:
        samples_dir = Path(__file__).resolve().parent / "samples"
    samples_dir = Path(samples_dir)
    samples_dir.mkdir(parents=True, exist_ok=True)

    results: List[CameraValidationReport] = []
    for w, h in mode_list:
        print(f"\n  Probing {w}x{h}...")
        try:
            report = validate_camera_mode_strict(
                index, backend_label, w, h, fps, num_frames, samples_dir)
            results.append(report)
            status = "PASS" if report.passed else "FAIL"
            print(f"    {status}: {report.frames_ok}/{report.frames_read} frames "
                  f"@ {report.measured_fps:.1f} FPS, "
                  f"sharpness={np.mean(report.sharpness_values):.0f} +/- {np.std(report.sharpness_values):.0f}")
        except Exception as exc:
            print(f"    ERROR: {exc}")
            results.append(CameraValidationReport(
                camera_index=index,
                backend=backend_label.upper(),
                requested_width=w,
                requested_height=h,
                requested_fps=fps,
                cap_prop_width=0, cap_prop_height=0, cap_prop_fps=0,
                actual_frame_shape=(0, 0, 0),
                measured_fps=0, fourcc_int=0, fourcc_str="",
                frames_read=0, frames_ok=0,
                passed=False, error_message=str(exc),
            ))
    return results


def print_validation_report(report: CameraValidationReport) -> None:
    """Print a formatted camera validation report."""
    status = "PASS" if report.passed else "FAIL"
    print()
    print("=" * 72)
    print(f"  CAMERA MODE VALIDATION — [{status}]")
    print("=" * 72)
    print(f"  Camera index:       {report.camera_index}")
    print(f"  Backend:            {report.backend}")
    print(f"  Requested:          {report.requested_width}x{report.requested_height} "
          f"@ {report.requested_fps:.0f} FPS")
    print(f"  CAP_PROP reports:   {report.cap_prop_width:.0f}x{report.cap_prop_height:.0f} "
          f"@ {report.cap_prop_fps:.2f} FPS")
    print(f"  Actual frame.shape: {report.actual_frame_shape}")
    print(f"  Measured FPS:       {report.measured_fps:.2f}")
    print(f"  FOURCC:             {report.fourcc_str}")
    print(f"  Frames:             {report.frames_ok}/{report.frames_read} OK")
    if report.sharpness_values:
        print(f"  Sharpness (Laplacian var, relative only): "
              f"mean={np.mean(report.sharpness_values):.1f} "
              f"std={np.std(report.sharpness_values):.1f}")
    if report.sample_paths:
        print(f"  Samples:")
        for sp in report.sample_paths:
            print(f"    {sp}")
    if report.report_path:
        print(f"  JSON report:        {report.report_path}")
    if not report.passed:
        print(f"  ERROR: {report.error_message}")
    print("=" * 72)


def print_probe_summary(reports: List[CameraValidationReport]) -> None:
    """Print a summary table of probe-mode results."""
    print()
    print("=" * 72)
    print("  CAMERA MODE PROBE SUMMARY")
    print("=" * 72)
    print(f"  {'Mode':<16} {'Status':<6} {'Frames':<8} {'FPS':<8} "
          f"{'Sharpness':<12} {'Error'}")
    print(f"  {'-'*16} {'-'*6} {'-'*8} {'-'*8} {'-'*12} {'-'*20}")
    for r in reports:
        mode = f"{r.requested_width}x{r.requested_height}"
        status = "PASS" if r.passed else "FAIL"
        frames = f"{r.frames_ok}/{r.frames_read}"
        fps_str = f"{r.measured_fps:.1f}"
        sharp = (f"{np.mean(r.sharpness_values):.0f} +/- "
                 f"{np.std(r.sharpness_values):.0f}"
                 if r.sharpness_values else "--")
        err = r.error_message[:40] if r.error_message else ""
        print(f"  {mode:<16} {status:<6} {frames:<8} {fps_str:<8} "
              f"{sharp:<12} {err}")
    print("=" * 72)

    # Note: we do NOT auto-select any camera — the user must decide
    passed = [r for r in reports if r.passed]
    if passed:
        print(f"\n  {len(passed)}/{len(reports)} modes passed strict validation.")
        print(f"  Candidate modes (for user to evaluate):")
        for r in sorted(passed,
                        key=lambda x: np.mean(x.sharpness_values) if x.sharpness_values else 0,
                        reverse=True):
            avg_sharp = (np.mean(r.sharpness_values)
                         if r.sharpness_values else 0)
            print(f"    Camera {r.camera_index} [{r.backend}] "
                  f"{r.requested_width}x{r.requested_height} "
                  f"@ {r.measured_fps:.1f} FPS  "
                  f"sharpness(relative)={avg_sharp:.0f}")
        print(f"\n  REMINDER: Sharpness is RELATIVE only. Higher resolution != better.")
        print(f"  Evaluate sample images visually before choosing a mode.")
    else:
        print(f"\n  No modes passed strict validation.")

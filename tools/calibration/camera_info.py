#!/usr/bin/env python3
"""Inspect available cameras and report their actual operating parameters.

For each detected camera, reports: frame.shape, backend name, FOURCC code,
actual FPS, and saves a sample image.  No heuristic IR-detection based on
resolution -- reports only measured parameters.

Usage:
    python tools/calibration/camera_info.py                    # scan all, preview camera 0
    python tools/calibration/camera_info.py --list-only        # scan only, no preview
    python tools/calibration/camera_info.py --camera 2         # preview camera 2
    python tools/calibration/camera_info.py --backend MSMF     # force MSMF backend
    python tools/calibration/camera_info.py --width 1280 --height 720 --fps 30
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

from camera_utils import (
    add_camera_args,
    CameraReport,
    CameraValidationReport,
    compute_sharpness,
    decode_fourcc,
    inspect_camera,
    open_camera_strict,
    print_probe_summary,
    print_validation_report,
    probe_camera_modes,
    resolve_backend,
    validate_camera_mode_strict,
)

_CALIB_DIR = Path(__file__).resolve().parent


# =======================================================================
# Multi-camera scan
# =======================================================================

def scan_cameras(
    max_index: int = 4,
    backend_label: str = "AUTO",
    width: int = 640,
    height: int = 480,
    fps: float = 0.0,
) -> List[CameraReport]:
    """Scan camera indices 0..max_index.  Reports actual parameters for each
    camera that can be opened at the probe resolution.

    Uses a low probe resolution to maximise the chance of opening every
    attached camera regardless of its native mode list.
    """
    reports: List[CameraReport] = []

    for idx in range(max_index + 1):
        try:
            report = inspect_camera(
                idx,
                backend_label=backend_label,
                width=width,
                height=height,
                fps=fps,
                save_sample=True,
                samples_dir=_CALIB_DIR / "samples",
            )
            reports.append(report)
        except RuntimeError as exc:
            # Probe resolution may not be supported -- try a second common one
            try:
                report = inspect_camera(
                    idx,
                    backend_label=backend_label,
                    width=320,
                    height=240,
                    fps=0,
                    save_sample=True,
                    samples_dir=_CALIB_DIR / "samples",
                )
                reports.append(report)
            except RuntimeError:
                pass  # camera truly unavailable at this index

    return reports


def print_report(reports: List[CameraReport]) -> None:
    """Print a formatted scan report."""
    print()
    print("=" * 78)
    print("  CAMERA SCAN REPORT")
    print("=" * 78)
    print(f"  Cameras detected: {len(reports)}")
    print()

    for r in reports:
        print(f"  [{r.index}] Camera {r.index}")
        print(f"      Backend:       {r.backend}")
        print(f"      frame.shape:   ({r.height}, {r.width}, {r.channels})")
        print(f"      Resolution:    {r.width} x {r.height}")
        print(f"      FOURCC:        {r.fourcc_str}  (0x{r.fourcc_int:08X})")
        print(f"      FPS:           {r.fps:.2f}")
        if r.sample_path and r.sample_path.exists():
            print(f"      Sample:        {r.sample_path}")
        print()

    if not reports:
        print("  [WARN] No cameras detected.")
        print("  Check USB connection, drivers, and that no other app is using the camera.")

    print("=" * 78)


# =======================================================================
# Preview
# =======================================================================

def show_preview(
    index: int,
    backend_label: str,
    width: int,
    height: int,
    fps: float = 0.0,
    calib_path: Optional[Path] = None,
) -> None:
    """Open a live-preview window for the specified camera.

    Controls:
      q / ESC  -- quit
      s        -- save snapshot to tools/calibration/snapshots/
      c        -- toggle crosshair / rule-of-thirds grid
      u        -- toggle undistort overlay (requires --calib)
    """
    cap, actual_backend = open_camera_strict(index, backend_label, width, height, fps)
    print(f"Preview: Camera {index} [{actual_backend}] {width}x{height}")

    # Load calibration for undistort toggle
    camera_matrix: Optional[np.ndarray] = None
    dist_coeffs: Optional[np.ndarray] = None
    if calib_path and calib_path.exists():
        data = np.load(calib_path)
        camera_matrix = data["camera_matrix"]
        dist_coeffs = data["dist_coeffs"]
        print(f"Loaded calibration: fx={camera_matrix[0,0]:.1f}")

    snapshot_dir = _CALIB_DIR / "snapshots"
    snapshot_dir.mkdir(parents=True, exist_ok=True)

    show_crosshair = False
    undistort_on = camera_matrix is not None
    frame_idx = 0
    fps_timer = time.time()

    cv2.namedWindow("Camera Preview", cv2.WINDOW_NORMAL)
    print("  q/ESC=quit  s=snapshot  c=crosshair  u=undistort")

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        frame_idx += 1
        h, w = frame.shape[:2]

        if undistort_on and camera_matrix is not None and dist_coeffs is not None:
            new_mtx, _ = cv2.getOptimalNewCameraMatrix(
                camera_matrix, dist_coeffs, (w, h), 1, (w, h))
            display = cv2.undistort(frame, camera_matrix, dist_coeffs, None, new_mtx)
        else:
            display = frame.copy()

        # Crosshair overlay
        if show_crosshair:
            dh, dw = display.shape[:2]
            cv2.line(display, (dw // 2, 0), (dw // 2, dh), (0, 255, 0), 1)
            cv2.line(display, (0, dh // 2), (dw, dh // 2), (0, 255, 0), 1)
            for f in (3,):
                cv2.line(display, (dw // f, 0), (dw // f, dh), (0, 255, 0), 1, cv2.LINE_AA)
                cv2.line(display, (dw - dw // f, 0), (dw - dw // f, dh), (0, 255, 0), 1, cv2.LINE_AA)
                cv2.line(display, (0, dh // f), (dw, dh // f), (0, 255, 0), 1, cv2.LINE_AA)
                cv2.line(display, (0, dh - dh // f), (dw, dh - dh // f), (0, 255, 0), 1, cv2.LINE_AA)

        # HUD
        if frame_idx % 30 == 0:
            fps_val = 30.0 / (time.time() - fps_timer)
            fps_timer = time.time()
        else:
            fps_val = 0.0

        cv2.putText(display, f"{w}x{h}  FPS:{fps_val:.1f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if undistort_on and camera_matrix is not None:
            cv2.putText(display, "UNDISTORT", (10, dh - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow("Camera Preview", display)
        key = cv2.waitKey(1) & 0xFF

        if key in (27, ord("q")):
            break
        elif key == ord("s"):
            stamp = time.strftime("%Y%m%d_%H%M%S")
            fname = snapshot_dir / f"snapshot_{stamp}.png"
            cv2.imwrite(str(fname), frame)
            print(f"Snapshot: {fname}")
        elif key == ord("c"):
            show_crosshair = not show_crosshair
        elif key == ord("u") and camera_matrix is not None:
            undistort_on = not undistort_on

    cap.release()
    cv2.destroyAllWindows()


# =======================================================================
# Main
# =======================================================================

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Inspect camera parameters using OpenCV",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    add_camera_args(parser)
    parser.add_argument("--list-only", action="store_true",
                        help="Scan cameras and print report, then exit (no preview)")
    parser.add_argument("--validate", action="store_true",
                        help="Strict mode acceptance test: read 30+ frames, check every "
                             "frame.shape, save samples and JSON report. "
                             "Implied when --width/--height are explicitly set "
                             "with a specific --backend (not AUTO).")
    parser.add_argument("--calib", type=Path, default=None,
                        help="Path to camera_calib.npz for undistort toggle in preview")
    parser.add_argument("--scan-width", type=int, default=640,
                        help="Probe resolution width for multi-camera scan (default: 640)")
    parser.add_argument("--scan-height", type=int, default=480,
                        help="Probe resolution height for multi-camera scan (default: 480)")
    parser.add_argument("--max-index", type=int, default=4,
                        help="Highest camera index to scan (default: 4)")
    parser.add_argument("--probe-modes", type=str, default=None,
                        help="Comma-separated resolutions to probe: "
                             '"640x480,1280x720,1920x1080". '
                             'Each mode is validated with actual frame reads.')
    parser.add_argument("--num-frames", type=int, default=30,
                        help="Number of frames to read in validate/probe mode (default: 30)")
    args = parser.parse_args()

    # -- Probe-modes: batch validate multiple resolutions --
    if args.probe_modes:
        mode_list: List[Tuple[int, int]] = []
        for item in args.probe_modes.split(","):
            parts = item.strip().split("x")
            if len(parts) != 2:
                print(f"[ERROR] Invalid probe mode: '{item}' — expected WxH format")
                return 1
            mode_list.append((int(parts[0]), int(parts[1])))

        if args.backend.upper() == "AUTO":
            print("[ERROR] --probe-modes requires an explicit --backend (MSMF or DSHOW)")
            return 1

        print(f"\nProbing {len(mode_list)} modes on Camera {args.camera} "
              f"[{args.backend}] (reading {args.num_frames} frames each)...")
        reports = probe_camera_modes(
            args.camera, args.backend, mode_list, args.fps, args.num_frames)
        print_probe_summary(reports)
        return 0 if any(r.passed for r in reports) else 1

    # -- Decide whether to do strict validation or simple scan+preview --
    # Explicit --validate flag, or explicit resolution+backend (not AUTO)
    do_validate = args.validate or (
        args.backend.upper() != "AUTO"
        and (args.width != 1920 or args.height != 1080)
        and args.width > 0 and args.height > 0
    )

    if do_validate:
        # -- Strict mode acceptance test --
        if args.backend.upper() == "AUTO":
            print("[ERROR] Validation mode requires an explicit --backend (MSMF or DSHOW).")
            print("  Use: --backend MSMF (or DSHOW)")
            return 1

        print(f"\nValidating Camera {args.camera} [{args.backend}] "
              f"{args.width}x{args.height} @ {args.fps:.0f} FPS...")
        try:
            report = validate_camera_mode_strict(
                args.camera, args.backend, args.width, args.height,
                args.fps, args.num_frames)
        except ValueError as exc:
            print(f"[ERROR] {exc}")
            return 1

        print_validation_report(report)
        return 0 if report.passed else 1

    # -- Legacy: scan + preview --
    reports = scan_cameras(
        max_index=args.max_index,
        backend_label=args.backend,
        width=args.scan_width,
        height=args.scan_height,
    )
    print_report(reports)

    if args.list_only:
        return 0 if reports else 1

    # -- Preview --
    available = {r.index for r in reports}
    preview_idx = args.camera if args.camera in available else (min(available) if available else None)
    if preview_idx is None:
        print("[ERROR] No camera available for preview.")
        return 1
    if preview_idx != args.camera:
        print(f"[INFO] Camera {args.camera} not found -- using camera {preview_idx}")

    show_preview(preview_idx, args.backend, args.width, args.height, args.fps, args.calib)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

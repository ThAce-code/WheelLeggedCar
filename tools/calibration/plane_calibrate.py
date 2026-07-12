#!/usr/bin/env python3
"""Interactively calibrate an undistorted-pixel to chessboard-plane mapping."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from camera_calibrate import detect_chessboard
from camera_source import add_capture_source_args, open_capture_source
from camera_utils import add_camera_args
from plane_calibration import compute_plane_calibration, save_plane_calibration


_DIR = Path(__file__).resolve().parent
_DEFAULT_CALIB = _DIR / "camera_calib.npz"
_DEFAULT_PLANE = _DIR / "plane_homography.npz"


def _detected_corner_count(corners: np.ndarray | None) -> int:
    return 0 if corners is None else len(corners)


def _validate_capture_request(
    calibration_size: tuple[int, int],
    calibration_backend: str,
    requested_size: tuple[int, int],
    use_ffmpeg: bool,
    requested_backend: str,
) -> None:
    if requested_size != calibration_size:
        raise ValueError(
            f"image size mismatch: camera calibration is {calibration_size}, "
            f"capture requested {requested_size}")
    calibration_is_ffmpeg = calibration_backend.casefold() == "ffmpeg-dshow"
    if calibration_is_ffmpeg != use_ffmpeg:
        raise ValueError(
            f"capture backend mismatch: calibration requires {calibration_backend}")
    if (not calibration_is_ffmpeg and
            calibration_backend.casefold() != requested_backend.casefold()):
        raise ValueError(
            f"capture backend mismatch: calibration requires "
            f"{calibration_backend}, requested {requested_backend}")


def _load_camera_calibration(path: Path) -> tuple[np.ndarray, np.ndarray, tuple[int, int], str]:
    with np.load(path) as data:
        matrix = data["camera_matrix"].copy()
        dist = data["dist_coeffs"].copy()
        resolution = tuple(int(value) for value in data["resolution"])
        backend_value = data["backend"].item()
    if isinstance(backend_value, bytes):
        backend_value = backend_value.decode("utf-8")
    return matrix, dist, resolution, str(backend_value)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    add_camera_args(parser)
    parser.add_argument("--calib", type=Path, default=_DEFAULT_CALIB)
    parser.add_argument("--output", type=Path, default=_DEFAULT_PLANE)
    parser.add_argument("--cols", type=int, default=9)
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--square-size-mm", type=float, required=True)
    parser.add_argument("--front-direction", choices=["left", "right"], required=True)
    parser.add_argument("--down-direction", choices=["down", "up"], default="down")
    add_capture_source_args(parser)
    args = parser.parse_args()

    K, D, calib_size, calib_backend = _load_camera_calibration(args.calib)
    requested_size = (args.width, args.height)
    _validate_capture_request(
        calib_size, calib_backend, requested_size, args.ffmpeg, args.backend)
    source, backend = open_capture_source(
        args.camera, args.backend, args.width, args.height, args.fps,
        args.ffmpeg, args.ffmpeg_name)
    if backend.casefold() != calib_backend.casefold():
        source.close()
        raise ValueError(
            f"capture backend mismatch: opened {backend}, "
            f"calibration requires {calib_backend}")
    latest_corners = None
    window = "Plane Calibration"
    print("Align the chessboard flat in the measurement plane; c=calibrate, q=quit")
    try:
        while True:
            frame = source.read(0.01)
            if frame is not None:
                latest_corners = detect_chessboard(frame, args.cols, args.rows)
                display = frame.copy()
                if latest_corners is not None:
                    cv2.drawChessboardCorners(
                        display, (args.cols, args.rows), latest_corners, True)
                status = (
                    f"{_detected_corner_count(latest_corners)}/"
                    f"{args.cols * args.rows} corners; "
                    "keep board flat and fill the measurement plane")
                color = (0, 255, 0) if latest_corners is not None else (0, 0, 255)
                cv2.putText(display, status, (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
                cv2.imshow(window, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("c"):
                required = args.cols * args.rows
                if latest_corners is None or len(latest_corners) != required:
                    print(f"Calibration requires exactly {required} detected corners")
                    continue
                calibration = compute_plane_calibration(
                    latest_corners, K, D, requested_size, str(args.calib),
                    backend, args.front_direction, args.down_direction,
                    args.cols, args.rows, args.square_size_mm)
                save_plane_calibration(args.output, calibration)
                print(
                    f"Saved {args.output.with_suffix('.npz')} and JSON; "
                    f"RMSE={calibration.rmse_mm:.4f} mm, "
                    f"inliers={calibration.inlier_count}/{required}")
    finally:
        source.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Audited mapping from undistorted camera pixels to chessboard-plane mm."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from camera_utils import undistort_points


def _normalize_calibration_path(path: str | Path) -> str:
    return os.path.normcase(str(Path(path).resolve()))


@dataclass
class PlaneCalibration:
    H: np.ndarray
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    image_size: tuple[int, int]
    calib_path: str
    backend: str
    front_direction: str
    down_direction: str
    board_cols: int
    board_rows: int
    square_size_mm: float
    rmse_mm: float
    src_points_undistorted_px: np.ndarray
    dst_points_mm: np.ndarray
    point_domain: str = "undistorted_px"
    inlier_count: int = 0

    def map_undistorted_points(self, points: np.ndarray) -> np.ndarray:
        points = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(points, self.H).reshape(-1, 2)


def map_undistorted_points(
    calibration: PlaneCalibration, points: np.ndarray,
) -> np.ndarray:
    return calibration.map_undistorted_points(points)


def compute_plane_calibration(
    distorted_corners: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    image_size: tuple[int, int],
    calib_path: str,
    backend: str,
    front_direction: str,
    down_direction: str,
    board_cols: int,
    board_rows: int,
    square_size_mm: float,
) -> PlaneCalibration:
    if front_direction not in ("left", "right"):
        raise ValueError("front direction must be 'left' or 'right'")
    if down_direction not in ("down", "up"):
        raise ValueError("down direction must be 'down' or 'up'")
    corners = np.asarray(distorted_corners, dtype=np.float64).reshape(-1, 2)
    expected = board_cols * board_rows
    if len(corners) != expected:
        raise ValueError(f"expected {expected} chessboard corners, got {len(corners)}")

    undistorted = undistort_points(corners, camera_matrix, dist_coeffs)
    grid = undistorted.reshape(board_rows, board_cols, 2)
    destinations = np.empty((board_rows, board_cols, 2), dtype=np.float64)
    for row in range(board_rows):
        for col in range(board_cols):
            x_index = board_cols - 1 - col if front_direction == "left" else col
            y_index = row if down_direction == "down" else board_rows - 1 - row
            destinations[row, col] = (
                x_index * square_size_mm, y_index * square_size_mm)

    src = grid.reshape(-1, 2)
    dst = destinations.reshape(-1, 2)
    H, mask = cv2.findHomography(
        src, dst, method=cv2.RANSAC, ransacReprojThreshold=1.0)
    if H is None or mask is None:
        raise ValueError("plane homography could not be computed")
    inliers = mask.reshape(-1).astype(bool)
    inlier_fraction = float(np.mean(inliers))
    if inlier_fraction < 0.8:
        raise ValueError(
            f"plane homography has only {inlier_fraction:.1%} inliers; require 80%")
    mapped = cv2.perspectiveTransform(src.reshape(-1, 1, 2), H).reshape(-1, 2)
    rmse = float(np.sqrt(np.mean(np.sum((mapped[inliers] - dst[inliers]) ** 2, axis=1))))
    if rmse > 0.5:
        raise ValueError(f"plane RMSE {rmse:.3f} mm exceeds 0.5 mm")
    return PlaneCalibration(
        H=H, camera_matrix=np.asarray(camera_matrix, dtype=np.float64),
        dist_coeffs=np.asarray(dist_coeffs, dtype=np.float64),
        image_size=(int(image_size[0]), int(image_size[1])),
        calib_path=_normalize_calibration_path(calib_path), backend=str(backend),
        front_direction=front_direction, down_direction=down_direction,
        board_cols=board_cols, board_rows=board_rows,
        square_size_mm=float(square_size_mm), rmse_mm=rmse,
        src_points_undistorted_px=src, dst_points_mm=dst,
        inlier_count=int(np.count_nonzero(inliers)),
    )


def _paths(path: Path | str) -> tuple[Path, Path]:
    base = Path(path)
    if base.suffix in (".npz", ".json"):
        base = base.with_suffix("")
    return base.with_suffix(".npz"), base.with_suffix(".json")


def save_plane_calibration(path: Path | str, calibration: PlaneCalibration) -> None:
    npz_path, json_path = _paths(path)
    npz_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        npz_path, H=calibration.H, camera_matrix=calibration.camera_matrix,
        dist_coeffs=calibration.dist_coeffs,
        src_points_undistorted_px=calibration.src_points_undistorted_px,
        dst_points_mm=calibration.dst_points_mm)
    provenance = {
        "image_size": list(calibration.image_size),
        "calib_path": calibration.calib_path,
        "backend": calibration.backend,
        "front_direction": calibration.front_direction,
        "down_direction": calibration.down_direction,
        "board_cols": calibration.board_cols,
        "board_rows": calibration.board_rows,
        "square_size_mm": calibration.square_size_mm,
        "rmse_mm": calibration.rmse_mm,
        "point_domain": calibration.point_domain,
        "inlier_count": calibration.inlier_count,
    }
    json_path.write_text(json.dumps(provenance, indent=2), encoding="utf-8")


def load_plane_calibration(path: Path | str) -> PlaneCalibration:
    npz_path, json_path = _paths(path)
    provenance = json.loads(json_path.read_text(encoding="utf-8"))
    with np.load(npz_path) as arrays:
        calibration = PlaneCalibration(
            H=arrays["H"], camera_matrix=arrays["camera_matrix"],
            dist_coeffs=arrays["dist_coeffs"],
            image_size=tuple(provenance["image_size"]),
            calib_path=provenance["calib_path"], backend=provenance["backend"],
            front_direction=provenance["front_direction"],
            down_direction=provenance["down_direction"],
            board_cols=int(provenance["board_cols"]),
            board_rows=int(provenance["board_rows"]),
            square_size_mm=float(provenance["square_size_mm"]),
            rmse_mm=float(provenance["rmse_mm"]),
            src_points_undistorted_px=arrays["src_points_undistorted_px"],
            dst_points_mm=arrays["dst_points_mm"],
            point_domain=provenance.get("point_domain", ""),
            inlier_count=int(provenance.get("inlier_count", 0)),
        )
    _validate_integrity(calibration)
    return calibration


def _validate_integrity(calibration: PlaneCalibration) -> None:
    H = np.asarray(calibration.H)
    if H.shape != (3, 3) or not np.all(np.isfinite(H)):
        raise ValueError("finite homography required with shape (3, 3)")
    if np.linalg.matrix_rank(H) < 3:
        raise ValueError("singular homography is invalid")
    source = np.asarray(calibration.src_points_undistorted_px)
    destination = np.asarray(calibration.dst_points_mm)
    if source.ndim != 2 or source.shape[1:] != (2,) or len(source) < 4:
        raise ValueError("source points must have shape (N, 2), N >= 4")
    if not np.all(np.isfinite(source)):
        raise ValueError("finite source points are required")
    if (destination.ndim != 2 or destination.shape[1:] != (2,) or
            len(destination) < 4 or len(destination) != len(source)):
        raise ValueError(
            "destination points must have shape (N, 2) matching source points")
    if not np.all(np.isfinite(destination)):
        raise ValueError("finite destination points are required")
    if calibration.board_cols <= 0:
        raise ValueError("board columns must be positive")
    if calibration.board_rows <= 0:
        raise ValueError("board rows must be positive")
    if not np.isfinite(calibration.square_size_mm) or calibration.square_size_mm <= 0:
        raise ValueError("square size must be positive")
    if not np.isfinite(calibration.rmse_mm) or calibration.rmse_mm > 0.5:
        raise ValueError("stored RMSE exceeds 0.5 mm or is non-finite")
    required_inliers = 0.8 * calibration.board_cols * calibration.board_rows
    if calibration.inlier_count < required_inliers:
        raise ValueError("stored RANSAC inliers are below 80% of board points")
    if calibration.inlier_count > calibration.board_cols * calibration.board_rows:
        raise ValueError("stored RANSAC inliers exceed board points")
    if calibration.point_domain != "undistorted_px":
        raise ValueError(
            f"point domain mismatch: expected undistorted_px, got {calibration.point_domain!r}")


def _same_calibration_path(stored: str, expected: str) -> bool:
    return (_normalize_calibration_path(stored) ==
            _normalize_calibration_path(expected))


def validate_plane_calibration(
    calibration: PlaneCalibration,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    image_size: tuple[int, int],
    front_direction: str,
    down_direction: str,
    backend: str,
    calib_path: str,
) -> None:
    _validate_integrity(calibration)
    if tuple(calibration.image_size) != tuple(image_size):
        raise ValueError("image size mismatch")
    if not np.allclose(calibration.camera_matrix, camera_matrix, rtol=0.0, atol=1e-9):
        raise ValueError("camera matrix mismatch")
    if (calibration.dist_coeffs.shape != np.asarray(dist_coeffs).shape or
            not np.allclose(calibration.dist_coeffs, dist_coeffs, rtol=0.0, atol=1e-9)):
        raise ValueError("distortion coefficients mismatch")
    if calibration.front_direction != front_direction:
        raise ValueError("front direction mismatch")
    if calibration.down_direction != down_direction:
        raise ValueError("down direction mismatch")
    if calibration.backend.casefold() != backend.casefold():
        raise ValueError("backend mismatch")
    if not _same_calibration_path(calibration.calib_path, calib_path):
        raise ValueError("calibration path mismatch")

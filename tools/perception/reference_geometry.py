"""Reference geometry and ROI rules shared by perception replay tests.

Vehicle coordinates are x-forward, y-left, z-up.  Camera coordinates are
x-right, y-down, z-forward.  Positive pitch is nose-up and positive roll
raises the vehicle's left side.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraCalibration:
    width: int
    height: int
    matrix: np.ndarray
    distortion: np.ndarray
    model: str

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("camera dimensions must be positive")
        if np.asarray(self.matrix).shape != (3, 3):
            raise ValueError("camera matrix must be 3x3")
        if self.model not in ("standard", "fisheye"):
            raise ValueError("model must be standard or fisheye")
        if self.model == "fisheye" and np.asarray(self.distortion).size != 4:
            raise ValueError("fisheye calibration requires four coefficients")


@dataclass(frozen=True)
class VehiclePose:
    x_m: float
    y_m: float
    yaw_rad: float
    roll_rad: float
    pitch_rad: float
    camera_height_m: float


@dataclass(frozen=True)
class ProjectionResult:
    valid: bool
    reason: str
    x_m: float = math.nan
    y_m: float = math.nan
    range_m: float = math.nan
    bearing_rad: float = math.nan


@dataclass(frozen=True)
class TrackWindow:
    left: int
    top: int
    right: int
    bottom: int


@dataclass(frozen=True)
class RoiPlan:
    discovery_enabled: bool
    discovery_top_y: np.ndarray
    vehicle_mask_top_y: int
    tracking_windows: list[TrackWindow]


def _rotation_x(angle_rad: float) -> np.ndarray:
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, cosine, -sine],
                     [0.0, sine, cosine]])


def _rotation_y(angle_rad: float) -> np.ndarray:
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return np.array([[cosine, 0.0, sine],
                     [0.0, 1.0, 0.0],
                     [-sine, 0.0, cosine]])


def _rotation_z(angle_rad: float) -> np.ndarray:
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return np.array([[cosine, -sine, 0.0],
                     [sine, cosine, 0.0],
                     [0.0, 0.0, 1.0]])


def _camera_ray(calibration: CameraCalibration, pixel: tuple[float, float]) -> np.ndarray:
    points = np.asarray(pixel, dtype=np.float64).reshape(1, 1, 2)
    matrix = np.asarray(calibration.matrix, dtype=np.float64)
    distortion = np.asarray(calibration.distortion, dtype=np.float64).reshape(-1)
    if calibration.model == "fisheye":
        normalized = cv2.fisheye.undistortPoints(
            points, matrix, distortion.reshape(4, 1)).reshape(2)
    else:
        normalized = cv2.undistortPoints(points, matrix, distortion).reshape(2)
    return np.array([normalized[0], normalized[1], 1.0])


def _world_ray(calibration: CameraCalibration, pose: VehiclePose,
               pixel: tuple[float, float]) -> np.ndarray:
    # camera x-right/y-down/z-forward -> vehicle x-forward/y-left/z-up
    vehicle_from_camera = np.array([[0.0, 0.0, 1.0],
                                    [-1.0, 0.0, 0.0],
                                    [0.0, -1.0, 0.0]])
    world_from_vehicle = (_rotation_z(pose.yaw_rad)
                          @ _rotation_y(-pose.pitch_rad)
                          @ _rotation_x(pose.roll_rad))
    return world_from_vehicle @ vehicle_from_camera @ _camera_ray(calibration, pixel)


def horizon_curve(calibration: CameraCalibration, pose: VehiclePose) -> np.ndarray:
    """Return the raw-image horizon y for every image column.

    The root is solved in the raw image, so it stays valid for both standard
    and fisheye calibration models without a full-frame remap.
    """
    horizon = np.empty(calibration.width, dtype=np.float64)
    low_y = -float(calibration.height) * 2.0
    high_y = float(calibration.height) * 3.0
    for x in range(calibration.width):
        low = low_y
        high = high_y
        low_z = _world_ray(calibration, pose, (float(x), low))[2]
        high_z = _world_ray(calibration, pose, (float(x), high))[2]
        if low_z == 0.0:
            horizon[x] = low
            continue
        if high_z == 0.0:
            horizon[x] = high
            continue
        if low_z * high_z > 0.0:
            horizon[x] = low if abs(low_z) < abs(high_z) else high
            continue
        for _ in range(36):
            middle = (low + high) * 0.5
            middle_z = _world_ray(calibration, pose, (float(x), middle))[2]
            if low_z * middle_z <= 0.0:
                high = middle
                high_z = middle_z
            else:
                low = middle
                low_z = middle_z
        horizon[x] = (low + high) * 0.5
    return horizon


def build_roi(frame_sequence: int, horizon_y: np.ndarray,
              track_windows: Iterable[TrackWindow], *, horizon_margin_px: int = 4,
              vehicle_mask_top_y: int = 108) -> RoiPlan:
    """Build discovery and track ROIs without rasterizing a full ROI mask."""
    horizon = np.asarray(horizon_y, dtype=np.float64)
    if horizon.shape != (188,):
        raise ValueError("horizon_y must contain one value per 188 image columns")
    discovery_top = np.clip(np.rint(horizon + horizon_margin_px), 0,
                            vehicle_mask_top_y).astype(np.int16)
    clipped_windows: list[TrackWindow] = []
    for window in track_windows:
        left = max(0, min(187, int(window.left)))
        right = max(0, min(187, int(window.right)))
        if left > right:
            continue
        horizon_top = int(discovery_top[(left + right) // 2])
        top = max(int(window.top), horizon_top)
        bottom = min(int(window.bottom), vehicle_mask_top_y - 1)
        if top <= bottom:
            clipped_windows.append(TrackWindow(left, top, right, bottom))
    return RoiPlan(
        discovery_enabled=(frame_sequence % 2) == 0,
        discovery_top_y=discovery_top,
        vehicle_mask_top_y=vehicle_mask_top_y,
        tracking_windows=clipped_windows,
    )


def project_pixel_to_ground(calibration: CameraCalibration, pose: VehiclePose,
                            pixel: tuple[float, float], *,
                            near_parallel_z: float = 0.05) -> ProjectionResult:
    """Intersect one undistorted camera ray with the z=0 ground plane."""
    if pose.camera_height_m <= 0.0:
        return ProjectionResult(False, "invalid_camera_height")
    ray = _world_ray(calibration, pose, pixel)
    if abs(ray[2]) < near_parallel_z:
        return ProjectionResult(False, "near_parallel")
    ray_scale = -pose.camera_height_m / ray[2]
    if ray_scale <= 0.0:
        return ProjectionResult(False, "behind_camera")
    x_m = pose.x_m + ray_scale * ray[0]
    y_m = pose.y_m + ray_scale * ray[1]
    dx = x_m - pose.x_m
    dy = y_m - pose.y_m
    return ProjectionResult(
        True,
        "",
        x_m=float(x_m),
        y_m=float(y_m),
        range_m=float(math.hypot(dx, dy)),
        bearing_rad=float(math.atan2(dy, dx)),
    )

"""Detect concentric circle-plus markers and keep their semantic roles stable."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class CrossCircleConfig:
    min_diameter_px: float = 16.0
    max_diameter_px: float = 240.0
    max_aspect_ratio: float = 1.30
    min_circularity: float = 0.60
    min_ring_score: float = 0.55
    min_cross_score: float = 0.55
    max_center_error_fraction: float = 0.16
    expected_size_ratio: float = 1.40
    size_ratio_tolerance: float = 0.18
    origin_lock_radius_px: float = 15.0
    min_confidence: float = 0.65


@dataclass(frozen=True)
class CrossCircleCandidate:
    center: Tuple[float, float]
    diameter_px: float
    ellipse: Tuple[Tuple[float, float], Tuple[float, float], float]
    circularity: float
    ring_score: float
    horizontal_score: float
    vertical_score: float
    center_error_px: float
    confidence: float
    assigned_role: Optional[str] = None


@dataclass(frozen=True)
class CrossCircleRoles:
    status: str
    origin: Optional[CrossCircleCandidate] = None
    wheel: Optional[CrossCircleCandidate] = None
    candidates: Tuple[CrossCircleCandidate, ...] = ()


class CrossCircleDetector:
    def __init__(self, config: CrossCircleConfig = CrossCircleConfig()):
        self.config = config
        self.locked = False
        self._origin_center: Optional[np.ndarray] = None
        self._current_roles = CrossCircleRoles("SEARCHING")

    @property
    def current_roles(self) -> CrossCircleRoles:
        return self._current_roles

    @property
    def locked_origin_center(self) -> Optional[Tuple[float, float]]:
        if self._origin_center is None:
            return None
        return (float(self._origin_center[0]), float(self._origin_center[1]))

    def reset(self) -> None:
        self.locked = False
        self._origin_center = None
        self._current_roles = CrossCircleRoles("SEARCHING")

    @staticmethod
    def _sample_score(binary: np.ndarray, cx: float, cy: float,
                      radius: float) -> Tuple[float, float, float]:
        size = max(16, int(np.ceil(radius * 2.5)))
        patch = cv2.getRectSubPix(binary, (size, size), (cx, cy))
        yy, xx = np.indices(patch.shape, dtype=np.float32)
        px = xx - (patch.shape[1] - 1) * 0.5
        py = yy - (patch.shape[0] - 1) * 0.5
        rr = np.hypot(px, py)
        ink = patch.astype(np.float32) / 255.0

        ring = (rr >= radius * 0.78) & (rr <= radius * 1.08)
        ring_score = float(np.mean(ink[ring])) if np.any(ring) else 0.0

        arm_limit = radius * 0.72
        half_width = max(1.5, radius * 0.10)
        horizontal = (np.abs(py) <= half_width) & (np.abs(px) <= arm_limit)
        vertical = (np.abs(px) <= half_width) & (np.abs(py) <= arm_limit)
        cross = horizontal | vertical
        off_axis = (rr <= radius * 0.62) & ~(
            (np.abs(px) <= half_width * 1.6) |
            (np.abs(py) <= half_width * 1.6))
        horizontal_score = (float(np.mean(ink[horizontal]))
                            if np.any(horizontal) else 0.0)
        vertical_score = (float(np.mean(ink[vertical]))
                          if np.any(vertical) else 0.0)
        white_score = 1.0 - float(np.mean(ink[off_axis])) if np.any(off_axis) else 0.0
        horizontal_score = 0.72 * horizontal_score + 0.28 * white_score
        vertical_score = 0.72 * vertical_score + 0.28 * white_score
        return ring_score, horizontal_score, vertical_score

    @staticmethod
    def _projection_center(binary: np.ndarray, cx: float, cy: float,
                           radius: float) -> np.ndarray:
        size = max(9, int(round(radius * 1.45)) | 1)
        patch = cv2.getRectSubPix(binary, (size, size), (cx, cy))
        col = patch.astype(np.float32).sum(axis=0)
        row = patch.astype(np.float32).sum(axis=1)

        def peak(values: np.ndarray) -> float:
            i = int(np.argmax(values))
            if 0 < i < len(values) - 1:
                a, b, c = values[i - 1:i + 2]
                denom = a - 2.0 * b + c
                if abs(float(denom)) > 1e-6:
                    return i + 0.5 * float(a - c) / float(denom)
            return float(i)

        local = np.array([peak(col), peak(row)], dtype=np.float64)
        top_left = np.array([cx, cy]) - (np.array(patch.shape[::-1]) - 1) * 0.5
        return top_left + local

    def detect(self, frame: np.ndarray) -> Tuple[CrossCircleCandidate, ...]:
        if frame is None or frame.size == 0:
            return ()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        enhanced = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
        block = max(31, (min(gray.shape[:2]) // 12) | 1)
        binary = cv2.adaptiveThreshold(
            enhanced, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, block, 5)
        edges = cv2.Canny(enhanced, 45, 135)
        contours, _ = cv2.findContours(
            cv2.bitwise_or(binary, edges), cv2.RETR_LIST,
            cv2.CHAIN_APPROX_NONE)

        found = []
        for contour in contours:
            if len(contour) < 20:
                continue
            area = float(cv2.contourArea(contour))
            perimeter = float(cv2.arcLength(contour, True))
            if area <= 0.0 or perimeter <= 0.0:
                continue
            ellipse = cv2.fitEllipse(contour)
            (cx, cy), (minor, major), _ = ellipse
            low, high = sorted((float(minor), float(major)))
            diameter = 0.5 * (low + high)
            if not (self.config.min_diameter_px <= diameter <=
                    self.config.max_diameter_px):
                continue
            aspect = high / max(low, 1e-6)
            circularity = 4.0 * np.pi * area / (perimeter * perimeter)
            if aspect > self.config.max_aspect_ratio or circularity < self.config.min_circularity:
                continue
            radius = diameter * 0.5
            ring_score, horizontal_score, vertical_score = self._sample_score(
                binary, cx, cy, radius)
            cross_score = min(horizontal_score, vertical_score)
            if ring_score < self.config.min_ring_score or cross_score < self.config.min_cross_score:
                continue
            projected = self._projection_center(binary, cx, cy, radius)
            ellipse_center = np.array([cx, cy], dtype=np.float64)
            center_error = float(np.linalg.norm(projected - ellipse_center))
            if center_error > diameter * self.config.max_center_error_fraction:
                continue
            center = ellipse_center * 0.95 + projected * 0.05
            shape_score = min(1.0, circularity) * min(1.0, 1.0 / aspect * 1.15)
            confidence = 0.30 * ring_score + 0.40 * cross_score + 0.30 * shape_score
            fitted_ellipse = (
                (float(ellipse[0][0]), float(ellipse[0][1])),
                (float(ellipse[1][0]), float(ellipse[1][1])),
                float(ellipse[2]))
            found.append(CrossCircleCandidate(
                center=(float(center[0]), float(center[1])),
                diameter_px=diameter, ellipse=fitted_ellipse,
                circularity=float(circularity), ring_score=ring_score,
                horizontal_score=horizontal_score,
                vertical_score=vertical_score,
                center_error_px=center_error,
                confidence=float(confidence)))

        return self._non_maximum_suppress(found)

    @staticmethod
    def _non_maximum_suppress(
            candidates: Sequence[CrossCircleCandidate],
    ) -> Tuple[CrossCircleCandidate, ...]:
        kept = []
        for candidate in sorted(
                candidates, key=lambda item: item.confidence, reverse=True):
            duplicate = any(
                np.linalg.norm(np.subtract(candidate.center, other.center)) <
                0.25 * min(candidate.diameter_px, other.diameter_px)
                for other in kept)
            if not duplicate:
                kept.append(candidate)
        return tuple(kept)

    def _set_current(self, roles: CrossCircleRoles) -> CrossCircleRoles:
        self._current_roles = roles
        return roles

    def update(self, frame: np.ndarray) -> CrossCircleRoles:
        candidates = tuple(c for c in self.detect(frame)
                           if c.confidence >= self.config.min_confidence)
        if len(candidates) < 2:
            return self._set_current(CrossCircleRoles(
                "MISSING", candidates=candidates))
        if len(candidates) > 2:
            return self._set_current(CrossCircleRoles(
                "AMBIGUOUS", candidates=candidates))

        small, large = sorted(candidates, key=lambda item: item.diameter_px)
        ratio = large.diameter_px / small.diameter_px
        if abs(ratio - self.config.expected_size_ratio) > self.config.size_ratio_tolerance:
            return self._set_current(CrossCircleRoles(
                "AMBIGUOUS", candidates=candidates))

        if not self.locked:
            origin, wheel = large, small
            self._origin_center = np.asarray(origin.center, dtype=np.float64)
            self.locked = True
        else:
            distances = [float(np.linalg.norm(
                np.asarray(c.center) - self._origin_center)) for c in candidates]
            index = int(np.argmin(distances))
            if distances[index] > self.config.origin_lock_radius_px:
                return self._set_current(CrossCircleRoles(
                    "AMBIGUOUS", candidates=candidates))
            origin, wheel = candidates[index], candidates[1 - index]
            if origin is not large:
                return self._set_current(CrossCircleRoles(
                    "AMBIGUOUS", candidates=candidates))
            self._origin_center = np.asarray(origin.center, dtype=np.float64)
        origin = replace(origin, assigned_role="origin")
        wheel = replace(wheel, assigned_role="wheel")
        return self._set_current(CrossCircleRoles(
            "VALID", origin, wheel, candidates))


__all__: Sequence[str] = (
    "CrossCircleConfig", "CrossCircleCandidate", "CrossCircleRoles",
    "CrossCircleDetector")

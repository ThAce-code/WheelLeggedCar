"""Detect concentric circle-plus markers and keep their semantic roles stable."""

from __future__ import annotations

from dataclasses import dataclass
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
    confidence: float
    ring_score: float
    cross_score: float


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

    def reset(self) -> None:
        self.locked = False
        self._origin_center = None

    @staticmethod
    def _sample_score(binary: np.ndarray, cx: float, cy: float,
                      radius: float) -> Tuple[float, float]:
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
        ink_score = float(np.mean(ink[cross])) if np.any(cross) else 0.0
        white_score = 1.0 - float(np.mean(ink[off_axis])) if np.any(off_axis) else 0.0
        cross_score = 0.72 * ink_score + 0.28 * white_score
        return ring_score, cross_score

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
            ring_score, cross_score = self._sample_score(binary, cx, cy, radius)
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
            found.append(CrossCircleCandidate(
                (float(center[0]), float(center[1])), diameter,
                float(confidence), ring_score, cross_score))

        kept = []
        for candidate in sorted(found, key=lambda item: item.confidence, reverse=True):
            duplicate = any(
                np.linalg.norm(np.subtract(candidate.center, other.center)) <
                0.25 * min(candidate.diameter_px, other.diameter_px)
                for other in kept)
            if not duplicate:
                kept.append(candidate)
        return tuple(kept)

    def update(self, frame: np.ndarray) -> CrossCircleRoles:
        candidates = tuple(c for c in self.detect(frame)
                           if c.confidence >= self.config.min_confidence)
        if len(candidates) < 2:
            return CrossCircleRoles("MISSING", candidates=candidates)
        if len(candidates) > 2:
            return CrossCircleRoles("AMBIGUOUS", candidates=candidates)

        small, large = sorted(candidates, key=lambda item: item.diameter_px)
        ratio = large.diameter_px / small.diameter_px
        if abs(ratio - self.config.expected_size_ratio) > self.config.size_ratio_tolerance:
            return CrossCircleRoles("AMBIGUOUS", candidates=candidates)

        if not self.locked:
            origin, wheel = large, small
            self._origin_center = np.asarray(origin.center, dtype=np.float64)
            self.locked = True
        else:
            distances = [float(np.linalg.norm(
                np.asarray(c.center) - self._origin_center)) for c in candidates]
            index = int(np.argmin(distances))
            if distances[index] > self.config.origin_lock_radius_px:
                return CrossCircleRoles("AMBIGUOUS", candidates=candidates)
            origin, wheel = candidates[index], candidates[1 - index]
            self._origin_center = np.asarray(origin.center, dtype=np.float64)
        return CrossCircleRoles("VALID", origin, wheel, candidates)


__all__: Sequence[str] = (
    "CrossCircleConfig", "CrossCircleCandidate", "CrossCircleRoles",
    "CrossCircleDetector")

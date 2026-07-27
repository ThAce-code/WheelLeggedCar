"""Detect concentric circle-plus markers and keep their semantic roles stable."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
from typing import Callable, Optional, Sequence, Tuple

import cv2
import numpy as np

from camera_utils import undistort_points


@dataclass(frozen=True)
class CrossCircleConfig:
    min_diameter_px: float = 16.0
    max_diameter_px: float = 240.0
    max_aspect_ratio: float = 1.30
    # A linkage can hide a short part of the printed ring.  The cross and
    # size-ratio checks remain the stronger discriminators in that case.
    min_circularity: float = 0.08
    min_ring_score: float = 0.55
    min_cross_score: float = 0.55
    max_center_error_fraction: float = 0.16
    expected_size_ratio: float = 1.40  # 14 mm wheel / 10 mm origin
    size_ratio_tolerance: float = 0.18
    min_pair_scale_separation: float = 1.20
    min_pair_confidence_separation: float = 0.05
    max_horizontal_to_vertical_ratio: float = 1.35
    fragmented_min_ring_score: float = 0.85
    fragmented_min_cross_score: float = 0.60
    strong_pattern_min_ring_score: float = 0.90
    strong_pattern_min_cross_score: float = 0.60
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


@dataclass(frozen=True)
class CrossCircleMeasurement:
    x_mm: float
    y_mm: float
    origin_u_px: float
    origin_v_px: float
    wheel_u_px: float
    wheel_v_px: float
    confidence: float
    valid_frames: int
    status: str


class CrossCircleMeasurementTracker:
    def __init__(self, detector, plane_calibration, camera_matrix,
                 dist_coeffs, jump_threshold_mm=20.0,
                 relative_bounds_mm=None):
        self.detector = detector
        self.plane_calibration = plane_calibration
        self.camera_matrix = np.asarray(camera_matrix, dtype=np.float64)
        self.dist_coeffs = np.asarray(dist_coeffs, dtype=np.float64)
        self.jump_threshold_mm = float(jump_threshold_mm)
        self.relative_bounds_mm = (None if relative_bounds_mm is None else
                                   tuple(float(value)
                                         for value in relative_bounds_mm))
        if (self.relative_bounds_mm is not None and
                len(self.relative_bounds_mm) != 4):
            raise ValueError("relative_bounds_mm must contain x_min, x_max, y_min, y_max")
        self._history = deque(maxlen=15)
        self._pending_jump_history = deque(maxlen=5)
        self.current_sample_accepted = False

    @property
    def valid_frame_count(self) -> int:
        return len(self._history)

    @staticmethod
    def _median_measurement(history, valid_frames):
        values = np.asarray([
            (item.x_mm, item.y_mm, item.origin_u_px, item.origin_v_px,
             item.wheel_u_px, item.wheel_v_px, item.confidence)
            for item in history], dtype=np.float64)
        medians = np.median(values, axis=0)
        return CrossCircleMeasurement(
            x_mm=float(medians[0]), y_mm=float(medians[1]),
            origin_u_px=float(medians[2]), origin_v_px=float(medians[3]),
            wheel_u_px=float(medians[4]), wheel_v_px=float(medians[5]),
            confidence=float(medians[6]), valid_frames=valid_frames,
            status="VALID")

    def process(self, frame):
        self.current_sample_accepted = False
        if self.relative_bounds_mm is None:
            roles = self.detector.update(frame)
        else:
            roles = self.detector.update(
                frame, pair_validator=self._pair_within_relative_bounds)
        if roles.status != "VALID" or roles.origin is None or roles.wheel is None:
            return self.live_measurement()

        centers = np.asarray(
            [roles.origin.center, roles.wheel.center], dtype=np.float64)
        undistorted = undistort_points(
            centers, self.camera_matrix, self.dist_coeffs)
        plane_points = self.plane_calibration.map_undistorted_points(
            undistorted)
        relative = plane_points[1] - plane_points[0]

        sample = CrossCircleMeasurement(
            x_mm=float(relative[0]), y_mm=float(relative[1]),
            origin_u_px=float(centers[0, 0]),
            origin_v_px=float(centers[0, 1]),
            wheel_u_px=float(centers[1, 0]),
            wheel_v_px=float(centers[1, 1]),
            confidence=float(min(
                roles.origin.confidence, roles.wheel.confidence)),
            valid_frames=1, status="VALID")

        if self._history:
            recent = list(self._history)[-5:]
            median_x = float(np.median([item.x_mm for item in recent]))
            median_y = float(np.median([item.y_mm for item in recent]))
            if (abs(sample.x_mm - median_x) > self.jump_threshold_mm or
                    abs(sample.y_mm - median_y) > self.jump_threshold_mm):
                if self._pending_jump_history:
                    pending = list(self._pending_jump_history)
                    pending_x = float(np.median(
                        [item.x_mm for item in pending]))
                    pending_y = float(np.median(
                        [item.y_mm for item in pending]))
                    if (abs(sample.x_mm - pending_x) > self.jump_threshold_mm or
                            abs(sample.y_mm - pending_y) >
                            self.jump_threshold_mm):
                        self._pending_jump_history.clear()
                self._pending_jump_history.append(sample)
                if (len(self._pending_jump_history) ==
                        self._pending_jump_history.maxlen):
                    self._history.clear()
                    self._history.extend(self._pending_jump_history)
                    self._pending_jump_history.clear()
                    self.current_sample_accepted = True
                return self.live_measurement()

        self._pending_jump_history.clear()
        self._history.append(sample)
        self.current_sample_accepted = True
        return self.live_measurement()

    def _pair_within_relative_bounds(self, origin, wheel) -> bool:
        centers = np.asarray(
            [origin.center, wheel.center], dtype=np.float64)
        undistorted = undistort_points(
            centers, self.camera_matrix, self.dist_coeffs)
        plane_points = self.plane_calibration.map_undistorted_points(
            undistorted)
        relative = plane_points[1] - plane_points[0]
        x_min, x_max, y_min, y_max = self.relative_bounds_mm
        return (x_min <= float(relative[0]) <= x_max and
                y_min <= float(relative[1]) <= y_max)

    def live_measurement(self):
        if not self._history:
            return None
        recent = list(self._history)[-5:]
        return self._median_measurement(recent, len(self._history))

    def capture(self):
        if len(self._history) < self._history.maxlen:
            return None
        return self._median_measurement(self._history, len(self._history))

    def reset(self):
        self.clear_history()
        self.detector.reset()

    def clear_history(self):
        self._history.clear()
        self._pending_jump_history.clear()
        self.current_sample_accepted = False


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

    def _shape_is_credible(self, *, aspect: float, circularity: float,
                           ring_score: float, cross_score: float) -> bool:
        if aspect > self.config.max_aspect_ratio:
            return False
        if circularity >= self.config.min_circularity:
            return True
        return (ring_score >= self.config.fragmented_min_ring_score and
                cross_score >= self.config.fragmented_min_cross_score)

    def _sample_score(self, binary: np.ndarray, cx: float, cy: float,
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

        def score_axes(axis_x: np.ndarray, axis_y: np.ndarray
                       ) -> Tuple[float, float]:
            horizontal = ((np.abs(axis_y) <= half_width) &
                          (np.abs(axis_x) <= arm_limit))
            vertical = ((np.abs(axis_x) <= half_width) &
                        (np.abs(axis_y) <= arm_limit))
            off_axis = (rr <= radius * 0.62) & ~(
                (np.abs(axis_x) <= half_width * 1.6) |
                (np.abs(axis_y) <= half_width * 1.6))
            white_score = (1.0 - float(np.mean(ink[off_axis]))
                           if np.any(off_axis) else 0.0)
            horizontal_score = (float(np.mean(ink[horizontal]))
                                if np.any(horizontal) else 0.0)
            vertical_score = (float(np.mean(ink[vertical]))
                              if np.any(vertical) else 0.0)
            return (0.72 * horizontal_score + 0.28 * white_score,
                    0.72 * vertical_score + 0.28 * white_score)

        axis_aligned = score_axes(px, py)
        if (ring_score < self.config.min_ring_score or
                min(axis_aligned) >= self.config.min_cross_score):
            return ring_score, axis_aligned[0], axis_aligned[1]

        # A plus sign repeats every 90 degrees.  Searching 0..45 degrees is
        # sufficient because swapping the two axes describes the other half.
        angles = np.deg2rad(np.arange(5.0, 46.0, 5.0))[:, None, None]
        cosine, sine = np.cos(angles), np.sin(angles)
        axis_x = px[None, :, :] * cosine + py[None, :, :] * sine
        axis_y = -px[None, :, :] * sine + py[None, :, :] * cosine
        horizontal = ((np.abs(axis_y) <= half_width) &
                      (np.abs(axis_x) <= arm_limit))
        vertical = ((np.abs(axis_x) <= half_width) &
                    (np.abs(axis_y) <= arm_limit))
        off_axis = (rr[None, :, :] <= radius * 0.62) & ~(
            (np.abs(axis_x) <= half_width * 1.6) |
            (np.abs(axis_y) <= half_width * 1.6))

        def masked_means(masks: np.ndarray) -> np.ndarray:
            counts = np.maximum(masks.sum(axis=(1, 2)), 1)
            return (masks * ink[None, :, :]).sum(axis=(1, 2)) / counts

        white_scores = 1.0 - masked_means(off_axis)
        horizontal_scores = 0.72 * masked_means(horizontal) + 0.28 * white_scores
        vertical_scores = 0.72 * masked_means(vertical) + 0.28 * white_scores
        minimum_scores = np.minimum(horizontal_scores, vertical_scores)
        average_scores = 0.5 * (horizontal_scores + vertical_scores)
        best = max(range(len(minimum_scores)),
                   key=lambda index: (minimum_scores[index],
                                      average_scores[index]))
        return (ring_score, float(horizontal_scores[best]),
                float(vertical_scores[best]))

    @staticmethod
    def _projection_center(binary: np.ndarray, cx: float, cy: float,
                           radius: float) -> np.ndarray:
        size = max(9, int(round(radius * 1.45)) | 1)
        patch = cv2.getRectSubPix(binary, (size, size), (cx, cy))
        yy, xx = np.indices(patch.shape, dtype=np.float64)
        local_center = (np.asarray(patch.shape[::-1], dtype=np.float64) - 1.0) * 0.5
        rr = np.hypot(xx - local_center[0], yy - local_center[1])
        weights = (patch.astype(np.float64) / 255.0) * (rr <= radius * 0.62)
        total = float(weights.sum())
        if total <= 1e-6:
            return np.array([cx, cy], dtype=np.float64)
        local = np.array([
            float((weights * xx).sum() / total),
            float((weights * yy).sum() / total)], dtype=np.float64)
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
        # Printed gray rings are often visible as edges but not as solid ink
        # after adaptive thresholding.  Thicken those edges only for pattern
        # scoring; retain the binary image for stable subpixel centering.
        score_mask = cv2.bitwise_or(
            binary, cv2.dilate(edges, np.ones((3, 3), dtype=np.uint8)))
        contours, _ = cv2.findContours(
            cv2.bitwise_or(binary, edges), cv2.RETR_LIST,
            cv2.CHAIN_APPROX_SIMPLE)

        found = []
        for contour in contours:
            if len(contour) < 5:
                continue
            _, _, width, height = cv2.boundingRect(contour)
            box_low, box_high = sorted((float(width), float(height)))
            rough_diameter = 0.5 * (box_low + box_high)
            if not (self.config.min_diameter_px * 0.70 <= rough_diameter <=
                    self.config.max_diameter_px * 1.30):
                continue
            if (box_high / max(box_low, 1.0) >
                    self.config.max_aspect_ratio * 1.20):
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
            if aspect > self.config.max_aspect_ratio:
                continue
            radius = diameter * 0.5
            projected = self._projection_center(binary, cx, cy, radius)
            ellipse_center = np.array([cx, cy], dtype=np.float64)
            center_error = float(np.linalg.norm(projected - ellipse_center))
            if center_error > diameter * self.config.max_center_error_fraction:
                continue
            ring_score, horizontal_score, vertical_score = self._sample_score(
                score_mask, cx, cy, radius)
            cross_score = min(horizontal_score, vertical_score)
            if ring_score < self.config.min_ring_score or cross_score < self.config.min_cross_score:
                continue
            if not self._shape_is_credible(
                    aspect=aspect, circularity=circularity,
                    ring_score=ring_score, cross_score=cross_score):
                continue
            center = ellipse_center * 0.95 + projected * 0.05
            shape_score = min(1.0, circularity) * min(1.0, 1.0 / aspect * 1.15)
            confidence = (0.35 * ring_score + 0.45 * cross_score +
                          0.20 * shape_score)
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

    def update(
            self, frame: np.ndarray,
            pair_validator: Optional[
                Callable[[CrossCircleCandidate, CrossCircleCandidate], bool]
            ] = None,
    ) -> CrossCircleRoles:
        candidates = tuple(
            candidate for candidate in self.detect(frame)
            if (candidate.confidence >= self.config.min_confidence or
                (candidate.ring_score >=
                 self.config.strong_pattern_min_ring_score and
                 min(candidate.horizontal_score, candidate.vertical_score) >=
                 self.config.strong_pattern_min_cross_score)))
        if len(candidates) < 2:
            return self._set_current(CrossCircleRoles(
                "MISSING", candidates=candidates))

        compatible_pairs = []
        for first_index in range(len(candidates) - 1):
            for second_index in range(first_index + 1, len(candidates)):
                small, large = sorted(
                    (candidates[first_index], candidates[second_index]),
                    key=lambda item: item.diameter_px)
                ratio = large.diameter_px / small.diameter_px
                if abs(ratio - self.config.expected_size_ratio) > self.config.size_ratio_tolerance:
                    continue
                delta_x = float(large.center[0] - small.center[0])
                delta_y = float(large.center[1] - small.center[1])
                if delta_y <= 0.0:
                    continue
                if (abs(delta_x) > delta_y *
                        self.config.max_horizontal_to_vertical_ratio):
                    continue
                if (pair_validator is not None and
                        not pair_validator(small, large)):
                    continue
                compatible_pairs.append((small, large))

        if self.locked and self._origin_center is not None:
            compatible_pairs = [
                pair for pair in compatible_pairs
                if float(np.linalg.norm(
                    np.asarray(pair[0].center) - self._origin_center)) <=
                self.config.origin_lock_radius_px]

        compatible_pairs.sort(
            key=lambda pair: pair[0].diameter_px, reverse=True)
        if len(compatible_pairs) > 1:
            largest, runner_up = compatible_pairs[:2]
            if (largest[0].diameter_px < runner_up[0].diameter_px *
                    self.config.min_pair_scale_separation):
                ranked = sorted(
                    compatible_pairs,
                    key=lambda pair: pair[0].confidence + pair[1].confidence,
                    reverse=True)
                best_score = ranked[0][0].confidence + ranked[0][1].confidence
                next_score = ranked[1][0].confidence + ranked[1][1].confidence
                if (best_score - next_score <
                        self.config.min_pair_confidence_separation):
                    return self._set_current(CrossCircleRoles(
                        "AMBIGUOUS", candidates=candidates))
                compatible_pairs = [ranked[0]]
            else:
                compatible_pairs = [largest]

        if not compatible_pairs:
            return self._set_current(CrossCircleRoles(
                "AMBIGUOUS", candidates=candidates))
        small, large = compatible_pairs[0]

        if not self.locked:
            origin, wheel = small, large
            self._origin_center = np.asarray(origin.center, dtype=np.float64)
            self.locked = True
        else:
            origin, wheel = small, large
            self._origin_center = np.asarray(origin.center, dtype=np.float64)
        origin = replace(origin, assigned_role="origin")
        wheel = replace(wheel, assigned_role="wheel")
        return self._set_current(CrossCircleRoles(
            "VALID", origin, wheel, candidates))


__all__: Sequence[str] = (
    "CrossCircleConfig", "CrossCircleCandidate", "CrossCircleRoles",
    "CrossCircleDetector", "CrossCircleMeasurement",
    "CrossCircleMeasurementTracker")

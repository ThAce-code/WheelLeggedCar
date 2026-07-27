"""One-pass row-run reference detector for 188x120 grayscale cone images."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from tools.perception.reference_geometry import RoiPlan, TrackWindow


SCORE_WEIGHTS = {
    "taper": 77,
    "band": 64,
    "symmetry": 38,
    "aspect": 26,
    "base": 26,
    "contrast": 25,
}
SCORE_ACCEPT = 166
_MAX_CANDIDATES = 24
_BRIGHT_THRESHOLD = 160


@dataclass(frozen=True)
class ImageQuality:
    p05: int
    p50: int
    p95: int
    clipped_dark_ratio: float
    clipped_bright_ratio: float
    map_observation_allowed: bool


@dataclass(frozen=True)
class Candidate:
    left: int
    top: int
    right: int
    bottom: int
    area_px: int
    bottom_center: tuple[int, int]
    slice_count: int
    components: dict[str, int]
    score: int


@dataclass(frozen=True)
class DetectorResult:
    quality: ImageQuality
    candidates: list[Candidate]
    accepted: list[Candidate]
    roi_pixel_count: int
    component_overflow_count: int


@dataclass(frozen=True)
class _Run:
    y: int
    left: int
    right: int
    label: int


class _UnionFind:
    def __init__(self) -> None:
        self.parent: list[int] = []

    def add(self) -> int:
        label = len(self.parent)
        self.parent.append(label)
        return label

    def find(self, label: int) -> int:
        while self.parent[label] != label:
            self.parent[label] = self.parent[self.parent[label]]
            label = self.parent[label]
        return label

    def union(self, first: int, second: int) -> int:
        first_root = self.find(first)
        second_root = self.find(second)
        if first_root == second_root:
            return first_root
        if first_root < second_root:
            self.parent[second_root] = first_root
            return first_root
        self.parent[first_root] = second_root
        return second_root


def assess_image_quality(image: np.ndarray) -> ImageQuality:
    """Use a sparse sample so quality checking cannot dominate frame time."""
    sampled = np.asarray(image, dtype=np.uint8)[::4, ::4]
    p05, p50, p95 = (int(value) for value in np.percentile(sampled, [5, 50, 95]))
    dark_ratio = float(np.mean(sampled <= 5))
    bright_ratio = float(np.mean(sampled >= 250))
    allowed = (dark_ratio <= 0.30 and bright_ratio <= 0.30
               and (p95 - p05) >= 24)
    return ImageQuality(p05, p50, p95, dark_ratio, bright_ratio, allowed)


def _in_tracking_window(x: int, y: int, windows: list[TrackWindow]) -> bool:
    return any(window.left <= x <= window.right and window.top <= y <= window.bottom
               for window in windows)


def _is_roi_pixel(roi: RoiPlan, x: int, y: int) -> bool:
    if y >= roi.vehicle_mask_top_y:
        return False
    if roi.discovery_enabled and y >= int(roi.discovery_top_y[x]):
        return True
    return _in_tracking_window(x, y, roi.tracking_windows)


def _is_foreground(image: np.ndarray, x: int, y: int) -> bool:
    pixel = int(image[y, x])
    left = max(0, x - 2)
    right = min(image.shape[1], x + 3)
    rolling_mean = int(np.mean(image[y, left:right]))
    return pixel >= _BRIGHT_THRESHOLD or pixel >= rolling_mean + 40


def _component_candidate(image: np.ndarray, runs: list[_Run]) -> Candidate | None:
    left = min(run.left for run in runs)
    right = max(run.right for run in runs)
    top = min(run.y for run in runs)
    bottom = max(run.y for run in runs)
    height = bottom - top + 1
    width = right - left + 1
    area = sum(run.right - run.left + 1 for run in runs)
    if height < 8 or width < 4 or area < 24:
        return None

    widths_by_row: dict[int, int] = {}
    weighted_x = 0.0
    values: list[int] = []
    band_values: list[int] = []
    band_top = top + int(round(height * 0.30))
    band_bottom = top + int(round(height * 0.60))
    for run in runs:
        run_width = run.right - run.left + 1
        widths_by_row[run.y] = widths_by_row.get(run.y, 0) + run_width
        weighted_x += (run.left + run.right) * 0.5 * run_width
        row_values = image[run.y, run.left:run.right + 1]
        values.extend(int(value) for value in row_values)
        if band_top <= run.y <= band_bottom:
            band_values.extend(int(value) for value in row_values)

    slice_count = 3 if height < 12 else 5
    slice_fractions = (0.20, 0.50, 0.80) if slice_count == 3 else (0.15, 0.30, 0.50, 0.70, 0.85)
    slice_widths = [widths_by_row.get(top + int(round((height - 1) * fraction)), 0)
                    for fraction in slice_fractions]
    top_width = max(1, slice_widths[0])
    bottom_width = max(1, slice_widths[-1])
    taper = int(np.clip(256.0 * max(0, bottom_width - top_width) / bottom_width, 0, 256))
    band_mean = float(np.mean(band_values)) if band_values else 0.0
    band = int(np.clip((band_mean - 180.0) * 256.0 / 75.0, 0, 256))
    centroid_x = weighted_x / area
    bbox_center_x = (left + right) * 0.5
    symmetry = int(np.clip(256.0 * (1.0 - abs(centroid_x - bbox_center_x)
                                    / max(width * 0.5, 1.0)), 0, 256))
    aspect_ratio = height / width
    aspect = int(np.clip(256.0 * (1.0 - abs(aspect_ratio - 1.3) / 1.3), 0, 256))
    base = int(np.clip(256.0 * bottom_width / max(slice_widths), 0, 256))
    foreground_mean = float(np.mean(values))
    background = image[max(0, top - 3):min(image.shape[0], bottom + 4),
                       max(0, left - 3):min(image.shape[1], right + 4)]
    contrast = int(np.clip(256.0 * (foreground_mean - float(np.mean(background))) / 100.0,
                           0, 256))
    components = {
        "taper": taper,
        "band": band,
        "symmetry": symmetry,
        "aspect": aspect,
        "base": base,
        "contrast": contrast,
    }
    score = (sum(SCORE_WEIGHTS[name] * value for name, value in components.items()) + 128) >> 8
    return Candidate(
        left, top, right, bottom, area,
        (int(round(centroid_x)), bottom), slice_count, components, score)


def detect(image: np.ndarray, roi: RoiPlan) -> DetectorResult:
    """Detect candidates via one sequential row-run pass over active ROI pixels."""
    gray = np.asarray(image, dtype=np.uint8)
    if gray.shape != (120, 188):
        raise ValueError("detector requires a 188x120 grayscale image")
    quality = assess_image_quality(gray)
    union_find = _UnionFind()
    previous_runs: list[_Run] = []
    all_runs: list[_Run] = []
    roi_pixel_count = 0
    for y in range(gray.shape[0]):
        current_runs: list[_Run] = []
        x = 0
        while x < gray.shape[1]:
            if not _is_roi_pixel(roi, x, y):
                x += 1
                continue
            roi_pixel_count += 1
            if not _is_foreground(gray, x, y):
                x += 1
                continue
            left = x
            while x + 1 < gray.shape[1] and _is_roi_pixel(roi, x + 1, y) and _is_foreground(gray, x + 1, y):
                x += 1
                roi_pixel_count += 1
            right = x
            overlaps = [run.label for run in previous_runs
                        if run.left <= right and run.right >= left]
            label = union_find.add() if not overlaps else overlaps[0]
            for other in overlaps[1:]:
                label = union_find.union(label, other)
            run = _Run(y, left, right, label)
            current_runs.append(run)
            all_runs.append(run)
            x += 1
        previous_runs = current_runs

    grouped: dict[int, list[_Run]] = {}
    for run in all_runs:
        grouped.setdefault(union_find.find(run.label), []).append(run)
    candidates = [candidate for candidate in
                  (_component_candidate(gray, runs) for runs in grouped.values())
                  if candidate is not None]
    candidates.sort(key=lambda candidate: (-candidate.score,
                                            -(candidate.bottom - candidate.top),
                                            candidate.left))
    overflow = max(0, len(candidates) - _MAX_CANDIDATES)
    candidates = candidates[:_MAX_CANDIDATES]
    accepted = ([candidate for candidate in candidates if candidate.score >= SCORE_ACCEPT]
                if quality.map_observation_allowed else [])
    return DetectorResult(quality, candidates, accepted, roi_pixel_count, overflow)

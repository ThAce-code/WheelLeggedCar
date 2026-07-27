"""Bounded static-cone tracking, frozen gap generation, and task states."""

from __future__ import annotations

import math
from dataclasses import dataclass, field


CHI2_GATE = 5.99


@dataclass(frozen=True)
class Observation:
    x_m: float
    y_m: float
    covariance_xx: float
    covariance_xy: float
    covariance_yy: float
    score: int
    far: bool = False


@dataclass
class Track:
    cone_id: int
    x_m: float
    y_m: float
    covariance_xx: float = 0.01
    covariance_yy: float = 0.01
    confirmed: bool = False
    history: list[int] = field(default_factory=list)
    miss_count: int = 0
    prediction_only: bool = False
    far: bool = False


@dataclass(frozen=True)
class Gap:
    gap_id: int
    left_cone_id: int
    right_cone_id: int
    center_x_m: float
    center_y_m: float
    heading_rad: float
    approach_x_m: float
    approach_y_m: float
    exit_x_m: float
    exit_y_m: float


@dataclass(frozen=True)
class GapMap:
    gaps: list[Gap]
    return_gaps: list[Gap]


class StaticConeTracker:
    def __init__(self, max_tracks: int = 32) -> None:
        self.max_tracks = max_tracks
        self.tracks: list[Track] = []
        self._next_id = 1
        self.overflow_count = 0

    @staticmethod
    def _distance_squared(track: Track, observation: Observation) -> float:
        dx = observation.x_m - track.x_m
        dy = observation.y_m - track.y_m
        return dx * dx + dy * dy

    @classmethod
    def _mahalanobis_squared(cls, track: Track, observation: Observation) -> float:
        variance_x = max(0.001, track.covariance_xx + observation.covariance_xx)
        variance_y = max(0.001, track.covariance_yy + observation.covariance_yy)
        dx = observation.x_m - track.x_m
        dy = observation.y_m - track.y_m
        return dx * dx / variance_x + dy * dy / variance_y

    def _update_confirmation(self, track: Track) -> None:
        window = 6 if track.far else 5
        needed = 4 if track.far else 3
        track.history = track.history[-window:]
        if sum(track.history) >= needed:
            track.confirmed = True

    def step(self, observations: list[Observation]) -> None:
        pairs: list[tuple[float, int, int]] = []
        for track_index, track in enumerate(self.tracks):
            for observation_index, observation in enumerate(observations):
                hard_gate = max(0.20, 3.0 * math.sqrt(
                    track.covariance_xx + track.covariance_yy
                    + observation.covariance_xx + observation.covariance_yy))
                if self._distance_squared(track, observation) > hard_gate * hard_gate:
                    continue
                d_squared = self._mahalanobis_squared(track, observation)
                if d_squared <= CHI2_GATE:
                    pairs.append((d_squared, track_index, observation_index))
        pairs.sort(key=lambda pair: (pair[0], -observations[pair[2]].score,
                                     self.tracks[pair[1]].cone_id))
        used_tracks: set[int] = set()
        used_observations: set[int] = set()
        for _distance, track_index, observation_index in pairs:
            if track_index in used_tracks or observation_index in used_observations:
                continue
            track = self.tracks[track_index]
            observation = observations[observation_index]
            track.x_m = observation.x_m
            track.y_m = observation.y_m
            track.covariance_xx = observation.covariance_xx
            track.covariance_yy = observation.covariance_yy
            track.far = track.far or observation.far
            track.history.append(1)
            track.miss_count = 0
            track.prediction_only = False
            self._update_confirmation(track)
            used_tracks.add(track_index)
            used_observations.add(observation_index)
        for track_index, track in enumerate(self.tracks):
            if track_index in used_tracks:
                continue
            track.history.append(0)
            track.miss_count += 1
            track.prediction_only = track.miss_count <= 8
            self._update_confirmation(track)
        self.tracks = [track for track in self.tracks if track.miss_count < 20]
        for observation_index, observation in enumerate(observations):
            if observation_index in used_observations:
                continue
            if len(self.tracks) >= self.max_tracks:
                self.overflow_count += 1
                continue
            track = Track(self._next_id, observation.x_m, observation.y_m,
                          observation.covariance_xx, observation.covariance_yy,
                          history=[1], far=observation.far)
            self._next_id += 1
            self._update_confirmation(track)
            self.tracks.append(track)

    def merge_overlaps(self, distance_m: float) -> None:
        survivors: list[Track] = []
        for track in sorted(self.tracks, key=lambda item: item.cone_id):
            overlap = next((existing for existing in survivors
                            if existing.confirmed and track.confirmed
                            and math.hypot(existing.x_m - track.x_m,
                                           existing.y_m - track.y_m) <= distance_m), None)
            if overlap is None:
                survivors.append(track)
            else:
                overlap.history = [max(a, b) for a, b in zip(
                    overlap.history[-6:], track.history[-6:])]
                overlap.miss_count = min(overlap.miss_count, track.miss_count)
        self.tracks = survivors


def lock_gap_map(tracks: list[Track], start_xy: tuple[float, float],
                 turn_xy: tuple[float, float]) -> GapMap:
    """Freeze confirmed cones by outbound axis and construct adjacent N-1 gaps."""
    axis_x = turn_xy[0] - start_xy[0]
    axis_y = turn_xy[1] - start_xy[1]
    axis_length = math.hypot(axis_x, axis_y)
    if axis_length <= 0.0:
        raise ValueError("start and turnaround points must differ")
    axis_x /= axis_length
    axis_y /= axis_length
    frozen = [track for track in tracks if track.confirmed]
    frozen.sort(key=lambda track: ((track.x_m - start_xy[0]) * axis_x
                                   + (track.y_m - start_xy[1]) * axis_y,
                                   track.cone_id))
    gaps: list[Gap] = []
    heading = math.atan2(-axis_y, -axis_x)
    if heading <= -math.pi:
        heading = math.pi
    for index, (left, right) in enumerate(zip(frozen, frozen[1:]), start=1):
        center_x = (left.x_m + right.x_m) * 0.5
        center_y = (left.y_m + right.y_m) * 0.5
        gaps.append(Gap(index, left.cone_id, right.cone_id, center_x, center_y,
                        heading, center_x + axis_x * 0.6, center_y + axis_y * 0.6,
                        center_x - axis_x * 0.6, center_y - axis_y * 0.6))
    return GapMap(gaps, list(reversed(gaps)))


class PerceptionStateMachine:
    def __init__(self) -> None:
        self.state = "READY"

    def step(self, *, task_start: bool = False, turnaround: bool = False,
             confirmed_cones: int = 0, complete: bool = False,
             critical_fault: bool = False) -> str:
        if critical_fault:
            self.state = "SAFE_STOP"
        elif self.state == "READY" and task_start:
            self.state = "OUTBOUND_MAP"
        elif self.state == "OUTBOUND_MAP" and turnaround:
            self.state = "TURN_LOCK"
        elif self.state == "TURN_LOCK":
            self.state = "RETURN_GAPS" if confirmed_cones >= 2 else "SAFE_STOP"
        elif self.state == "RETURN_GAPS" and complete:
            self.state = "FINISH"
        return self.state

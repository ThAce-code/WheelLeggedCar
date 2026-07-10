#!/usr/bin/env python3
"""Evaluate wheel-leg IK calibration samples and print candidate offsets.

This script is intentionally offline-only: it never edits firmware files.  The
first fitting pass adjusts only x/y model offsets from direct LIK samples; link
length and branch changes must be accepted manually after reviewing residuals.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class KinematicsConfig:
    l1_mm: float = 90.0
    l2_mm: float = 60.0
    l3_mm: float = 60.0
    l4_mm: float = 90.0
    l5_mm: float = 38.0
    x_min_mm: float = -35.0
    x_max_mm: float = 35.0
    y_min_mm: float = 35.0
    y_max_mm: float = 150.0
    x_offset_mm: float = 0.0
    y_offset_mm: float = 0.0


@dataclass(frozen=True)
class Sample:
    sample_id: str
    label: str
    servo: Tuple[float, float, float, float]
    measured_x_mm: float
    measured_y_mm: float


@dataclass(frozen=True)
class Prediction:
    sample: Sample
    predicted_x_mm: float
    predicted_y_mm: float
    error_x_mm: float
    error_y_mm: float


def finite_float(value: str) -> Optional[float]:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def first_float(row: dict, names: Sequence[str]) -> Optional[float]:
    for name in names:
        if name in row:
            parsed = finite_float(row[name])
            if parsed is not None:
                return parsed
    return None


def read_samples(path: Path) -> List[Sample]:
    samples: List[Sample] = []
    with path.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            note = (row.get("note") or "").lower()
            if "skip" in note:
                continue
            measured_x = first_float(row, ("measured_x_mm", "x_mm"))
            measured_y = first_float(row, ("measured_y_mm", "y_mm"))
            servo = (
                first_float(row, ("cmd_a0_deg", "servo0_output_deg", "servo0_deg")),
                first_float(row, ("cmd_a1_deg", "servo1_output_deg", "servo1_deg")),
                first_float(row, ("cmd_a2_deg", "servo2_output_deg", "servo2_deg")),
                first_float(row, ("cmd_a3_deg", "servo3_output_deg", "servo3_deg")),
            )
            if measured_x is None or measured_y is None or any(v is None for v in servo):
                continue
            samples.append(
                Sample(
                    sample_id=row.get("sample_id", str(len(samples))),
                    label=row.get("label", ""),
                    servo=(servo[0], servo[1], servo[2], servo[3]),  # type: ignore[arg-type]
                    measured_x_mm=measured_x,
                    measured_y_mm=measured_y,
                )
            )
    return samples


def parse_current_config(config_path: Path) -> KinematicsConfig:
    if not config_path.exists():
        return KinematicsConfig()

    text = config_path.read_text(encoding="utf-8", errors="ignore")
    pattern = re.compile(
        r"(?P<l1>[-+]?\d+(?:\.\d+)?)f,\s*/\*\s*L1[\s\S]*?"
        r"(?P<l2>[-+]?\d+(?:\.\d+)?)f,\s*/\*\s*L2[\s\S]*?"
        r"(?P<l3>[-+]?\d+(?:\.\d+)?)f,\s*/\*\s*L3[\s\S]*?"
        r"(?P<l4>[-+]?\d+(?:\.\d+)?)f,\s*/\*\s*L4[\s\S]*?"
        r"(?P<l5>[-+]?\d+(?:\.\d+)?)f,\s*/\*\s*L5[\s\S]*?"
        r"(?P<x_min>[-+]?\d+(?:\.\d+)?)f,\s*"
        r"(?P<x_max>[-+]?\d+(?:\.\d+)?)f,\s*"
        r"(?P<y_min>[-+]?\d+(?:\.\d+)?)f,\s*"
        r"(?P<y_max>[-+]?\d+(?:\.\d+)?)f,\s*"
        r"(?P<x_offset>[-+]?\d+(?:\.\d+)?)f,\s*"
        r"(?P<y_offset>[-+]?\d+(?:\.\d+)?)f,",
        re.MULTILINE,
    )
    match = pattern.search(text)
    if not match:
        return KinematicsConfig()

    values = {name: float(value) for name, value in match.groupdict().items()}
    return KinematicsConfig(
        l1_mm=values["l1"],
        l2_mm=values["l2"],
        l3_mm=values["l3"],
        l4_mm=values["l4"],
        l5_mm=values["l5"],
        x_min_mm=values["x_min"],
        x_max_mm=values["x_max"],
        y_min_mm=values["y_min"],
        y_max_mm=values["y_max"],
        x_offset_mm=values["x_offset"],
        y_offset_mm=values["y_offset"],
    )


def circle_fk_candidates(cfg: KinematicsConfig, servo_a_deg: float, servo_b_deg: float) -> List[Tuple[float, float, str]]:
    alpha = math.radians(servo_a_deg)
    beta = math.radians(servo_b_deg)
    c_x = cfg.l1_mm * math.cos(alpha)
    c_y = cfg.l1_mm * math.sin(alpha)
    d_x = cfg.l5_mm + cfg.l4_mm * math.cos(beta)
    d_y = cfg.l4_mm * math.sin(beta)
    dx = d_x - c_x
    dy = d_y - c_y
    distance = math.hypot(dx, dy)
    if distance <= 1e-6:
        return []

    projection = ((cfg.l2_mm * cfg.l2_mm) - (cfg.l3_mm * cfg.l3_mm) + (distance * distance)) / (2.0 * distance)
    root_term = (cfg.l2_mm * cfg.l2_mm) - (projection * projection)
    if root_term < 0.0:
        return []

    height = math.sqrt(max(root_term, 0.0))
    base_x = c_x + projection * dx / distance
    base_y = c_y + projection * dy / distance
    candidates = [
        (base_x - dy * height / distance, base_y + dx * height / distance, "plus"),
        (base_x + dy * height / distance, base_y - dx * height / distance, "minus"),
    ]
    valid = []
    for x_mm, y_mm, root in candidates:
        if y_mm <= 0.0:
            continue
        if not (cfg.x_min_mm <= x_mm - cfg.x_offset_mm <= cfg.x_max_mm):
            continue
        if not (cfg.y_min_mm <= y_mm - cfg.y_offset_mm <= cfg.y_max_mm):
            continue
        valid.append((x_mm, y_mm, root))
    return valid


def forward_pair(cfg: KinematicsConfig, servo_a_deg: float, servo_b_deg: float) -> Optional[Tuple[float, float]]:
    candidates = circle_fk_candidates(cfg, servo_a_deg, servo_b_deg)
    if not candidates:
        return None

    def sort_key(candidate: Tuple[float, float, str]) -> Tuple[float, int]:
        x_mm, _y_mm, root = candidate
        root_order = 0 if root == "minus" else 1
        return (abs(x_mm - cfg.x_offset_mm), root_order)

    selected_x, selected_y, _root = sorted(candidates, key=sort_key)[0]
    return selected_x - cfg.x_offset_mm, selected_y - cfg.y_offset_mm


def predict_sample(cfg: KinematicsConfig, sample: Sample) -> Optional[Tuple[float, float]]:
    left = forward_pair(cfg, sample.servo[0], sample.servo[2])
    right = forward_pair(cfg, sample.servo[1], sample.servo[3])
    pairs = [pair for pair in (left, right) if pair is not None]
    if not pairs:
        return None
    return (
        sum(pair[0] for pair in pairs) / len(pairs),
        sum(pair[1] for pair in pairs) / len(pairs),
    )


def evaluate(cfg: KinematicsConfig, samples: Iterable[Sample]) -> List[Prediction]:
    predictions: List[Prediction] = []
    for sample in samples:
        predicted = predict_sample(cfg, sample)
        if predicted is None:
            continue
        pred_x, pred_y = predicted
        predictions.append(
            Prediction(
                sample=sample,
                predicted_x_mm=pred_x,
                predicted_y_mm=pred_y,
                error_x_mm=pred_x - sample.measured_x_mm,
                error_y_mm=pred_y - sample.measured_y_mm,
            )
        )
    return predictions


def rmse(values: Sequence[float]) -> float:
    if not values:
        return float("nan")
    return math.sqrt(sum(value * value for value in values) / len(values))


def fit_offset_candidate(cfg: KinematicsConfig, predictions: Sequence[Prediction]) -> KinematicsConfig:
    if not predictions:
        return cfg
    mean_error_x = sum(pred.error_x_mm for pred in predictions) / len(predictions)
    mean_error_y = sum(pred.error_y_mm for pred in predictions) / len(predictions)
    return replace(
        cfg,
        x_offset_mm=cfg.x_offset_mm + mean_error_x,
        y_offset_mm=cfg.y_offset_mm + mean_error_y,
    )


def print_summary(name: str, predictions: Sequence[Prediction]) -> None:
    print(f"{name}_predicted_rows={len(predictions)}")
    print(f"{name}_rmse_x_mm={rmse([pred.error_x_mm for pred in predictions]):.3f}")
    print(f"{name}_rmse_y_mm={rmse([pred.error_y_mm for pred in predictions]):.3f}")
    if predictions:
        worst = max(predictions, key=lambda pred: abs(pred.error_y_mm))
        print(
            f"{name}_worst_y_error sample={worst.sample.sample_id} label={worst.sample.label} "
            f"error_y_mm={worst.error_y_mm:.3f}"
        )


def print_candidate_config(cfg: KinematicsConfig) -> None:
    print("candidate_leg_config")
    print("/* candidate kinematics block; review before copying into project/code/leg_config.c")
    print(f" * l1_mm = {cfg.l1_mm:.3f}f")
    print(f" * l2_mm = {cfg.l2_mm:.3f}f")
    print(f" * l3_mm = {cfg.l3_mm:.3f}f")
    print(f" * l4_mm = {cfg.l4_mm:.3f}f")
    print(f" * l5_mm = {cfg.l5_mm:.3f}f")
    print(f" * x_offset_mm = {cfg.x_offset_mm:.3f}f")
    print(f" * y_offset_mm = {cfg.y_offset_mm:.3f}f")
    print(" */")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="Calibration CSV from calib_ik_servo.ps1")
    parser.add_argument("--config", default=Path("project/code/leg_config.c"), type=Path)
    parser.add_argument("--max-iter", default=100, type=int, help="Reserved for future nonlinear fitting")
    args = parser.parse_args()

    samples = read_samples(args.input)
    cfg = parse_current_config(args.config)
    current_predictions = evaluate(cfg, samples)
    candidate = fit_offset_candidate(cfg, current_predictions)
    candidate_predictions = evaluate(candidate, samples)

    print(f"usable_rows={len(samples)}")
    print_summary("current", current_predictions)
    print_summary("candidate", candidate_predictions)
    print(f"rmse_x_mm={rmse([pred.error_x_mm for pred in candidate_predictions]):.3f}")
    print(f"rmse_y_mm={rmse([pred.error_y_mm for pred in candidate_predictions]):.3f}")
    print_candidate_config(candidate)

    if len(samples) == 0:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

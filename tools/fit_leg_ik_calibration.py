#!/usr/bin/env python3
"""Evaluate wheel-leg IK calibration samples and print candidate offsets.

Now with train/validation split (requirement 10):
  - Fits kinematics offsets on a TRAINING subset only
  - Evaluates RMSE on an independent VALIDATION subset
  - Reports both train and val metrics separately

Usage:
    python tools/fit_leg_ik_calibration.py --input data/ik_calib.csv
    python tools/fit_leg_ik_calibration.py --input data.csv --val-split 0.2
    python tools/fit_leg_ik_calibration.py --input data.csv --val-samples 5
    python tools/fit_leg_ik_calibration.py --input data.csv --kfold 5
    python tools/fit_leg_ik_calibration.py --input data.csv --validation-report report.json
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
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
                    servo=(servo[0], servo[1], servo[2], servo[3]),
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


def circle_fk_candidates(cfg: KinematicsConfig, servo_a_deg: float, servo_b_deg: float
                         ) -> List[Tuple[float, float, str]]:
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


def forward_pair(cfg: KinematicsConfig, servo_a_deg: float, servo_b_deg: float
                 ) -> Optional[Tuple[float, float]]:
    candidates = circle_fk_candidates(cfg, servo_a_deg, servo_b_deg)
    if not candidates:
        return None
    def sort_key(c: Tuple[float, float, str]) -> Tuple[float, int]:
        return (abs(c[0] - cfg.x_offset_mm), 0 if c[2] == "minus" else 1)
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


# =======================================================================
# Train / validation split helpers (requirement 10)
# =======================================================================

def split_train_val(
    samples: List[Sample],
    val_split: float = 0.2,
    val_count: int = 0,
    seed: int = 42,
) -> Tuple[List[Sample], List[Sample]]:
    """Split samples into training and validation sets.

    Stratified by label when possible -- each unique label appears
    proportionally in both sets.

    Args:
        samples: All samples.
        val_split: Fraction of samples for validation (0.0-1.0).
        val_count: Absolute number of validation samples (overrides val_split if > 0).
        seed: Random seed for reproducibility.

    Returns:
        (train_samples, val_samples)
    """
    if len(samples) < 2:
        return list(samples), []

    rng = random.Random(seed)

    # Group by label for stratified split
    from collections import defaultdict
    by_label: dict[str, List[Sample]] = defaultdict(list)
    for s in samples:
        by_label[s.label].append(s)

    train: List[Sample] = []
    val: List[Sample] = []

    total_val = val_count if val_count > 0 else max(1, int(len(samples) * val_split))

    # Try stratified: take proportional val samples from each label group
    allocated = 0
    label_items = sorted(by_label.items(), key=lambda x: len(x[1]), reverse=True)

    for i, (label, group) in enumerate(label_items):
        group_copy = list(group)
        rng.shuffle(group_copy)

        if i == len(label_items) - 1:
            # Last group: take whatever remains to hit the target
            n_val = total_val - allocated
        else:
            n_val = max(0, min(len(group_copy) - 1,
                               int(len(group_copy) / len(samples) * total_val)))

        n_val = max(0, min(n_val, len(group_copy) - 1))
        val.extend(group_copy[:n_val])
        train.extend(group_copy[n_val:])
        allocated += n_val

    # If we didn't hit the target (e.g. single label), simple random split
    if len(val) < 1 and len(samples) >= 2:
        rng.shuffle(samples)
        val_count_actual = max(1, int(len(samples) * val_split))
        return samples[val_count_actual:], samples[:val_count_actual]

    return train, val


def kfold_splits(samples: List[Sample], k: int = 5, seed: int = 42
                 ) -> List[Tuple[List[Sample], List[Sample]]]:
    """Generate k train/validation fold pairs.

    Returns k tuples of (train_samples, val_samples).
    """
    rng = random.Random(seed)
    shuffled = list(samples)
    rng.shuffle(shuffled)

    fold_size = len(shuffled) // k
    folds: List[Tuple[List[Sample], List[Sample]]] = []

    for i in range(k):
        val_start = i * fold_size
        val_end = (i + 1) * fold_size if i < k - 1 else len(shuffled)
        val = shuffled[val_start:val_end]
        train = shuffled[:val_start] + shuffled[val_end:]
        folds.append((train, val))

    return folds


def print_summary(name: str, predictions: Sequence[Prediction]) -> None:
    print(f"  {name}_n={len(predictions)}")
    print(f"  {name}_rmse_x_mm={rmse([p.error_x_mm for p in predictions]):.3f}")
    print(f"  {name}_rmse_y_mm={rmse([p.error_y_mm for p in predictions]):.3f}")
    if predictions:
        worst = max(predictions, key=lambda p: abs(p.error_y_mm))
        print(f"  {name}_worst_y sample={worst.sample.sample_id} "
              f"label={worst.sample.label} error_y_mm={worst.error_y_mm:.3f}")


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
    parser = argparse.ArgumentParser(
        description="Evaluate IK calibration with train/validation split",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--config", default=Path("project/code/leg_config.c"), type=Path)
    parser.add_argument("--val-split", default=0.2, type=float,
                        help="Fraction of samples for validation (default: 0.2)")
    parser.add_argument("--val-samples", default=0, type=int,
                        help="Absolute number of validation samples (overrides --val-split)")
    parser.add_argument("--kfold", default=0, type=int,
                        help="Run k-fold cross-validation (e.g. --kfold 5)")
    parser.add_argument("--seed", default=42, type=int,
                        help="Random seed for splits (default: 42)")
    parser.add_argument("--no-split", action="store_true",
                        help="Use all data for both fit and eval (legacy behavior)")
    parser.add_argument("--validation-report", type=Path, default=None,
                        help="Path to validate_measurement.py JSON report. "
                             "If provided, prints MAE/RMSE/max_error/repeatability "
                             "before fitting.")
    args = parser.parse_args()

    # -- Read and display validation report if provided --
    if args.validation_report:
        if not args.validation_report.exists():
            print(f"[ERROR] Validation report not found: {args.validation_report}")
            return 1
        try:
            report = json.loads(args.validation_report.read_text(encoding="utf-8"))
            required = ["mae_mm", "rmse_mm", "max_error_mm", "repeatability_std_mm"]
            missing = [k for k in required if k not in report]
            if missing:
                print(f"[ERROR] Validation report missing required fields: {missing}")
                print(f"  Found fields: {list(report.keys())}")
                return 1
            print(f"\n{'='*60}")
            print(f"  MEASUREMENT SYSTEM VALIDATION (pre-fit gate)")
            print(f"  Report: {args.validation_report}")
            print(f"{'='*60}")
            print(f"  MAE:                {report['mae_mm']:.3f} mm")
            print(f"  RMSE:               {report['rmse_mm']:.3f} mm")
            print(f"  Max error:          {report['max_error_mm']:.3f} mm")
            print(f"  Repeatability std:  {report.get('repeatability_std_mm', 0):.3f} mm")
            print(f"{'='*60}")
            print(f"  Reference thresholds (guidance only, not enforced):")
            if report['mae_mm'] <= 1.0:
                print(f"    MAE <= 1.0 mm  : Excellent")
            elif report['mae_mm'] <= 2.0:
                print(f"    MAE 1.0-2.0 mm : Usually acceptable")
            else:
                print(f"    MAE > 3.0 mm   : Recommend investigating measurement system")
        except (json.JSONDecodeError, KeyError) as exc:
            print(f"[ERROR] Failed to parse validation report: {exc}")
            return 1

    samples = read_samples(args.input)
    if len(samples) == 0:
        print("[ERROR] No usable samples found.")
        return 2

    cfg = parse_current_config(args.config)

    if args.no_split:
        # -- Legacy: fit and evaluate on all data --
        print(f"\n{'='*60}")
        print(f"  IK FIT (legacy: all {len(samples)} samples)")
        print(f"{'='*60}")
        current_pred = evaluate(cfg, samples)
        candidate = fit_offset_candidate(cfg, current_pred)
        candidate_pred = evaluate(candidate, samples)
        print_summary("current", current_pred)
        print_summary("candidate", candidate_pred)
        print_candidate_config(candidate)
        print(f"\n[WARN] Fit + eval on same data -- optimistic. "
              f"Use --val-split for honest metrics.")
        return 0

    if args.kfold > 0:
        # -- K-fold cross-validation --
        print(f"\n{'='*60}")
        print(f"  IK FIT -- {args.kfold}-FOLD CROSS-VALIDATION")
        print(f"  Total samples: {len(samples)}")
        print(f"{'='*60}")

        folds = kfold_splits(samples, args.kfold, args.seed)
        fold_train_rmse_x: List[float] = []
        fold_train_rmse_y: List[float] = []
        fold_val_rmse_x: List[float] = []
        fold_val_rmse_y: List[float] = []
        all_candidates: List[KinematicsConfig] = []

        for i, (train, val) in enumerate(folds):
            # Fit on training fold
            train_pred = evaluate(cfg, train)
            candidate = fit_offset_candidate(cfg, train_pred)
            all_candidates.append(candidate)

            # Evaluate on training fold
            train_metrics = evaluate(candidate, train)
            tr_x = rmse([p.error_x_mm for p in train_metrics])
            tr_y = rmse([p.error_y_mm for p in train_metrics])

            # Evaluate on validation fold (UNSEEN data)
            val_pred = evaluate(candidate, val)
            v_x = rmse([p.error_x_mm for p in val_pred])
            v_y = rmse([p.error_y_mm for p in val_pred])

            fold_train_rmse_x.append(tr_x)
            fold_train_rmse_y.append(tr_y)
            fold_val_rmse_x.append(v_x)
            fold_val_rmse_y.append(v_y)

            print(f"\n  Fold {i+1}/{args.kfold}: "
                  f"train n={len(train)} val n={len(val)}")
            print(f"    train RMSE: x={tr_x:.3f}  y={tr_y:.3f} mm")
            print(f"    val   RMSE: x={v_x:.3f}  y={v_y:.3f} mm")
            print(f"    x_offset={candidate.x_offset_mm:.3f}  "
                  f"y_offset={candidate.y_offset_mm:.3f}")

        print(f"\n{'-'*60}")
        print(f"  CROSS-VALIDATION SUMMARY ({args.kfold} folds)")
        print(f"  Train RMSE x: "
              f"{sum(fold_train_rmse_x)/len(fold_train_rmse_x):.3f} +/- "
              f"{np_std(fold_train_rmse_x):.3f} mm")
        print(f"  Train RMSE y: "
              f"{sum(fold_train_rmse_y)/len(fold_train_rmse_y):.3f} +/- "
              f"{np_std(fold_train_rmse_y):.3f} mm")
        print(f"  Val   RMSE x: "
              f"{sum(fold_val_rmse_x)/len(fold_val_rmse_x):.3f} +/- "
              f"{np_std(fold_val_rmse_x):.3f} mm")
        print(f"  Val   RMSE y: "
              f"{sum(fold_val_rmse_y)/len(fold_val_rmse_y):.3f} +/- "
              f"{np_std(fold_val_rmse_y):.3f} mm")

        # Report the mean candidate (averaged offsets)
        mean_x_off = sum(c.x_offset_mm for c in all_candidates) / len(all_candidates)
        mean_y_off = sum(c.y_offset_mm for c in all_candidates) / len(all_candidates)
        mean_candidate = replace(cfg, x_offset_mm=mean_x_off, y_offset_mm=mean_y_off)
        print(f"\n  Mean candidate (k={args.kfold} folds):")
        print_candidate_config(mean_candidate)

    else:
        # -- Single train/val split --
        train, val = split_train_val(samples, args.val_split, args.val_samples, args.seed)
        print(f"\n{'='*60}")
        print(f"  IK FIT -- TRAIN/VALIDATION SPLIT")
        print(f"  Total: {len(samples)}  |  "
              f"Train: {len(train)}  |  Val: {len(val)}")
        print(f"  Split: val_split={args.val_split}  seed={args.seed}")
        print(f"{'='*60}")

        if len(val) == 0:
            print("[ERROR] Validation set is empty. Reduce --val-split or add more samples.")
            return 1

        # -- Fit on TRAINING ONLY --
        train_pred = evaluate(cfg, train)
        candidate = fit_offset_candidate(cfg, train_pred)

        # -- Evaluate on TRAINING --
        train_metrics = evaluate(candidate, train)
        print(f"\n  TRAINING SET ({len(train)} samples):")
        print_summary("train", train_metrics)

        # -- Evaluate on VALIDATION (UNSEEN) --
        val_metrics = evaluate(candidate, val)
        print(f"\n  VALIDATION SET ({len(val)} samples) -- UNSEEN during fit:")
        print_summary("val", val_metrics)

        # Also evaluate current config on both
        current_train = evaluate(cfg, train)
        current_val = evaluate(cfg, val)
        print(f"\n  Current config baseline:")
        print(f"    train RMSE: x={rmse([p.error_x_mm for p in current_train]):.3f}  "
              f"y={rmse([p.error_y_mm for p in current_train]):.3f} mm")
        print(f"    val   RMSE: x={rmse([p.error_x_mm for p in current_val]):.3f}  "
              f"y={rmse([p.error_y_mm for p in current_val]):.3f} mm")

        print(f"\n{'-'*60}")
        print(f"  CANDIDATE CONFIG (fitted on TRAINING set only)")
        print_candidate_config(candidate)

        # Print individual validation predictions for manual inspection
        if val_metrics:
            print(f"\n  Validation set detail:")
            print(f"  {'label':<16} {'meas_x':>8} {'meas_y':>8} "
                  f"{'pred_x':>8} {'pred_y':>8} {'err_x':>8} {'err_y':>8}")
            for p in sorted(val_metrics, key=lambda p: abs(p.error_y_mm), reverse=True):
                print(f"  {p.sample.label:<16} "
                      f"{p.sample.measured_x_mm:>8.2f} {p.sample.measured_y_mm:>8.2f} "
                      f"{p.predicted_x_mm:>8.2f} {p.predicted_y_mm:>8.2f} "
                      f"{p.error_x_mm:>+8.3f} {p.error_y_mm:>+8.3f}")

    return 0


def np_std(values: List[float]) -> float:
    """Compute sample standard deviation."""
    if len(values) < 2:
        return 0.0
    mean = sum(values) / len(values)
    variance = sum((v - mean) ** 2 for v in values) / (len(values) - 1)
    return math.sqrt(variance)


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Fit visible-leg commands to absolute physical wheel-center coordinates.

The fixed chassis marker is physical (0, 0), +X is vehicle forward, and +Y is
downward.  The three named 90-degree reference captures anchor the fitted
similarity; they are not subtracted to create a neutral-relative coordinate
system.

Usage:
    python tools/fit_leg_ik_calibration.py \
        --input data/ik_calib_visible_leg_20pt.csv --kfold 5
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import least_squares


REFERENCE_LABELS = ("ref_start", "ref_mid", "ref_end")
NEUTRAL_COMMAND_DEG = 90.0
FIT_ROOT = "plus"


@dataclass(frozen=True)
class KinematicsConfig:
    l1_mm: float = 60.0
    l2_mm: float = 90.0
    l3_mm: float = 90.0
    l4_mm: float = 60.0
    l5_mm: float = 37.0


@dataclass(frozen=True)
class Sample:
    sample_id: str
    label: str
    servo: Tuple[float, float, float, float]
    measured_x_mm: float
    measured_y_mm: float


@dataclass(frozen=True)
class SimilarityCandidate:
    alpha_ref_deg: float
    beta_ref_deg: float
    direction_a: int
    direction_b: int
    determinant: int
    rotation_deg: float
    scale: float
    matrix: Tuple[float, float, float, float]
    reference_physical_x_mm: float
    reference_physical_y_mm: float
    reference_local_x_mm: float
    reference_local_y_mm: float
    objective_sse: float


@dataclass(frozen=True)
class Prediction:
    sample: Sample
    predicted_x_mm: float
    predicted_y_mm: float
    error_x_mm: float
    error_y_mm: float
    radial_error_mm: float


@dataclass(frozen=True)
class FitMetrics:
    predictions: Tuple[Prediction, ...]
    rmse_x_mm: float
    rmse_y_mm: float
    radial_rmse_mm: float
    max_radial_error_mm: float


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
            if "skip" in (row.get("note") or "").lower():
                continue
            measured_x = first_float(row, ("measured_x_mm", "x_mm"))
            measured_y = first_float(row, ("measured_y_mm", "y_mm"))
            servo_values = tuple(
                first_float(row, names)
                for names in (
                    ("cmd_a0_deg", "servo0_output_deg", "servo0_deg"),
                    ("cmd_a1_deg", "servo1_output_deg", "servo1_deg"),
                    ("cmd_a2_deg", "servo2_output_deg", "servo2_deg"),
                    ("cmd_a3_deg", "servo3_output_deg", "servo3_deg"),
                )
            )
            if (measured_x is None or measured_y is None or
                    any(value is None for value in servo_values)):
                continue
            samples.append(Sample(
                sample_id=row.get("sample_id", str(len(samples))),
                label=row.get("label", ""),
                servo=(servo_values[0], servo_values[1],
                       servo_values[2], servo_values[3]),
                measured_x_mm=measured_x,
                measured_y_mm=measured_y,
            ))
    return samples


def validate_sample_coverage(
    samples: Sequence[Sample], min_x_span_mm: float, min_y_span_mm: float,
) -> Optional[str]:
    if not samples:
        return "no usable samples"
    if any(sample.measured_y_mm <= 0.0 for sample in samples):
        return ("measured Y violates the vehicle coordinate convention: "
                "wheel-center down must be positive")
    x_values = [sample.measured_x_mm for sample in samples]
    y_values = [sample.measured_y_mm for sample in samples]
    x_span = max(x_values) - min(x_values)
    y_span = max(y_values) - min(y_values)
    if x_span < min_x_span_mm:
        return (f"X coverage is only {x_span:.3f} mm; require at least "
                f"{min_x_span_mm:.3f} mm")
    if y_span < min_y_span_mm:
        return (f"Y coverage is only {y_span:.3f} mm; require at least "
                f"{min_y_span_mm:.3f} mm")
    return None


def validate_reference_samples(samples: Sequence[Sample]) -> Optional[str]:
    selected = {label: [s for s in samples if s.label == label]
                for label in REFERENCE_LABELS}
    if any(len(selected[label]) != 1 for label in REFERENCE_LABELS):
        return "reference fit requires exactly one ref_start, ref_mid, and ref_end"
    for label in REFERENCE_LABELS:
        if any(abs(value - NEUTRAL_COMMAND_DEG) > 1e-6
               for value in selected[label][0].servo):
            return f"{label} must contain the four 90-degree commands"
    return None


def reference_physical_point(samples: Sequence[Sample]) -> Tuple[float, float]:
    error = validate_reference_samples(samples)
    if error is not None:
        raise ValueError(error)
    references = [sample for sample in samples
                  if sample.label in REFERENCE_LABELS]
    return (
        sum(sample.measured_x_mm for sample in references) / len(references),
        sum(sample.measured_y_mm for sample in references) / len(references),
    )


def validate_reference_drift(
    samples: Sequence[Sample], max_drift_mm: float,
) -> Optional[str]:
    start = [sample for sample in samples if sample.label == "ref_start"]
    end = [sample for sample in samples if sample.label == "ref_end"]
    if not start and not end:
        return None
    if len(start) != 1 or len(end) != 1:
        return "reference drift check requires exactly one ref_start and ref_end"
    drift_mm = math.hypot(
        end[0].measured_x_mm - start[0].measured_x_mm,
        end[0].measured_y_mm - start[0].measured_y_mm)
    if drift_mm > max_drift_mm:
        return (f"reference drift is {drift_mm:.3f} mm; limit is "
                f"{max_drift_mm:.3f} mm")
    return None


def reference_drift_mm(samples: Sequence[Sample]) -> float:
    start = next(sample for sample in samples if sample.label == "ref_start")
    end = next(sample for sample in samples if sample.label == "ref_end")
    return math.hypot(end.measured_x_mm - start.measured_x_mm,
                      end.measured_y_mm - start.measured_y_mm)


def parse_current_config(config_path: Path) -> KinematicsConfig:
    if not config_path.exists():
        return KinematicsConfig()
    text = config_path.read_text(encoding="utf-8", errors="ignore")
    values = {}
    for name in ("l1_mm", "l2_mm", "l3_mm", "l4_mm", "l5_mm"):
        match = re.search(
            rf"\.{name}\s*=\s*([-+]?\d+(?:\.\d+)?)f", text)
        if match is None:
            return KinematicsConfig()
        values[name] = float(match.group(1))
    return KinematicsConfig(**values)


def circle_fk_candidates(
    cfg: KinematicsConfig, alpha_deg: float, beta_deg: float,
) -> List[Tuple[float, float, str]]:
    alpha = math.radians(alpha_deg)
    beta = math.radians(beta_deg)
    c_x = cfg.l1_mm * math.cos(alpha)
    c_y = cfg.l1_mm * math.sin(alpha)
    d_x = cfg.l5_mm + cfg.l4_mm * math.cos(beta)
    d_y = cfg.l4_mm * math.sin(beta)
    dx = d_x - c_x
    dy = d_y - c_y
    distance = math.hypot(dx, dy)
    if distance <= 1e-9:
        return []
    projection = ((cfg.l2_mm ** 2 - cfg.l3_mm ** 2 + distance ** 2) /
                  (2.0 * distance))
    root_term = cfg.l2_mm ** 2 - projection ** 2
    if root_term < -1e-7:
        return []
    height = math.sqrt(max(root_term, 0.0))
    base_x = c_x + projection * dx / distance
    base_y = c_y + projection * dy / distance
    return [
        (base_x - dy * height / distance,
         base_y + dx * height / distance, "plus"),
        (base_x + dy * height / distance,
         base_y - dx * height / distance, "minus"),
    ]


def five_bar_forward(
    cfg: KinematicsConfig, alpha_deg: float, beta_deg: float,
    root: str = FIT_ROOT,
) -> Optional[Tuple[float, float]]:
    for x_mm, y_mm, candidate_root in circle_fk_candidates(
            cfg, alpha_deg, beta_deg):
        if candidate_root == root and y_mm > 0.0:
            return (x_mm, y_mm)
    return None


def similarity_matrix(rotation_deg: float, determinant: int
                      ) -> Tuple[float, float, float, float]:
    angle = math.radians(rotation_deg)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return (cosine, -determinant * sine,
            sine, determinant * cosine)


def _candidate_from_parameters(
    cfg: KinematicsConfig,
    parameters: Sequence[float],
    direction_a: int,
    direction_b: int,
    determinant: int,
    reference_physical: Tuple[float, float],
    objective_sse: float,
) -> Optional[SimilarityCandidate]:
    alpha_ref_deg, beta_ref_deg, rotation_deg, log_scale = parameters
    reference_local = five_bar_forward(
        cfg, alpha_ref_deg, beta_ref_deg, FIT_ROOT)
    if reference_local is None:
        return None
    return SimilarityCandidate(
        alpha_ref_deg=float(alpha_ref_deg),
        beta_ref_deg=float(beta_ref_deg),
        direction_a=direction_a,
        direction_b=direction_b,
        determinant=determinant,
        rotation_deg=float(rotation_deg),
        scale=float(math.exp(log_scale)),
        matrix=similarity_matrix(rotation_deg, determinant),
        reference_physical_x_mm=reference_physical[0],
        reference_physical_y_mm=reference_physical[1],
        reference_local_x_mm=reference_local[0],
        reference_local_y_mm=reference_local[1],
        objective_sse=float(objective_sse),
    )


def model_to_physical(
    candidate: SimilarityCandidate, x_mm: float, y_mm: float,
) -> Tuple[float, float]:
    m00, m01, m10, m11 = candidate.matrix
    dx = x_mm - candidate.reference_local_x_mm
    dy = y_mm - candidate.reference_local_y_mm
    return (
        candidate.reference_physical_x_mm +
        candidate.scale * (m00 * dx + m01 * dy),
        candidate.reference_physical_y_mm +
        candidate.scale * (m10 * dx + m11 * dy),
    )


def physical_to_model(
    candidate: SimilarityCandidate, x_mm: float, y_mm: float,
) -> Tuple[float, float]:
    m00, m01, m10, m11 = candidate.matrix
    dx = (x_mm - candidate.reference_physical_x_mm) / candidate.scale
    dy = (y_mm - candidate.reference_physical_y_mm) / candidate.scale
    return (
        candidate.reference_local_x_mm + m00 * dx + m10 * dy,
        candidate.reference_local_y_mm + m01 * dx + m11 * dy,
    )


def predict_candidate(
    candidate: SimilarityCandidate, sample: Sample,
    cfg: Optional[KinematicsConfig] = None,
) -> Optional[Tuple[float, float]]:
    cfg = cfg or KinematicsConfig()
    alpha_deg = (candidate.alpha_ref_deg + candidate.direction_a *
                 (sample.servo[0] - NEUTRAL_COMMAND_DEG))
    beta_deg = (candidate.beta_ref_deg + candidate.direction_b *
                (sample.servo[2] - NEUTRAL_COMMAND_DEG))
    local = five_bar_forward(cfg, alpha_deg, beta_deg, FIT_ROOT)
    if local is None:
        return None
    return model_to_physical(candidate, *local)


def _fit_residuals(
    parameters: Sequence[float],
    samples: Sequence[Sample],
    cfg: KinematicsConfig,
    direction_a: int,
    direction_b: int,
    determinant: int,
    reference_physical: Tuple[float, float],
) -> np.ndarray:
    candidate = _candidate_from_parameters(
        cfg, parameters, direction_a, direction_b, determinant,
        reference_physical, 0.0)
    if candidate is None:
        return np.full(len(samples) * 2, 1000.0, dtype=np.float64)
    residuals = []
    for sample in samples:
        predicted = predict_candidate(candidate, sample, cfg)
        if predicted is None:
            residuals.extend((1000.0, 1000.0))
        else:
            residuals.extend((predicted[0] - sample.measured_x_mm,
                              predicted[1] - sample.measured_y_mm))
    return np.asarray(residuals, dtype=np.float64)


def fit_similarity_candidate(
    samples: Sequence[Sample],
    cfg: Optional[KinematicsConfig] = None,
    reference_physical: Optional[Tuple[float, float]] = None,
) -> SimilarityCandidate:
    if len(samples) < 4:
        raise ValueError("at least four samples are required")
    cfg = cfg or KinematicsConfig()
    if reference_physical is None:
        reference_physical = reference_physical_point(samples)

    starts = (
        (170.0, 0.0, 175.0, math.log(0.95)),
        (150.0, -20.0, 0.0, 0.0),
        (190.0, 20.0, -180.0, 0.0),
    )
    lower = np.asarray((120.0, -60.0, -360.0, math.log(0.5)))
    upper = np.asarray((220.0, 60.0, 360.0, math.log(1.5)))
    best: Optional[SimilarityCandidate] = None

    for direction_a in (-1, 1):
        for direction_b in (-1, 1):
            for determinant in (-1, 1):
                for start in starts:
                    result = least_squares(
                        _fit_residuals,
                        np.asarray(start, dtype=np.float64),
                        bounds=(lower, upper),
                        args=(samples, cfg, direction_a, direction_b,
                              determinant, reference_physical),
                        max_nfev=3000,
                        xtol=1e-11,
                        ftol=1e-11,
                        gtol=1e-11,
                    )
                    candidate = _candidate_from_parameters(
                        cfg, result.x, direction_a, direction_b, determinant,
                        reference_physical, float(np.dot(result.fun, result.fun)))
                    if candidate is not None and (
                            best is None or
                            candidate.objective_sse < best.objective_sse):
                        best = candidate
    if best is None:
        raise ValueError("no finite similarity candidate could be fitted")
    return best


def _rmse(values: Sequence[float]) -> float:
    if not values:
        return float("nan")
    return math.sqrt(sum(value * value for value in values) / len(values))


def evaluate_candidate(
    candidate: SimilarityCandidate,
    samples: Iterable[Sample],
    cfg: Optional[KinematicsConfig] = None,
) -> FitMetrics:
    cfg = cfg or KinematicsConfig()
    predictions = []
    for sample in samples:
        predicted = predict_candidate(candidate, sample, cfg)
        if predicted is None:
            continue
        error_x = predicted[0] - sample.measured_x_mm
        error_y = predicted[1] - sample.measured_y_mm
        predictions.append(Prediction(
            sample=sample,
            predicted_x_mm=predicted[0],
            predicted_y_mm=predicted[1],
            error_x_mm=error_x,
            error_y_mm=error_y,
            radial_error_mm=math.hypot(error_x, error_y),
        ))
    return FitMetrics(
        predictions=tuple(predictions),
        rmse_x_mm=_rmse([item.error_x_mm for item in predictions]),
        rmse_y_mm=_rmse([item.error_y_mm for item in predictions]),
        radial_rmse_mm=_rmse([item.radial_error_mm for item in predictions]),
        max_radial_error_mm=max(
            (item.radial_error_mm for item in predictions), default=float("nan")),
    )


def convex_hull(points: Sequence[Tuple[float, float]]
                ) -> List[Tuple[float, float]]:
    unique = sorted(set(points))
    if len(unique) <= 1:
        return unique

    def cross(origin, first, second):
        return ((first[0] - origin[0]) * (second[1] - origin[1]) -
                (first[1] - origin[1]) * (second[0] - origin[0]))

    lower: List[Tuple[float, float]] = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper: List[Tuple[float, float]] = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def point_in_inset_hull(
    point: Tuple[float, float],
    hull: Sequence[Tuple[float, float]],
    inset_mm: float,
) -> bool:
    if len(hull) < 3 or inset_mm < 0.0:
        return False
    for index, first in enumerate(hull):
        second = hull[(index + 1) % len(hull)]
        edge_x = second[0] - first[0]
        edge_y = second[1] - first[1]
        edge_length = math.hypot(edge_x, edge_y)
        if edge_length <= 1e-9:
            return False
        signed_distance = (
            edge_x * (point[1] - first[1]) -
            edge_y * (point[0] - first[0])) / edge_length
        if signed_distance < inset_mm - 1e-9:
            return False
    return True


def kfold_splits(samples: Sequence[Sample], k: int, seed: int
                 ) -> List[Tuple[List[Sample], List[Sample]]]:
    if k < 2 or k > len(samples):
        raise ValueError("kfold must be between 2 and the sample count")
    shuffled = list(samples)
    random.Random(seed).shuffle(shuffled)
    buckets = [shuffled[index::k] for index in range(k)]
    result = []
    for index in range(k):
        validation = buckets[index]
        training = [item for bucket_index, bucket in enumerate(buckets)
                    if bucket_index != index for item in bucket]
        result.append((training, validation))
    return result


def print_candidate(candidate: SimilarityCandidate,
                    hull: Sequence[Tuple[float, float]]) -> None:
    m00, m01, m10, m11 = candidate.matrix
    print("\ncandidate_leg_physical_calibration")
    print("/* anchored physical-coordinate candidate; review before copying */")
    print(f".physical_reference_x_mm = {candidate.reference_physical_x_mm:.6f}f,")
    print(f".physical_reference_y_mm = {candidate.reference_physical_y_mm:.6f}f,")
    print(f".alpha_reference_deg = {candidate.alpha_ref_deg:.6f}f,")
    print(f".beta_reference_deg = {candidate.beta_ref_deg:.6f}f,")
    print(f".command_direction_a = {candidate.direction_a:.1f}f,")
    print(f".command_direction_b = {candidate.direction_b:.1f}f,")
    print(f".model_to_physical_scale = {candidate.scale:.9f}f,")
    print(f".model_to_physical_m00 = {m00:.9f}f,")
    print(f".model_to_physical_m01 = {m01:.9f}f,")
    print(f".model_to_physical_m10 = {m10:.9f}f,")
    print(f".model_to_physical_m11 = {m11:.9f}f,")
    print(f".model_reference_x_mm = {candidate.reference_local_x_mm:.6f}f,")
    print(f".model_reference_y_mm = {candidate.reference_local_y_mm:.6f}f,")
    print(f".physical_workspace_vertex_count = {len(hull)}U,")
    print(".physical_workspace = {")
    for x_mm, y_mm in hull:
        print(f"    {{{x_mm:.3f}f, {y_mm:.3f}f}},")
    print("},")
    print(".physical_workspace_inset_mm = 2.000f,")


def print_metrics(name: str, metrics: FitMetrics) -> None:
    print(f"  {name}: n={len(metrics.predictions)} "
          f"x_rmse={metrics.rmse_x_mm:.3f} mm "
          f"y_rmse={metrics.rmse_y_mm:.3f} mm "
          f"radial_rmse={metrics.radial_rmse_mm:.3f} mm "
          f"max={metrics.max_radial_error_mm:.3f} mm")


def _load_validation_report(path: Optional[Path]) -> Optional[str]:
    if path is None:
        return None
    if not path.exists():
        return f"Validation report not found: {path}"
    try:
        report = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        return f"Failed to parse validation report: {exc}"
    required = ("mae_mm", "rmse_mm", "max_error_mm",
                "repeatability_std_mm")
    missing = [name for name in required if name not in report]
    if missing:
        return f"Validation report missing required fields: {missing}"
    print("MEASUREMENT SYSTEM VALIDATION")
    for name in required:
        print(f"  {name}={float(report[name]):.3f}")
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--config", default=Path("project/code/leg_config.c"),
                        type=Path)
    parser.add_argument("--kfold", default=0, type=int)
    parser.add_argument("--seed", default=42, type=int)
    parser.add_argument("--no-split", action="store_true")
    parser.add_argument("--validation-report", type=Path, default=None)
    parser.add_argument("--min-x-span-mm", type=float, default=10.0)
    parser.add_argument("--min-y-span-mm", type=float, default=10.0)
    parser.add_argument("--max-reference-drift-mm", type=float, default=2.0)
    parser.add_argument("--expected-samples", type=int, default=20)
    args = parser.parse_args()

    report_error = _load_validation_report(args.validation_report)
    if report_error is not None:
        print(f"[ERROR] {report_error}")
        return 1

    samples = read_samples(args.input)
    if len(samples) != args.expected_samples:
        print(f"[ERROR] expected {args.expected_samples} usable samples; "
              f"found {len(samples)}")
        return 2
    reference_error = validate_reference_samples(samples)
    if reference_error is not None:
        print(f"[ERROR] {reference_error}")
        return 3
    coverage_error = validate_sample_coverage(
        samples, args.min_x_span_mm, args.min_y_span_mm)
    if coverage_error is not None:
        print(f"[ERROR] {coverage_error}")
        return 3
    drift_error = validate_reference_drift(
        samples, args.max_reference_drift_mm)
    if drift_error is not None:
        print(f"[ERROR] {drift_error}")
        return 3

    cfg = parse_current_config(args.config)
    reference = reference_physical_point(samples)
    candidate = fit_similarity_candidate(samples, cfg, reference)
    metrics = evaluate_candidate(candidate, samples, cfg)
    if len(metrics.predictions) != len(samples):
        print("[ERROR] fitted model did not predict every sample")
        return 4

    print("\nANCHORED PHYSICAL-COORDINATE FIT")
    print(f"  samples={len(samples)}")
    print(f"  reference=({reference[0]:.6f},{reference[1]:.6f}) mm")
    print(f"  reference_drift={reference_drift_mm(samples):.3f} mm")
    print(f"  directions=({candidate.direction_a},{candidate.direction_b})")
    print(f"  determinant={candidate.determinant}")
    print(f"  scale={candidate.scale:.9f}")
    print_metrics("full", metrics)

    cv_radial = []
    cv_directions = []
    if args.kfold > 0:
        print(f"\n{args.kfold}-FOLD CROSS-VALIDATION")
        for index, (training, validation) in enumerate(
                kfold_splits(samples, args.kfold, args.seed), start=1):
            fold_candidate = fit_similarity_candidate(
                training, cfg, reference)
            fold_metrics = evaluate_candidate(
                fold_candidate, validation, cfg)
            print_metrics(f"fold_{index}_validation", fold_metrics)
            print(f"    directions=({fold_candidate.direction_a},"
                  f"{fold_candidate.direction_b}) "
                  f"determinant={fold_candidate.determinant} "
                  f"scale={fold_candidate.scale:.6f}")
            cv_radial.append(fold_metrics.radial_rmse_mm)
            cv_directions.append((fold_candidate.direction_a,
                                  fold_candidate.direction_b,
                                  fold_candidate.determinant))

    errors = []
    if not 0.8 <= candidate.scale <= 1.2:
        errors.append(f"scale {candidate.scale:.6f} is outside [0.8,1.2]")
    if metrics.radial_rmse_mm > 2.0:
        errors.append(f"full radial RMSE {metrics.radial_rmse_mm:.3f} exceeds 2 mm")
    if metrics.max_radial_error_mm > 3.0:
        errors.append(f"full max error {metrics.max_radial_error_mm:.3f} exceeds 3 mm")
    if cv_radial and sum(cv_radial) / len(cv_radial) > 2.0:
        errors.append("mean cross-validation radial RMSE exceeds 2 mm")
    if cv_directions and len(set(cv_directions)) != 1:
        errors.append("direction/reflection selection is unstable across folds")
    if cv_directions and cv_directions[0] != (
            candidate.direction_a, candidate.direction_b,
            candidate.determinant):
        errors.append("full-fit direction/reflection differs from cross-validation")
    if errors:
        for error in errors:
            print(f"[ERROR] {error}")
        print("[ERROR] Refusing to print a firmware candidate.")
        return 5

    hull = convex_hull([(sample.measured_x_mm, sample.measured_y_mm)
                        for sample in samples])
    print_candidate(candidate, hull)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

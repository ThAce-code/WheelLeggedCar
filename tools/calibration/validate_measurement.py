#!/usr/bin/env python3
"""Validate camera measurement accuracy against known ground-truth distances.

Given a set of marker-pair distances measured by the camera and their
known true values (from calipers or CAD), computes:

  - MAE  (mean absolute error, mm)
  - RMSE (root-mean-square error, mm)
  - Max error (mm)
  - Repeatability std-dev (mm) when multiple measurements of the same pair exist
  - Per-pair error breakdown

Input formats:
  1. JSON with known distances and measurements:
     {
       "pairs": [
         {"id": "0-1", "true_mm": 90.0, "measurements": [89.2, 90.5, 89.8]},
         ...
       ]
     }
  2. CSV from detect_markers.py + a ground-truth JSON mapping pair IDs to distances
  3. Cross-circle IK CSV + position truth JSON keyed by sample label

Usage:
    python tools/calibration/validate_measurement.py --input validation.json
    python tools/calibration/validate_measurement.py --csv measurements.csv --truth truth.json
    python tools/calibration/validate_measurement.py --ik-csv data/validation.csv --position-truth truth.json
    python tools/calibration/validate_measurement.py --pairs "0-1=90,1-2=60,0-3=180"
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np


@dataclass
class PairResult:
    pair_id: str
    true_mm: float
    measurements: List[float]
    n: int = 0
    mean_mm: float = 0.0
    bias_mm: float = 0.0
    std_mm: float = 0.0
    max_error_mm: float = 0.0

    def compute(self):
        self.n = len(self.measurements)
        if self.n == 0:
            return
        arr = np.array(self.measurements)
        self.mean_mm = float(np.mean(arr))
        self.bias_mm = self.mean_mm - self.true_mm
        self.std_mm = float(np.std(arr, ddof=1)) if self.n >= 2 else 0.0
        self.max_error_mm = float(np.max(np.abs(arr - self.true_mm)))


@dataclass
class ValidationReport:
    pairs: List[PairResult]
    total_measurements: int = 0
    mae_mm: float = 0.0
    rmse_mm: float = 0.0
    max_error_mm: float = 0.0
    # Pooled repeatability (sqrt of mean of per-pair variances)
    repeatability_std_mm: float = 0.0

    def compute(self):
        all_errors = []
        all_vars = []
        for p in self.pairs:
            p.compute()
            for m in p.measurements:
                all_errors.append(abs(m - p.true_mm))
            if p.n >= 2:
                all_vars.append(p.std_mm ** 2)

        self.total_measurements = sum(p.n for p in self.pairs)
        if all_errors:
            self.mae_mm = float(np.mean(all_errors))
            self.rmse_mm = float(np.sqrt(np.mean(np.array(all_errors) ** 2)))
            self.max_error_mm = float(np.max(all_errors))

        # Pooled repeatability = sqrt(mean(per-pair variance))
        # Only includes pairs with >= 2 measurements
        if all_vars:
            self.repeatability_std_mm = float(np.sqrt(np.mean(all_vars)))


@dataclass
class PositionResult:
    label: str
    true_x_mm: float
    true_y_mm: float
    measured_points: List[Tuple[float, float]]
    n: int = 0
    mean_x_mm: float = 0.0
    mean_y_mm: float = 0.0
    mean_dx_mm: float = 0.0
    mean_dy_mm: float = 0.0
    repeatability_std_mm: float = 0.0
    max_error_mm: float = 0.0
    errors_mm: Optional[List[float]] = None

    def compute(self) -> None:
        points = np.asarray(self.measured_points, dtype=np.float64)
        self.n = len(points)
        self.mean_x_mm = float(np.mean(points[:, 0]))
        self.mean_y_mm = float(np.mean(points[:, 1]))
        self.mean_dx_mm = self.mean_x_mm - self.true_x_mm
        self.mean_dy_mm = self.mean_y_mm - self.true_y_mm
        deltas = points - np.array([self.true_x_mm, self.true_y_mm])
        errors = np.linalg.norm(deltas, axis=1)
        self.errors_mm = [float(value) for value in errors]
        self.max_error_mm = float(np.max(errors))
        self.repeatability_std_mm = float(np.sqrt(np.sum(
            np.var(points, axis=0, ddof=1)))) if self.n >= 2 else 0.0


@dataclass
class PositionValidationReport:
    positions: List[PositionResult]
    total_measurements: int = 0
    rmse_mm: float = 0.0
    max_error_mm: float = 0.0
    repeatability_std_mm: float = 0.0
    x_bias_span_mm: float = 0.0
    y_bias_span_mm: float = 0.0
    no_systematic_error: bool = False
    passed: bool = False

    def compute(self) -> None:
        for position in self.positions:
            position.compute()
        errors = np.asarray([
            error for position in self.positions
            for error in (position.errors_mm or [])], dtype=np.float64)
        self.total_measurements = int(errors.size)
        self.rmse_mm = float(np.sqrt(np.mean(errors ** 2)))
        self.max_error_mm = float(np.max(errors))
        self.repeatability_std_mm = float(np.sqrt(np.mean([
            position.repeatability_std_mm ** 2
            for position in self.positions])))
        x_biases = [position.mean_dx_mm for position in self.positions]
        y_biases = [position.mean_dy_mm for position in self.positions]
        self.x_bias_span_mm = float(max(x_biases) - min(x_biases))
        self.y_bias_span_mm = float(max(y_biases) - min(y_biases))
        self.no_systematic_error = (
            self.x_bias_span_mm <= 2.0 and self.y_bias_span_mm <= 2.0)
        self.passed = (
            self.repeatability_std_mm <= 1.0 and
            self.rmse_mm <= 2.0 and
            self.max_error_mm <= 3.0 and
            self.no_systematic_error)


def load_from_json(path: Path) -> List[PairResult]:
    """Load validation data from a JSON file."""
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    pairs: List[PairResult] = []
    for item in data.get("pairs", []):
        pairs.append(PairResult(
            pair_id=item["id"],
            true_mm=float(item["true_mm"]),
            measurements=[float(x) for x in item.get("measurements", [])],
        ))
    return pairs


def load_from_csv_and_truth(
    csv_path: Path,
    truth: Dict[str, float],
) -> List[PairResult]:
    """Load measurements from a detect_markers CSV, pair them by marker IDs,
    and match against ground-truth distances.

    CSV columns: label, m0_x_mm, m0_y_mm, m0_z_mm, m1_x_mm, ...
    For each row, compute distances between every pair of markers.
    Group by pair-id (e.g. "0-1") and match to truth dict.
    """
    from collections import defaultdict

    # Read CSV rows
    rows: List[dict] = []
    with open(csv_path, "r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            rows.append(row)

    # For each row, extract marker positions
    pair_measurements: Dict[str, List[float]] = defaultdict(list)

    for row in rows:
        # Find all marker_id -> (x, y, z) in this row
        markers: Dict[int, Tuple[float, float, float]] = {}
        for key in row:
            if key.startswith("m") and "_x_mm" in key:
                mid = int(key.split("_")[0][1:])  # "m0_x_mm" -> 0
                try:
                    x = float(row[f"m{mid}_x_mm"])
                    y = float(row[f"m{mid}_y_mm"])
                    z = float(row[f"m{mid}_z_mm"])
                    markers[mid] = (x, y, z)
                except (KeyError, ValueError):
                    continue

        # Compute distances between all pairs
        marker_ids = sorted(markers.keys())
        for i in range(len(marker_ids)):
            for j in range(i + 1, len(marker_ids)):
                a_id = marker_ids[i]
                b_id = marker_ids[j]
                a = np.array(markers[a_id])
                b = np.array(markers[b_id])
                dist = float(np.linalg.norm(a - b))
                pair_key = f"{a_id}-{b_id}"
                pair_measurements[pair_key].append(dist)

    # Match to ground truth
    pairs: List[PairResult] = []
    for pair_id, measurements in sorted(pair_measurements.items()):
        true_val = truth.get(pair_id)
        if true_val is None:
            # Also try reversed key
            a, b = pair_id.split("-")
            true_val = truth.get(f"{b}-{a}")
        if true_val is None:
            print(f"  [WARN] No ground truth for pair {pair_id}, skipping")
            continue
        pairs.append(PairResult(
            pair_id=pair_id,
            true_mm=true_val,
            measurements=measurements,
        ))

    return pairs


def load_cross_circle_ik_csv(csv_path: Path,
                             truth_path: Path) -> List[PositionResult]:
    """Load cross-circle IK rows and calculate 2-D position errors by label.

    Truth JSON format::

        {"positions": {"x0_y50": {"x_mm": 0, "y_mm": 50}}}

    Repeated CSV rows with the same label form the repeatability group.
    """
    from collections import defaultdict

    truth_data = json.loads(truth_path.read_text(encoding="utf-8"))
    positions = truth_data.get("positions", {})
    measured_points: Dict[str, List[Tuple[float, float]]] = defaultdict(list)
    with csv_path.open("r", newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {"label", "measured_x_mm", "measured_y_mm"}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError(
                "IK CSV requires label, measured_x_mm, measured_y_mm columns")
        for row in reader:
            label = row["label"]
            if label not in positions:
                measured_points[label].append((float("nan"), float("nan")))
                continue
            expected = positions[label]
            try:
                measured_x = float(row["measured_x_mm"])
                measured_y = float(row["measured_y_mm"])
                dx = measured_x - float(expected["x_mm"])
                dy = measured_y - float(expected["y_mm"])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"invalid position data for label {label}") from exc
            measured_points[label].append((measured_x, measured_y))
    truth_labels = set(positions)
    csv_labels = set(measured_points)
    unknown = sorted(csv_labels - truth_labels)
    if unknown:
        raise ValueError(f"unknown CSV labels: {', '.join(unknown)}")
    missing = sorted(truth_labels - csv_labels)
    if missing:
        raise ValueError(f"missing truth positions in CSV: {', '.join(missing)}")
    too_few = sorted(label for label, points in measured_points.items()
                     if len(points) < 5)
    if too_few:
        raise ValueError(
            f"at least 5 repeats required for: {', '.join(too_few)}")
    return [PositionResult(
        label=label,
        true_x_mm=float(positions[label]["x_mm"]),
        true_y_mm=float(positions[label]["y_mm"]),
        measured_points=measured_points[label])
        for label in sorted(truth_labels)]


def _position_report_payload(report: PositionValidationReport) -> dict:
    return {
        "passed": report.passed,
        "rmse_mm": report.rmse_mm,
        "max_error_mm": report.max_error_mm,
        "repeatability_std_mm": report.repeatability_std_mm,
        "x_bias_span_mm": report.x_bias_span_mm,
        "y_bias_span_mm": report.y_bias_span_mm,
        "gates": {
            "repeatability_le_1_mm": report.repeatability_std_mm <= 1.0,
            "rmse_le_2_mm": report.rmse_mm <= 2.0,
            "max_error_le_3_mm": report.max_error_mm <= 3.0,
            "no_systematic_error": report.no_systematic_error,
        },
        "positions": [{
            "label": item.label, "true_x_mm": item.true_x_mm,
            "true_y_mm": item.true_y_mm, "mean_x_mm": item.mean_x_mm,
            "mean_y_mm": item.mean_y_mm, "mean_dx_mm": item.mean_dx_mm,
            "mean_dy_mm": item.mean_dy_mm,
            "repeatability_std_mm": item.repeatability_std_mm,
            "max_error_mm": item.max_error_mm, "n": item.n,
        } for item in report.positions],
    }


def print_position_report(report: PositionValidationReport) -> None:
    print("\nCROSS-CIRCLE POSITION VALIDATION")
    print("label                 mean_dx  mean_dy  repeat_std  max_error  n")
    for item in report.positions:
        print(f"{item.label:<20} {item.mean_dx_mm:>8.3f} "
              f"{item.mean_dy_mm:>8.3f} {item.repeatability_std_mm:>11.3f} "
              f"{item.max_error_mm:>10.3f} {item.n:>2}")
    print(f"RMSE={report.rmse_mm:.3f} mm  max={report.max_error_mm:.3f} mm  "
          f"repeatability={report.repeatability_std_mm:.3f} mm")
    print(f"bias spans: X={report.x_bias_span_mm:.3f} mm  "
          f"Y={report.y_bias_span_mm:.3f} mm (limit 2.000 mm)")
    print("PASS" if report.passed else "FAIL")


def load_from_pairs_arg(pairs_str: str, measurements_json: Optional[Path] = None
                        ) -> List[PairResult]:
    """Parse --pairs "0-1=90.0,1-2=60.0,..." with optional measurements file."""
    truth: Dict[str, float] = {}
    for item in pairs_str.split(","):
        item = item.strip()
        if "=" in item:
            pid, val = item.split("=", 1)
            truth[pid.strip()] = float(val.strip())

    if measurements_json and measurements_json.exists():
        with open(measurements_json, "r", encoding="utf-8") as f:
            data = json.load(f)
        pairs: List[PairResult] = []
        for pdata in data.get("pairs", []):
            pid = pdata.get("id", "")
            if pid in truth:
                pairs.append(PairResult(
                    pair_id=pid,
                    true_mm=truth[pid],
                    measurements=[float(x) for x in pdata.get("measurements", [])],
                ))
        return pairs

    # No measurements provided -- just create the truth skeleton
    return [PairResult(pair_id=k, true_mm=v, measurements=[]) for k, v in truth.items()]


def print_report(report: ValidationReport) -> None:
    """Print formatted validation report."""
    print()
    print("=" * 72)
    print("  MEASUREMENT SYSTEM VALIDATION REPORT")
    print("=" * 72)
    print(f"  Pairs evaluated:       {len(report.pairs)}")
    print(f"  Total measurements:    {report.total_measurements}")
    print()
    print(f"  MAE:                   {report.mae_mm:.3f} mm")
    print(f"  RMSE:                  {report.rmse_mm:.3f} mm")
    print(f"  Max error:             {report.max_error_mm:.3f} mm")
    if report.repeatability_std_mm > 0:
        print(f"  Repeatability (1std):    {report.repeatability_std_mm:.3f} mm")
    print()
    print(f"  {'Pair':<12} {'True mm':>8} {'Mean mm':>8} {'Bias':>8} "
          f"{'Std':>8} {'Max err':>8} {'N':>4}")
    print(f"  {'-'*12} {'-'*8} {'-'*8} {'-'*8} {'-'*8} {'-'*8} {'-'*4}")

    for p in report.pairs:
        n_str = str(p.n) if p.n > 0 else "--"
        print(f"  {p.pair_id:<12} {p.true_mm:>8.2f} {p.mean_mm:>8.2f} "
              f"{p.bias_mm:>+8.3f} {p.std_mm:>8.3f} {p.max_error_mm:>8.3f} {n_str:>4}")

    print("=" * 72)

    # Pass/fail thresholds (typical machine vision guidance)
    print()
    if report.rmse_mm <= 1.0:
        print("  [PASS] RMSE <= 1.0 mm -- Excellent for IK calibration")
    elif report.rmse_mm <= 2.0:
        print("  ~ RMSE <= 2.0 mm -- Acceptable for IK calibration")
    elif report.rmse_mm <= 5.0:
        print("  [WARN] RMSE <= 5.0 mm -- Marginal; verify marker placement and calibration")
    else:
        print("  [FAIL] RMSE > 5.0 mm -- Unacceptable; redo camera calibration and marker setup")
    if report.repeatability_std_mm > 0:
        if report.repeatability_std_mm <= 0.5:
            print("  [PASS] Repeatability std <= 0.5 mm -- Excellent")
        elif report.repeatability_std_mm <= 1.0:
            print("  ~ Repeatability std <= 1.0 mm -- Acceptable")
        else:
            print("  [WARN] Repeatability std > 1.0 mm -- Improve camera/marker stability")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate camera measurements against ground-truth distances",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--input", type=Path, default=None,
                        help="JSON file with pairs (true_mm + measurements)")
    parser.add_argument("--csv", type=Path, default=None,
                        help="CSV from detect_markers.py")
    parser.add_argument("--truth", type=Path, default=None,
                        help="JSON mapping pair IDs to true distances: "
                             '{"0-1": 90.0, "1-2": 60.0}')
    parser.add_argument("--ik-csv", type=Path, default=None,
                        help="Cross-circle IK CSV with measured X/Y columns")
    parser.add_argument("--position-truth", type=Path, default=None,
                        help="JSON positions keyed by IK CSV label")
    parser.add_argument("--pairs", type=str, default=None,
                        help='Inline pairs: "0-1=90.0,1-2=60.0,0-3=180.0"')
    parser.add_argument("--measurements", type=Path, default=None,
                        help="JSON file with measurement arrays (used with --pairs)")
    parser.add_argument("--output", type=Path, default=None,
                        help="Save report as JSON")
    args = parser.parse_args()

    # Determine input source
    pairs: List[PairResult] = []

    if args.ik_csv or args.position_truth:
        if not args.ik_csv or not args.position_truth:
            print("[ERROR] --ik-csv and --position-truth are required together")
            return 1
        try:
            position_report = PositionValidationReport(
                load_cross_circle_ik_csv(args.ik_csv, args.position_truth))
        except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
            print(f"[ERROR] {exc}")
            return 1
        position_report.compute()
        print_position_report(position_report)
        if args.output:
            args.output.write_text(
                json.dumps(_position_report_payload(position_report), indent=2),
                encoding="utf-8")
            print(f"\nReport saved: {args.output}")
        return 0 if position_report.passed else 2
    elif args.input:
        pairs = load_from_json(args.input)
    elif args.csv and args.truth:
        if not args.truth.exists():
            print(f"[ERROR] Truth file not found: {args.truth}")
            return 1
        truth = json.loads(args.truth.read_text(encoding="utf-8"))
        pairs = load_from_csv_and_truth(args.csv, truth)
    elif args.pairs:
        pairs = load_from_pairs_arg(args.pairs, args.measurements)
    else:
        # Interactive mode: enter pairs manually
        print("Interactive validation mode.")
        print("Enter pair ID, true distance, and measurements (comma-separated).")
        print("Example: 0-1, 90.0, 89.2, 90.5, 89.8")
        print("Empty line to finish.")
        pair_list: List[dict] = []
        while True:
            line = input("> ").strip()
            if not line:
                break
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 3:
                print("  Need at least: pair_id, true_mm, measurement1")
                continue
            pid = parts[0]
            true_mm = float(parts[1])
            measurements = [float(x) for x in parts[2:]]
            pair_list.append({"id": pid, "true_mm": true_mm, "measurements": measurements})

        for pdata in pair_list:
            pairs.append(PairResult(
                pair_id=pdata["id"],
                true_mm=pdata["true_mm"],
                measurements=pdata["measurements"],
            ))

    if not pairs:
        print("[ERROR] No validation pairs provided.")
        return 1

    report = ValidationReport(pairs=pairs)
    report.compute()
    print_report(report)

    if args.output:
        output = {
            "mae_mm": report.mae_mm,
            "rmse_mm": report.rmse_mm,
            "max_error_mm": report.max_error_mm,
            "repeatability_std_mm": report.repeatability_std_mm,
            "pairs": [
                {
                    "id": p.pair_id,
                    "true_mm": p.true_mm,
                    "mean_mm": p.mean_mm,
                    "bias_mm": p.bias_mm,
                    "std_mm": p.std_mm,
                    "max_error_mm": p.max_error_mm,
                    "n": p.n,
                    "measurements": p.measurements,
                }
                for p in report.pairs
            ],
        }
        args.output.write_text(json.dumps(output, indent=2), encoding="utf-8")
        print(f"\nReport saved: {args.output}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Replay MT9V03X BMP frames through the reference cone detector."""

from __future__ import annotations

import argparse
import csv
import glob
import json
import re
import sys
import time
from pathlib import Path

import cv2
import numpy as np

if __package__ in (None, ""):
    _PROJECT_ROOT = Path(__file__).resolve().parents[2]
    if str(_PROJECT_ROOT) not in sys.path:
        sys.path.insert(0, str(_PROJECT_ROOT))

from tools.perception.reference_detector import detect
from tools.perception.reference_geometry import (
    CameraCalibration,
    VehiclePose,
    build_roi,
    horizon_curve,
)


def frame_timestamp(path: Path) -> int:
    """Use the final decimal sequence in a BMP stem as its timestamp."""
    match = re.search(r"(\d+)(?!.*\d)", path.stem)
    if match is None:
        raise ValueError(f"frame name has no numeric timestamp: {path.name}")
    return int(match.group(1))


def load_calibration(path: Path) -> CameraCalibration:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    return CameraCalibration(
        int(payload["image_size"][0]), int(payload["image_size"][1]),
        np.asarray(payload["K"], dtype=np.float64),
        np.asarray(payload["D"], dtype=np.float64), payload["model"])


def load_poses(path: Path) -> dict[int, VehiclePose]:
    poses: dict[int, VehiclePose] = {}
    with Path(path).open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            timestamp = int(row["timestamp_us"])
            poses[timestamp] = VehiclePose(
                float(row["x_m"]), float(row["y_m"]), float(row["yaw_rad"]),
                float(row["roll_rad"]), float(row["pitch_rad"]),
                float(row["camera_height_m"]))
    return poses


def _write_overlay(image: np.ndarray, result, output_path: Path) -> None:
    overlay = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    for candidate in result.candidates:
        color = (0, 255, 0) if candidate in result.accepted else (0, 180, 255)
        cv2.rectangle(overlay, (candidate.left, candidate.top),
                      (candidate.right, candidate.bottom), color, 1)
        cv2.putText(overlay, str(candidate.score), (candidate.left, candidate.top - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1, cv2.LINE_AA)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), overlay)


def replay_frames(frame_paths: list[Path], calibration: CameraCalibration,
                  poses: dict[int, VehiclePose], output_csv: Path,
                  overlay_dir: Path | None = None) -> None:
    """Replay frames in numeric timestamp order and write one deterministic row each."""
    ordered = sorted((Path(path) for path in frame_paths), key=frame_timestamp)
    output_csv = Path(output_csv)
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=[
            "timestamp_us", "image_path", "quality_allowed", "p05", "p50", "p95",
            "roi_pixels", "candidate_count", "accepted_count", "top_score",
            "top_components", "elapsed_us",
        ])
        writer.writeheader()
        for path in ordered:
            timestamp = frame_timestamp(path)
            if timestamp not in poses:
                raise ValueError(f"missing pose for frame timestamp {timestamp}")
            image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if image is None:
                raise ValueError(f"cannot read image: {path}")
            start = time.perf_counter_ns()
            horizon = horizon_curve(calibration, poses[timestamp])
            result = detect(image, build_roi(timestamp, horizon, []))
            elapsed_us = (time.perf_counter_ns() - start) // 1000
            top = result.candidates[0] if result.candidates else None
            writer.writerow({
                "timestamp_us": timestamp,
                "image_path": str(path),
                "quality_allowed": int(result.quality.map_observation_allowed),
                "p05": result.quality.p05,
                "p50": result.quality.p50,
                "p95": result.quality.p95,
                "roi_pixels": result.roi_pixel_count,
                "candidate_count": len(result.candidates),
                "accepted_count": len(result.accepted),
                "top_score": top.score if top is not None else "",
                "top_components": json.dumps(top.components, sort_keys=True) if top else "",
                "elapsed_us": elapsed_us,
            })
            if overlay_dir is not None:
                _write_overlay(image, result, Path(overlay_dir) / f"{path.stem}.png")


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay BMP cone-perception frames")
    parser.add_argument("--images", required=True, help="BMP glob pattern")
    parser.add_argument("--pose-csv", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--overlay-dir", type=Path, default=None)
    args = parser.parse_args()
    paths = [Path(path) for path in glob.glob(args.images)]
    if not paths:
        parser.error("--images matched no files")
    try:
        replay_frames(paths, load_calibration(args.calibration), load_poses(args.pose_csv),
                      args.output_csv, args.overlay_dir)
    except ValueError as exc:
        print(f"[ERROR] {exc}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

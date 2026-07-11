#!/usr/bin/env python3
"""Integrated IK calibration workflow using camera-based measurement.

Bridges between calib_ik_servo.ps1 (MCU servo commands), camera marker
detection (pose -> wheel-center mm), and fit_leg_ik_calibration.py (model fitting).

Coordinate pipeline audited (requirement 8): all measurements are computed
from undistorted marker corner points, not resampled images.

Usage:
    python tools/calibration/calibrate_with_camera.py --manual
    python tools/calibration/calibrate_with_camera.py --camera 2 --backend MSMF
    python tools/calibration/calibrate_with_camera.py --images calib_frames/
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from camera_utils import (
    add_camera_args,
    open_camera_strict,
    undistort_points,
    undistort_image_for_display,
    compute_homography,
    apply_homography,
)

_CALIB_DIR = Path(__file__).resolve().parent
_DEFAULT_CALIB = _CALIB_DIR / "camera_calib.npz"
_DEFAULT_MARKER_CONFIG = _CALIB_DIR / "marker_config.json"
_DEFAULT_OUT_CSV = Path("data/ik_calib_camera.csv")


# =======================================================================
# CalibrationData and MarkerDetector (duplicated to keep script self-contained
# but imported from detect_markers when available)
# =======================================================================

try:
    from detect_markers import CalibrationData, MarkerDetector, draw_detections
except ImportError:
    # Inline fallback
    from dataclasses import dataclass as _dc, field as _f

    @_dc
    class CalibrationData:
        camera_matrix: np.ndarray
        dist_coeffs: np.ndarray
        image_size: Tuple[int, int]
        camera_index: int = -1
        backend: str = ""
        rmse: float = 0.0

        @classmethod
        def load(cls, path: Path) -> "CalibrationData":
            if not path.exists():
                raise FileNotFoundError(f"Calibration not found: {path}")
            data = np.load(path, allow_pickle=True)
            return cls(
                camera_matrix=data["camera_matrix"],
                dist_coeffs=data["dist_coeffs"],
                image_size=(int(data["resolution"][0]), int(data["resolution"][1])),
                camera_index=int(data.get("camera_index", -1)),
                backend=str(data.get("backend", "")),
                rmse=float(data.get("rmse", 0.0)),
            )

    @_dc
    class MarkerDetection:
        marker_id: int
        corners_distorted: np.ndarray
        corners_undistorted: np.ndarray
        tvec: np.ndarray
        rvec: np.ndarray
        center_undistorted: np.ndarray

    class MarkerDetector:
        def __init__(self, calib: CalibrationData, marker_size_mm: float = 30.0):
            self.calib = calib
            self.marker_size_mm = marker_size_mm
            self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, params)
            half = marker_size_mm / 2.0
            self.marker_obj_points = np.array([
                [-half, -half, 0], [half, -half, 0],
                [half,  half, 0], [-half,  half, 0],
            ], dtype=np.float32)

        def detect(self, frame: np.ndarray) -> Dict[int, MarkerDetection]:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)
            detections: Dict[int, MarkerDetection] = {}
            if ids is None:
                return detections
            K = self.calib.camera_matrix
            D = self.calib.dist_coeffs
            for i, mid in enumerate(ids.flatten()):
                corners_dist = corners[i][0]
                corners_undist = undistort_points(corners_dist, K, D)
                success, rvec, tvec = cv2.solvePnP(
                    self.marker_obj_points, corners_undist.reshape(4, 1, 2),
                    K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not success:
                    continue
                center = corners_undist.mean(axis=0)
                detections[int(mid)] = MarkerDetection(
                    marker_id=int(mid),
                    corners_distorted=corners_dist.copy(),
                    corners_undistorted=corners_undist,
                    tvec=tvec, rvec=rvec,
                    center_undistorted=center,
                )
            return detections

    def draw_detections(display, detections, calib):
        K = calib.camera_matrix
        for mid, det in detections.items():
            cv2.polylines(display, [det.corners_undistorted.astype(np.int32)], True, (0, 255, 0), 2)
            cv2.drawFrameAxes(display, K, np.zeros(5), det.rvec, det.tvec, 15.0, 2)
            cx, cy = int(det.center_undistorted[0]), int(det.center_undistorted[1])
            d = float(np.linalg.norm(det.tvec))
            cv2.putText(display, f"ID:{mid} {d:.0f}mm",
                        (cx - 40, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return display


# =======================================================================
# Robot geometry
# =======================================================================

class RobotGeometry:
    def __init__(self, config_path: Optional[Path] = None):
        if config_path and config_path.exists():
            cfg = json.loads(config_path.read_text(encoding="utf-8"))
        else:
            cfg = {
                "origin_marker_id": 0, "wheel_marker_id": 4,
                "ground_offset_mm": 0.0,
                "markers": [
                    {"id": 0, "label": "origin",       "dx_mm": 0.0,  "dy_mm": 0.0,  "dz_mm": 0.0},
                    {"id": 1, "label": "left_servo",    "dx_mm": -90.0, "dy_mm": 0.0, "dz_mm": 0.0},
                    {"id": 2, "label": "right_servo",   "dx_mm": 90.0,  "dy_mm": 0.0, "dz_mm": 0.0},
                    {"id": 3, "label": "leg_lower",     "dx_mm": 0.0,  "dy_mm": -60.0, "dz_mm": 0.0},
                    {"id": 4, "label": "wheel_center",  "dx_mm": 0.0,  "dy_mm": -98.0, "dz_mm": 38.0},
                ],
            }
        self.origin_marker_id: int = cfg["origin_marker_id"]
        self.wheel_marker_id: int = cfg["wheel_marker_id"]

    def compute_wheel_center(self, detections: Dict[int, MarkerDetection]
                             ) -> Optional[Tuple[float, float]]:
        if self.wheel_marker_id in detections:
            det = detections[self.wheel_marker_id]
            t = det.tvec.flatten()
            x_mm = float(t[2])
            y_mm = float(-t[1])
            if self.origin_marker_id in detections:
                o = detections[self.origin_marker_id].tvec.flatten()
                x_mm -= float(o[2])
                y_mm -= float(-o[1])
            return (x_mm, y_mm)
        return None


# =======================================================================
# Calibration poses
# =======================================================================

DEFAULT_POSES = [
    (90, 90, 90, 90, "mid_center"),
    (80, 80, 80, 80, "low_center"),
    (100, 100, 100, 100, "high_center"),
    (82, 82, 78, 78, "low_fwd"),
    (78, 78, 82, 82, "low_bwd"),
    (92, 92, 88, 88, "mid_fwd"),
    (88, 88, 92, 92, "mid_bwd"),
    (102, 102, 98, 98, "high_fwd"),
    (98, 98, 102, 102, "high_bwd"),
    (78, 82, 78, 82, "low_left"),
    (82, 78, 82, 78, "low_right"),
    (88, 92, 88, 92, "mid_left"),
    (92, 88, 92, 88, "mid_right"),
]


# =======================================================================
# Manual measurement
# =======================================================================

def manual_measure(
    camera_index: int,
    backend_label: str,
    width: int,
    height: int,
    fps: float,
    marker_size_mm: float,
    calib_path: Path,
    out_csv: Path,
) -> None:
    calib = CalibrationData.load(calib_path)
    detector = MarkerDetector(calib, marker_size_mm)
    geometry = RobotGeometry(_DEFAULT_MARKER_CONFIG)

    cap, actual_backend = open_camera_strict(camera_index, backend_label, width, height, fps)

    poses = DEFAULT_POSES
    pose_index = 0
    samples: List[dict] = []
    snapshot_dir = _CALIB_DIR / "snapshots"
    snapshot_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n{'='*60}")
    print(f"  IK CALIBRATION (Camera)")
    print(f"  Camera: {camera_index} [{actual_backend}] {width}x{height}")
    print(f"  Poses: {len(poses)}")
    print(f"{'='*60}")
    print("  SPACE=capture  p=prev  s=skip  r=remove  q=quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        detections = detector.detect(frame)
        display = undistort_image_for_display(
            frame, calib.camera_matrix, calib.dist_coeffs)
        display = draw_detections(display, detections, calib)

        wheel_center = geometry.compute_wheel_center(detections)

        h, w = display.shape[:2]
        cv2.rectangle(display, (0, 0), (w, 80), (0, 0, 0), -1)

        if pose_index < len(poses):
            a0, a1, a2, a3, label = poses[pose_index]
            cv2.putText(display,
                        f"POSE {pose_index+1}/{len(poses)} [{label}] "
                        f"({a0},{a1},{a2},{a3}) deg",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            cv2.putText(display, "ALL POSES COMPLETE",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.putText(display,
                    f"Markers: {len(detections)} | Samples: {len(samples)}",
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        if wheel_center:
            cv2.putText(display,
                        f"Wheel: x={wheel_center[0]:.1f} y={wheel_center[1]:.1f} mm",
                        (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.imshow("IK Calibration", display)
        key = cv2.waitKey(1) & 0xFF

        if key in (27, ord("q")):
            break
        elif key == ord(" ") and pose_index < len(poses):
            a0, a1, a2, a3, label = poses[pose_index]
            if wheel_center:
                note = "camera_measured"
                mx, my = f"{wheel_center[0]:.2f}", f"{wheel_center[1]:.2f}"
            else:
                note = "NO_WHEEL_DETECTED"
                mx = my = ""
            samples.append({
                "sample_id": len(samples), "label": label,
                "cmd_a0_deg": a0, "cmd_a1_deg": a1,
                "cmd_a2_deg": a2, "cmd_a3_deg": a3,
                "measured_x_mm": mx, "measured_y_mm": my,
                "marker_count": len(detections),
                "marker_ids": ",".join(str(mid) for mid in sorted(detections.keys())),
                "note": note,
            })
            stamp = time.strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(str(snapshot_dir / f"pose_{pose_index:03d}_{label}_{stamp}.png"),
                        frame)
            print(f"  [{label}] {'OK' if wheel_center else 'NO DETECT'}")
            pose_index += 1
        elif key == ord("p") and pose_index > 0:
            pose_index -= 1
            if samples and samples[-1]["label"] == poses[pose_index][4]:
                samples.pop()
        elif key == ord("s") and pose_index < len(poses):
            pose_index += 1
        elif key == ord("r") and samples:
            samples.pop()
            pose_index = max(0, pose_index - 1)

    cap.release()
    cv2.destroyAllWindows()

    if samples:
        with open(out_csv, "w", newline="", encoding="utf-8") as f:
            fields = ["sample_id", "label",
                      "cmd_a0_deg", "cmd_a1_deg", "cmd_a2_deg", "cmd_a3_deg",
                      "measured_x_mm", "measured_y_mm",
                      "marker_count", "marker_ids", "note"]
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            w.writerows(samples)
        print(f"\nSaved {len(samples)} samples -> {out_csv}")
        print(f"Next: python tools/fit_leg_ik_calibration.py --input {out_csv} --val-split 0.2")


def batch_from_images(
    image_dir: Path,
    calib_path: Path,
    marker_size_mm: float,
    out_csv: Path,
) -> None:
    calib = CalibrationData.load(calib_path)
    detector = MarkerDetector(calib, marker_size_mm)
    geometry = RobotGeometry(_DEFAULT_MARKER_CONFIG)

    files = sorted(p for p in image_dir.iterdir()
                   if p.suffix.lower() in (".jpg", ".jpeg", ".png", ".bmp", ".tiff"))
    print(f"Processing {len(files)} images from {image_dir}")

    samples: List[dict] = []
    for f in files:
        img = cv2.imread(str(f))
        if img is None:
            print(f"  [SKIP] {f.name}")
            continue
        detections = detector.detect(img)
        wc = geometry.compute_wheel_center(detections)
        label = f.stem.split("_", 1)[-1] if "_" in f.stem else f.stem
        samples.append({
            "sample_id": len(samples), "label": label,
            "cmd_a0_deg": "", "cmd_a1_deg": "", "cmd_a2_deg": "", "cmd_a3_deg": "",
            "measured_x_mm": f"{wc[0]:.2f}" if wc else "",
            "measured_y_mm": f"{wc[1]:.2f}" if wc else "",
            "marker_count": len(detections),
            "marker_ids": ",".join(str(mid) for mid in sorted(detections.keys())),
            "note": f"batch:{f.name}" if wc else f"NO_DETECT:{f.name}",
        })
        print(f"  {f.name}: {'OK' if wc else 'NO DETECT'}")

    if samples:
        with open(out_csv, "w", newline="", encoding="utf-8") as f:
            fields = ["sample_id", "label",
                      "cmd_a0_deg", "cmd_a1_deg", "cmd_a2_deg", "cmd_a3_deg",
                      "measured_x_mm", "measured_y_mm",
                      "marker_count", "marker_ids", "note"]
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            w.writerows(samples)
        print(f"\nSaved {len(samples)} samples -> {out_csv}")


# =======================================================================
# Main
# =======================================================================

def main() -> int:
    parser = argparse.ArgumentParser(
        description="IK calibration with camera measurement",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    add_camera_args(parser)
    parser.add_argument("--calib", type=Path, default=_DEFAULT_CALIB)
    parser.add_argument("--marker-size", type=float, default=30.0)
    parser.add_argument("--marker-config", type=Path, default=_DEFAULT_MARKER_CONFIG)
    parser.add_argument("--output", type=Path, default=_DEFAULT_OUT_CSV)
    parser.add_argument("--manual", action="store_true", default=True)
    parser.add_argument("--images", type=Path, default=None)
    parser.add_argument("--generate-config", action="store_true")
    args = parser.parse_args()

    if args.generate_config:
        cfg = {
            "origin_marker_id": 0, "wheel_marker_id": 4,
            "markers": [
                {"id": 0, "label": "origin",       "dx_mm": 0.0,  "dy_mm": 0.0,  "dz_mm": 0.0},
                {"id": 1, "label": "left_servo",    "dx_mm": -90.0, "dy_mm": 0.0, "dz_mm": 0.0},
                {"id": 2, "label": "right_servo",   "dx_mm": 90.0,  "dy_mm": 0.0, "dz_mm": 0.0},
                {"id": 3, "label": "leg_lower",     "dx_mm": 0.0,  "dy_mm": -60.0, "dz_mm": 0.0},
                {"id": 4, "label": "wheel_center",  "dx_mm": 0.0,  "dy_mm": -98.0, "dz_mm": 38.0},
            ],
        }
        args.marker_config.write_text(json.dumps(cfg, indent=2), encoding="utf-8")
        print(f"Config saved: {args.marker_config}")
        return 0

    if not args.calib.exists():
        print(f"[ERROR] Calibration not found: {args.calib}")
        return 1

    if args.images:
        batch_from_images(args.images, args.calib, args.marker_size, args.output)
    else:
        manual_measure(args.camera, args.backend, args.width, args.height,
                       args.fps, args.marker_size, args.calib, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

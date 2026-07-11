#!/usr/bin/env python3
"""ArUco marker detection for robot position measurement.

Detects ArUco markers in camera frames (strict resolution), estimates
3D pose via PnP, and measures wheel-center position in robot coordinates.

The coordinate pipeline is audited so that H (homography) computation
and measurement points use **consistent undistorted pixel coordinates**
via cv2.undistortPoints -- never a resampled undistorted image.

Usage:
    python tools/calibration/detect_markers.py                       # live detection
    python tools/calibration/detect_markers.py --generate            # A4 marker sheet
    python tools/calibration/detect_markers.py --camera 2 --backend MSMF
    python tools/calibration/detect_markers.py --image photo.jpg     # from saved image
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from dataclasses import dataclass, field
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
from camera_calibrate import generate_aruco_a4_svg, _test_existing_markers_png
from geometry_utils import solve_pnp_checked, HomographyData, validate_homography_match

_CALIB_DIR = Path(__file__).resolve().parent
_DEFAULT_CALIB = _CALIB_DIR / "camera_calib.npz"


# =======================================================================
# Data types
# =======================================================================

@dataclass
class MarkerDetection:
    marker_id: int
    corners_distorted: np.ndarray       # (4, 2) original (distorted) pixel coords
    corners_undistorted: np.ndarray     # (4, 2) undistorted pixel coords
    tvec: np.ndarray                    # (3, 1) camera-frame translation (mm)
    rvec: np.ndarray                    # (3, 1) rotation vector
    center_undistorted: np.ndarray      # (2,) undistorted pixel coords


@dataclass
class CalibrationData:
    """Loaded camera calibration."""
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    image_size: Tuple[int, int]
    camera_index: int = -1
    backend: str = ""
    rmse: float = 0.0

    @classmethod
    def load(cls, path: Path) -> "CalibrationData":
        if not path.exists():
            raise FileNotFoundError(
                f"Calibration not found: {path}\n"
                f"Run: python tools/calibration/camera_calibrate.py"
            )
        data = np.load(path, allow_pickle=True)
        return cls(
            camera_matrix=data["camera_matrix"],
            dist_coeffs=data["dist_coeffs"],
            image_size=(int(data["resolution"][0]), int(data["resolution"][1])),
            camera_index=int(data.get("camera_index", -1)),
            backend=str(data.get("backend", "")),
            rmse=float(data.get("rmse", 0.0)),
        )


# =======================================================================
# Marker detector with audited coordinate pipeline
# =======================================================================

class MarkerDetector:
    """Detect ArUco markers with consistent undistorted coordinate handling.

    All measurement points are computed via cv2.undistortPoints on the
    original distorted corner detections -- never from a resampled
    undistorted image (requirement 8 audit).
    """

    def __init__(self, calib: CalibrationData, marker_size_mm: float = 30.0):
        self.calib = calib
        self.marker_size_mm = marker_size_mm
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, params)

        half = marker_size_mm / 2.0
        self.marker_obj_points = np.array([
            [-half, -half, 0],
            [ half, -half, 0],
            [ half,  half, 0],
            [-half,  half, 0],
        ], dtype=np.float32)

    def detect(self, frame: np.ndarray) -> Dict[int, MarkerDetection]:
        """Detect markers in a raw (distorted) frame.

        Uses solve_pnp_checked() in 'raw_pixel' domain (Mode A):
          raw ArUco corner points + K + D
        This is the simplest and least error-prone mode — no pre-undistortion,
        no risk of double-undistortion.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        detections: Dict[int, MarkerDetection] = {}
        if ids is None:
            return detections

        K = self.calib.camera_matrix
        D = self.calib.dist_coeffs

        for i, marker_id in enumerate(ids.flatten()):
            # raw_points_px: original distorted corner coordinates from ArUco detector
            raw_points_px = corners[i][0]  # (4, 2) distorted pixel coords

            # Mode A: raw_pixel — pass raw points + K + D directly to solvePnP
            success, rvec, tvec = solve_pnp_checked(
                self.marker_obj_points,
                raw_points_px,
                K,
                D,
                point_domain="raw_pixel",
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not success:
                continue

            # Derive undistorted center for homography and display
            undistorted_points_px = undistort_points(raw_points_px, K, D)
            center_undistorted_px = undistorted_points_px.mean(axis=0)

            detections[int(marker_id)] = MarkerDetection(
                marker_id=int(marker_id),
                corners_distorted=raw_points_px.copy(),
                corners_undistorted=undistorted_points_px,
                tvec=tvec,
                rvec=rvec,
                center_undistorted=center_undistorted_px,
            )

        return detections

    def measure_distance_mm(self, d1: MarkerDetection, d2: MarkerDetection) -> float:
        return float(np.linalg.norm(d1.tvec - d2.tvec))

    def compute_world_from_homography(
        self,
        detections: Dict[int, MarkerDetection],
        world_ref_points: Dict[int, Tuple[float, float]],
    ) -> Optional[Dict[int, Tuple[float, float]]]:
        """Map detected marker centers -> world coordinates via homography.

        Uses markers with known world positions as reference points to
        compute H, then applies H to all detected markers' undistorted
        pixel centers.  Returns {marker_id: (x_mm, y_mm)}.
        """
        src_pts = []
        dst_pts = []
        ref_ids = []
        for mid, world_xy in world_ref_points.items():
            if mid in detections:
                src_pts.append(detections[mid].center_undistorted)
                dst_pts.append(world_xy)
                ref_ids.append(mid)

        if len(src_pts) < 4:
            return None

        src = np.array(src_pts, dtype=np.float64)
        dst = np.array(dst_pts, dtype=np.float64)
        H = compute_homography(src, dst)

        result: Dict[int, Tuple[float, float]] = {}
        for mid, det in detections.items():
            world = apply_homography(H, det.center_undistorted.reshape(1, 2))
            result[mid] = (float(world[0, 0]), float(world[0, 1]))

        return result


# =======================================================================
# Drawing (for display only -- uses undistorted image)
# =======================================================================

def draw_detections(
    frame_undist_display: np.ndarray,
    detections: Dict[int, MarkerDetection],
    calib: CalibrationData,
) -> np.ndarray:
    """Draw markers on an undistorted-for-display image."""
    display = frame_undist_display.copy()
    K = calib.camera_matrix

    for marker_id, det in detections.items():
        corners_int = det.corners_undistorted.astype(np.int32)
        cv2.polylines(display, [corners_int], True, (0, 255, 0), 2)

        # Draw axes (no distortion since points are already undistorted)
        cv2.drawFrameAxes(display, K, np.zeros(5), det.rvec, det.tvec, 15.0, 2)

        cx, cy = int(det.center_undistorted[0]), int(det.center_undistorted[1])
        d = float(np.linalg.norm(det.tvec))
        cv2.putText(display, f"ID:{marker_id} {d:.0f}mm",
                    (cx - 40, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    return display


# =======================================================================
# Interactive measurement
# =======================================================================

def interactive_measure(
    camera_index: int,
    backend_label: str,
    width: int,
    height: int,
    fps: float,
    marker_size_mm: float,
    calib_path: Path,
    out_csv: Optional[Path],
) -> None:
    """Interactive marker measurement with strict camera mode."""

    calib = CalibrationData.load(calib_path)
    print(f"Calibration: camera {calib.camera_index} [{calib.backend}] "
          f"{calib.image_size[0]}x{calib.image_size[1]} RMSE={calib.rmse:.4f}px")

    detector = MarkerDetector(calib, marker_size_mm)

    cap, actual_backend = open_camera_strict(camera_index, backend_label, width, height, fps)
    print(f"Camera: {camera_index} [{actual_backend}] {width}x{height}")

    measurements: List[dict] = []

    print(f"\n{'='*60}")
    print(f"  MARKER MEASUREMENT")
    print(f"{'='*60}")
    print("  SPACE=capture  r=remove last  s=save  q=quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        # Detect in distorted frame, undistort corners individually
        detections = detector.detect(frame)

        # Undistort image for display only
        display = undistort_image_for_display(
            frame, calib.camera_matrix, calib.dist_coeffs)

        # Draw on display image
        display = draw_detections(display, detections, calib)

        # HUD
        h, w = display.shape[:2]
        cv2.rectangle(display, (0, 0), (w, 44), (0, 0, 0), -1)
        color = (0, 255, 0) if detections else (0, 0, 255)
        cv2.putText(display,
                    f"Markers: {len(detections)}  |  "
                    f"Meas: {len(measurements)}  |  "
                    f"Size: {marker_size_mm:.0f}mm",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        cv2.imshow("Marker Detection", display)
        key = cv2.waitKey(1) & 0xFF

        if key in (27, ord("q")):
            break
        elif key == ord(" ") and detections:
            entry = {
                "label": f"meas_{len(measurements):03d}",
                "marker_count": len(detections),
                "marker_ids": ",".join(str(mid) for mid in sorted(detections.keys())),
            }
            for mid, det in detections.items():
                entry[f"m{mid}_x_mm"] = f"{det.tvec[0, 0]:.2f}"
                entry[f"m{mid}_y_mm"] = f"{det.tvec[1, 0]:.2f}"
                entry[f"m{mid}_z_mm"] = f"{det.tvec[2, 0]:.2f}"
                entry[f"m{mid}_u_px"] = f"{det.center_undistorted[0]:.2f}"
                entry[f"m{mid}_v_px"] = f"{det.center_undistorted[1]:.2f}"
            measurements.append(entry)
            print(f"  {entry['label']}: {len(detections)} markers")
        elif key == ord("r") and measurements:
            removed = measurements.pop()
            print(f"  Removed: {removed['label']}")
        elif key == ord("s") and measurements:
            _save_csv(measurements, out_csv)

    cap.release()
    cv2.destroyAllWindows()

    if measurements:
        _save_csv(measurements, out_csv)


def detect_from_image(
    image_path: Path,
    calib_path: Path,
    marker_size_mm: float,
    output: Optional[Path],
) -> Dict[int, MarkerDetection]:
    """Detect markers in a saved image (for testing/validation)."""
    calib = CalibrationData.load(calib_path)
    detector = MarkerDetector(calib, marker_size_mm)

    img = cv2.imread(str(image_path))
    if img is None:
        raise ValueError(f"Cannot read: {image_path}")

    detections = detector.detect(img)

    print(f"\n{len(detections)} markers in {image_path.name}:")
    for mid, det in sorted(detections.items()):
        d = float(np.linalg.norm(det.tvec))
        print(f"  ID {mid}: ({det.tvec[0,0]:.1f}, {det.tvec[1,0]:.1f}, "
              f"{det.tvec[2,0]:.1f}) mm  dist={d:.1f}")

    display = undistort_image_for_display(
        img, calib.camera_matrix, calib.dist_coeffs)
    display = draw_detections(display, detections, calib)

    if output:
        cv2.imwrite(str(output), display)
        print(f"Annotated: {output}")
    else:
        cv2.imshow("Markers", display)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return detections


def _save_csv(measurements: List[dict], out_csv: Optional[Path]) -> None:
    if out_csv is None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        out_csv = _CALIB_DIR / f"measurements_{stamp}.csv"

    # Collect all field names
    fieldnames = ["label", "marker_count", "marker_ids"]
    extra = set()
    for m in measurements:
        extra.update(k for k in m if k not in fieldnames)
    fieldnames.extend(sorted(extra))

    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(measurements)

    print(f"Saved {len(measurements)} measurements -> {out_csv}")


# =======================================================================
# Main
# =======================================================================

def main() -> int:
    parser = argparse.ArgumentParser(
        description="ArUco marker detection with audited coordinate pipeline",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    add_camera_args(parser)
    parser.add_argument("--calib", type=Path, default=_DEFAULT_CALIB)
    parser.add_argument("--marker-size", type=float, default=30.0)
    parser.add_argument("--image", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--generate", action="store_true",
                        help="Generate A4 ArUco marker sheet and exit")
    parser.add_argument("--marker-ids", type=str, default="0,1,2,3,4,5,6,7")
    parser.add_argument("--dictionary", type=str, default="DICT_4X4_50")
    args = parser.parse_args()

    if args.generate:
        ids = [int(x.strip()) for x in args.marker_ids.split(",")]
        generate_aruco_a4_svg(
            marker_ids=ids,
            marker_black_size_mm=args.marker_size,
            dictionary_name=args.dictionary,
            output=args.output,
        )
        return 0

    if not args.calib.exists():
        print(f"[ERROR] Calibration not found: {args.calib}")
        print("Run: python tools/calibration/camera_calibrate.py")
        return 1

    if args.image:
        detect_from_image(args.image, args.calib, args.marker_size, args.output)
        return 0

    interactive_measure(
        args.camera, args.backend, args.width, args.height, args.fps,
        args.marker_size, args.calib, args.output,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

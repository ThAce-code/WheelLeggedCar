#!/usr/bin/env python3
"""Camera intrinsic calibration using chessboard or ChArUco patterns.

Generates physically-accurate vector PDF/SVG patterns (250x175 mm default).
Captures frames at a strict, user-specified resolution and computes K, D,
RMS, and per-view reprojection errors, saved with camera metadata.

Usage:
    python tools/calibration/camera_calibrate.py                          # interactive
    python tools/calibration/camera_calibrate.py --camera 2 --backend MSMF
    python tools/calibration/camera_calibrate.py --width 1920 --height 1080
    python tools/calibration/camera_calibrate.py --generate               # print pattern
    python tools/calibration/camera_calibrate.py --images "frames/*.jpg"  # batch
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

from camera_utils import (
    add_camera_args,
    decode_fourcc,
    open_camera_strict,
    resolve_backend,
)

# -- Board geometry: 9x6 inner corners -> 10x7 squares at 25mm -> 250x175mm --
# NOTE: "board pattern size" (250x175mm) != "SVG page size" (270x195mm with 10mm margins)
_DEFAULT_COLS = 9
_DEFAULT_ROWS = 6
_DEFAULT_SQUARE_MM = 25.0
_BOARD_PATTERN_W_MM = (_DEFAULT_COLS + 1) * _DEFAULT_SQUARE_MM   # 250 mm
_BOARD_PATTERN_H_MM = (_DEFAULT_ROWS + 1) * _DEFAULT_SQUARE_MM   # 175 mm
_DEFAULT_MARGIN_MM = 10.0
_PAGE_W_MM = _BOARD_PATTERN_W_MM + 2 * _DEFAULT_MARGIN_MM         # 270 mm
_PAGE_H_MM = _BOARD_PATTERN_H_MM + 2 * _DEFAULT_MARGIN_MM         # 195 mm

_CALIB_DIR = Path(__file__).resolve().parent
_DEFAULT_OUTPUT = _CALIB_DIR / "camera_calib.npz"


# =======================================================================
# Pattern generation (requirement 6, 7)
# =======================================================================

def generate_chessboard_svg(
    cols: int = _DEFAULT_COLS,
    rows: int = _DEFAULT_ROWS,
    square_mm: float = _DEFAULT_SQUARE_MM,
    margin_mm: float = 10.0,
    output: Optional[Path] = None,
) -> Path:
    """Generate a physically-accurate SVG chessboard.

    SVG uses mm units natively -- prints at exact size when "Actual size"
    is selected in the print dialog.
    """
    patterns_dir = _CALIB_DIR / "patterns"
    patterns_dir.mkdir(parents=True, exist_ok=True)

    if output is None:
        output = patterns_dir / f"chessboard_{cols}x{rows}_{square_mm:.0f}mm.svg"

    board_w = (cols + 1) * square_mm
    board_h = (rows + 1) * square_mm
    total_w = board_w + 2 * margin_mm
    total_h = board_h + 2 * margin_mm

    lines: list[str] = []
    lines.append('<?xml version="1.0" encoding="UTF-8"?>')
    lines.append(f'<svg xmlns="http://www.w3.org/2000/svg" '
                 f'width="{total_w}mm" height="{total_h}mm" '
                 f'viewBox="0 0 {total_w} {total_h}">')
    lines.append(f'<!-- Chessboard: {cols}x{rows} inner corners, '
                 f'{square_mm}mm squares, total {board_w}x{board_h}mm -->')
    lines.append(f'<rect width="{total_w}" height="{total_h}" fill="white"/>')

    for row in range(rows + 1):
        for col in range(cols + 1):
            if (row + col) % 2 == 0:
                x = margin_mm + col * square_mm
                y = margin_mm + row * square_mm
                lines.append(
                    f'<rect x="{x}" y="{y}" width="{square_mm}" '
                    f'height="{square_mm}" fill="black"/>'
                )

    # Dimension labels
    lines.append(f'<text x="{total_w/2}" y="{total_h - 2}" '
                 f'text-anchor="middle" font-size="3" fill="gray">'
                 f'{board_w:.0f}x{board_h:.0f} mm | {cols}x{rows} corners | '
                 f'{square_mm:.0f}mm squares</text>')

    lines.append('</svg>')

    output.write_text("\n".join(lines), encoding="utf-8")
    print(f"Chessboard SVG saved: {output}  "
          f"(pattern: {board_w:.0f}x{board_h:.0f} mm, "
          f"page: {total_w:.0f}x{total_h:.0f} mm)")

    # Save companion metadata JSON
    metadata = {
        "pattern_inner_corners": [cols, rows],
        "square_count": [cols + 1, rows + 1],
        "nominal_square_size_mm": square_mm,
        "board_pattern_width_mm": board_w,
        "board_pattern_height_mm": board_h,
        "page_width_mm": total_w,
        "page_height_mm": total_h,
        "margin_mm": margin_mm,
        "print_scale": "100_percent_actual_size",
        "note": ("For best accuracy, measure 5 consecutive squares with calipers "
                 "and divide by 5. Use --square-size-mm with the measured value."),
    }
    json_path = output.with_suffix(".json")
    json_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(f"Chessboard metadata: {json_path}")

    # Also generate PNG for quick preview
    _generate_chessboard_png(cols, rows, square_mm, margin_mm, output.with_suffix(".png"))

    return output


def _generate_chessboard_png(
    cols: int, rows: int, square_mm: float, margin_mm: float, output: Path,
) -> None:
    """Generate a high-DPI PNG of the chessboard for on-screen preview."""
    dpi = 300
    px_per_mm = dpi / 25.4
    total_w = (cols + 1) * square_mm + 2 * margin_mm
    total_h = (rows + 1) * square_mm + 2 * margin_mm
    img_w = int(total_w * px_per_mm)
    img_h = int(total_h * px_per_mm)

    img = np.ones((img_h, img_w), dtype=np.uint8) * 255
    margin_px = int(margin_mm * px_per_mm)
    square_px = int(square_mm * px_per_mm)

    for row in range(rows + 1):
        for col in range(cols + 1):
            if (row + col) % 2 == 0:
                x0 = margin_px + col * square_px
                y0 = margin_px + row * square_px
                img[y0:y0 + square_px, x0:x0 + square_px] = 0

    cv2.imwrite(str(output), img)


def generate_aruco_a4_svg(
    marker_ids: Optional[List[int]] = None,
    marker_black_size_mm: float = 20.0,
    quiet_zone_mm: float = 5.0,
    dictionary_name: str = "DICT_4X4_50",
    page_size: str = "A4",
    orientation: str = "landscape",
    output: Optional[Path] = None,
) -> Path:
    """Generate an A4 SVG sheet with ArUco markers at known physical sizes.

    Args:
        marker_ids: List of ArUco marker IDs. Defaults to [0..7].
        marker_black_size_mm: Outer side length of the black encoded area
            (NOT including the white quiet zone). This is the physical size
            of the bit-pattern region that the ArUco detector reads.
        quiet_zone_mm: White border width added around each marker.
            Total marker tile = marker_black_size_mm + 2 * quiet_zone_mm.
        dictionary_name: OpenCV ArUco dictionary name (e.g. "DICT_4X4_50").
        page_size: Paper size string ("A4").
        orientation: "landscape" or "portrait".
        output: Output SVG path. Auto-generated from parameters if None.

    Returns:
        Path to the saved SVG file.
    """
    markers_dir = _CALIB_DIR / "markers"
    markers_dir.mkdir(parents=True, exist_ok=True)

    if marker_ids is None:
        marker_ids = list(range(8))

    # ---------- page geometry ----------
    if page_size.upper() != "A4":
        raise ValueError(f"Unsupported page size: {page_size}")
    if orientation == "landscape":
        page_w_mm = 297.0
        page_h_mm = 210.0
    else:
        page_w_mm = 210.0
        page_h_mm = 297.0

    # Total marker tile = black area + quiet zone on both sides
    marker_total_mm = marker_black_size_mm + 2.0 * quiet_zone_mm

    # ---------- output filename ----------
    if output is None:
        ids_sorted = sorted(marker_ids)
        if ids_sorted == list(range(ids_sorted[0], ids_sorted[-1] + 1)):
            ids_str = f"ids{ids_sorted[0]}-{ids_sorted[-1]}"
        else:
            ids_str = "ids" + "_".join(str(i) for i in ids_sorted)
        output = markers_dir / (
            f"aruco_{dictionary_name}_{ids_str}_"
            f"{marker_black_size_mm:.0f}mm_{page_size}.svg"
        )

    # ---------- layout: fit markers on page ----------
    n = len(marker_ids)
    # Try to fit in a grid, starting from wider layouts
    rows = cols = 1
    for try_cols in range(min(4, n), 0, -1):
        try_rows = (n + try_cols - 1) // try_cols
        cell_w = page_w_mm / try_cols
        cell_h = page_h_mm / try_rows
        if cell_w >= marker_total_mm + 10.0 and cell_h >= marker_total_mm + 15.0:
            cols = try_cols
            rows = try_rows
            break

    cell_w = page_w_mm / cols
    cell_h = page_h_mm / rows

    # ---------- generate markers & build SVG ----------
    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, dictionary_name))

    dpi = 300
    px_per_mm = dpi / 25.4

    lines: list[str] = []
    lines.append('<?xml version="1.0" encoding="UTF-8"?>')
    lines.append(
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'xmlns:xlink="http://www.w3.org/1999/xlink" '
        f'width="{page_w_mm}mm" height="{page_h_mm}mm" '
        f'viewBox="0 0 {page_w_mm} {page_h_mm}">'
    )
    lines.append(
        f'<!-- ArUco markers: {dictionary_name}, '
        f'black_size={marker_black_size_mm}mm, '
        f'quiet_zone={quiet_zone_mm}mm, IDs={marker_ids} -->'
    )
    lines.append(
        f'<rect width="{page_w_mm}" height="{page_h_mm}" fill="white"/>'
    )

    for i, marker_id in enumerate(marker_ids):
        row = i // cols
        col = i % cols
        cx = col * cell_w + cell_w / 2.0
        cy = row * cell_h + cell_h / 2.0

        # Build marker PNG with quiet zone baked in (white border)
        black_px = int(marker_black_size_mm * px_per_mm)
        qz_px = int(quiet_zone_mm * px_per_mm)
        total_px = black_px + 2 * qz_px
        marker_img = cv2.aruco.generateImageMarker(
            dictionary, marker_id, black_px)
        img_with_qz = np.ones((total_px, total_px), dtype=np.uint8) * 255
        img_with_qz[qz_px:qz_px + black_px,
                     qz_px:qz_px + black_px] = marker_img

        _, png_buf = cv2.imencode(".png", img_with_qz)
        png_b64 = __import__("base64").b64encode(png_buf.tobytes()).decode()

        x = cx - marker_total_mm / 2.0
        y = cy - marker_total_mm / 2.0

        lines.append(
            f'<image x="{x:.1f}" y="{y:.1f}" '
            f'width="{marker_total_mm}" height="{marker_total_mm}" '
            f'xlink:href="data:image/png;base64,{png_b64}" '
            f'preserveAspectRatio="none"/>'
        )

        # Label OUTSIDE the quiet zone (below the total marker tile)
        label_y = y + marker_total_mm + 4.0
        lines.append(
            f'<text x="{cx:.1f}" y="{label_y:.1f}" '
            f'text-anchor="middle" font-size="3" fill="black">'
            f'ID:{marker_id}</text>'
        )

    # Page metadata footer
    lines.append(
        f'<text x="{page_w_mm/2:.1f}" y="{page_h_mm - 5:.1f}" '
        f'text-anchor="middle" font-size="3" fill="gray">'
        f'{dictionary_name} | black_size={marker_black_size_mm}mm | '
        f'quiet_zone={quiet_zone_mm}mm | IDs={marker_ids} | '
        f'{page_w_mm:.0f}x{page_h_mm:.0f}mm'
        f'</text>'
    )
    lines.append('</svg>')

    output.write_text("\n".join(lines), encoding="utf-8")
    print(f"ArUco SVG saved: {output}")
    print(f"  Dictionary:      {dictionary_name}")
    print(f"  Marker black:    {marker_black_size_mm:.1f} mm (encoded area)")
    print(f"  Quiet zone:      {quiet_zone_mm:.1f} mm")
    print(f"  Marker total:    {marker_total_mm:.1f} mm")
    print(f"  Markers:         {marker_ids}")
    print(f"  Page:            {page_size} {orientation} "
          f"({page_w_mm:.0f}x{page_h_mm:.0f} mm)")
    print(f"  Layout:          {rows} rows x {cols} cols")

    # ---------- companion JSON metadata ----------
    json_path = output.with_suffix(".json")
    metadata: dict = {
        "dictionary": dictionary_name,
        "marker_ids": marker_ids,
        "marker_black_size_mm": marker_black_size_mm,
        "quiet_zone_mm": quiet_zone_mm,
        "marker_total_size_mm": marker_total_mm,
        "page_size": page_size,
        "orientation": orientation,
        "page_width_mm": page_w_mm,
        "page_height_mm": page_h_mm,
        "layout_rows": rows,
        "layout_cols": cols,
        "print_scale": "100_percent_actual_size",
    }
    json_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(f"  Metadata saved:  {json_path}")

    # ---------- auto-validation ----------
    _validate_generated_svg(
        dictionary_name=dictionary_name,
        marker_ids=marker_ids,
        marker_black_size_mm=marker_black_size_mm,
        quiet_zone_mm=quiet_zone_mm,
        rows=rows,
        cols=cols,
        page_w_mm=page_w_mm,
        page_h_mm=page_h_mm,
        px_per_mm=px_per_mm,
    )

    return output


def _validate_generated_svg(
    dictionary_name: str,
    marker_ids: List[int],
    marker_black_size_mm: float,
    quiet_zone_mm: float,
    rows: int,
    cols: int,
    page_w_mm: float,
    page_h_mm: float,
    px_per_mm: float,
) -> None:
    """Auto-validate the generated SVG layout by rasterising markers at the
    same DPI/positions as the SVG and running cv2.aruco.detectMarkers.

    On any failure this prints details and calls sys.exit(1).
    """
    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, dictionary_name))

    marker_total_mm = marker_black_size_mm + 2.0 * quiet_zone_mm
    cell_w_mm = page_w_mm / cols
    cell_h_mm = page_h_mm / rows

    page_w_px = int(page_w_mm * px_per_mm)
    page_h_px = int(page_h_mm * px_per_mm)
    page_img = np.ones((page_h_px, page_w_px), dtype=np.uint8) * 255

    for i, marker_id in enumerate(marker_ids):
        row = i // cols
        col = i % cols
        cx_mm = col * cell_w_mm + cell_w_mm / 2.0
        cy_mm = row * cell_h_mm + cell_h_mm / 2.0

        black_px = int(marker_black_size_mm * px_per_mm)
        qz_px = int(quiet_zone_mm * px_per_mm)
        total_px = black_px + 2 * qz_px

        marker_img = cv2.aruco.generateImageMarker(
            dictionary, marker_id, black_px)
        img_with_qz = np.ones((total_px, total_px), dtype=np.uint8) * 255
        img_with_qz[qz_px:qz_px + black_px,
                     qz_px:qz_px + black_px] = marker_img

        x0 = int(round((cx_mm - marker_total_mm / 2.0) * px_per_mm))
        y0 = int(round((cy_mm - marker_total_mm / 2.0) * px_per_mm))
        x0 = max(0, min(x0, page_w_px - total_px))
        y0 = max(0, min(y0, page_h_px - total_px))

        roi = page_img[y0:y0 + total_px, x0:x0 + total_px]
        if roi.shape[0] != total_px or roi.shape[1] != total_px:
            print(f"[VALIDATION FAIL] Marker {marker_id} does not fit on page "
                  f"(x0={x0}, y0={y0}, roi={roi.shape}, expected={total_px})")
            sys.exit(1)
        roi[:] = img_with_qz

    # Detect
    parameters = cv2.aruco.DetectorParameters()
    detector_obj = cv2.aruco.ArucoDetector(dictionary, parameters)
    corners, ids, _rejected = detector_obj.detectMarkers(page_img)

    expected_ids = set(marker_ids)

    if ids is None:
        print("\n[VALIDATION FAIL] No markers detected in generated image!")
        sys.exit(1)

    detected_ids = set(int(i) for i in ids.flatten())

    print(f"\n[VALIDATION] Detected IDs: {sorted(detected_ids)}")
    print(f"[VALIDATION] Expected IDs:  {sorted(expected_ids)}")

    # Must detect all expected IDs
    missing = expected_ids - detected_ids
    if missing:
        print(f"[VALIDATION FAIL] Missing IDs: {sorted(missing)}")
        sys.exit(1)

    # Must NOT detect extra IDs
    extra = detected_ids - expected_ids
    if extra:
        print(f"[VALIDATION FAIL] Extra IDs detected: {sorted(extra)}")
        sys.exit(1)

    # Each marker must have exactly 4 corners
    for idx, corner in enumerate(corners):
        pts = corner[0]  # shape (4, 2)
        if pts.shape[0] != 4:
            print(f"[VALIDATION FAIL] Marker id={ids[idx][0]} has "
                  f"{pts.shape[0]} corners, expected 4")
            sys.exit(1)

    # Verify no overlapping bounding boxes
    bboxes: list = []
    for corner in corners:
        pts = corner[0]
        bboxes.append((
            float(pts[:, 0].min()), float(pts[:, 1].min()),
            float(pts[:, 0].max()), float(pts[:, 1].max()),
        ))

    for i in range(len(bboxes)):
        for j in range(i + 1, len(bboxes)):
            ax1, ay1, ax2, ay2 = bboxes[i]
            bx1, by1, bx2, by2 = bboxes[j]
            if ax1 < bx2 and ax2 > bx1 and ay1 < by2 and ay2 > by1:
                print(f"[VALIDATION FAIL] Overlapping markers: "
                      f"id={ids[i][0]} and id={ids[j][0]}")
                sys.exit(1)

    print(f"[VALIDATION PASS] All {len(detected_ids)} markers detected, "
          f"4 corners each, no overlap.")


def _test_existing_markers_png(
    png_path: Path,
    dictionary_name: str = "DICT_4X4_50",
    expected_ids: Optional[List[int]] = None,
) -> None:
    """Read a PNG file and run ArUco detection to verify marker IDs.

    This PNG has no reliable physical print scale -- only suitable for
    algorithm preview and detection testing.
    """
    if expected_ids is None:
        expected_ids = list(range(8))

    if not png_path.exists():
        print(f"[ERROR] PNG not found: {png_path}")
        sys.exit(1)

    img = cv2.imread(str(png_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"[ERROR] Cannot read PNG: {png_path}")
        sys.exit(1)

    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, dictionary_name))
    parameters = cv2.aruco.DetectorParameters()
    detector_obj = cv2.aruco.ArucoDetector(dictionary, parameters)
    corners, ids, _rejected = detector_obj.detectMarkers(img)

    expected_set = set(expected_ids)

    print(f"\n[PNG TEST] File:       {png_path}")
    print(f"[PNG TEST] Dictionary:  {dictionary_name}")
    print(f"[PNG TEST] Note: This PNG has no reliable physical print scale -- "
          f"only suitable for algorithm preview and detection testing.")

    if ids is None:
        detected_set: set = set()
    else:
        detected_set = set(int(i) for i in ids.flatten())

    print(f"[PNG TEST] Detected IDs: {sorted(detected_set)}")
    print(f"[PNG TEST] Expected IDs:  {sorted(expected_set)}")

    missing = expected_set - detected_set
    extra = detected_set - expected_set

    if missing:
        print(f"[PNG TEST FAIL] Missing IDs: {sorted(missing)}")
        sys.exit(1)
    if extra:
        print(f"[PNG TEST FAIL] Extra IDs: {sorted(extra)}")
        sys.exit(1)

    print(f"[PNG TEST PASS] All {len(detected_set)} expected markers detected.")


# =======================================================================
# Calibration core
# =======================================================================

def detect_chessboard(frame: np.ndarray, cols: int, rows: int
                      ) -> Optional[np.ndarray]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH +
             cv2.CALIB_CB_NORMALIZE_IMAGE +
             cv2.CALIB_CB_FAST_CHECK)
    ret, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags)
    return corners if ret else None


def calibrate_from_frames(
    objpoints_all: List[np.ndarray],
    imgpoints_all: List[np.ndarray],
    image_size: Tuple[int, int],
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], float, List[float]]:
    """Run calibrateCamera, return (mtx, dist, rmse, per_view_errors)."""
    if len(objpoints_all) < 3:
        print("[ERROR] Need at least 3 frames.")
        return None, None, float("nan"), []

    print(f"\nCalibrating with {len(objpoints_all)} frames...")
    flags = cv2.CALIB_ZERO_TANGENT_DIST + cv2.CALIB_FIX_K3
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints_all, imgpoints_all, image_size,
        None, None, flags=flags, criteria=criteria,
    )

    rmse = float(ret)
    per_view: List[float] = []
    for i, (op, ip) in enumerate(zip(objpoints_all, imgpoints_all)):
        projected, _ = cv2.projectPoints(op, rvecs[i], tvecs[i], mtx, dist)
        err = cv2.norm(ip, projected, cv2.NORM_L2) / len(projected)
        per_view.append(float(err))

    return mtx, dist, rmse, per_view


def save_calibration(
    output_path: Path,
    camera_index: int,
    backend: str,
    width: int,
    height: int,
    mtx: np.ndarray,
    dist: np.ndarray,
    rmse: float,
    per_view_errors: List[float],
    cols: int,
    rows: int,
    square_mm: float,
) -> None:
    """Save calibration NPZ with full metadata (requirement 5)."""
    np.savez(
        output_path,
        camera_index=camera_index,
        backend=backend,
        resolution=np.array([width, height], dtype=np.int32),
        camera_matrix=mtx,
        dist_coeffs=dist,
        rmse=rmse,
        per_view_errors=np.array(per_view_errors, dtype=np.float32),
        cols=cols,
        rows=rows,
        square_mm=square_mm,
    )
    print(f"\nSaved: {output_path}")
    print(f"  camera_index: {camera_index}")
    print(f"  backend:      {backend}")
    print(f"  resolution:   {width}x{height}")
    print(f"  K:\n{mtx}")
    print(f"  D: {dist.ravel()}")
    print(f"  RMSE: {rmse:.4f} px")
    if per_view_errors:
        print(f"  per_view min/mean/max: "
              f"{min(per_view_errors):.4f} / "
              f"{sum(per_view_errors)/len(per_view_errors):.4f} / "
              f"{max(per_view_errors):.4f} px")

    # Also save a human-readable JSON summary
    json_path = output_path.with_suffix(".json")
    json_path.write_text(json.dumps({
        "camera_index": camera_index,
        "backend": backend,
        "resolution": [width, height],
        "fx": float(mtx[0, 0]),
        "fy": float(mtx[1, 1]),
        "cx": float(mtx[0, 2]),
        "cy": float(mtx[1, 2]),
        "k1": float(dist[0, 0]),
        "k2": float(dist[0, 1]),
        "p1": float(dist[0, 2]),
        "p2": float(dist[0, 3]),
        "rmse_px": rmse,
        "per_view_errors": [float(e) for e in per_view_errors],
        "cols": cols,
        "rows": rows,
        "square_mm": square_mm,
    }, indent=2), encoding="utf-8")
    print(f"  JSON summary: {json_path}")


# =======================================================================
# Interactive capture
# =======================================================================

_last_rmse: float = 0.0


def interactive_capture(
    camera_index: int,
    backend_label: str,
    width: int,
    height: int,
    fps: float,
    cols: int,
    rows: int,
    square_mm: float,
    output_path: Path,
) -> None:
    """Open camera with strict mode, capture chessboard frames, calibrate."""

    cap, actual_backend = open_camera_strict(camera_index, backend_label, width, height, fps)
    print(f"Camera {camera_index} [{actual_backend}] {width}x{height}")

    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_mm

    objpoints: List[np.ndarray] = []
    imgpoints: List[np.ndarray] = []
    mtx: Optional[np.ndarray] = None
    dist: Optional[np.ndarray] = None
    undistort_preview = False
    display_rmse: float = 0.0

    print(f"\n{'='*60}")
    print(f"  CAMERA CALIBRATION")
    print(f"  Pattern: {cols}x{rows} corners, {square_mm}mm squares")
    print(f"  Board pattern: {_BOARD_PATTERN_W_MM}x{_BOARD_PATTERN_H_MM} mm "
          f"(page: {_PAGE_W_MM}x{_PAGE_H_MM} mm)")
    print(f"  Resolution: {width}x{height}")
    print(f"{'='*60}")
    print("  SPACE=capture  d=delete  c=calibrate  u=undistort  q=quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        display = frame.copy()
        h, w = display.shape[:2]
        corners = detect_chessboard(frame, cols, rows)

        if corners is not None:
            cv2.drawChessboardCorners(display, (cols, rows), corners, True)
            board_ready = True
        else:
            board_ready = False

        # Status bar
        cv2.rectangle(display, (0, 0), (w, 50), (0, 0, 0), -1)
        color = (0, 255, 0) if board_ready else (0, 0, 255)
        cv2.putText(display,
                    f"{'BOARD' if board_ready else 'NO BOARD'}  "
                    f"Frames: {len(objpoints)}  "
                    f"RMSE: {display_rmse:.4f}" if mtx else "",
                    (10, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        if undistort_preview and mtx is not None and dist is not None:
            new_mtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
            display = cv2.undistort(display, mtx, dist, None, new_mtx)

        cv2.imshow("Camera Calibration", display)
        key = cv2.waitKey(1) & 0xFF

        if key in (27, ord("q")):
            break
        elif key == ord(" ") and board_ready:
            corners = detect_chessboard(frame, cols, rows)
            if corners is not None:
                objpoints.append(objp)
                imgpoints.append(corners)
                print(f"  Frame #{len(objpoints)} captured")
        elif key == ord("d") and objpoints:
            objpoints.pop()
            imgpoints.pop()
            print(f"  Deleted -> {len(objpoints)} frames")
        elif key == ord("c") and len(objpoints) >= 3:
            mtx, dist, rmse, per_view = calibrate_from_frames(
                objpoints, imgpoints, (width, height))
            display_rmse = rmse
            if mtx is not None and dist is not None:
                print(f"\n  RMSE: {rmse:.4f} px")
                print(f"  fx={mtx[0,0]:.1f} fy={mtx[1,1]:.1f} "
                      f"cx={mtx[0,2]:.1f} cy={mtx[1,2]:.1f}")
                save_calibration(
                    output_path, camera_index, actual_backend,
                    width, height, mtx, dist, rmse, per_view,
                    cols, rows, square_mm)
        elif key == ord("u"):
            undistort_preview = not undistort_preview

    cap.release()
    cv2.destroyAllWindows()

    # Final calibration if enough frames
    if len(objpoints) >= 3:
        mtx, dist, rmse, per_view = calibrate_from_frames(
            objpoints, imgpoints, (width, height))
        if mtx is not None and dist is not None:
            save_calibration(
                output_path, camera_index, actual_backend,
                width, height, mtx, dist, rmse, per_view,
                cols, rows, square_mm)
            print("\nPer-view errors:")
            for i, err in enumerate(per_view):
                bar = "#" * min(int(err / 0.1), 40)
                print(f"  frame {i:3d}: {err:6.4f} px  {bar}")
    else:
        print(f"\n[INFO] {len(objpoints)} frames -- need >=3 for calibration.")


def calibrate_from_images(
    image_paths: List[Path],
    cols: int,
    rows: int,
    square_mm: float,
    output_path: Path,
) -> None:
    """Batch calibration from pre-captured image files."""
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_mm

    objpoints: List[np.ndarray] = []
    imgpoints: List[np.ndarray] = []
    image_size: Optional[Tuple[int, int]] = None

    for path in image_paths:
        if not path.exists():
            print(f"  [SKIP] {path} -- not found")
            continue
        img = cv2.imread(str(path))
        if img is None:
            print(f"  [SKIP] {path} -- cannot read")
            continue
        if image_size is None:
            image_size = (img.shape[1], img.shape[0])

        corners = detect_chessboard(img, cols, rows)
        if corners is not None:
            objpoints.append(objp)
            imgpoints.append(corners)
            print(f"  {path.name} -- OK")
        else:
            print(f"  {path.name} -- no chessboard")

    if not objpoints:
        print("[ERROR] No valid frames.")
        return

    mtx, dist, rmse, per_view = calibrate_from_frames(
        objpoints, imgpoints, image_size)

    if mtx is None or dist is None:
        return

    save_calibration(
        output_path, -1, "batch", image_size[0], image_size[1],
        mtx, dist, rmse, per_view, cols, rows, square_mm)

    print("\nPer-view errors:")
    for i, err in enumerate(per_view):
        bar = "#" * min(int(err / 0.1), 40)
        print(f"  frame {i:3d}: {err:6.4f} px  {bar}")


# =======================================================================
# Main
# =======================================================================

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Camera intrinsic calibration with chessboard",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    add_camera_args(parser)
    parser.add_argument("--cols", type=int, default=_DEFAULT_COLS)
    parser.add_argument("--rows", type=int, default=_DEFAULT_ROWS)
    parser.add_argument("--square", type=float, default=_DEFAULT_SQUARE_MM,
                        help="Nominal square size in mm. For best accuracy, "
                             "measure 5 squares with calipers and use "
                             "square_size_mm / 5 as the value.")
    parser.add_argument("--square-size-mm", type=float, default=None,
                        help="Measured square size in mm (overrides --square for "
                             "calibration). Measure: 5 consecutive squares with "
                             "calipers, divide by 5.")
    parser.add_argument("--output", type=Path, default=_DEFAULT_OUTPUT)
    parser.add_argument("--images", type=str, default=None,
                        help="Glob pattern for batch calibration from image files")
    parser.add_argument("--generate", action="store_true",
                        help="Generate printable SVG chessboard and exit")
    parser.add_argument("--generate-aruco", action="store_true",
                        help="Generate formal ArUco marker A4 SVG sheet and exit")
    parser.add_argument("--aruco-dictionary", type=str, default="DICT_4X4_50",
                        help="ArUco dictionary name (for --generate-aruco)")
    parser.add_argument("--marker-ids", type=str, default="0,1,2,3,4,5,6,7",
                        help="Comma-separated marker IDs (for --generate-aruco)")
    parser.add_argument("--marker-size-mm", type=float, default=20.0,
                        help="Marker black encoded area size in mm "
                             "(for --generate-aruco). NOT including quiet zone.")
    parser.add_argument("--quiet-zone-mm", type=float, default=5.0,
                        help="Quiet zone white border width in mm "
                             "(for --generate-aruco)")
    parser.add_argument("--page-size", type=str, default="A4",
                        help="Page size (for --generate-aruco)")
    parser.add_argument("--orientation", type=str, default="landscape",
                        choices=["landscape", "portrait"],
                        help="Page orientation (for --generate-aruco)")
    parser.add_argument("--svg", type=Path, default=None,
                        help="Output path for generated SVG")
    parser.add_argument("--test-markers-png", type=Path, default=None,
                        help="Run ArUco detection test on an existing PNG file "
                             "and exit")
    args = parser.parse_args()

    if args.generate:
        generate_chessboard_svg(args.cols, args.rows, args.square, output=args.svg)
        return 0

    if args.generate_aruco:
        ids = [int(x.strip()) for x in args.marker_ids.split(",")]
        generate_aruco_a4_svg(
            marker_ids=ids,
            marker_black_size_mm=args.marker_size_mm,
            quiet_zone_mm=args.quiet_zone_mm,
            dictionary_name=args.aruco_dictionary,
            page_size=args.page_size,
            orientation=args.orientation,
            output=args.svg,
        )
        return 0

    if args.test_markers_png:
        ids = [int(x.strip()) for x in args.marker_ids.split(",")]
        _test_existing_markers_png(
            png_path=args.test_markers_png,
            dictionary_name=args.aruco_dictionary,
            expected_ids=ids,
        )
        return 0

    # Use measured square size if provided, otherwise nominal
    square_mm = args.square_size_mm if args.square_size_mm is not None else args.square
    if args.square_size_mm is not None:
        print(f"Using measured square size: {square_mm:.2f} mm "
              f"(nominal was {args.square:.2f} mm)")

    if args.images:
        import glob as g
        paths = [Path(p) for p in sorted(g.glob(args.images))]
        print(f"Found {len(paths)} images")
        calibrate_from_images(paths, args.cols, args.rows, square_mm, args.output)
        return 0

    interactive_capture(
        camera_index=args.camera,
        backend_label=args.backend,
        width=args.width,
        height=args.height,
        fps=args.fps,
        cols=args.cols,
        rows=args.rows,
        square_mm=square_mm,
        output_path=args.output,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

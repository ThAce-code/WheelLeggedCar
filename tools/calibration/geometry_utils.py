# Coordinate domain naming convention used throughout this module:
#   raw_points_px         — original distorted image points
#   undistorted_points_px — after cv2.undistortPoints(..., P=K)
#   normalized_points     — after cv2.undistortPoints(..., P=None)
#   plane_points_mm       — world coordinates on the plane (Z=0)
# Never use ambiguous names like pts1, pts2, points across domain boundaries.

"""Unified coordinate-domain-checked solvePnP and homography handling.

Provides strict wrappers around OpenCV's solvePnP and homography routines
that enforce explicit coordinate domain tracking, robust input validation,
and reproducible serialisation of computed transforms.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Optional, Tuple

import cv2
import numpy as np

# -- Type aliases -----------------------------------------------------------

PointDomain = Literal["raw_pixel", "undistorted_pixel", "normalized"]


# =========================================================================
# solvePnP -- strict coordinate-domain-aware wrapper
# =========================================================================

def solve_pnp_checked(
    object_points: np.ndarray,       # (N, 3) world points
    image_points: np.ndarray,        # (N, 2) image points
    camera_matrix: np.ndarray,       # (3, 3)
    dist_coeffs: np.ndarray,         # (4,), (5,), or (8,) distortion coefficients
    point_domain: PointDomain = "raw_pixel",
    flags: int = cv2.SOLVEPNP_IPPE_SQUARE,
    debug: bool = False,
) -> Tuple[bool, np.ndarray, np.ndarray]:
    """Strict wrapper around cv2.solvePnP with explicit coordinate-domain handling.

    This function never silently assumes a coordinate domain — the caller must
    declare whether *image_points* are in raw-pixel, undistorted-pixel, or
    normalised-image coordinates.  The appropriate pre-processing and camera
    parameters are chosen accordingly.

    Parameters
    ----------
    object_points : ndarray of shape (N, 3), float32 or float64
        3-D world coordinates of the calibration object.
    image_points : ndarray of shape (N, 2), float32 or float64
        Observed image points whose domain is given by *point_domain*.
    camera_matrix : ndarray of shape (3, 3), float64
        Intrinsic camera matrix.
    dist_coeffs : ndarray of shape (4,), (5,), or (8,), float64
        Distortion coefficients (k1, k2, p1, p2, ...).
    point_domain : {'raw_pixel', 'undistorted_pixel', 'normalized'}
        Coordinate domain of *image_points*.
    flags : int
        OpenCV solvePnP flags (default: cv2.SOLVEPNP_IPPE_SQUARE).
    debug : bool
        When True, prints the coordinate domain used, K matrix summary,
        distortion usage, and point count.

    Returns
    -------
    (success, rvec, tvec) : (bool, ndarray, ndarray)
        *success* is True when solvePnP succeeds; *rvec* (3x1) and *tvec*
        (3x1) are the rotation and translation vectors.  On failure both
        are zero arrays.

    Raises
    ------
    ValueError
        If any input fails shape, dtype, or finite-value validation.
    """
    # -- Validate shapes ----------------------------------------------------

    _validate_2d_image_points(image_points)
    _validate_3d_object_points(object_points)
    _validate_camera_matrix(camera_matrix)

    n = object_points.shape[0]
    if n < 4:
        raise ValueError(
            f"solvePnP requires at least 4 points; got {n}"
        )

    # -- Convert dtypes to float64 for OpenCV -------------------------------

    if image_points.dtype != np.float64:
        image_points = image_points.astype(np.float64)
    if object_points.dtype != np.float64:
        object_points = object_points.astype(np.float64)
    camera_matrix = camera_matrix.astype(np.float64)

    # -- Select coordinate pipeline -----------------------------------------

    zero_distortion = np.zeros(5, dtype=np.float64)

    if point_domain == "raw_pixel":
        # Mode A: pass everything straight through
        if dist_coeffs is None:
            raise ValueError(
                "point_domain='raw_pixel' requires distortion coefficients; "
                "got dist_coeffs=None.  If points are already undistorted, use "
                "point_domain='undistorted_pixel' instead."
            )
        dist_coeffs = dist_coeffs.astype(np.float64)
        if dist_coeffs.size < 4:
            raise ValueError(
                f"point_domain='raw_pixel' requires at least 4 distortion "
                f"coefficients; got {dist_coeffs.size}"
            )

        # Warn if distortion appears to be all-zero (user might have the wrong domain)
        if np.allclose(dist_coeffs, 0.0):
            import warnings
            warnings.warn(
                "point_domain='raw_pixel' but dist_coeffs are all zero. "
                "If your points are already undistorted, switch to "
                "point_domain='undistorted_pixel'."
            )

        use_K = camera_matrix
        use_D = dist_coeffs
        image_pts_for_solve = image_points

    elif point_domain == "undistorted_pixel":
        # Mode B: undistort points with P=K, then solvePnP with K + zero D
        if dist_coeffs is None:
            raise ValueError(
                "point_domain='undistorted_pixel' requires distortion coefficients; "
                "got dist_coeffs=None."
            )
        dist_coeffs = dist_coeffs.astype(np.float64)
        if dist_coeffs.size < 4:
            raise ValueError(
                f"point_domain='undistorted_pixel' requires at least 4 distortion "
                f"coefficients; got {dist_coeffs.size}"
            )

        if np.allclose(dist_coeffs, 0.0):
            import warnings
            warnings.warn(
                "point_domain='undistorted_pixel' but dist_coeffs are all zero — "
                "proceeding with identity undistortion (points are assumed already "
                "undistorted)."
            )

        pts = image_points.reshape(-1, 1, 2).astype(np.float64)
        image_pts_for_solve = cv2.undistortPoints(
            pts, camera_matrix, dist_coeffs, P=camera_matrix
        ).reshape(-1, 2)

        use_K = camera_matrix
        use_D = zero_distortion

    elif point_domain == "normalized":
        # Mode C: undistort without P (normalised coords), then solvePnP
        # with identity K + zero D
        if dist_coeffs is None:
            raise ValueError(
                "point_domain='normalized' requires distortion coefficients; "
                "got dist_coeffs=None."
            )
        dist_coeffs = dist_coeffs.astype(np.float64)
        if dist_coeffs.size < 4:
            raise ValueError(
                f"point_domain='normalized' requires at least 4 distortion "
                f"coefficients; got {dist_coeffs.size}"
            )

        if np.allclose(dist_coeffs, 0.0):
            import warnings
            warnings.warn(
                "point_domain='normalized' but dist_coeffs are all zero — "
                "proceeding with identity undistortion (points are assumed already "
                "undistorted)."
            )

        pts = image_points.reshape(-1, 1, 2).astype(np.float64)
        image_pts_for_solve = cv2.undistortPoints(
            pts, camera_matrix, dist_coeffs, P=None
        ).reshape(-1, 2)

        use_K = np.eye(3, dtype=np.float64)
        use_D = zero_distortion

    else:
        raise ValueError(
            f"Unknown point_domain='{point_domain}'. "
            f"Must be one of: raw_pixel, undistorted_pixel, normalized."
        )

    # -- Debug output -------------------------------------------------------

    if debug:
        print(f"[solve_pnp_checked] point_domain  = {point_domain}")
        print(f"[solve_pnp_checked] K (fx, fy, cx, cy) = "
              f"({use_K[0, 0]:.3f}, {use_K[1, 1]:.3f}, "
              f"{use_K[0, 2]:.3f}, {use_K[1, 2]:.3f})")
        if point_domain == "raw_pixel":
            print(f"[solve_pnp_checked] distortion = yes (k1..p2 = {use_D.ravel()[:4]})")
        else:
            print(f"[solve_pnp_checked] distortion = zero (already undistorted)")
        print(f"[solve_pnp_checked] num points  = {n}")

    # -- Call OpenCV --------------------------------------------------------

    try:
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_pts_for_solve,
            use_K,
            use_D,
            flags=flags,
        )
    except cv2.error as e:
        raise RuntimeError(f"cv2.solvePnP failed: {e}") from e

    return bool(success), rvec, tvec


# =========================================================================
# Input validation helpers (internal)
# =========================================================================

def _validate_2d_image_points(points: np.ndarray) -> None:
    """Validate (N, 2) image point array: shape, dtype, no NaN/Inf."""
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError(
            f"image_points must be (N, 2); got shape {points.shape}"
        )
    if points.dtype not in (np.float32, np.float64):
        raise ValueError(
            f"image_points dtype must be float32 or float64; got {points.dtype}"
        )
    if not np.isfinite(points).all():
        raise ValueError("NaN/Inf detected in image_points")


def _validate_3d_object_points(points: np.ndarray) -> None:
    """Validate (N, 3) object point array: shape, dtype, no NaN/Inf."""
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(
            f"object_points must be (N, 3); got shape {points.shape}"
        )
    if points.dtype not in (np.float32, np.float64):
        raise ValueError(
            f"object_points dtype must be float32 or float64; got {points.dtype}"
        )
    if not np.isfinite(points).all():
        raise ValueError("NaN/Inf detected in object_points")


def _validate_camera_matrix(K: np.ndarray) -> None:
    """Validate camera matrix shape (3, 3)."""
    if K.shape != (3, 3):
        raise ValueError(
            f"camera_matrix must be (3, 3); got {K.shape}"
        )


# =========================================================================
# HomographyData — fully self-describing homography container
# =========================================================================

@dataclass
class HomographyData:
    """Fully self-describing homography from undistorted-pixel to world-mm.

    Every field carries provenance so that downstream consumers can verify
    compatibility before applying the transform.  The design assumes
    homographies are always computed on *undistorted* pixel coordinates
    (requirement from the camera_utils pipeline), hence the *point_domain*
    is always ``"undistorted_pixel"``.
    """

    H: np.ndarray                    # (3, 3) homography matrix
    point_domain: str                # "undistorted_pixel"
    image_width: int
    image_height: int
    camera_index: int
    backend: str
    camera_matrix: np.ndarray        # (3, 3)
    dist_coeffs: np.ndarray          # (k1, k2, p1, p2, ...)
    calib_path: str                  # path to calibration file used
    src_points_undistorted_px: Optional[np.ndarray] = None  # for debugging
    dst_points_mm: Optional[np.ndarray] = None               # for debugging


# =========================================================================
# save / load homography
# =========================================================================

def save_homography(path: Path, homography_data: HomographyData) -> None:
    """Persist a HomographyData object to an NPZ archive with a JSON sidecar.

    The NPZ stores all numeric arrays; the JSON sidecar stores scalar
    metadata and paths so that the homography is self-documenting even
    when NumPy is not available.

    Parameters
    ----------
    path : Path
        Base file path (without extension — ``.npz`` and ``.json`` are appended).
    homography_data : HomographyData
        Data to serialise.
    """
    npz_path = path.with_suffix(".npz")
    json_path = path.with_suffix(".json")

    # -- Build save dict for np.savez ---------------------------------------
    save_dict = {
        "H": homography_data.H,
        "camera_matrix": homography_data.camera_matrix,
        "dist_coeffs": homography_data.dist_coeffs,
    }

    # Optional debug arrays
    if homography_data.src_points_undistorted_px is not None:
        save_dict["src_points_undistorted_px"] = homography_data.src_points_undistorted_px
    if homography_data.dst_points_mm is not None:
        save_dict["dst_points_mm"] = homography_data.dst_points_mm

    np.savez_compressed(str(npz_path), **save_dict)

    # -- JSON sidecar -------------------------------------------------------
    metadata = {
        "point_domain": homography_data.point_domain,
        "image_width": homography_data.image_width,
        "image_height": homography_data.image_height,
        "camera_index": homography_data.camera_index,
        "backend": homography_data.backend,
        "calib_path": homography_data.calib_path,
    }
    json_path.write_text(json.dumps(metadata, indent=2, ensure_ascii=False))


def load_homography(path: Path) -> HomographyData:
    """Load a HomographyData object from a previously saved NPZ/JSON pair.

    Parameters
    ----------
    path : Path
        Base file path (without extension — ``.npz`` and ``.json`` are read).

    Returns
    -------
    HomographyData
        Reconstructed object.

    Raises
    ------
    FileNotFoundError
        If either the NPZ or JSON file is missing.
    KeyError
        If a required field is missing from the archive.
    """
    npz_path = path.with_suffix(".npz")
    json_path = path.with_suffix(".json")

    if not npz_path.is_file():
        raise FileNotFoundError(f"NPZ archive not found: {npz_path}")
    if not json_path.is_file():
        raise FileNotFoundError(f"JSON sidecar not found: {json_path}")

    data = np.load(str(npz_path), allow_pickle=False)

    # Validate required NPZ fields
    _require_field(data, "H")
    _require_field(data, "camera_matrix")
    _require_field(data, "dist_coeffs")

    metadata = json.loads(json_path.read_text(encoding="utf-8"))

    # Validate required JSON fields
    _require_json_field(metadata, "point_domain")
    _require_json_field(metadata, "image_width")
    _require_json_field(metadata, "image_height")
    _require_json_field(metadata, "camera_index")
    _require_json_field(metadata, "backend")
    _require_json_field(metadata, "calib_path")

    return HomographyData(
        H=data["H"],
        point_domain=metadata["point_domain"],
        image_width=metadata["image_width"],
        image_height=metadata["image_height"],
        camera_index=metadata["camera_index"],
        backend=metadata["backend"],
        camera_matrix=data["camera_matrix"],
        dist_coeffs=data["dist_coeffs"],
        calib_path=metadata["calib_path"],
        src_points_undistorted_px=data.get("src_points_undistorted_px"),
        dst_points_mm=data.get("dst_points_mm"),
    )


def _require_field(data: dict, key: str) -> None:
    if key not in data:
        raise KeyError(
            f"Required field '{key}' not found in NPZ archive"
        )


def _require_json_field(metadata: dict, key: str) -> None:
    if key not in metadata:
        raise KeyError(
            f"Required field '{key}' not found in JSON sidecar"
        )


# =========================================================================
# validate_homography_match
# =========================================================================

def validate_homography_match(
    homography_data: HomographyData,
    current_camera_matrix: np.ndarray,
    current_dist_coeffs: np.ndarray,
    current_resolution: Tuple[int, int],
    tolerance_fx: float = 0.05,  # 5% relative tolerance
) -> None:
    """Verify that a loaded homography is compatible with the current camera setup.

    Checks resolution, intrinsic parameters (fx/fy relative to tolerance),
    and point domain.  Raises ``RuntimeError`` with a detailed diagnostic
    message on the first mismatch encountered.

    Parameters
    ----------
    homography_data : HomographyData
        Previously saved homography.
    current_camera_matrix : ndarray of shape (3, 3)
        Current camera intrinsic matrix.
    current_dist_coeffs : ndarray
        Current distortion coefficients.
    current_resolution : (width, height)
        Current image resolution.
    tolerance_fx : float
        Maximum allowed relative deviation in fx (default 5%).

    Raises
    ------
    RuntimeError
        On any detected mismatch.
    """
    current_w, current_h = current_resolution

    # 1. Resolution
    if homography_data.image_width != current_w or homography_data.image_height != current_h:
        raise RuntimeError(
            f"Resolution mismatch: "
            f"current={current_w}x{current_h}, "
            f"expected={homography_data.image_width}x{homography_data.image_height}, "
            f"suggestion=Re-compute homography at current resolution."
        )

    # 2. Point domain
    if homography_data.point_domain != "undistorted_pixel":
        raise RuntimeError(
            f"Point domain mismatch: "
            f"current='undistorted_pixel', "
            f"expected='{homography_data.point_domain}', "
            f"suggestion=Re-compute homography with consistent undistortion pipeline."
        )

    # 3. fx / fy tolerance
    K_saved = homography_data.camera_matrix
    K_curr = current_camera_matrix.astype(np.float64)

    fx_saved = K_saved[0, 0]
    fy_saved = K_saved[1, 1]
    fx_curr = K_curr[0, 0]
    fy_curr = K_curr[1, 1]

    if abs(fx_saved) < 1e-9 or abs(fy_saved) < 1e-9:
        raise RuntimeError(
            "Saved camera matrix has zero or near-zero focal length; "
            "homography is likely invalid."
        )

    rel_fx = abs(fx_curr - fx_saved) / abs(fx_saved)
    rel_fy = abs(fy_curr - fy_saved) / abs(fy_saved)

    if rel_fx > tolerance_fx:
        raise RuntimeError(
            f"fx mismatch: "
            f"current={fx_curr:.3f}, expected={fx_saved:.3f} "
            f"(relative diff={rel_fx:.4f} > tolerance={tolerance_fx}), "
            f"suggestion=Re-calibrate camera or re-compute homography."
        )
    if rel_fy > tolerance_fx:
        raise RuntimeError(
            f"fy mismatch: "
            f"current={fy_curr:.3f}, expected={fy_saved:.3f} "
            f"(relative diff={rel_fy:.4f} > tolerance={tolerance_fx}), "
            f"suggestion=Re-calibrate camera or re-compute homography."
        )

    # 4. Distortion coefficients (informational — distortion is zeroed at
    #    homography time, so we only check that D is at least 4 elements)
    if current_dist_coeffs is not None and current_dist_coeffs.size < 4:
        import warnings
        warnings.warn(
            "Current dist_coeffs have fewer than 4 elements; "
            "undistortion may be incomplete."
        )


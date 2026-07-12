# Cross-Circle Wheel-Center Measurement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Detect a coplanar 14 mm origin marker and 10 mm wheel-center cross-circle marker from the MF-200 feed and record the wheel center as robot-relative X/Y millimetres.

**Architecture:** A shared camera-source adapter provides responsive OpenCV or ffmpeg capture. A one-time chessboard command saves an audited undistorted-pixel-to-plane-mm calibration. A dedicated geometric detector and measurement tracker identify the two marker roles, transform their centers, reject ambiguous frames, and feed both the live measurement and IK collection commands while leaving ArUco mode intact.

**Tech Stack:** Python 3.8+, OpenCV 5, NumPy, `unittest`, ffmpeg DirectShow, NPZ/JSON calibration artifacts.

## Global Constraints

- MF-200 mode is exactly 1920x1080 through ffmpeg DirectShow.
- Camera focus, camera pose, resolution, and marker plane remain fixed after plane calibration.
- Origin uses the 14 mm PDF marker; wheel center uses the 10 mm PDF marker.
- Printed marker faces are coplanar; the approved spacer removes the approximately 30 mm depth offset.
- Robot `+X` points toward vehicle front, which is camera-image left; robot `+Y` points down.
- Measurement points originate in the distorted input frame, are undistorted as points, and are never read from a resampled display image.
- Existing ArUco functionality and tests remain supported.
- Invalid or ambiguous detections never update filters or CSV measurements.
- Preserve all unrelated dirty-worktree files; every commit stages only files named by its task.

## File Structure

- Create `tools/calibration/camera_source.py`: common OpenCV/ffmpeg capture adapter and CLI arguments.
- Create `tools/calibration/plane_calibration.py`: plane-calibration data model, computation, persistence, validation, and point mapping.
- Create `tools/calibration/plane_calibrate.py`: interactive chessboard plane-calibration command.
- Create `tools/calibration/cross_circle_detector.py`: marker candidate detection, scoring, role assignment, temporal locking, and filtered measurement tracker.
- Modify `tools/calibration/detect_markers.py`: retain ArUco path and add cross-circle live/image path.
- Modify `tools/calibration/calibrate_with_camera.py`: retain ArUco path and add cross-circle IK capture path.
- Modify `tools/calibration/README.md`: document plane calibration, marker mounting, commands, controls, and acceptance gate.
- Create focused tests under `tools/calibration/tests/` for every new boundary.

---

### Task 1: Shared Responsive Camera Source

**Files:**
- Create: `tools/calibration/camera_source.py`
- Create: `tools/calibration/tests/test_camera_source.py`
- Add existing required backend: `tools/calibration/ffmpeg_camera.py`
- Reuse: `tools/calibration/camera_utils.py`

**Interfaces:**
- Consumes: `open_camera_strict(index, backend, width, height, fps)` and `FfmpegCamera(name, width, height, fps)`.
- Produces: `add_capture_source_args(parser)`, `open_capture_source(...) -> tuple[CaptureSource, str]`, and `CaptureSource.read(timeout_sec=0.01) -> Optional[np.ndarray]` plus `close()`.

- [ ] **Step 1: Write the failing adapter tests**

Create `test_camera_source.py` with real wrapper behavior and injected openers:

```python
from __future__ import annotations

import argparse
import sys
import unittest
from pathlib import Path

import numpy as np

CALIB = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(CALIB))

from camera_source import add_capture_source_args, open_capture_source


class FakeCap:
    def __init__(self):
        self.frame = np.zeros((4, 6, 3), dtype=np.uint8)
        self.released = False

    def read(self):
        return True, self.frame

    def release(self):
        self.released = True


class FakeFfmpeg:
    def __init__(self, name, width, height, fps):
        self.args = (name, width, height, fps)
        self.opened = False
        self.closed = False

    def open(self):
        self.opened = True

    def read(self, timeout_sec=0.01):
        return np.ones((4, 6, 3), dtype=np.uint8)

    def close(self):
        self.closed = True


class TestCaptureSource(unittest.TestCase):
    def test_ffmpeg_source_forwards_timeout_and_closes(self):
        created = []
        source, backend = open_capture_source(
            camera_index=0, backend_label="AUTO", width=1920, height=1080,
            fps=30, use_ffmpeg=True, ffmpeg_name="USB Camera",
            ffmpeg_factory=lambda *args: created.append(FakeFfmpeg(*args)) or created[-1])
        self.assertEqual(backend, "ffmpeg-dshow")
        self.assertEqual(source.read(0.01).shape, (4, 6, 3))
        source.close()
        self.assertTrue(created[0].opened)
        self.assertTrue(created[0].closed)

    def test_opencv_source_normalizes_read_contract(self):
        cap = FakeCap()
        source, backend = open_capture_source(
            2, "MSMF", 1920, 1080, 30, False, "USB Camera",
            opencv_opener=lambda *args: (cap, "MSMF"))
        self.assertEqual(backend, "MSMF")
        self.assertIs(source.read(0.01), cap.frame)
        source.close()
        self.assertTrue(cap.released)

    def test_cli_arguments_default_to_opencv(self):
        parser = argparse.ArgumentParser()
        add_capture_source_args(parser)
        args = parser.parse_args([])
        self.assertFalse(args.ffmpeg)
        self.assertEqual(args.ffmpeg_name, "USB Camera")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```powershell
python -m unittest tools/calibration/tests/test_camera_source.py -v
```

Expected: import failure because `camera_source.py` does not exist.

- [ ] **Step 3: Implement the adapter**

Create `camera_source.py` with these complete public contracts:

```python
from __future__ import annotations

import argparse
from typing import Callable, Optional, Protocol, Tuple

import numpy as np

from camera_utils import open_camera_strict


class CaptureSource(Protocol):
    def read(self, timeout_sec: float = 0.01) -> Optional[np.ndarray]: ...
    def close(self) -> None: ...


class OpenCVCaptureSource:
    def __init__(self, cap):
        self.cap = cap
        self._queue = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._reader_loop, daemon=True,
            name="opencv-camera-reader")
        self._thread.start()

    def _reader_loop(self) -> None:
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            if not ok:
                self._stop.wait(0.01)
                continue
            try:
                self._queue.put_nowait(frame)
            except queue.Full:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    pass
                self._queue.put_nowait(frame)

    def read(self, timeout_sec: float = 0.01) -> Optional[np.ndarray]:
        try:
            return self._queue.get(timeout=timeout_sec)
        except queue.Empty:
            return None

    def close(self) -> None:
        self._stop.set()
        self.cap.release()
        self._thread.join(timeout=2.0)


class FfmpegCaptureSource:
    def __init__(self, camera):
        self.camera = camera

    def read(self, timeout_sec: float = 0.01) -> Optional[np.ndarray]:
        return self.camera.read(timeout_sec=timeout_sec)

    def close(self) -> None:
        self.camera.close()


def add_capture_source_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--ffmpeg", action="store_true",
                        help="Capture through ffmpeg DirectShow")
    parser.add_argument("--ffmpeg-name", default="USB Camera",
                        help="DirectShow friendly device name")


def open_capture_source(
    camera_index: int,
    backend_label: str,
    width: int,
    height: int,
    fps: float,
    use_ffmpeg: bool,
    ffmpeg_name: str,
    opencv_opener: Callable = open_camera_strict,
    ffmpeg_factory: Optional[Callable] = None,
) -> Tuple[CaptureSource, str]:
    if use_ffmpeg:
        if ffmpeg_factory is None:
            from ffmpeg_camera import FfmpegCamera
            ffmpeg_factory = FfmpegCamera
        camera = ffmpeg_factory(
            ffmpeg_name, width, height, int(fps) if fps > 0 else 30)
        camera.open()
        return FfmpegCaptureSource(camera), "ffmpeg-dshow"
    cap, backend = opencv_opener(
        camera_index, backend_label, width, height, fps)
    return OpenCVCaptureSource(cap), backend
```

The existing `FfmpegCamera` reader must also use a stop event and nonblocking
latest-frame queue insertion. When its queue is full it drops the oldest frame
before inserting the newest frame or shutdown sentinel. Neither normal frame
delivery nor the reader's `finally` block may block indefinitely on
`queue.put()`. Tests must assert timeout forwarding, `(False, None)` handling,
bounded reads while the OpenCV backend stalls, latest-frame replacement when a
queue is full, and reader-thread termination on close.

- [ ] **Step 4: Run the focused and baseline tests**

```powershell
python -m unittest tools/calibration/tests/test_camera_source.py -v
python -m unittest discover -s tools/calibration/tests -v
```

Expected: all tests pass.

- [ ] **Step 5: Commit Task 1 only**

```powershell
git add -- tools/calibration/camera_source.py tools/calibration/ffmpeg_camera.py tools/calibration/tests/test_camera_source.py
git commit -m "feat: add shared camera source adapter"
```

---

### Task 2: Audited Chessboard Plane Calibration

**Files:**
- Create: `tools/calibration/plane_calibration.py`
- Create: `tools/calibration/plane_calibrate.py`
- Create: `tools/calibration/tests/test_plane_calibration.py`
- Reuse: `tools/calibration/camera_calibrate.py`
- Reuse: `tools/calibration/camera_utils.py`
- Reuse: `tools/calibration/camera_source.py`

**Interfaces:**
- Produces `PlaneCalibration`, `compute_plane_calibration(...)`, `save_plane_calibration(...)`, `load_plane_calibration(...)`, `validate_plane_calibration(...)`, and `map_undistorted_points(...)`.
- `plane_calibrate.py` produces `tools/calibration/plane_homography.npz` and `.json`.

- [ ] **Step 1: Write failing geometry, persistence, and compatibility tests**

Create tests that generate a known projective transform, distort its pixel points through the calibrated camera model, recover the plane mapping, and assert:

```python
def make_synthetic_plane_calibration(
    front_direction="left", down_direction="down",
):
    src = np.array([
        [300.0, 200.0], [1500.0, 240.0],
        [1460.0, 900.0], [340.0, 860.0],
    ], dtype=np.float64)
    dst = np.array([
        [250.0, 0.0], [0.0, 0.0],
        [0.0, 175.0], [250.0, 175.0],
    ], dtype=np.float64)
    H, _ = cv2.findHomography(src, dst)
    K = np.array([
        [1097.6, 0.0, 950.7],
        [0.0, 1097.6, 492.8],
        [0.0, 0.0, 1.0],
    ])
    D = np.array([-0.0114, 0.00355, 0.0, 0.0])
    mapped = cv2.perspectiveTransform(
        src.reshape(-1, 1, 2), H).reshape(-1, 2)
    rmse = float(np.sqrt(np.mean(np.sum((mapped - dst) ** 2, axis=1))))
    return PlaneCalibration(
        H=H, camera_matrix=K, dist_coeffs=D,
        image_size=(1920, 1080), calib_path="camera_calib.npz",
        backend="ffmpeg-dshow", front_direction=front_direction,
        down_direction=down_direction, board_cols=9, board_rows=6,
        square_size_mm=25.0, rmse_mm=rmse,
        src_points_undistorted_px=src, dst_points_mm=dst)


class TestPlaneCalibration(unittest.TestCase):
    def test_compute_recovers_left_positive_down_positive_plane(self):
        calibration = make_synthetic_plane_calibration(
            front_direction="left", down_direction="down")
        recovered = calibration.map_undistorted_points(
            calibration.src_points_undistorted_px)
        np.testing.assert_allclose(
            recovered, calibration.dst_points_mm, atol=0.05)

    def test_round_trip_preserves_provenance(self):
        calibration = make_synthetic_plane_calibration()
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp) / "plane_homography"
            save_plane_calibration(base, calibration)
            loaded = load_plane_calibration(base)
        self.assertEqual(loaded.front_direction, "left")
        self.assertEqual(loaded.down_direction, "down")
        self.assertEqual(loaded.image_size, (1920, 1080))
        np.testing.assert_allclose(loaded.H, calibration.H)

    def test_rejects_camera_matrix_mismatch(self):
        calibration = make_synthetic_plane_calibration()
        wrong_K = calibration.camera_matrix.copy()
        wrong_K[0, 0] += 20.0
        with self.assertRaisesRegex(ValueError, "camera matrix"):
            validate_plane_calibration(
                calibration, wrong_K, calibration.dist_coeffs,
                (1920, 1080), "left", "down")
```

The test helper constructs `PlaneCalibration` explicitly and uses
`cv2.getPerspectiveTransform`/`cv2.perspectiveTransform`; it must not read a
hardware camera.

- [ ] **Step 2: Run the focused test and verify RED**

```powershell
python -m unittest tools/calibration/tests/test_plane_calibration.py -v
```

Expected: import failure for `plane_calibration`.

- [ ] **Step 3: Implement the plane-calibration data boundary**

Define:

```python
@dataclass
class PlaneCalibration:
    H: np.ndarray
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    image_size: tuple[int, int]
    calib_path: str
    backend: str
    front_direction: str
    down_direction: str
    board_cols: int
    board_rows: int
    square_size_mm: float
    rmse_mm: float
    src_points_undistorted_px: np.ndarray
    dst_points_mm: np.ndarray

    def map_undistorted_points(self, points: np.ndarray) -> np.ndarray:
        points = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(points, self.H).reshape(-1, 2)
```

`compute_plane_calibration` must:

1. accept `(N, 2)` distorted chessboard corners plus K/D;
2. call `undistort_points` exactly once;
3. reshape corners into `(rows, cols, 2)`;
4. assign destination grid millimetres so X increases toward image left when
   `front_direction="left"` and Y increases down when `down_direction="down"`;
5. call `cv2.findHomography(..., method=cv2.RANSAC, ransacReprojThreshold=1.0)`;
6. compute plane RMSE from the inlier set;
7. reject fewer than 80% inliers or RMSE above 0.5 mm.

Persistence writes numeric arrays to NPZ and all scalar provenance to a JSON
sidecar. Validation rejects image-size, K, D, point-domain, front-direction,
down-direction, backend, or calibration-path mismatch with a field-specific
`ValueError`. Loading/validation also rejects a non-finite or singular H,
malformed point arrays, non-positive board dimensions or square size, stored
RMSE above 0.5 mm, and stored RANSAC inliers below 80% of board points.

- [ ] **Step 4: Implement the interactive command**

`plane_calibrate.py` must parse existing camera args plus:

```python
parser.add_argument("--calib", type=Path, default=_DEFAULT_CALIB)
parser.add_argument("--output", type=Path, default=_DEFAULT_PLANE)
parser.add_argument("--cols", type=int, default=9)
parser.add_argument("--rows", type=int, default=6)
parser.add_argument("--square-size-mm", type=float, required=True)
parser.add_argument("--front-direction", choices=["left", "right"], required=True)
parser.add_argument("--down-direction", choices=["down", "up"], default="down")
add_capture_source_args(parser)
```

The loop uses `source.read(0.01)` and `cv2.waitKey(1)` on every iteration.
It overlays detected corners and alignment instructions. Pressing `c` requires
54 corners, computes/saves calibration, prints RMSE/inlier count, and keeps the
preview open; `q` exits. Before opening capture, the command requires requested
width/height to equal the loaded camera calibration resolution and requires the
selected capture family to match its backend provenance. Thus the current
MF-200 calibration automatically requires ffmpeg-dshow at 1920x1080, while a
future calibration made with an OpenCV backend remains usable through that
matching backend. Cleanup is in `finally`.

- [ ] **Step 5: Verify Task 2**

```powershell
python -m unittest tools/calibration/tests/test_plane_calibration.py -v
python -m py_compile tools/calibration/plane_calibration.py tools/calibration/plane_calibrate.py
python -m unittest discover -s tools/calibration/tests -v
```

Expected: all tests pass and compilation exits zero.

- [ ] **Step 6: Commit Task 2 only**

```powershell
git add -- tools/calibration/plane_calibration.py tools/calibration/plane_calibrate.py tools/calibration/tests/test_plane_calibration.py
git commit -m "feat: add chessboard plane calibration"
```

---

### Task 3: Cross-Circle Detection and Role Locking

**Files:**
- Create: `tools/calibration/cross_circle_detector.py`
- Create: `tools/calibration/tests/test_cross_circle_detector.py`

**Interfaces:**
- Produces `CrossCircleConfig`, `CrossCircleCandidate`, `CrossCircleRoles`, and `CrossCircleDetector.detect(frame)`, `.update(frame)`, `.reset()`.
- Role names are exact strings `origin` and `wheel`; states are `SEARCHING`, `VALID`, `MISSING`, and `AMBIGUOUS`.
- `CrossCircleCandidate` retains fitted ellipse, circularity, ring score,
  separate horizontal/vertical cross scores, center-agreement error,
  confidence, and optional assigned role for downstream audit/debug display.
- The detector exposes initial/reset `SEARCHING` through a public current-state
  accessor even before a frame is processed.

- [ ] **Step 1: Write a synthetic marker renderer inside the test module**

The renderer creates a white canvas, draws a black ring and centered black plus,
then optionally applies rotation, perspective, Gaussian blur, brightness
gradient, and clutter. Tests cover:

```python
def draw_cross_circle(image, center, diameter):
    cx, cy = (int(round(center[0])), int(round(center[1])))
    radius = int(round(diameter / 2.0))
    thickness = max(2, int(round(diameter * 0.08)))
    arm = int(round(radius * 0.68))
    cv2.circle(image, (cx, cy), radius, (0, 0, 0), thickness,
               lineType=cv2.LINE_AA)
    cv2.line(image, (cx - arm, cy), (cx + arm, cy),
             (0, 0, 0), thickness, cv2.LINE_AA)
    cv2.line(image, (cx, cy - arm), (cx, cy + arm),
             (0, 0, 0), thickness, cv2.LINE_AA)


def render_pair(origin=(400.0, 280.0), wheel=(720.0, 600.0),
                origin_diameter=70, wheel_diameter=50,
                angle_deg=4.0, blur_sigma=0.8):
    image = np.full((900, 1200, 3), 255, dtype=np.uint8)
    draw_cross_circle(image, origin, origin_diameter)
    draw_cross_circle(image, wheel, wheel_diameter)
    matrix = cv2.getRotationMatrix2D((600, 450), angle_deg, 1.0)
    image = cv2.warpAffine(
        image, matrix, (1200, 900), borderValue=(255, 255, 255))
    transformed = {}
    for name, point in {"origin": origin, "wheel": wheel}.items():
        transformed[name] = matrix @ np.array([point[0], point[1], 1.0])
    if blur_sigma > 0:
        image = cv2.GaussianBlur(image, (0, 0), blur_sigma)
    return image, transformed


def render_one():
    image = np.full((900, 1200, 3), 255, dtype=np.uint8)
    draw_cross_circle(image, (400, 300), 70)
    return image


def render_three():
    image, _ = render_pair(angle_deg=0, blur_sigma=0)
    draw_cross_circle(image, (950, 450), 60)
    return image


class TestCrossCircleDetector(unittest.TestCase):
    def setUp(self):
        self.detector = CrossCircleDetector()

    def test_detects_subpixel_centers_under_rotation_and_blur(self):
    image, expected = render_pair(origin=(400.3, 280.6), wheel=(720.4, 600.2))
    roles = self.detector.update(image)
    self.assertEqual(roles.status, "VALID")
    np.testing.assert_allclose(roles.origin.center, expected["origin"], atol=0.5)
    np.testing.assert_allclose(roles.wheel.center, expected["wheel"], atol=0.5)

def test_assigns_larger_marker_to_origin(self):
    image, _ = render_pair(origin_diameter=70, wheel_diameter=50)
    roles = self.detector.update(image)
    self.assertGreater(roles.origin.diameter_px, roles.wheel.diameter_px)

def test_rejects_invalid_candidate_counts(self):
    self.assertEqual(self.detector.update(render_one()).status, "MISSING")
    self.assertEqual(self.detector.update(render_three()).status, "AMBIGUOUS")

def test_rejects_wrong_size_ratio(self):
    image, _ = render_pair(origin_diameter=60, wheel_diameter=55)
    self.assertEqual(self.detector.update(image).status, "AMBIGUOUS")

def test_reset_clears_role_lock(self):
    roles = self.detector.update(render_pair()[0])
    self.assertEqual(roles.status, "VALID")
    self.detector.reset()
    self.assertFalse(self.detector.locked)
```

- [ ] **Step 2: Run tests and verify RED**

```powershell
python -m unittest tools/calibration/tests/test_cross_circle_detector.py -v
```

Expected: import failure for `cross_circle_detector`.

- [ ] **Step 3: Implement geometric candidate extraction**

Use a single `CrossCircleConfig` containing exact tunables:

```python
@dataclass(frozen=True)
class CrossCircleConfig:
    min_diameter_px: float = 16.0
    max_diameter_px: float = 240.0
    max_aspect_ratio: float = 1.30
    min_circularity: float = 0.60
    min_ring_score: float = 0.55
    min_cross_score: float = 0.55
    max_center_error_fraction: float = 0.16
    expected_size_ratio: float = 1.40
    size_ratio_tolerance: float = 0.18
    origin_lock_radius_px: float = 15.0
    min_confidence: float = 0.65
```

`detect(frame)` performs CLAHE, adaptive inverse thresholding, Canny edges,
`findContours(RETR_LIST)`, ellipse fitting, circularity/aspect checks, ring
sampling, and horizontal/vertical projection scoring. It fuses ellipse center
and the maximum row/column projections for the final center. Candidates closer
than 25% of the smaller diameter are non-maximum-suppressed by confidence.

`update(frame)` requires exactly two candidates above confidence, verifies the
diameter ratio, locks larger as origin, and on later frames chooses the origin
candidate nearest the locked origin within `origin_lock_radius_px`. A locked
update must still satisfy that the spatially matched origin is the larger
candidate and the 14:10 ratio remains valid; otherwise return `AMBIGUOUS`
without swapping roles or changing the saved lock. It never silently relocks
after an ambiguous or missing frame; only `reset()` clears the role lock.
Focused tests verify retained lock center across missing/ambiguous frames,
rejection beyond the lock radius, recovery near the original lock, NMS at the
25% boundary, and confidence-threshold rejection.

- [ ] **Step 4: Verify Task 3**

```powershell
python -m unittest tools/calibration/tests/test_cross_circle_detector.py -v
python -m unittest discover -s tools/calibration/tests -v
```

Expected: all tests pass.

- [ ] **Step 5: Commit Task 3 only**

```powershell
git add -- tools/calibration/cross_circle_detector.py tools/calibration/tests/test_cross_circle_detector.py
git commit -m "feat: detect cross-circle marker centers"
```

---

### Task 4: Relative Millimetre Measurement and Robust Capture

**Files:**
- Modify: `tools/calibration/cross_circle_detector.py`
- Create: `tools/calibration/tests/test_cross_circle_measurement.py`
- Reuse: `tools/calibration/plane_calibration.py`
- Reuse: `tools/calibration/camera_utils.py`

**Interfaces:**
- Produces `CrossCircleMeasurement` and `CrossCircleMeasurementTracker.process(frame)`, `.live_measurement()`, `.capture()`, `.reset()`.
- `capture()` returns `None` until 15 valid frames exist.
- Constructor: `CrossCircleMeasurementTracker(detector, plane_calibration, camera_matrix, dist_coeffs, jump_threshold_mm=20.0)`.

- [ ] **Step 1: Write failing tracker tests with a fake detector and known H**

```python
class FakeDetector:
    def __init__(self, frames):
        self.frames = deque(frames)

    def update(self, frame):
        del frame
        return self.frames.popleft()

    def reset(self):
        self.frames.clear()


def candidate(center, diameter, confidence=0.9):
    return CrossCircleCandidate(
        center=np.asarray(center, dtype=np.float64),
        diameter_px=float(diameter), ellipse=None, circularity=0.95,
        ring_score=0.9, horizontal_score=0.9, vertical_score=0.9,
        center_error_px=0.1, confidence=confidence)


def roles(origin=(0, 0), wheel=(0, 0), status="VALID"):
    return CrossCircleRoles(
        origin=candidate(origin, 70), wheel=candidate(wheel, 50),
        status=status)


def ambiguous_roles():
    return CrossCircleRoles(origin=None, wheel=None, status="AMBIGUOUS")


def jump_roles():
    return roles(origin=(0, 0), wheel=(1000, 1000))


def blank_frame():
    return np.zeros((8, 8, 3), dtype=np.uint8)


def make_tracker(frames):
    plane = PlaneCalibration(
        H=np.diag([-1.0, 1.0, 1.0]),
        camera_matrix=np.eye(3), dist_coeffs=np.zeros(4),
        image_size=(1920, 1080), calib_path="camera_calib.npz",
        backend="test", front_direction="left", down_direction="down",
        board_cols=9, board_rows=6, square_size_mm=25.0, rmse_mm=0.0,
        src_points_undistorted_px=np.zeros((4, 2)),
        dst_points_mm=np.zeros((4, 2)))
    return CrossCircleMeasurementTracker(
        FakeDetector(frames), plane, np.eye(3), np.zeros(4))


class TestCrossCircleMeasurement(unittest.TestCase):
    def test_relative_axes_are_left_positive_and_down_positive(self):
    tracker = make_tracker(
        frames=[roles(origin=(600, 400), wheel=(500, 520))] * 15)
    for _ in range(15):
        tracker.process(blank_frame())
    captured = tracker.capture()
    self.assertAlmostEqual(captured.x_mm, 100.0, places=6)
    self.assertAlmostEqual(captured.y_mm, 120.0, places=6)

    def test_capture_requires_fifteen_valid_frames(self):
        tracker = make_tracker(frames=[roles()] * 14)
    for _ in range(14):
        tracker.process(blank_frame())
    self.assertIsNone(tracker.capture())

    def test_live_value_is_median_of_latest_five(self):
        sequence = [roles(origin=(0, 0), wheel=(-x, 0))
                    for x in [10, 11, 100, 12, 13]]
        tracker = make_tracker(frames=sequence)
    for _ in range(5):
        tracker.process(blank_frame())
    self.assertEqual(tracker.live_measurement().x_mm, 12.0)

    def test_invalid_and_jump_frames_do_not_enter_history(self):
        tracker = make_tracker(frames=[roles(), ambiguous_roles(), jump_roles()])
    for _ in range(3):
        tracker.process(blank_frame())
    self.assertEqual(tracker.valid_frame_count, 1)
```

- [ ] **Step 2: Run tests and verify RED**

```powershell
python -m unittest tools/calibration/tests/test_cross_circle_measurement.py -v
```

Expected: missing tracker interfaces.

- [ ] **Step 3: Implement the tracker**

Define the measurement record:

```python
@dataclass(frozen=True)
class CrossCircleMeasurement:
    x_mm: float
    y_mm: float
    origin_u_px: float
    origin_v_px: float
    wheel_u_px: float
    wheel_v_px: float
    confidence: float
    valid_frames: int
    status: str
```

`process` must undistort only the two detected centers with `undistort_points`,
map both through `PlaneCalibration.map_undistorted_points`, subtract origin
from wheel, and append only valid non-jump values to `deque(maxlen=15)`. A jump
is rejected when either component differs by more than 20 mm from the latest
five-frame median. `live_measurement` returns the component-wise median of the
latest five valid frames; `capture` returns the component-wise median of all 15
history entries only when the deque is full. Confidence is the smaller of the
two role confidences.

- [ ] **Step 4: Verify Task 4**

```powershell
python -m unittest tools/calibration/tests/test_cross_circle_measurement.py -v
python -m unittest discover -s tools/calibration/tests -v
```

Expected: all tests pass.

- [ ] **Step 5: Commit Task 4 only**

```powershell
git add -- tools/calibration/cross_circle_detector.py tools/calibration/tests/test_cross_circle_measurement.py
git commit -m "feat: measure cross-circle wheel coordinates"
```

---

### Task 5: Add Cross-Circle Mode to Live Measurement

**Files:**
- Modify: `tools/calibration/detect_markers.py`
- Create: `tools/calibration/tests/test_detect_markers_cross_circle.py`

**Interfaces:**
- Consumes: `open_capture_source`, `PlaneCalibration`, and `CrossCircleMeasurementTracker`.
- CLI adds `--marker-type`, `--plane-homography`, `--ffmpeg`, and `--ffmpeg-name`.
- ArUco default remains `--marker-type aruco` for compatibility.

- [ ] **Step 1: Write failing parser and CSV tests**

Extract `build_parser()` and `_cross_circle_csv_row(measurement, label)` so tests can assert:

```python
def sample_measurement():
    return CrossCircleMeasurement(
        x_mm=25.5, y_mm=80.25,
        origin_u_px=900.0, origin_v_px=400.0,
        wheel_u_px=760.0, wheel_v_px=620.0,
        confidence=0.91, valid_frames=15, status="VALID")


def test_parser_accepts_cross_circle_ffmpeg_mode(self):
    args = build_parser().parse_args([
        "--marker-type", "cross-circle", "--ffmpeg",
        "--ffmpeg-name", "USB Camera"])
    self.assertEqual(args.marker_type, "cross-circle")
    self.assertTrue(args.ffmpeg)

def test_cross_circle_csv_contains_traceability_fields(self):
    row = _cross_circle_csv_row(sample_measurement(), "meas_000")
    self.assertEqual(row["x_mm"], "25.50")
    self.assertIn("origin_u_px", row)
    self.assertIn("wheel_v_px", row)
    self.assertIn("confidence", row)
    self.assertEqual(row["valid_frames"], 15)
```

- [ ] **Step 2: Run tests and verify RED**

```powershell
python -m unittest tools/calibration/tests/test_detect_markers_cross_circle.py -v
```

Expected: missing parser/row helpers.

- [ ] **Step 3: Add a separate cross-circle live loop without disturbing ArUco**

Implement `interactive_measure_cross_circle(args)` that:

- loads `CalibrationData` and `PlaneCalibration`;
- calls `validate_plane_calibration` before opening the camera;
- uses `open_capture_source` and a 0.01-second read timeout;
- calls `cv2.waitKey(1)` every loop, including no-frame loops;
- draws origin in blue, wheel in green, and shows status/confidence/X/Y/history;
- maps `r` to tracker reset, SPACE to 15-frame capture, `s` to CSV save, and
  `q`/Escape to cleanup;
- rejects SPACE with an explicit `need N more valid frames` message;
- rejects SPACE unless the detector's current frame state is exactly `VALID`,
  even when 15 older valid history entries exist;
- writes fields `label,x_mm,y_mm,origin_u_px,origin_v_px,wheel_u_px,wheel_v_px,confidence,valid_frames,status`.

Camera/plane validation explicitly compares the loaded camera calibration
backend with the requested backend family and plane backend, in addition to
K/D/resolution/path. One `try/finally` covers every operation after capture is
opened, including tracker construction and actual-backend checks, so setup
exceptions cannot leak the source.

Keep current `interactive_measure` and ArUco image mode unchanged. Dispatch in
`main()` based on `args.marker_type`.

- [ ] **Step 4: Verify Task 5**

```powershell
python -m unittest tools/calibration/tests/test_detect_markers_cross_circle.py -v
python -m py_compile tools/calibration/detect_markers.py
python -m unittest discover -s tools/calibration/tests -v
python tools/calibration/detect_markers.py --help
```

Expected: all tests pass; help lists both marker types and ffmpeg flags.

- [ ] **Step 5: Commit Task 5 only**

```powershell
git add -- tools/calibration/detect_markers.py tools/calibration/tests/test_detect_markers_cross_circle.py
git commit -m "feat: add cross-circle live measurement mode"
```

---

### Task 6: Integrate IK Collection, Documentation, and Final Gates

**Files:**
- Modify: `tools/calibration/calibrate_with_camera.py`
- Modify: `tools/calibration/README.md`
- Create: `tools/calibration/tests/test_calibrate_with_cross_circle.py`

**Interfaces:**
- Consumes: shared camera source, plane calibration, and measurement tracker.
- Preserves current output columns and fills `measured_x_mm`/`measured_y_mm` from a 15-frame cross-circle capture.
- Produces `CrossCirclePoseState(poses)` with `pose_index` and `capture(measurement) -> bool` so invalid captures cannot advance the wizard.

- [ ] **Step 1: Write failing parser and IK-row tests**

Extract `build_parser()` and `_build_cross_circle_ik_row(...)`:

```python
def sample_measurement():
    return CrossCircleMeasurement(
        x_mm=25.5, y_mm=80.25,
        origin_u_px=900.0, origin_v_px=400.0,
        wheel_u_px=760.0, wheel_v_px=620.0,
        confidence=0.91, valid_frames=15, status="VALID")


def test_ik_parser_accepts_cross_circle_ffmpeg(self):
    args = build_parser().parse_args([
        "--marker-type", "cross-circle", "--ffmpeg"])
    self.assertEqual(args.marker_type, "cross-circle")
    self.assertTrue(args.ffmpeg)

def test_valid_capture_populates_existing_measurement_columns(self):
    row = _build_cross_circle_ik_row(
        sample_id=0, pose=(90, 90, 90, 90, "mid_center"),
        measurement=sample_measurement())
    self.assertEqual(row["measured_x_mm"], "25.50")
    self.assertEqual(row["measured_y_mm"], "80.25")
    self.assertEqual(row["note"], "camera_measured_cross_circle")
    self.assertEqual(row["valid_frames"], 15)

def test_missing_capture_does_not_advance_pose(self):
    state = CrossCirclePoseState(DEFAULT_POSES)
    self.assertFalse(state.capture(None))
    self.assertEqual(state.pose_index, 0)
```

- [ ] **Step 2: Run tests and verify RED**

```powershell
python -m unittest tools/calibration/tests/test_calibrate_with_cross_circle.py -v
```

Expected: missing cross-circle parser/row/state interfaces.

- [ ] **Step 3: Implement cross-circle IK mode**

Add parser fields identical to Task 5 plus `--plane-homography`. Dispatch to a
new `manual_measure_cross_circle(args)` while retaining the ArUco function.
Implement the wizard gate exactly as:

```python
class CrossCirclePoseState:
    def __init__(self, poses):
        self.poses = list(poses)
        self.pose_index = 0

    def capture(self, measurement):
        if measurement is None or self.pose_index >= len(self.poses):
            return False
        self.pose_index += 1
        return True
```

The loop shows the current servo pose and live X/Y, but SPACE advances the pose
only if `tracker.capture()` returns a valid 15-frame measurement. CSV keeps all
existing columns and adds:

```text
origin_u_px,origin_v_px,wheel_u_px,wheel_v_px,confidence,valid_frames
```

`marker_count` is `2`, `marker_ids` is `origin,wheel`, and `note` is
`camera_measured_cross_circle`. Snapshot filenames retain the existing pose
index/label/timestamp convention. `r` removes the most recent sample and moves
back one pose; a separate `l` key clears marker role lock so `r` keeps its
existing meaning.

- [ ] **Step 4: Update the README with exact physical and command workflow**

Document:

- print at 100%; 14 mm origin, 10 mm wheel, 12 mm unused;
- spacer requirement so faces are coplanar;
- chessboard alignment and `plane_calibrate.py` command;
- camera/focus/plane immobility rule;
- live and IK commands from the approved design;
- `r`/`l`/SPACE controls;
- validation positions and thresholds from the design spec;
- explicit statement that ArUco remains available.

- [ ] **Step 5: Run complete software verification**

```powershell
git diff --check
python -m py_compile tools/calibration/camera_source.py tools/calibration/plane_calibration.py tools/calibration/plane_calibrate.py tools/calibration/cross_circle_detector.py tools/calibration/detect_markers.py tools/calibration/calibrate_with_camera.py
python -m unittest discover -s tools/calibration/tests -v
python tools/calibration/plane_calibrate.py --help
python tools/calibration/detect_markers.py --help
python tools/calibration/calibrate_with_camera.py --help
python tools/calibration/ffmpeg_camera.py --name "USB Camera" --width 1920 --height 1080 --fps 30 --validate
```

Expected: zero diff-check errors, compilation success, all tests pass, all help
commands list approved flags, and the MF-200 reads 30 frames at 1920x1080.

- [ ] **Step 6: Commit Task 6 only**

```powershell
git add -- tools/calibration/calibrate_with_camera.py tools/calibration/README.md tools/calibration/tests/test_calibrate_with_cross_circle.py
git commit -m "feat: collect IK data from cross-circle markers"
```

- [ ] **Step 7: Perform the hardware acceptance gate with the user**

After software completion, guide the user through plane calibration and collect
at least five repeats at selected points spanning `X=-50,0,+50 mm` and
`Y=50,100,150 mm`. Run:

```powershell
python tools/calibration/validate_measurement.py --help
```

Then invoke its existing accepted CLI with the captured CSV. Do not run
`fit_leg_ik_calibration.py` unless repeatability standard deviation is at most
1.0 mm, overall RMSE is at most 2.0 mm, no maximum error exceeds 3.0 mm, and no
position-dependent systematic error is visible.

---

## Plan Self-Review Mapping

- Shared ffmpeg responsiveness: Tasks 1, 5, and 6.
- Audited plane calibration and metadata rejection: Task 2.
- Geometric circle/cross validation and role locking: Task 3.
- Point-domain correctness, median filters, jump rejection, and 15-frame capture: Task 4.
- Live HUD and traceable CSV: Task 5.
- IK CSV integration while preserving ArUco: Task 6.
- Synthetic, regression, real-camera, and hardware acceptance evidence: Tasks 2-6.

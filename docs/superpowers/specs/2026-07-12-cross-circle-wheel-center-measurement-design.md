# Cross-Circle Wheel-Center Measurement Design

## Goal

Replace ArUco markers in the wheel-center measurement path with two compact
cross-circle markers while preserving millimetre-scale planar coordinates for
IK calibration. The system tracks a fixed origin and a moving wheel center,
then reports the wheel center relative to the origin.

## Confirmed Physical Setup

- Camera: MF-200 through ffmpeg DirectShow at 1920x1080.
- Camera intrinsics: `tools/calibration/camera_calib.npz`.
- Marker artwork: `cross_circle_markers_A4_10_12_14mm.pdf`, printed at 100%.
- Origin marker: 14 mm total white-square size.
- Wheel marker: 10 mm total white-square size.
- The 12 mm marker is reserved and is not used by this workflow.
- The two printed marker faces must be coplanar. A spacer is permitted to
  remove the approximately 30 mm original depth difference.
- The camera remains fixed after measurement-plane calibration.
- In the camera image, vehicle front is to the left.
- Robot coordinates use the origin marker center as `(0, 0)`, `+X` toward
  vehicle front, and `+Y` vertically down.

## Chosen Approach

Use one-time chessboard plane calibration plus geometric cross-circle center
detection. The existing 9x6 inner-corner chessboard with measured 25 mm squares
defines a homography from undistorted image pixels to the common marker plane.
At runtime, marker centers are undistorted individually and mapped through this
homography. The result does not depend on the lens's nominal 2.8 mm focal length.

Direct pixel-diameter scaling is rejected because it cannot fully compensate
perspective. Multi-scale template matching is rejected because it is less
robust to rotation, blur, illumination, and scale changes and provides weaker
center accuracy.

## Architecture

### `cross_circle_detector.py`

This new module owns cross-circle detection and role tracking. It exposes a
detector that accepts one distorted BGR frame and returns candidates containing:

- subpixel center in distorted pixels;
- fitted outer ellipse and effective diameter;
- circle/ellipse quality score;
- horizontal-cross score;
- vertical-cross score;
- center-agreement error;
- aggregate confidence;
- assigned role (`origin` or `wheel`) when identity is locked.

The module performs no camera opening, homography loading, CSV writing, or IK
logic.

### `plane_calibrate.py`

This new command loads camera intrinsics, opens either the existing OpenCV
camera source or the MF-200 ffmpeg source, detects all 54 chessboard corners,
undistorts those corner coordinates, and fits a homography using all available
points. It saves `tools/calibration/plane_homography.npz` with:

- homography matrix;
- source undistorted pixel points;
- destination plane points in millimetres;
- camera matrix and distortion coefficients;
- image width and height;
- camera backend and calibration path;
- square size and chessboard geometry;
- axis convention (`front_direction=left`, `down_direction=down`);
- fit residual statistics.

The chessboard horizontal grid direction must be parallel to vehicle
front/rear, and its vertical grid direction must be parallel to vehicle up/down.
The command resolves chessboard ordering so the saved mapping has `+X` toward
the left side of the camera image and `+Y` downward.

### Existing Commands

`detect_markers.py` gains explicit cross-circle mode while retaining ArUco mode:

```text
--marker-type aruco|cross-circle
--ffmpeg
--ffmpeg-name "USB Camera"
--plane-homography tools/calibration/plane_homography.npz
```

Cross-circle mode displays both pixel centers, role/confidence, and relative
`X/Y` millimetres. Captured CSV rows include coordinates, pixel centers,
confidence, and the number of valid frames used.

`calibrate_with_camera.py` consumes the same detector and plane mapping. Its
existing `measured_x_mm` and `measured_y_mm` output fields receive the filtered
relative wheel-center coordinates. ArUco mode remains available.

Both live commands use the responsive ffmpeg polling pattern already used by
`camera_calibrate.py`: short frame-read timeouts, continuous OpenCV event
processing, and no heavy synchronous detection work that can starve the GUI.

## Coordinate Pipeline

The measurement path is:

```text
raw MF-200 frame
  -> detect cross-circle centers in distorted pixels
  -> undistort only the detected center coordinates with camera K and D
  -> apply saved plane homography
  -> obtain origin and wheel positions in plane millimetres
  -> subtract origin from wheel
  -> apply saved axis convention
  -> filter, display, and record X/Y
```

Measurement coordinates must never be derived from a resampled undistorted
display image. This preserves the existing audited coordinate-domain rule.

## Detection and Identity Rules

1. Convert to grayscale and enhance local contrast.
2. Produce adaptive-threshold and edge representations.
3. Find closed contours and fit ellipses to candidates with sufficient points.
4. Reject candidates outside configured bounds for diameter, circularity,
   ellipse aspect ratio, border clearance, or contrast.
5. In each candidate ROI, require a dark horizontal stroke and a dark vertical
   stroke crossing near the fitted ellipse center.
6. Fuse the ellipse center and cross intersection into a subpixel center.
7. During initial lock, require exactly two strong candidates and a diameter
   ratio consistent with 14:10. The larger candidate becomes `origin`; the
   smaller candidate becomes `wheel`.
8. After lock, require the origin to remain near its tracked location while the
   wheel may move. Pressing `r` clears the lock and restarts role assignment.

Thresholds are configuration values covered by synthetic and recorded-frame
tests rather than unexplained constants embedded throughout the code.

## Filtering, Capture, and Failure Behaviour

- Live HUD coordinates use the component-wise median of the latest five valid
  frames.
- A capture uses the component-wise median of the latest 15 valid frames.
- Capture is rejected until 15 valid frames are available.
- Missing origin or wheel produces no valid coordinate.
- More than two plausible candidates, an invalid size ratio, low confidence,
  or an excessive single-frame jump produces `AMBIGUOUS` or `INVALID`; such
  frames do not update filters or CSV output.
- A plane-homography mismatch in camera matrix, distortion, resolution, point
  domain, or axis metadata prevents measurement from starting and reports the
  exact mismatch.
- Loss of ffmpeg frames must not freeze the GUI.

The system prefers a missing measurement over a plausible but unverified
measurement.

## User Workflow

Plane calibration:

```powershell
python tools/calibration/plane_calibrate.py `
  --ffmpeg `
  --ffmpeg-name "USB Camera" `
  --width 1920 `
  --height 1080 `
  --square-size-mm 25 `
  --front-direction left
```

Live verification:

```powershell
python tools/calibration/detect_markers.py `
  --marker-type cross-circle `
  --ffmpeg `
  --ffmpeg-name "USB Camera" `
  --width 1920 `
  --height 1080
```

IK calibration collection:

```powershell
python tools/calibration/calibrate_with_camera.py `
  --marker-type cross-circle `
  --ffmpeg `
  --ffmpeg-name "USB Camera" `
  --width 1920 `
  --height 1080
```

The camera, focus ring, resolution, and marker plane must not move after plane
calibration. Moving any of them invalidates `plane_homography.npz` and requires
plane recalibration.

## Automated Verification

Tests generate synthetic 10 mm and 14 mm cross-circle images and cover scale,
rotation, perspective, illumination gradients, blur, and background clutter.
Required assertions are:

- center error no greater than 0.5 pixels in accepted synthetic cases;
- stable 14 mm origin and 10 mm wheel role assignment;
- rejection of one candidate, more than two candidates, and invalid size ratio;
- no coordinate update from low-confidence or jump-rejected frames;
- homography recovery of known synthetic plane coordinates;
- rejection of stale homography metadata;
- correct `+X` left and `+Y` down signs;
- median-window and 15-frame capture behaviour;
- responsive GUI event processing during ffmpeg frame stalls;
- preserved ArUco tests and existing coordinate-domain tests.

## Hardware Acceptance Gate

Validate known wheel-center positions spanning at least:

```text
X = -50, 0, +50 mm
Y = 50, 100, 150 mm
```

Collect at least five repeated samples at each selected position and run
`validate_measurement.py`. Acceptance requires:

- repeatability standard deviation no greater than 1.0 mm;
- overall RMSE no greater than 2.0 mm;
- no maximum error greater than 3.0 mm;
- no visible position-dependent systematic error.

IK parameter fitting remains blocked until this measurement validation passes.

## Compatibility and Scope

- Existing ArUco generation, detection, and tests remain supported.
- Camera intrinsic calibration is not redesigned.
- Firmware, servo control, IK geometry, and motor-control limits are outside
  this change.
- The supplied PDF is consumed as a physical asset; this change does not
  regenerate or alter it.

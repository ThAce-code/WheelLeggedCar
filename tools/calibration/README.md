# Camera-Based IK Calibration Tools

Camera-assisted measurement for wheel-leg robot inverse kinematics calibration.
Replaces manual ruler measurements with computer vision for better accuracy and repeatability.

## Quick Start

```powershell
# Step 1: Check your camera
python tools/calibration/camera_info.py

# Step 2: Generate & print a calibration pattern
python tools/calibration/camera_calibrate.py --generate

# Step 3: Calibrate the camera (interactive)
python tools/calibration/camera_calibrate.py

# Step 4: Generate & print ArUco markers
python tools/calibration/detect_markers.py --generate

# Step 5: Place markers on robot, then measure
python tools/calibration/detect_markers.py

# Step 6: Run IK calibration with camera
python tools/calibration/calibrate_with_camera.py --manual
```

## Tool Overview

| Script | Purpose | When to Use |
|--------|---------|-------------|
| `camera_info.py` | Inspect camera parameters, live preview | First step — verify camera works |
| `camera_calibrate.py` | Intrinsic calibration (camera matrix, distortion) | Once per camera setup |
| `detect_markers.py` | Detect ArUco markers on robot, estimate pose | Calibration measurement step |
| `calibrate_with_camera.py` | Integrated workflow: pose → measure → CSV | Full IK calibration run |

## Cross-Circle Wheel-Center Workflow

ArUco remains available and is still the default marker type. Use the
cross-circle mode when measuring the wheel center in the calibrated side plane.

### Physical setup

Print `cross_circle_markers_A4_10_12_14mm.pdf` at **Actual size / 100%** (never
Fit to page). Mount the 14 mm marker as the fixed origin and the 10 mm marker at
the wheel center. The 12 mm marker is unused in this workflow. The two printed
marker faces must be coplanar; install a rigid spacer to remove the original
approximately 30 mm face-depth difference.

Align the 9x6-inner-corner, 25 mm-square chessboard so its horizontal grid is
parallel to vehicle front/rear and its vertical grid is parallel to vehicle
up/down. With the vehicle front appearing on image-left, create the plane map:

```powershell
python tools/calibration/plane_calibrate.py `
  --ffmpeg `
  --ffmpeg-name "USB Camera" `
  --width 1920 `
  --height 1080 `
  --square-size-mm 25 `
  --front-direction left
```

After this step, do not move the camera, focus ring, resolution, or marker
plane. Any change invalidates `plane_homography.npz` and requires plane
recalibration.

### Live verification and IK collection

```powershell
python tools/calibration/detect_markers.py `
  --marker-type cross-circle `
  --ffmpeg `
  --ffmpeg-name "USB Camera" `
  --width 1920 `
  --height 1080

python tools/calibration/calibrate_with_camera.py `
  --marker-type cross-circle `
  --ffmpeg `
  --ffmpeg-name "USB Camera" `
  --width 1920 `
  --height 1080
```

In cross-circle IK collection, `SPACE` captures only when the current frame is
VALID and the tracker has accumulated 15 valid frames. `r` removes the latest
sample and returns to its pose without changing the marker-role lock. `l`
clears the role lock and filtered history. `q` or `ESC` exits. The CSV preserves
the existing IK columns and adds origin/wheel pixel centers, confidence, and
valid-frame count.

### Measurement acceptance gate

Before fitting IK parameters, validate known points spanning at least
`X=-50,0,+50 mm` and `Y=50,100,150 mm`, with at least five repeats at every
selected position. Build a validation CSV using the cross-circle IK column
names; repeated rows use the same position label:

```text
label,measured_x_mm,measured_y_mm
x0_y50,0.2,50.1
x0_y50,-0.1,49.8
```

Create `data/cross_circle_position_truth.json` with the exact physical ground
truth for every label:

```json
{
  "positions": {
    "xm50_y50": {"x_mm": -50.0, "y_mm": 50.0},
    "x0_y50": {"x_mm": 0.0, "y_mm": 50.0},
    "xp50_y50": {"x_mm": 50.0, "y_mm": 50.0}
  }
}
```

Run the IK-compatible position validator (the older `--csv --truth` mode is
only for ArUco marker-pair distance CSVs and is not compatible with IK rows):

```powershell
python tools/calibration/validate_measurement.py `
  --ik-csv data/cross_circle_position_validation.csv `
  --position-truth data/cross_circle_position_truth.json `
  --output data/cross_circle_position_validation_report.json
```

The command is strict: every truth label must appear in the CSV, every CSV
label must exist in truth, and every label must have at least five rows.
Schema or coverage failures exit `1`; a completed measurement that fails a
quality gate exits `2`; only a complete passing report exits `0`.

The JSON report contains `mean_dx_mm` and `mean_dy_mm` for every position so
axis-direction bias is visible. `x_bias_span_mm` and `y_bias_span_mm` are the
maximum-minus-minimum signed biases across positions; either span above
2.0 mm fails `no_systematic_error`. The other enforced gates are pooled 2-D
repeatability <=1.0 mm, radial position RMSE <=2.0 mm, and radial maximum error
<=3.0 mm. The top-level `passed` is true only when all four gates pass.

Continue only when repeatability standard deviation is at most 1.0 mm,
overall RMSE is at most 2.0 mm, maximum error never exceeds 3.0 mm, and the
per-position results show no systematic trend across X/Y. Do not run
`fit_leg_ik_calibration.py` until all four conditions pass.

## Detailed Usage

### 1. Camera Inspection (`camera_info.py`)

```bash
# List all cameras
python tools/calibration/camera_info.py --list-only

# Preview camera 0
python tools/calibration/camera_info.py

# Preview with undistortion (after calibration)
python tools/calibration/camera_info.py --calib tools/calibration/camera_calib.npz

# Use camera 1 at specific resolution
python tools/calibration/camera_info.py --camera 1 --width 1920 --height 1080
```

Controls in preview window:
- `q` / `ESC` — quit
- `s` — save snapshot to `tools/calibration/snapshots/`
- `c` — toggle crosshair overlay
- `u` — toggle undistort (if calibration loaded)

### 2. Camera Calibration (`camera_calibrate.py`)

**Generate a calibration pattern:**
```bash
# Chessboard (recommended for beginners)
python tools/calibration/camera_calibrate.py --generate --cols 9 --rows 6 --square 25

# With measured square size (recommended for accuracy):
# Measure 5 consecutive squares with calipers, divide by 5.
# Example: 5 squares = 124.8mm -> square_size = 24.96mm
python tools/calibration/camera_calibrate.py --generate --square 25 --square-size-mm 24.96
```

Print the generated SVG from `tools/calibration/patterns/`. Use "Actual size" (100% scale)
in the print dialog. Glue the printout to a flat rigid board.

**IMPORTANT — terminology:**
- **Board pattern size** = 250 × 175 mm (10×7 squares at 25mm each)
- **SVG page size** = 270 × 195 mm (includes 10mm margins on all sides)
- The SVG page is LARGER than the board pattern. Do not confuse the two.
- For best accuracy, measure the actual printed square size with calipers:
  `square_size_mm = measured_5_squares_mm / 5`

**Calibration output:** `tools/calibration/camera_calib.npz` + `.json` sidecar
Contains: `camera_index`, `backend`, `resolution`, `camera_matrix`, `dist_coeffs`,
`rmse`, `per_view_errors`, `cols`, `rows`, `square_mm`.

**The square size used for calibration is critical.** If you measure your printed
board and find the actual square size differs from the nominal 25mm, use
`--square-size-mm` to specify the measured value. This directly affects the
accuracy of all downstream measurements.

### 3. Marker Detection (`detect_markers.py`)

**Generate formal ArUco A4 calibration board:**
```bash
# Default: DICT_4X4_50, IDs 0-7, 20mm black area, 5mm quiet zone, A4 landscape
python tools/calibration/camera_calibrate.py --generate-aruco

# Custom parameters:
python tools/calibration/camera_calibrate.py --generate-aruco \
    --aruco-dictionary DICT_4X4_50 \
    --marker-ids "0,1,2,3,4,5,6,7" \
    --marker-size-mm 20 \
    --quiet-zone-mm 5 \
    --page-size A4 \
    --orientation landscape
```

Print the SVG from `tools/calibration/markers/` at 100% scale (no scaling).
The companion JSON file records all physical dimensions for traceability.

**IMPORTANT — marker size definition:**
- `marker_size_mm` = **black encoded area outer side length ONLY**
- The white quiet zone is ADDITIONAL (default +5mm on each side)
- Total marker tile = black_size + 2 × quiet_zone
- For PnP in `detect_markers.py`, use the black area size (not the total tile size)
- Do NOT confuse the black encoded area with the total printed marker size

**Existing markers_0-7.png**: This PNG is ONLY for algorithm preview and detection
testing. It has NO reliable physical print scale. Do not use it as a formal
calibration board. Always use `--generate-aruco` to produce a dimensioned SVG.

**Test existing markers PNG:**
```bash
python tools/calibration/camera_calibrate.py --test-markers-png tools/calibration/markers/markers_0-7.png

**Live detection:**
```bash
python tools/calibration/detect_markers.py
```

**Detect in saved image:**
```bash
python tools/calibration/detect_markers.py --image photo.jpg --output annotated.jpg
```

Controls:
- `SPACE` — capture measurement (records marker positions)
- `w` — toggle wizard mode (guided step-by-step)
- `r` — remove last measurement
- `s` — save measurements to CSV
- `+`/`-` — adjust marker size ±1mm
- `q` / `ESC` — quit

### 4. Integrated Calibration (`calibrate_with_camera.py`)

**Manual workflow (recommended):**
```bash
python tools/calibration/calibrate_with_camera.py --manual
```

This shows you the target servo angles for each calibration pose. You command the robot
to move to that pose (via `calib_ik_servo.ps1` in another terminal), then press SPACE
to capture the camera measurement.

**Batch from pre-saved images:**
```bash
python tools/calibration/calibrate_with_camera.py --images calib_frames/
```

**Generate marker config (customize marker positions):**
```bash
python tools/calibration/calibrate_with_camera.py --generate-config
# Edit tools/calibration/marker_config.json with actual offsets
```

## Calibration Pipeline (End-to-End)

```
┌─────────────────────┐
│  1. Camera Setup    │
│  camera_info.py     │  ← Verify camera works, choose resolution
└────────┬────────────┘
         ▼
┌─────────────────────┐
│  2. Camera Calib    │
│  camera_calibrate   │  ← Print chessboard, capture frames, get intrinsics
└────────┬────────────┘
         ▼
┌─────────────────────┐
│  3. Marker Setup    │
│  detect_markers     │  ← Generate ArUco markers, place on robot
│  --generate         │
└────────┬────────────┘
         ▼
┌─────────────────────────────────────────┐
│  4. IK Calibration Data Collection      │
│                                         │
│  Terminal A:                            │
│    calib_ik_servo.ps1                   │  ← Commands robot to each pose
│                                         │
│  Terminal B:                            │
│    calibrate_with_camera.py --manual    │  ← Captures wheel-center position
└────────┬────────────────────────────────┘
         ▼
┌─────────────────────┐
│  5. Model Fitting   │
│  fit_leg_ik_        │  ← Fits kinematics offsets from measured data
│  calibration.py     │
└─────────────────────┘
```

## Camera Setup Recommendations

For best results with leg IK calibration:

1. **Position**: Camera perpendicular to the robot's side, at leg height
2. **Distance**: Fill ~60-80% of the frame with the leg mechanism
3. **Lighting**: Diffuse, even lighting — avoid harsh shadows on markers
4. **Background**: Plain, contrasting background behind the robot
5. **Focus**: Manual focus if available — autofocus can shift between captures
6. **Stability**: Use a tripod — camera must not move during measurement

## Marker Placement Recommendations

```
        ┌──────────────────┐
        │  Robot Body      │
        │     [M0] ← origin│
        │                  │
 [M1]───┤  drive   drive ├───[M2]
   ↑    │  servo   servo  │    ↑
 left   │   axis    axis  │  right
 servo  │                  │  servo
        │      ╲  ╱       │
        │       ╲╱        │
        │       ╱╲        │
        │      ╱  ╲       │
        │     [M3] ← leg  │
        │      │   lower  │
        │      │          │
        │     [M4] ← wheel│
        │    center       │
        └──────────────────┘
```

- M0: Fixed reference on robot chassis (origin of measurement)
- M1, M2: On the drive servo horns (rotating parts)
- M3: On the lower leg link (below the 4-bar joint)
- M4: At the wheel center/axle

Marker size: 20-30mm works well at 0.5-1m distance with 1080p camera.

## File Structure

```
tools/calibration/
├── README.md
├── camera_utils.py              ← Shared: strict camera open, FOURCC, undistortion
├── geometry_utils.py            ← Unified solvePnP, HomographyData, coordinate audit
├── camera_info.py               ← Camera scan, strict validation, probe-modes
├── camera_calibrate.py          ← Camera intrinsic calibration, pattern generation
├── detect_markers.py            ← ArUco marker detection & pose measurement
├── calibrate_with_camera.py     ← Integrated IK calibration workflow
├── validate_measurement.py      ← Measurement accuracy validation (MAE/RMSE/repeatability)
├── camera_calib.npz             ← Calibration output (generated)
├── marker_config.json           ← Robot geometry → marker mapping
├── patterns/                    ← Generated calibration patterns (+ JSON metadata)
│   ├── chessboard_9x6_25mm.svg
│   ├── chessboard_9x6_25mm.json
│   └── chessboard_9x6_25mm.png  ← (preview only)
├── markers/                     ← Generated ArUco markers (+ JSON metadata)
│   ├── aruco_DICT_4X4_50_ids0-7_20mm_A4.svg
│   ├── aruco_DICT_4X4_50_ids0-7_20mm_A4.json
│   └── markers_0-7.png          ← (preview only - no physical scale)
├── samples/                     ← Camera validation samples + JSON reports
├── snapshots/                   ← Captured frames during calibration
└── tests/                       ← Automated tests
    ├── test_camera_mode_validation.py
    ├── test_coordinate_domains.py
    ├── test_homography_domain.py
    ├── test_aruco_generation.py
    └── test_pattern_geometry.py
```

## IK Fitting Gate (MANDATORY prerequisites)

**Do NOT run `fit_leg_ik_calibration.py` until ALL of the following are complete:**

1. Camera mode strict acceptance test passed (`camera_info.py --validate`)
2. Camera intrinsic calibration complete (`camera_calibrate.py`)
3. RMS and per-view reprojection errors reviewed (RMS < 1.0 px recommended)
4. Plane homography calibrated
5. Measurement validation complete using known distances:
   - 50mm, 100mm, 150mm, 200mm recommended
6. `validate_measurement.py` report generated and reviewed
7. No obvious position-dependent systematic error in the report
   (e.g., errors that grow with distance from image center)

**Running with a validation report:**
```bash
python tools/fit_leg_ik_calibration.py --input data/ik_calib.csv --validation-report validation_report.json
```

This prints MAE, RMSE, max error, and repeatability std before fitting,
so you can verify the measurement system quality.

**Reference thresholds** (guidance only, not enforced):
| Metric | Excellent | Usually acceptable | Investigate |
|--------|-----------|-------------------|-------------|
| MAE | <= 1.0 mm | 1.0–2.0 mm | > 3.0 mm |
| RMSE | <= 1.0 mm | 1.0–2.0 mm | > 3.0 mm |
| Repeatability (1σ) | <= 0.5 mm | 0.5–1.0 mm | > 1.0 mm |

If your measurement system exceeds these thresholds, investigate camera
calibration quality, marker placement, lighting, and camera stability
before proceeding to IK fitting.

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Camera not found | Try `--camera 1`, check USB connection, close other apps using camera |
| Board not detected | Improve lighting, check pattern is flat, try larger board size |
| High reprojection error (>1px) | Capture more frames at board edges, check focus, ensure board is clean |
| Markers not detected | Check lighting, ensure markers are flat, adjust `--marker-size` |
| Wrong marker size | Use calipers to measure actual printed marker size |
| Wheel position seems wrong | Verify marker_config.json offsets, check camera hasn't moved |
| OpenCV import error | `pip install opencv-python opencv-contrib-python` |

## Dependencies

- Python 3.8+
- opencv-python (cv2)
- opencv-contrib-python (ArUco module)
- numpy

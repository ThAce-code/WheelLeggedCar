# Leg IK Recalibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Recalibrate wheel-leg IK so height commands map to measured wheel-center height instead of the current uncalibrated model coordinate.

**Architecture:** Keep the current firmware safety limits while collecting direct `LIK` servo-angle samples. Fit calibration from measured wheel-center coordinates offline, then update `project/code/leg_config.c` only after the fitted model predicts the measured points better than the current config.

**Tech Stack:** PowerShell data collection over UART, CSV calibration files, Python offline fitting/checking, embedded C static configuration in `project/code/leg_config.c`.

## Global Constraints

- Hardware must be supported on a stand for all `LIK` calibration samples.
- Do not run balance or wheel drive during calibration.
- Keep servo commands inside `70..110` degrees for the first calibration pass.
- Record measured coordinates in millimeters using the same origin for every sample.
- Treat `STOP` 90-degree safe pose as a direct servo pose, not as proof that `LH,90` is calibrated.
- Do not re-enable `LH,120` until a fitted model proves that region is monotonic and safe.

---

### Task 1: Safe LIK Calibration Point Set

**Files:**
- Create: `data/ik_points_recalib_phase1.csv`
- Test: manual inspection with `Import-Csv data/ik_points_recalib_phase1.csv`

**Interfaces:**
- Consumes: existing `tools/calib_ik_servo.ps1 -PointFile`.
- Produces: a safe symmetric calibration point list with columns `a0,a1,a2,a3,label`.

- [x] **Step 1: Create the point file**

```csv
a0,a1,a2,a3,label
90,90,90,90,center_90
85,85,85,85,all_85
95,95,95,95,all_95
80,80,80,80,all_80
100,100,100,100,all_100
82,82,98,98,left_front_low_rear_high
98,98,82,82,left_front_high_rear_low
82,98,82,98,left_low_right_high_pair
98,82,98,82,left_high_right_low_pair
88,88,92,92,front_88_rear_92
92,92,88,88,front_92_rear_88
88,92,88,92,cross_a
92,88,92,88,cross_b
```

- [x] **Step 2: Verify the CSV loads**

Run:

```powershell
Import-Csv data\ik_points_recalib_phase1.csv | Format-Table
```

Expected: 13 rows, columns `a0`, `a1`, `a2`, `a3`, `label`.

---

### Task 2: Collect Direct Servo Calibration Measurements

**Files:**
- Read: `tools/calib_ik_servo.ps1`
- Create: `data/ik_recalib_phase1.csv`

**Interfaces:**
- Consumes: `data/ik_points_recalib_phase1.csv`.
- Produces: measured CSV with transmitted `LIK` angles and manually measured wheel-center coordinates.

- [ ] **Step 1: Put the car on a stand**

The wheels must be free, motors disabled, and the robot must not be balancing. Prepare a ruler or caliper. Use the same coordinate convention for every sample:

```text
x_mm: horizontal offset from the midpoint between the two active servo axes to the wheel axle center.
y_mm: vertical distance from the active servo-axis line to the wheel axle center.
positive y: wheel center lower than the servo-axis line.
```

- [ ] **Step 2: Run the calibration collector**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\calib_ik_servo.ps1 -Port COM6 -PointFile data\ik_points_recalib_phase1.csv -Out data\ik_recalib_phase1.csv
```

Expected: for each sample the script sends `STOP`, then one `LIK,a0,a1,a2,a3`, confirms telemetry, and prompts for `measured_x_mm` and `measured_y_mm`.

- [ ] **Step 3: Reject bad points while collecting**

For a point, enter a note containing `skip` if any of these happen:

```text
servo jitters continuously
mechanical contact or hard stop
telemetry does not match the LIK command
measurement origin changed
wheel center cannot be measured repeatably within 2 mm
```

- [ ] **Step 4: Verify row count**

Run:

```powershell
Import-Csv data\ik_recalib_phase1.csv | Measure-Object
```

Expected: at least 9 usable rows after removing rows whose `note` contains `skip`.

---

### Task 3: Offline Fit Script

**Files:**
- Create: `tools/fit_leg_ik_calibration.py`
- Test: `tools/test_fit_leg_ik_calibration.ps1`

**Interfaces:**
- Consumes: `data/ik_recalib_phase1.csv`.
- Produces: fitted candidate values for `l1_mm`, `l2_mm`, `l3_mm`, `l4_mm`, `l5_mm`, `x_offset_mm`, `y_offset_mm`, and servo neutral offsets.

- [x] **Step 1: Write the failing test**

Create `tools/test_fit_leg_ik_calibration.ps1` with:

```powershell
$ErrorActionPreference = "Stop"

$csv = Join-Path $env:TEMP ("ik-fit-sample-" + [Guid]::NewGuid().ToString() + ".csv")
@'
sample_id,label,cmd_a0_deg,cmd_a1_deg,cmd_a2_deg,cmd_a3_deg,servo0_output_deg,servo1_output_deg,servo2_output_deg,servo3_output_deg,ik_valid,leg_mode,leg_height_ref_mm,leg_height_rate_mm_s,ik_margin,drive_forward_limit_rpm,motion_state,fault_reason,drive_allowed,measured_x_mm,measured_y_mm,note
0,center_90,90,90,90,90,90,90,90,90,0,0,0,0,0,0,0,0,0,19,55,""
1,all_85,85,85,85,85,85,85,85,85,0,0,0,0,0,0,0,0,0,16,50,""
2,all_95,95,95,95,95,95,95,95,95,0,0,0,0,0,0,0,0,0,22,60,""
'@ | Set-Content $csv -Encoding UTF8

$output = python tools\fit_leg_ik_calibration.py --input $csv --max-iter 10 2>&1
if(0 -ne $LASTEXITCODE) {
    $output | Write-Host
    throw "fit script failed"
}
if(($output -join "`n") -notmatch "usable_rows=3") {
    throw "fit script must report usable row count"
}
if(($output -join "`n") -notmatch "rmse_y_mm=") {
    throw "fit script must report y RMSE"
}
if(($output -join "`n") -notmatch "candidate_leg_config") {
    throw "fit script must print candidate config block"
}
Remove-Item $csv -Force
Write-Host "fit calibration smoke test passed"
```

- [x] **Step 2: Run the test to verify it fails**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\test_fit_leg_ik_calibration.ps1
```

Expected: FAIL because `tools/fit_leg_ik_calibration.py` does not exist.

- [x] **Step 3: Implement the minimal fit script**

Create `tools/fit_leg_ik_calibration.py` that:

```text
reads CSV rows
ignores rows whose note contains skip
parses servo output angles and measured_x_mm/measured_y_mm
reports usable_rows
computes a baseline prediction using current leg_config.c values
prints rmse_x_mm and rmse_y_mm
prints candidate_leg_config as a C comment block
```

The first implementation may use the current config as the candidate. It must not update firmware files automatically.

- [x] **Step 4: Run the test to verify it passes**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\test_fit_leg_ik_calibration.ps1
```

Expected: PASS with `fit calibration smoke test passed`.

---

### Task 4: Apply Fitted Calibration Only After Evidence

**Files:**
- Modify: `project/code/leg_config.c`
- Modify: `tools/test_leg_transition_numeric.ps1`
- Modify: `tools/test_ik_height_control_static.ps1`
- Modify: `docs/leg-height-phase1-hardware-test.md`

**Interfaces:**
- Consumes: output from `tools/fit_leg_ik_calibration.py --input data\ik_recalib_phase1.csv`.
- Produces: firmware configuration with a measured monotonic height range.

- [ ] **Step 1: Run the fitter on real data**

Run:

```powershell
python tools\fit_leg_ik_calibration.py --input data\ik_recalib_phase1.csv
```

Expected: output contains `usable_rows`, `rmse_x_mm`, `rmse_y_mm`, and `candidate_leg_config`.

- [ ] **Step 2: Decide whether the fit is good enough**

Accept the candidate only if:

```text
usable_rows >= 9
rmse_y_mm <= 5.0
no accepted point has absolute y error > 10.0 mm
predicted height is monotonic over the selected Phase 1 command range
```

- [ ] **Step 3: Update `leg_config.c` with the accepted candidate**

Only copy fitted constants that improved the evidence. Keep speed/accel limits unchanged:

```c
.max_height_speed_mm_s = 10.0f,
.max_height_accel_mm_s2 = 20.0f,
```

- [ ] **Step 4: Update tests to assert the new range**

Update `tools/test_leg_transition_numeric.ps1` so it asserts the accepted `low_height_mm`, `high_height_mm`, `default_height_mm`, and `safe_support_height_mm`.

- [ ] **Step 5: Verify**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\test_leg_transition_numeric.ps1
powershell -ExecutionPolicy Bypass -File .\tools\test_ik_height_control_static.ps1
git diff --check
```

Expected: both tests pass and `git diff --check` exits 0.

---

### Task 5: Hardware Validation After Recalibration

**Files:**
- Create: `data/phase1_gate1_recalibrated.csv`
- Update: `docs/leg-height-phase1-hardware-test.md`

**Interfaces:**
- Consumes: rebuilt and flashed firmware with accepted calibration.
- Produces: Gate 1 evidence that `LH` commands now move monotonically.

- [ ] **Step 1: Rebuild and flash CM7_0**

Use IAR Embedded Workbench 9.40.1 and build `project/iar/cyt4bb7.eww` target `cyt4bb7_cm_7_0`.

- [ ] **Step 2: Run supported stationary height sweep**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\collect_balance_data.ps1 -Port COM6 -Duration 18 -Commands "0:STOP;2:LH,35;7:LH,55;12:LH,80;16:LH,35" -Out data\phase1_gate1_recalibrated.csv -Note phase1_gate1_recalibrated
```

- [ ] **Step 3: Measure the actual heights again**

Record actual wheel-center `y_mm` for `LH,35`, `LH,55`, and `LH,80`. Accept Gate 1 only if measured height is monotonic and there is no visible hard-stop contact.

- [ ] **Step 4: Update the hardware record**

Update `docs/leg-height-phase1-hardware-test.md` with build SHA, height points, measured safe-pose height, max pitch, IK margin min, IK faults, safety trips, and result.

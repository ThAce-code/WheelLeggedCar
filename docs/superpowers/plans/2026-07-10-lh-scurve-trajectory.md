# LH S-Curve Trajectory Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make Phase 1 `LH` motions jerk-limited and synchronized without changing `LIK` behaviour.

**Architecture:** `control_leg.c` owns height position, velocity, and acceleration. At each existing update, position error produces a bounded desired rate, rate error produces a bounded desired acceleration, and jerk limits the acceleration change before integration. The existing empirical mapping consumes the single height reference and emits all four servo angles from the same state.

**Tech Stack:** C99 firmware for CYT4BB/IAR, PowerShell static and numeric verification scripts.

## Global Constraints

- `LH` bounds remain exactly 45--65 mm.
- Maximum speed is exactly 10 mm/s; maximum acceleration is exactly 10 mm/s2; maximum jerk is exactly 80 mm/s3; position and rate gains are exactly 1.0 s-1 and 4.0 s-1.
- `LIK` remains a direct calibration/debug command and receives no trajectory-planner change.
- Do not change pins, scheduler periods, motor limits, balance laws, chassis laws, or the PWM 90 deg/s secondary limiter.
- Actual wheel height and servo position are not measured on this PWM-only platform; telemetry is an open-loop command estimate.

---

## File Structure

- `project/code/leg_config.h` owns the height-profile jerk and damping-gain types.
- `project/code/leg_config.c` owns its Phase 1 value.
- `project/code/control_leg.c` owns planner state, stopping logic, and empirical four-servo synchronization.
- `tools/test_leg_transition_numeric.ps1` simulates the planner and checks its dynamic bounds.
- `tools/test_ik_height_control_static.ps1` locks the source/configuration contract.
- `docs/leg-height-phase1-hardware-test.md` supplies the post-flash S-curve collection procedure.

### Task 1: Specify the jerk-limited planner contract in tests

**Files:**
- Modify: `tools/test_leg_transition_numeric.ps1`
- Modify: `tools/test_ik_height_control_static.ps1`

**Interfaces:**
- Consumes: `max_height_speed_mm_s`, `max_height_accel_mm_s2`, `max_height_jerk_mm_s3`, `height_position_kp_s`, and `height_rate_kp_s`.
- Produces: numeric expectations for position, velocity, acceleration, jerk, re-targeting, and the empirical four-servo pattern.

- [x] **Step 1: Write the failing numeric model and static assertions**

Replace the two-state PowerShell supervisor with a three-state function that takes `ReferenceMm`, `RateMmS`, `AccelMmS2`, `TargetMm`, `MaxSpeedMmS`, `MaxAccelMmS2`, `MaxJerkMmS3`, and `DtS`. Its result is `@($nextReferenceMm, $nextRateMmS, $nextAccelMmS2)`.

Use this decision order in the test model:

```powershell
$errorMm = $TargetMm - $ReferenceMm
$desiredRateMmS = [math]::Max(-$MaxSpeedMmS,
    [math]::Min($errorMm * $PositionKpS, $MaxSpeedMmS))
$desiredAccelMmS2 = [math]::Max(-$MaxAccelMmS2,
    [math]::Min($RateKpS * ($desiredRateMmS - $RateMmS), $MaxAccelMmS2))
$accelDeltaMmS2 = [math]::Max(-$MaxJerkMmS3 * $DtS,
    [math]::Min($desiredAccelMmS2 - $AccelMmS2, $MaxJerkMmS3 * $DtS))
$AccelMmS2 += $accelDeltaMmS2
$RateMmS = [math]::Max(-$MaxSpeedMmS,
    [math]::Min($RateMmS + ($AccelMmS2 * $DtS), $MaxSpeedMmS))
$nextReferenceMm = $ReferenceMm + ($RateMmS * $DtS)
```

Assert at every 10 ms sample: absolute rate <= 10.0, absolute acceleration <= 10.0, and absolute acceleration delta <= 0.8. Simulate `55 -> 65`, retarget to `45` at 2.0 s, then require final 45 mm with zero rate and zero acceleration. At every sample assert the empirical output equations:

```powershell
$halfDeltaDeg = 0.5 * (($referenceMm - 55.0) / 0.595)
Assert-Near $servoFl (90.0 + $halfDeltaDeg) 0.0001
Assert-Near $servoFr (90.0 - $halfDeltaDeg) 0.0001
Assert-Near $servoRl (90.0 - $halfDeltaDeg) 0.0001
Assert-Near $servoRr (90.0 + $halfDeltaDeg) 0.0001
```

Add source assertions requiring `max_height_jerk_mm_s3`, `control_leg_height_accel_mm_s2`, and a jerk-ramping helper in the firmware.

- [x] **Step 2: Run the numeric/static checks and verify RED**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\test_leg_transition_numeric.ps1
powershell -ExecutionPolicy Bypass -File .\tools\test_ik_height_control_static.ps1
```

Expected: both fail because neither the jerk profile field nor the acceleration state exists in the production source.

### Task 2: Add the bounded jerk state and planner implementation

**Files:**
- Modify: `project/code/leg_config.h`
- Modify: `project/code/leg_config.c`
- Modify: `project/code/control_leg.c`

**Interfaces:**
- Consumes: `leg_height_profile_struct.max_height_jerk_mm_s3`.
- Produces: a height reference whose rate and acceleration are bounded before `control_leg_apply_empirical_height()` is called.

- [x] **Step 1: Add the configuration field and Phase 1 setting**

Insert the field immediately after acceleration in `leg_height_profile_struct`:

```c
float max_height_jerk_mm_s3;
float height_position_kp_s;
float height_rate_kp_s;
```

Set its only profile initializer value immediately after `.max_height_accel_mm_s2`:

```c
.max_height_jerk_mm_s3 = 80.0f,
.height_position_kp_s = 1.0f,
.height_rate_kp_s = 4.0f,
```

- [x] **Step 2: Add state reset and helpers in `control_leg.c`**

Add this module state beside `control_leg_height_rate_mm_s`:

```c
static float control_leg_height_accel_mm_s2;
```

Reset it to `0.0f` in initialization, lock mode, fault entry, and fault recovery. Add a helper that clamps a value between `-limit` and `limit`, then use it to change acceleration by at most `max_height_jerk_mm_s3 * dt_s` per update.

- [x] **Step 3: Replace the trapezoidal rate code with the jerk-limited update**

In `LEG_MODE_HEIGHT`, replace the `sqrtf` / trapezoidal-rate block with these variables:

```c
float desired_rate_mm_s;
float desired_accel_mm_s2;
float accel_delta_mm_s2;
```

Calculate `desired_rate_mm_s = clamp(error_mm * height_position_kp_s, -max_speed, max_speed)`. Calculate `desired_accel_mm_s2 = clamp((desired_rate_mm_s - rate_mm_s) * height_rate_kp_s, -max_accel, max_accel)`. Ramp current acceleration by the jerk-limited delta; integrate and clamp rate to `+/- max_height_speed_mm_s`; then integrate the height reference.

Only when position error is at most 0.01 mm, rate is at most 0.05 mm/s, and acceleration is at most one jerk step may the implementation set reference exactly to target and clear rate and acceleration. Otherwise it must continue its jerk-limited braking/reversal. Check all planner state values with `control_leg_is_finite()`; on failure call `control_leg_enter_fault(LEG_FAULT_SERVO_LIMIT)` before generating servo angles.

Do not change `control_leg_apply_empirical_height()`, `control_leg_set_calib_angles()`, or `actuator_servo.c`.

- [x] **Step 4: Run the checks and verify GREEN**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\test_leg_transition_numeric.ps1
powershell -ExecutionPolicy Bypass -File .\tools\test_ik_height_control_static.ps1
powershell -ExecutionPolicy Bypass -File .\tools\test_fit_leg_ik_calibration.ps1
git diff --check -- project/code/leg_config.h project/code/leg_config.c project/code/control_leg.c tools/test_leg_transition_numeric.ps1 tools/test_ik_height_control_static.ps1
```

Expected: all three scripts report passed and diff check reports no whitespace errors.

### Task 3: Document and prepare the hardware A/B validation

**Files:**
- Modify: `docs/leg-height-phase1-hardware-test.md`

**Interfaces:**
- Consumes: telemetry already written by `tools/collect_balance_data.ps1`.
- Produces: a repeatable supported-bench validation record.

- [x] **Step 1: Add an S-curve test row**

Add a Gate 1 row with the exact command sequence and output file:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\collect_balance_data.ps1 -Port COM6 -Duration 18 -Commands "0:STOP;2:LH,55;4:LH,65;6:LH,45;12:LH,55" -Out data\phase1_gate1_lh_scurve_retarget.csv -Note phase1_gate1_lh_scurve_retarget
```

Record maximum absolute pitch/rate, servo target-versus-output mismatch, any fault state, and observed jerk. A stationary 55 mm shake is listed as a hardware investigation, not as an S-curve failure.

- [x] **Step 2: Final verification and commit**

Run all Task 2 checks again, inspect the staged file list, stage only the Task 1--3 source/test/doc files, then commit:

```powershell
git add -- project/code/leg_config.h project/code/leg_config.c project/code/control_leg.c tools/test_leg_transition_numeric.ps1 tools/test_ik_height_control_static.ps1 docs/leg-height-phase1-hardware-test.md
git commit -m "Smooth LH transitions with jerk limiting"
```

Do not stage experiment CSV files unless the user explicitly asks to preserve a particular result.

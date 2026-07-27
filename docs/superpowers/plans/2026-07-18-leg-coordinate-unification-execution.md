# Leg Coordinate Unification Adjusted Execution Plan

> **For agentic workers:** REQUIRED SUB-SKILLS: use `superpowers:test-driven-development` while implementing each task. Each task must finish with focused tests, the affected regression suite, self-review, and a commit.

**Goal:** Unify every public wheel-center position on calibrated `BODY_WHEEL` coordinates while preserving current `LH/LHF` servo and balance behavior.

**Architecture:** Physical/model transforms, hull validation, IK/FK, and logical-command FK live in `leg_kinematics`. `control_leg` publishes physical command estimates from actuator `output_deg`; legacy stance values remain numerically identical but are explicitly named as nonphysical units. Telemetry remains exactly 55 floats.

**Reason for this adjusted plan:** The approved six-task plan removed configuration and diagnostic fields before their consumers migrated. This five-task order keeps every task boundary buildable by moving `LIKREF/LXY` into the physical-foundation task and moving telemetry together with physical pose publication.

## Global Constraints

- Work only in `D:\smartcar\WheelLeggedCar_cyt4bb7_v1\.worktree\balance-fast-mode-spec` on `codex/balance-fast-mode-spec`.
- Public frame: `BODY_WHEEL`, origin at the chassis cross-circle, millimetres, `+X` forward, `+Y` downward.
- Reference point: `(-20.766667f, 47.356667f) mm`; it is not the origin.
- `LXY` is absolute physical millimetres; `LIKREF` uses the reference point; `LXY,0,55` is rejected.
- Use the fitted counter-clockwise eight-vertex hull and `2.0f mm` inset, not a rectangle.
- Runtime pose is an open-loop command estimate, never actual or measured position.
- Preserve `LH/LHF` numeric range `30/55/80`, S7 duration `500U`, 300 Hz PWM, balance gains, chassis limits, and all safety gates.
- Servo 0 pulse trim is `-11.111111f us`; servo 1-3 trims are `0.0f us`.
- Left pose source is measured calibration; right pose source is mirror assumption until independent hardware validation.
- Do not implement race stance, wheel shift, speed mapping, LQR changes, motor-limit changes, or virtual-lean changes.
- Do not touch existing untracked `data/*.csv`, `fast_tune_kit.zip`, or unrelated `.superpowers` files.

---

### Task 1: Physical kinematics foundation and physical command path

**Files:**
- Create: `tools/test_leg_physical_ik_static.ps1`
- Modify: `project/code/leg_config.h`
- Modify: `project/code/leg_config.c`
- Modify: `project/code/leg_kinematics.h`
- Modify: `project/code/leg_kinematics.c`
- Modify: `project/code/control_leg.c`
- Modify: `tools/test_leg_ik_zero_calibration_static.ps1`

**Requirements:**

1. Start with a failing test derived only from `c07702a:tools/test_leg_physical_ik_static.ps1`. It must verify the exact reference, transform, eight hull vertices, 2 mm inset, physical/model round trip, reference inside, `(0,55)` outside, private model solver, and absence of additive offsets/cross-band validation.
2. Port from `c07702a` only the physical calibration configuration and physical kinematics changes. Do not port camera, perception, marker tooling, or unrelated runtime code.
3. Kinematics configuration values are exactly:

```c
.x_min_mm = 10.0f,
.x_max_mm = 50.0f,
.y_min_mm = 25.0f,
.y_max_mm = 100.0f,
.physical_reference_x_mm = -20.766667f,
.physical_reference_y_mm = 47.356667f,
.alpha_reference_deg = 170.536799f,
.beta_reference_deg = -4.081158f,
.model_reference_x_mm = 22.830129f,
.model_reference_y_mm = 46.929213f,
.model_to_physical_scale = 0.955219899f,
.model_to_physical_m00 = -0.996313812f,
.model_to_physical_m01 = 0.085783378f,
.model_to_physical_m10 = 0.085783378f,
.model_to_physical_m11 = 0.996313812f,
.physical_workspace_inset_mm = 2.0f,
```

Hull vertices, counter-clockwise:

```c
{-40.620f,47.370f}, {-30.910f,39.630f}, {-20.380f,32.170f}, {-15.040f,47.600f},
{-22.030f,88.490f}, {-31.420f,74.120f}, {-37.940f,59.340f}, {-39.580f,53.010f}
```

4. Calibrated servo directions are FL/RL `-1.0f`, FR/RR `+1.0f`; neutral remains 90 degrees.
5. Public APIs:

```c
uint8 leg_kinematics_target_valid(float x_mm, float y_mm);
uint8 leg_kinematics_forward_command(uint8 right_side,
                                     float servo_a_command_deg,
                                     float servo_b_command_deg,
                                     float *x_mm,
                                     float *y_mm);
```

6. `leg_kinematics_solve` consumes physical coordinates, validates the inset hull, transforms to model coordinates, then calls private `leg_kinematics_solve_model`. `leg_kinematics_forward` returns physical coordinates.
7. `leg_kinematics_forward_command` reconstructs geometric alpha/beta from the calibrated reference and per-servo `neutral_deg`, `ik_offset_deg`, and `direction`, calls physical FK, and leaves caller outputs unchanged on failure.
8. In the same task, migrate `control_leg_set_ik_reference` and `LEG_MODE_IK_REFERENCE` to `physical_reference_x_mm/physical_reference_y_mm`. Remove `control_leg_ik_validation_point_valid`; `control_leg_set_xy` delegates to `leg_kinematics_target_valid` and rejects without clamping. This requirement keeps the intermediate commit buildable after old config fields are removed.
9. Update the existing zero-calibration test so reference commands still map to four logical 90-degree commands but use physical `P0`, and assert `LXY,0,55` rejection.

**Tests:**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_leg_physical_ik_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_leg_ik_zero_calibration_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_leg_transition_numeric.ps1
powershell -ExecutionPolicy Bypass -File tools/test_ik_height_control_static.ps1
```

**Commit:** `Unify leg kinematics on physical coordinates`

---

### Task 2: Calibrated servo pulse trim

**Files:**
- Modify: `project/code/app_config.h`
- Modify: `project/code/actuator_servo.c`
- Modify: `tools/test_servo_pwm_resolution_static.ps1`

**Requirements:**

1. Add a failing test for exact trim constants, the trimmed hardware-write path, clamping, and numeric duties.
2. Add:

```c
#define APP_SERVO0_PULSE_TRIM_US (-11.111111f)
#define APP_SERVO1_PULSE_TRIM_US (0.0f)
#define APP_SERVO2_PULSE_TRIM_US (0.0f)
#define APP_SERVO3_PULSE_TRIM_US (0.0f)
```

3. Apply channel trim only inside `actuator_servo_write` through a private `actuator_servo_angle_to_duty_with_trim`. Clamp trimmed pulse to 500-2500 us.
4. Keep public `actuator_servo_angle_to_duty` untrimmed so 90 degrees remains duty 9000 at `PWM_DUTY_MAX=20000`, 300 Hz. Servo-0 hardware duty at logical 90 is 8933.

**Tests:**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_servo_pwm_resolution_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_servo_300hz_integration_static.ps1
```

**Commit:** `Apply calibrated servo pulse trim`

---

### Task 3: Physical command-pose diagnostics and fixed 55-float telemetry

**Files:**
- Create: `tools/test_leg_coordinate_contract_static.ps1`
- Modify: `project/code/app_types.h`
- Modify: `project/code/control_leg.c`
- Modify: `project/code/telemetry.c`
- Modify: `tools/collect_balance_data.ps1`
- Modify: `tools/test_collect_balance_data.ps1`
- Modify: `tools/calib_ik_servo.ps1`
- Modify: `tools/test_ik_height_control_static.ps1`
- Modify: `tools/test_balance_drive_v2_static.ps1`
- Modify: `tools/test_servo_300hz_integration_static.ps1`
- Modify: `tools/test_timing_noise_regressions.ps1`

**Requirements:**

1. Start with failing contract/collector tests. Reject fabricated `x=0`, legacy scalar copied to physical Y, and `actual_height_mm`.
2. Add:

```c
typedef enum
{
    LEG_POSE_SOURCE_NONE = 0,
    LEG_POSE_SOURCE_MEASURED_CALIBRATION = 1,
    LEG_POSE_SOURCE_MIRROR_ASSUMPTION = 2
}leg_pose_source_enum;

typedef struct
{
    float x_mm;
    float y_mm;
    leg_pose_source_enum source;
    uint8 valid;
}leg_pose_command_estimate_struct;
```

3. `leg_diag_struct` contains left/right `*_command_pose_body_mm`; delete flat physical X/Y fields and `actual_height_mm`. For this task only, the remaining legacy scalar fields keep their current names so every boundary compiles; Task 4 renames them atomically across all consumers.
4. In `control_leg_publish_diag`, derive both poses from actuator `output_deg` using `leg_kinematics_forward_command`. Left source is measured calibration; right is mirror assumption. On failure preserve last X/Y and set `valid=0`; never substitute legacy values.
5. Remove all mode-specific manual X/Y assignments.
6. Keep `float vofa_data[55]`. Use mapping:

```text
13 target_height_mm (temporary legacy name)
14 height_ref_mm (temporary legacy name)
15 height_norm (temporary legacy name)
16 pose_status_flags
17 output_enable
33 left command X mm
34 left command Y mm
35 right command X mm
36 right command Y mm
37 IK margin
38-54 unchanged
```

7. Status bits: bit 0 IK valid, bit 1 left pose valid, bit 2 right pose valid, bit 3 left measured source, bit 4 right mirror source.
8. Collectors decode all five status bits, four physical coordinates, and both source strings. Keep 55 wire floats and below-50-percent line-utilization tests. Update calibration CSV to record coordinates, validity, and source.

**Tests:**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_leg_coordinate_contract_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_leg_physical_ik_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_collect_balance_data.ps1
powershell -ExecutionPolicy Bypass -File tools/test_ik_height_control_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_servo_300hz_integration_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_timing_noise_regressions.ps1
```

**Commit:** `Report physical leg command poses`

---

### Task 4: Atomic legacy stance semantic rename

**Files:**
- Modify: `project/code/leg_config.h`
- Modify: `project/code/leg_config.c`
- Modify: `project/code/control_leg.h`
- Modify: `project/code/control_leg.c`
- Modify: `project/code/host_command.c`
- Modify: `project/code/app_config.h`
- Modify: `project/code/app_types.h`
- Modify: `project/code/control_balance.c`
- Modify: `project/code/control_chassis.c`
- Modify: `project/code/telemetry.c`
- Modify: `tools/collect_balance_data.ps1`
- Modify: `tools/test_collect_balance_data.ps1`
- Modify: `tools/calib_ik_servo.ps1`
- Modify: `tools/test_leg_coordinate_contract_static.ps1`
- Modify: `tools/test_leg_transition_numeric.ps1`
- Modify: `tools/test_ik_height_control_static.ps1`
- Modify: `tools/test_leg_first_height_frame_static.ps1`

**Requirements:**

1. Apply one atomic rename across producers and every consumer. Preserve all numeric values and host wire commands `LH/LHF`.
2. Exact core mappings:

```text
leg_height_profile_struct -> leg_stance_profile_struct
leg_config_get_height_profile -> leg_config_get_stance_profile
low/high/default_height_mm -> legacy_low/high/default_units
max_height_speed/accel/jerk_mm_* -> legacy_max_rate/accel/jerk_units_*
height_position/rate_kp_s -> legacy_position/rate_kp_s
height_settle_error_mm -> legacy_settle_error_units
height_settle_ms -> legacy_settle_ms
fast_height_transition_ms -> fast_stance_transition_ms
safe_support_height_mm -> legacy_safe_support_units
height_min/height_max -> legacy_body_min_units/legacy_body_max_units
LEG_MODE_HEIGHT/FAST_HEIGHT/DIRECT_STEP -> LEG_MODE_LEGACY_STANCE/FAST_LEGACY_STANCE/DIRECT_LEGACY_STANCE
control_leg_set_height -> control_leg_set_legacy_stance
control_leg_set_fast_height -> control_leg_set_fast_legacy_stance
control_leg_set_direct_step_height -> control_leg_set_direct_legacy_stance
target_height_mm -> legacy_stance_target_units
height_ref_mm -> legacy_stance_ref_units
height_rate_mm_s -> legacy_stance_rate_units_s
height_norm and leg_height_norm -> legacy_stance_norm
```

3. Rename matching internal variables/helpers/constants. The empirical formula remains numerically identical but uses `stance_units`, `CONTROL_LEG_LEGACY_CENTER_UNITS=55.0f`, and `CONTROL_LEG_LEGACY_UNITS_PER_DELTA_DEG=0.595f`. Remove the false `Y_real ~=` comment.
4. Balance/chassis still interpolate the same gains and limits from the same numeric 0-1 legacy norm. Do not substitute physical Y.
5. Telemetry wire indices stay exactly as Task 3; only CSV/runtime names change. Remove all runtime names that claim the legacy scalar is millimetres or physical height.
6. Update numeric/static tests and retain exact `30/55/80`, rate 20, acceleration 20, jerk 80, settle error 1, and S7 500 ms assertions.

**Tests:**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_leg_coordinate_contract_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_leg_transition_numeric.ps1
powershell -ExecutionPolicy Bypass -File tools/test_leg_first_height_frame_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_ik_height_control_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_collect_balance_data.ps1
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_servo_300hz_integration_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_timing_noise_regressions.ps1
```

**Commit:** `Clarify legacy leg stance units`

---

### Task 5: Documentation, full regression, and hardware handoff

**Files:**
- Modify: `docs/leg-ik-zero-calibration-hardware-test.md`
- Modify: `docs/leg-height-phase1-hardware-test.md`
- Modify: `tools/test_leg_coordinate_contract_static.ps1`

**Requirements:**

1. Document `BODY_WHEEL`, exact P0, absolute `LXY`, command-estimate/open-loop meaning, left measured/right mirror provenance, inset hull, and `LXY,0,55` rejection.
2. Replace claims that `LH/LHF` values are millimetres with legacy stance units while retaining the existing hardware gate order.
3. Add a motor-disabled table for target X/Y, measured X/Y, component errors, pose-status flags, and servo outputs at `LIKREF` plus two inset points on each side.
4. Right leg requires independent measurements before dynamic wheel shift. Do not claim IAR or hardware validation without evidence.
5. Final stale-name scan must find no runtime `actual_height_mm`, `target_height_mm`, `height_ref_mm`, `height_norm`, old reference fields, cross-band validator, or fabricated physical coordinate assignments.
6. Run all affected tests, `git diff --check`, and attempt the CM7_0 IAR build only if IAR is available. Report software, IAR, and hardware status separately.

**Full tests:**

```powershell
$tests = @(
  'tools/test_leg_physical_ik_static.ps1',
  'tools/test_leg_coordinate_contract_static.ps1',
  'tools/test_servo_pwm_resolution_static.ps1',
  'tools/test_servo_300hz_integration_static.ps1',
  'tools/test_leg_ik_zero_calibration_static.ps1',
  'tools/test_leg_transition_numeric.ps1',
  'tools/test_leg_first_height_frame_static.ps1',
  'tools/test_ik_height_control_static.ps1',
  'tools/test_collect_balance_data.ps1',
  'tools/test_balance_drive_v2_static.ps1',
  'tools/test_timing_noise_regressions.ps1'
)
foreach($test in $tests) {
  powershell -ExecutionPolicy Bypass -File $test
  if($LASTEXITCODE -ne 0) { throw "$test failed" }
}
git diff --check
```

**Commit:** `Document physical leg coordinate gates`

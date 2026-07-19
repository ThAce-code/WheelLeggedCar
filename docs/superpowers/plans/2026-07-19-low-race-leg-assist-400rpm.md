# Low-race Automatic Leg Assist Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a fail-closed, automatically accelerating and braking low-race leg-assist path that can be validated at 250, 300, 350, and finally 400 RPM without weakening manual `LXY`, existing emergency stop, or motor-layer limits.

**Architecture:** `control_chassis` remains the owner of speed intent and calls a new pure `control_race_assist` supervisor for the normalized assist request and staged limits. `control_leg` gains a drive-allowed Cartesian race mode that tracks the supervisor's signed request through one shared S7 scalar, solves both legs on the persisted branches at every 10 ms sample, and freezes rather than commanding a recovery pose on a race-path fault. `control_balance` consumes a runtime output cap; host commands and appended VOFA fields make every gate observable and reversible.

**Tech Stack:** Embedded C99 for CYT4BB/Traveo II, IAR Embedded Workbench 9.40.1, PowerShell static/numeric harnesses, host GCC for numeric tests, UART0 VOFA+ JustFloat telemetry.

**Approved references:** `docs/superpowers/specs/2026-07-19-low-race-leg-assist-400rpm-design.md` and `docs/superpowers/specs/2026-07-19-low-race-leg-assist-formulas.md`.

## Global Constraints

- Work in `D:\smartcar\WheelLeggedCar_cyt4bb7_v1\.worktree\balance-fast-mode-spec` on `codex/balance-fast-mode-spec`.
- Keep `BODY_WHEEL=(-18.83,25.08) mm` as the race zero pose; `+X` is forward and `+Y` is down.
- Initial path endpoints are `(-20.83,27.08) mm` for acceleration and `(-16.83,27.08) mm` for normal braking.
- Keep the global IK minimum margin at `0.02`, preserve branch identity, and validate both legs and all servo limits at every trajectory sample.
- Keep manual `LXY` stopped and bench-oriented. Never route moving assist through its host-command handler.
- Keep `APP_MOTOR_RPM_TARGET_LIMIT=1000.0f` unchanged.
- Keep speed-loop `Ki=0` and `APP_BALANCE_FAST_SPEED_FF_GAIN=0` in this feature.
- Start with `APP_RACE_ASSIST_MAX_VALIDATED_LEVEL=1U`; levels 2--4 remain compiled but rejected until the preceding hardware gate is accepted and this single constant is deliberately raised.
- Runtime assist gains default to zero after reset. The first powered test must set conservative gains explicitly through the host command.
- Use 90 deg/s as the first race-assist servo command-rate limit.
- Preserve the current 55 VOFA indices. Append race fields at indices 55--71, use a 72-float frame, and move telemetry from 10 ms to 20 ms so 460800-baud wire occupancy remains below 50%.
- Emergency stop, host timeout, IMU fault, and application safety fault retain priority over every leg-assist state.
- Do not claim measured leg position: actuator and forward-kinematic pose fields remain command estimates.
- Do not add cyclic leg pumping, high-speed turn tuning, or automatic persistence of hardware validation level.

## File and responsibility map

- Create `project/code/control_race_assist.h`: pure supervisor types and public API.
- Create `project/code/control_race_assist.c`: state machine, level profiles, signed assist formula, pitch gates, staged speed/output caps, and turn derating.
- Modify `project/code/app_config.h`: fail-closed race constants, tuning bounds, telemetry count/rate, and level gate.
- Modify `project/code/app_types.h`: chassis, leg, and balance diagnostic fields; supervisor-specific types stay in `control_race_assist.h`.
- Modify `project/code/control_leg.h` and `project/code/control_leg.c`: drive-allowed Cartesian race mode, scalar S7 path, path preflight, branch/margin publication, and race fault hold.
- Modify `project/code/control_chassis.h` and `project/code/control_chassis.c`: runtime enable/level/gain APIs, supervisor inputs, command-to-leg handoff, and race-aware motion policy.
- Modify `project/code/control_balance.c`: runtime output cap and race motion-state acceptance.
- Modify `project/code/host_command.c`: `BRA,<level>` and `BRG,<Ka>,<Ke>,<hold>` commands plus reset semantics.
- Modify `project/code/telemetry.c`: append fields 55--71 without moving fields 0--54.
- Modify `project/iar/project_config/cyt4bb7_cm_7_0.ewp`: compile `control_race_assist.c` on CM7_0.
- Create `tools/test_race_assist_numeric.ps1`: supervisor state, bounds, sign, hysteresis, and level tests against production C.
- Create `tools/test_low_race_leg_assist_static.ps1`: whole-feature wiring and fail-closed contract checks.
- Modify `tools/test_leg_transition_numeric.ps1`: production IK sweep of both signed race paths.
- Modify `tools/collect_balance_data.ps1`, `tools/calib_ik_servo.ps1`, and their contract tests: parse the 72-float frame.
- Modify telemetry-count assertions in `tools/test_balance_drive_v2_static.ps1`, `tools/test_ik_height_control_static.ps1`, `tools/test_servo_300hz_integration_static.ps1`, `tools/test_timing_noise_regressions.ps1`, and `tools/test_leg_coordinate_contract_static.ps1`.
- Create `docs/low-race-leg-assist-hardware-test.md`: supported-chassis, level-1 ground, A/B, and promotion gates.

---

### Task 1: Add the pure race-assist supervisor and fail-closed configuration

**Files:**
- Create: `project/code/control_race_assist.h`
- Create: `project/code/control_race_assist.c`
- Modify: `project/code/app_config.h:11-160`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_0.ewp:1235-1260`
- Create: `tools/test_race_assist_numeric.ps1`

**Interfaces:**
- Consumes: scalar speed, IMU, leg-readiness, and runtime-command snapshots supplied later by `control_chassis`.
- Produces: `control_race_assist_init()`, `control_race_assist_set_level(uint8)`, `control_race_assist_set_gains(float,float,float)`, `control_race_assist_report_leg_path_fault()`, `control_race_assist_update(const race_assist_input_struct *)`, `control_race_assist_get_output()`, and `control_race_assist_get_level_profile(uint8, race_assist_level_profile_struct *)`.

- [ ] **Step 1: Write the failing numeric supervisor test**

Create a host-GCC harness in `tools/test_race_assist_numeric.ps1`. Follow the repository's temporary-directory pattern and compile copied production `control_race_assist.c/.h` with a minimal host `zf_common_headfile.h`. The C assertions must cover these exact cases:

```c
static int expect_near(float actual, float expected, float tolerance)
{
    return (fabsf(actual - expected) <= tolerance) ? 0 : 1;
}

static int check_level_profiles(void)
{
    static const float target_limit[4] = {250.0f, 300.0f, 350.0f, 400.0f};
    static const float output_limit[4] = {300.0f, 350.0f, 410.0f, 460.0f};
    uint8 level;

    for(level = 1U; level <= 4U; level++)
    {
        race_assist_level_profile_struct profile;
        if((APP_TRUE != control_race_assist_get_level_profile(level, &profile)) ||
           expect_near(profile.forward_limit_rpm, target_limit[level - 1U], 0.001f) ||
           expect_near(profile.balance_limit_rpm, output_limit[level - 1U], 0.001f) ||
           expect_near(profile.dx_mm, 2.0f, 0.001f) ||
           expect_near(profile.dy_mm, 2.0f, 0.001f))
        {
            return 1;
        }
    }
    return 0;
}
```

Call `control_race_assist_set_gains(0.005f, 0.005f, 0.15f)` before the positive and negative request cases. Add update sequences that require:

```text
level 0 -> DISABLED, u_request 0, runtime balance cap 300
level 2 rejected while APP_RACE_ASSIST_MAX_VALIDATED_LEVEL is 1
level 1 + low pose + fast mode + healthy feedback below 230 -> LOW_RACE
measured 240 with healthy gates -> ARMED
positive requested acceleration at 250 -> BOOST and u_request > 0
steady high target -> CRUISE_HOLD and u_request >= 0
falling target or negative speed error -> BRAKE and u_request < 0
measured speed below 200 during disable -> RECENTER then DISABLED after u_actual reaches 0
pitch above abort threshold -> FAULT_HOLD without a negative recovery command
NaN/Inf input -> FAULT_HOLD and finite zero/frozen output
turn_scale 1 at 300, 0.5 at 350, and 0 at 400 RPM
```

- [ ] **Step 2: Run the supervisor test to verify it fails**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_race_assist_numeric.ps1
```

Expected: FAIL because `project/code/control_race_assist.c` and its public API do not exist.

- [ ] **Step 3: Add exact configuration constants**

Add these constants to `project/code/app_config.h`:

```c
#define APP_RACE_ASSIST_MAX_VALIDATED_LEVEL       (1U)
#define APP_RACE_ASSIST_ARM_START_RPM             (230.0f)
#define APP_RACE_ASSIST_FULL_RPM                  (250.0f)
#define APP_RACE_ASSIST_RECENTER_RPM              (200.0f)
#define APP_RACE_ASSIST_SPEED_ERROR_DEADBAND_RPM  (5.0f)
#define APP_RACE_ASSIST_ACCEL_DEADBAND_RPM_S      (5.0f)
#define APP_RACE_ASSIST_PITCH_ARM_LIMIT_DEG       (10.0f)
#define APP_RACE_ASSIST_PITCH_ABORT_LIMIT_DEG     (15.0f)
#define APP_RACE_ASSIST_RATE_ARM_LIMIT_DPS        (100.0f)
#define APP_RACE_ASSIST_RATE_ABORT_LIMIT_DPS      (180.0f)
#define APP_RACE_ASSIST_PITCH_OFFSET_LIMIT_DEG    (7.0f)
#define APP_RACE_ASSIST_GAIN_A_DEFAULT            (0.0f)
#define APP_RACE_ASSIST_GAIN_E_DEFAULT            (0.0f)
#define APP_RACE_ASSIST_HOLD_DEFAULT              (0.0f)
#define APP_RACE_ASSIST_GAIN_A_MAX                (0.02f)
#define APP_RACE_ASSIST_GAIN_E_MAX                (0.05f)
#define APP_RACE_ASSIST_HOLD_MAX                  (0.50f)
#define APP_RACE_ASSIST_ZERO_X_MM                 (-18.83f)
#define APP_RACE_ASSIST_ZERO_Y_MM                 (25.08f)
#define APP_RACE_ASSIST_INITIAL_DX_MM             (2.0f)
#define APP_RACE_ASSIST_INITIAL_DY_MM             (2.0f)
#define APP_RACE_ASSIST_POSE_TOLERANCE_MM         (0.75f)
#define APP_RACE_ASSIST_REQUEST_DEADBAND          (0.02f)
#define APP_RACE_ASSIST_PATH_SAMPLE_COUNT         (21U)
#define APP_RACE_ASSIST_SERVO_SPEED_DPS           (90.0f)
```

Do not change `APP_MOTOR_RPM_TARGET_LIMIT`, speed `Ki`, or speed feedforward.

- [ ] **Step 4: Define supervisor contracts**

Create `project/code/control_race_assist.h` with these public types and names:

```c
#ifndef _control_race_assist_h_
#define _control_race_assist_h_

#include "app_types.h"

typedef enum
{
    RACE_ASSIST_DISABLED = 0,
    RACE_ASSIST_LOW_RACE,
    RACE_ASSIST_ARMED,
    RACE_ASSIST_BOOST,
    RACE_ASSIST_CRUISE_HOLD,
    RACE_ASSIST_BRAKE,
    RACE_ASSIST_RECENTER,
    RACE_ASSIST_FAULT_HOLD
}race_assist_state_enum;

typedef enum
{
    RACE_ASSIST_FAULT_NONE = 0,
    RACE_ASSIST_FAULT_INPUT_INVALID,
    RACE_ASSIST_FAULT_LEG_NOT_READY,
    RACE_ASSIST_FAULT_LEG_PATH,
    RACE_ASSIST_FAULT_PITCH_LIMIT
}race_assist_fault_reason_enum;

typedef struct
{
    float forward_limit_rpm;
    float balance_limit_rpm;
    float dx_mm;
    float dy_mm;
}race_assist_level_profile_struct;

typedef struct
{
    float target_rpm;
    float ramped_rpm;
    float measured_rpm;
    float pitch_deg;
    float pitch_rate_dps;
    float leg_u_actual;
    float dt_s;
    uint8 fast_enable;
    uint8 feedback_healthy;
    uint8 low_pose_ready;
    uint8 leg_path_fault;
}race_assist_input_struct;

typedef struct
{
    race_assist_state_enum state;
    race_assist_fault_reason_enum fault_reason;
    float requested_accel_rpm_s;
    float speed_error_rpm;
    float speed_blend;
    float u_request;
    float forward_limit_rpm;
    float balance_limit_rpm;
    float turn_scale;
    float pitch_offset_limit_deg;
    float dx_mm;
    float dy_mm;
    uint8 enable;
    uint8 level;
    uint8 leg_command_enable;
}race_assist_output_struct;

void control_race_assist_init(void);
uint8 control_race_assist_set_level(uint8 level);
uint8 control_race_assist_set_gains(float accel_gain, float error_gain, float hold_bias);
void control_race_assist_report_leg_path_fault(void);
void control_race_assist_update(const race_assist_input_struct *input);
const race_assist_output_struct *control_race_assist_get_output(void);
uint8 control_race_assist_get_level_profile(uint8 level,
                                            race_assist_level_profile_struct *profile);

#endif
```

The implementation must use a `switch(level)` with these exact profiles:

```c
case 1U: 250.0f, 300.0f, 2.0f, 2.0f;
case 2U: 300.0f, 350.0f, 2.0f, 2.0f;
case 3U: 350.0f, 410.0f, 2.0f, 2.0f;
case 4U: 400.0f, 460.0f, 2.0f, 2.0f;
```

Travel stays at 2 mm for all compiled levels. Increasing travel is a separate hardware-reviewed constant change.

- [ ] **Step 5: Implement the pure update equations and state transitions**

In `control_race_assist.c`, calculate:

```c
requested_accel_rpm_s = (input->ramped_rpm - previous_ramped_rpm) / input->dt_s;
speed_error_rpm = input->ramped_rpm - input->measured_rpm;
t = clamp01((fabsf(input->measured_rpm) - APP_RACE_ASSIST_ARM_START_RPM) /
            (APP_RACE_ASSIST_FULL_RPM - APP_RACE_ASSIST_ARM_START_RPM));
speed_blend = t * t * (3.0f - (2.0f * t));
u_raw = (accel_gain * requested_accel_rpm_s) +
        (error_gain * speed_error_rpm) +
        (hold_bias * speed_blend);
u_request = clamp(u_raw, -1.0f, 1.0f);
turn_scale = clamp01((400.0f - fabsf(input->measured_rpm)) / 100.0f);
```

Apply acceleration and error dead bands before multiplying. When a pitch abort, invalid input, or leg path fault occurs, retain `input->leg_u_actual` as the requested hold value and enter `RACE_ASSIST_FAULT_HOLD`; do not command a forward recovery impulse. On a runtime disable, cap forward intent at the recenter threshold, wait until measured speed is below 200 RPM, command `u=0`, then enter `DISABLED` only after `abs(leg_u_actual)<=0.02`.

- [ ] **Step 6: Add the source to the CM7_0 IAR project**

Add this file entry adjacent to `control_chassis.c` in `project/iar/project_config/cyt4bb7_cm_7_0.ewp`:

```xml
<file>
    <name>$PROJ_DIR$\..\..\code\control_race_assist.c</name>
</file>
```

- [ ] **Step 7: Run the numeric test and existing static baseline**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_race_assist_numeric.ps1
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
```

Expected: both exit 0; the new harness prints `race assist numeric checks passed`.

- [ ] **Step 8: Commit the supervisor foundation**

```powershell
git add project/code/app_config.h project/code/control_race_assist.h project/code/control_race_assist.c project/iar/project_config/cyt4bb7_cm_7_0.ewp tools/test_race_assist_numeric.ps1
git commit -m "Add fail-closed race assist supervisor"
```

---

### Task 2: Add a drive-allowed Cartesian race trajectory to the leg controller

**Files:**
- Modify: `project/code/control_leg.h:10-38`
- Modify: `project/code/control_leg.c:110-230,430-1005,1008-1140`
- Modify: `project/code/app_types.h:250-320`
- Modify: `tools/test_leg_transition_numeric.ps1:1000-1290`
- Create: `tools/test_low_race_leg_assist_static.ps1`

**Interfaces:**
- Consumes: signed `u_request`, `dx_mm`, `dy_mm`, and time from `control_chassis`.
- Produces: `control_leg_set_race_assist_request(float,float,float,uint32)`, `control_leg_disable_race_assist(uint32)`, `LEG_MODE_RACE_ASSIST`, `LEG_MOTION_RACE_ASSIST`, `LEG_MOTION_RACE_FAULT_HOLD`, and leg diagnostics for actual assist, side margins, and branch flags.

- [ ] **Step 1: Extend the production IK numeric harness with both signed paths**

In the generated C harness inside `tools/test_leg_transition_numeric.ps1`, add this path gate before `return 0`:

```c
static int check_race_path(float sign)
{
    const leg_kinematics_config_struct *cfg = leg_config_get_kinematics();
    leg_ik_result_struct left_reference = {0};
    leg_ik_result_struct right_reference = {0};
    leg_ik_result_struct left_previous = {0};
    leg_ik_result_struct right_previous = {0};
    int step;

    if((0 == cfg) ||
       (APP_TRUE != leg_kinematics_solve(APP_FALSE,
                                         cfg->physical_reference_x_mm,
                                         cfg->physical_reference_y_mm,
                                         0, &left_reference)) ||
       (APP_TRUE != leg_kinematics_solve(APP_TRUE,
                                         cfg->physical_reference_x_mm,
                                         cfg->physical_reference_y_mm,
                                         0, &right_reference)) ||
       (APP_TRUE != leg_kinematics_solve(APP_FALSE, -18.83f, 25.08f,
                                         0, &left_previous)) ||
       (APP_TRUE != leg_kinematics_solve(APP_TRUE, -18.83f, 25.08f,
                                         0, &right_previous)))
    {
        return 1;
    }

    for(step = 0; step <= 20; step++)
    {
        const float u = sign * (float)step / 20.0f;
        const float x_mm = -18.83f - (2.0f * u);
        const float y_mm = 25.08f + (2.0f * fabsf(u));
        leg_ik_result_struct left;
        leg_ik_result_struct right;
        float servo_deg[LEG_SERVO_COUNT];

        if((APP_TRUE != leg_kinematics_solve(APP_FALSE, x_mm, y_mm,
                                             &left_previous, &left)) ||
           (APP_TRUE != leg_kinematics_solve(APP_TRUE, x_mm, y_mm,
                                             &right_previous, &right)) ||
           (left.alpha_branch != left_previous.alpha_branch) ||
           (left.beta_branch != left_previous.beta_branch) ||
           (right.alpha_branch != right_previous.alpha_branch) ||
           (right.beta_branch != right_previous.beta_branch) ||
           (0.02f > left.singularity_margin) ||
           (0.02f > right.singularity_margin) ||
           (APP_TRUE != leg_kinematics_map_target_pose(&left_reference,
                                                        &right_reference,
                                                        &left, &right,
                                                        servo_deg)))
        {
            printf("race path rejected sign %.0f step %d target %.3f %.3f\n",
                   sign, step, x_mm, y_mm);
            return 1;
        }
        left_previous = left;
        right_previous = right;
    }
    return 0;
}
```

Call both `check_race_path(1.0f)` and `check_race_path(-1.0f)`.

- [ ] **Step 2: Run the path test to verify the leg feature contract fails**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_leg_transition_numeric.ps1
powershell -ExecutionPolicy Bypass -File tools/test_low_race_leg_assist_static.ps1
```

Expected: the production IK sweep passes, while the new static script fails because race mode, API, and diagnostics do not exist.

- [ ] **Step 3: Add explicit leg modes and diagnostics**

Append `LEG_MODE_RACE_ASSIST` to `leg_mode_enum`. Extend `leg_motion_state_enum` without renumbering existing values:

```c
LEG_MOTION_RACE_ASSIST,
LEG_MOTION_RACE_FAULT_HOLD
```

Append these fields to `leg_diag_struct`:

```c
float race_assist_request;
float race_assist_actual;
float race_target_x_mm;
float race_target_y_mm;
float left_ik_margin;
float right_ik_margin;
uint8 ik_branch_flags;
uint8 race_path_valid;
```

Use bits 0--3 of `ik_branch_flags` for left alpha, left beta, right alpha, and right beta; a set bit means `LEG_IK_BRANCH_MINUS`.

- [ ] **Step 4: Add the public race-leg API**

Add to `control_leg.h`:

```c
uint8 control_leg_set_race_assist_request(float u_request,
                                          float dx_mm,
                                          float dy_mm,
                                          uint32 now_ms);
void control_leg_disable_race_assist(uint32 now_ms);
```

The setter must reject non-finite inputs, `abs(u_request)>1`, non-positive travel, missing IK reference, a generic leg fault, or a full signed path that cannot be traversed on both persisted branches.

`control_leg_disable_race_assist()` requests `u=0` and keeps `LEG_MODE_RACE_ASSIST` active at the low-race zero after the scalar trajectory and actuator output settle. A later explicit `LH`, `LHF`, `LIKREF`, `LXY`, or lock command may change the mode. The disable path must never substitute the all-90-degree reference or legacy 55-unit pose.

- [ ] **Step 5: Implement a scalar S7 trajectory in Cartesian space**

Add `LEG_TRAJECTORY_RACE_ASSIST` and state variables:

```c
static float control_leg_race_u_start;
static float control_leg_race_u_target;
static float control_leg_race_u_pending;
static float control_leg_race_u_actual;
static float control_leg_race_dx_mm;
static float control_leg_race_dy_mm;
static uint32 control_leg_race_start_ms;
static uint32 control_leg_race_duration_ms;
static uint8 control_leg_race_pending_valid;
static uint8 control_leg_race_path_valid;
```

For every leg update:

```c
progress = clamp((float)(now_ms - control_leg_race_start_ms) /
                 (float)control_leg_race_duration_ms, 0.0f, 1.0f);
blend = control_leg_s7_blend(progress);
control_leg_race_u_actual = control_leg_race_u_start +
    ((control_leg_race_u_target - control_leg_race_u_start) * blend);
x_mm = APP_RACE_ASSIST_ZERO_X_MM -
       (control_leg_race_dx_mm * control_leg_race_u_actual);
y_mm = APP_RACE_ASSIST_ZERO_Y_MM +
       (control_leg_race_dy_mm * control_leg_absf(control_leg_race_u_actual));
```

Solve both legs from `control_leg_ik_previous_left/right`, map all four commands, publish them, then update the persisted results. When the requested sign differs from the current sign, first target `u=0`; store the signed final request in `control_leg_race_u_pending` and start the second S7 only after the zero segment completes. This removes the derivative corner from `abs(u)`.

While a segment is running, a request change smaller than `APP_RACE_ASSIST_REQUEST_DEADBAND` is ignored. A larger change replaces only `control_leg_race_u_pending`; it does not restart the active S7 every 5 ms. When the segment completes, start one new segment toward the latest pending request.

Calculate duration from the maximum mapped endpoint command change using the existing S7 factor and the 90 deg/s race limit:

```c
duration_s = (2.1875f * max_delta_deg) / APP_RACE_ASSIST_SERVO_SPEED_DPS;
if(duration_s < 0.10f)
{
    duration_s = 0.10f;
}
```

Do not use `control_leg_pose_start_if_changed()` for this mode because joint-space interpolation would not follow the preflighted X/Y curve.

- [ ] **Step 6: Implement race-specific fault hold**

Add a private helper that retains the current planned commands:

```c
static void control_leg_enter_race_fault_hold(leg_fault_reason_enum reason)
{
    control_leg_motion_state = LEG_MOTION_RACE_FAULT_HOLD;
    control_leg_fault_reason = reason;
    control_leg_trajectory_mode = LEG_TRAJECTORY_NONE;
    control_leg_race_u_target = control_leg_race_u_actual;
    control_leg_race_u_pending = control_leg_race_u_actual;
    control_leg_race_pending_valid = APP_FALSE;
    control_leg_diag.ik_error_count++;
}
```

The top-level fault branch must retain `control_leg_servo_cmd` for this state. Generic `LEG_MOTION_FAULT` behavior remains unchanged. Publish `drive_allowed=APP_TRUE` only while application safety is healthy, servo output remains enabled, and the race fault hold contains finite commands; publish a zero forward limit so the chassis ramp reduces speed.

- [ ] **Step 7: Make race motion drive-allowed without weakening manual transitions**

In `control_leg_publish_diag()`, report `LEG_MOTION_RACE_ASSIST` as drive-allowed with a forward limit supplied later by the chassis. Keep `LEG_MOTION_TRANSITION` at the existing 30 RPM limit. Do not change `control_leg_set_xy()` or `LEG_MODE_IK_VALIDATE` semantics.

- [ ] **Step 8: Run leg numeric and regression tests**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_leg_transition_numeric.ps1
powershell -ExecutionPolicy Bypass -File tools/test_leg_ik_zero_calibration_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_leg_physical_ik_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_servo_motion_numeric.ps1
powershell -ExecutionPolicy Bypass -File tools/test_low_race_leg_assist_static.ps1
```

Expected: all exit 0; both race paths retain branches and margins; manual `LXY` remains stopped-test-only.

- [ ] **Step 9: Commit the leg trajectory**

```powershell
git add project/code/app_types.h project/code/control_leg.h project/code/control_leg.c tools/test_leg_transition_numeric.ps1 tools/test_low_race_leg_assist_static.ps1
git commit -m "Add drive-safe race leg trajectory"
```

---

### Task 3: Integrate automatic assist and staged forward limits in the chassis

**Files:**
- Modify: `project/code/control_chassis.h:10-20`
- Modify: `project/code/control_chassis.c:105-440,462-590`
- Modify: `project/code/app_types.h:136-182`
- Modify: `tools/test_race_assist_numeric.ps1`
- Modify: `tools/test_low_race_leg_assist_static.ps1`

**Interfaces:**
- Consumes: supervisor API from Task 1 and race-leg API/diagnostics from Task 2.
- Produces: `control_chassis_set_race_assist_level()`, `control_chassis_set_race_assist_gains()`, race fields in `chassis_output_struct`, and same-cycle request handoff to `control_leg`.

- [ ] **Step 1: Add failing chassis-wiring assertions**

Require the static test to find:

```powershell
Require-Pattern $chassis 'control_race_assist_update' 'Chassis must run the race supervisor.'
Require-Pattern $chassis 'control_leg_set_race_assist_request' 'Chassis must hand the signed request to the leg controller.'
Require-Pattern $chassis 'LEG_MOTION_RACE_ASSIST' 'Race leg motion must bypass only the generic transition limit.'
Require-Pattern $chassis 'APP_RACE_ASSIST_PITCH_OFFSET_LIMIT_DEG' 'Race mode must use the initial 7 degree virtual-pitch cap.'
Reject-Pattern $host 'control_leg_set_xy[\s\S]{0,400}control_leg_set_race_assist_request' 'Manual LXY must not become the moving assist path.'
```

- [ ] **Step 2: Run the static test to verify it fails**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_low_race_leg_assist_static.ps1
```

Expected: FAIL on missing chassis integration.

- [ ] **Step 3: Add public runtime setters**

Add to `control_chassis.h`:

```c
uint8 control_chassis_set_race_assist_level(uint8 level, uint32 now_ms);
uint8 control_chassis_set_race_assist_gains(float accel_gain,
                                            float error_gain,
                                            float hold_bias);
```

The level setter delegates to `control_race_assist_set_level()`. Level zero requests safe disable/recenter. Levels above `APP_RACE_ASSIST_MAX_VALIDATED_LEVEL` return `APP_FALSE` and do not change the active level.

- [ ] **Step 4: Feed the supervisor from live snapshots**

After the ramped forward command and average wheel speed are available, fill:

```c
race_input.target_rpm = control_chassis_cmd.target_forward_rpm;
race_input.ramped_rpm = control_chassis_cmd.actual_forward_rpm;
race_input.measured_rpm = avg_wheel_speed_rpm;
race_input.pitch_deg = imu->pitch;
race_input.pitch_rate_dps = imu->pitch_rate_dps;
race_input.leg_u_actual = leg->race_assist_actual;
race_input.dt_s = dt_s;
race_input.fast_enable = control_chassis_cmd.fast_enable;
race_input.feedback_healthy =
    ((APP_TRUE == imu->healthy) &&
     (APP_TRUE == wheel_feedback->online) &&
     (APP_TRUE == wheel_feedback->left_online) &&
     (APP_TRUE == wheel_feedback->right_online) &&
     (APP_CHASSIS_IMU_MAX_AGE_MS >= imu_age_ms) &&
     (APP_CHASSIS_WHEEL_MAX_AGE_MS >= wheel_age_ms)) ? APP_TRUE : APP_FALSE;
race_input.low_pose_ready = low_pose_ready;
race_input.leg_path_fault =
    ((LEG_MOTION_RACE_FAULT_HOLD == leg->motion_state) ||
     (APP_FALSE == leg->race_path_valid)) ? APP_TRUE : APP_FALSE;
```

`low_pose_ready` requires settled actuator output, valid left/right command estimates, and both estimates within `APP_RACE_ASSIST_POSE_TOLERANCE_MM` of `(-18.83,25.08)`.

- [ ] **Step 5: Hand the supervisor request to the leg controller**

When `race_output.leg_command_enable` is true:

```c
if(APP_TRUE != control_leg_set_race_assist_request(race_output.u_request,
                                                    race_output.dx_mm,
                                                    race_output.dy_mm,
                                                    now_ms))
{
    control_race_assist_report_leg_path_fault();
}
```

Use the Task 1 `control_race_assist_report_leg_path_fault()` API rather than faking an input on the next cycle. When assist reaches `DISABLED`, call `control_leg_disable_race_assist(now_ms)` only after the leg has returned to `u=0`.

- [ ] **Step 6: Update motion policy without broadening ordinary transitions**

Extend `control_chassis_resolve_leg_motion_policy()` with explicit cases:

```c
else if((LEG_MOTION_RACE_ASSIST == leg->motion_state) ||
        (LEG_MOTION_RACE_FAULT_HOLD == leg->motion_state))
{
    *forward_limit_rpm = (APP_TRUE == race_output.enable) ?
                         race_output.forward_limit_rpm :
                         APP_CHASSIS_FORWARD_RPM_LIMIT;
    *fast_forward_limit_rpm = (APP_TRUE == race_output.enable) ?
                              race_output.forward_limit_rpm :
                              APP_CHASSIS_FAST_FORWARD_RPM_LIMIT;
    *effective_fast_enable = fast_requested;
}
```

The existing `LEG_MOTION_TRANSITION` branch remains exactly 30 RPM and disables fast blend.

- [ ] **Step 7: Publish chassis race diagnostics**

Append to `chassis_output_struct`:

```c
float wheel_speed_measured_rpm;
float speed_error_rpm;
float requested_accel_rpm_s;
float race_u_request;
float race_balance_limit_rpm;
float race_turn_scale;
uint8 race_assist_enable;
uint8 race_assist_level;
uint8 race_assist_state;
uint8 race_assist_fault_reason;
```

Initialize and clear every field on init, stop, unhealthy feedback, and command timeout.

- [ ] **Step 8: Apply the assisted pitch cap and turn scale**

When race assist is enabled, use:

```c
speed_pitch_limit_deg = control_chassis_lerp(
    APP_CHASSIS_SPEED_PITCH_LIMIT_DEG,
    APP_RACE_ASSIST_PITCH_OFFSET_LIMIT_DEG,
    race_output.speed_blend);
target_turn_dps *= race_output.turn_scale;
```

Do not raise the cap toward the ordinary fast 12 deg value while assisted.

- [ ] **Step 9: Run chassis and supervisor tests**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_race_assist_numeric.ps1
powershell -ExecutionPolicy Bypass -File tools/test_low_race_leg_assist_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_tune_drive_loops_static.ps1
```

Expected: all exit 0; ordinary fast mode remains capped by the legacy stance profile when assist is disabled.

- [ ] **Step 10: Commit chassis integration**

```powershell
git add project/code/app_types.h project/code/control_chassis.h project/code/control_chassis.c project/code/control_race_assist.h project/code/control_race_assist.c tools/test_race_assist_numeric.ps1 tools/test_low_race_leg_assist_static.ps1
git commit -m "Integrate automatic race leg assist"
```

---

### Task 4: Add runtime balance authority and race-aware fault deceleration

**Files:**
- Modify: `project/code/app_config.h:140-150`
- Modify: `project/code/control_balance.c:188-430`
- Modify: `project/code/app_types.h:184-233`
- Modify: `tools/test_low_race_leg_assist_static.ps1`
- Modify: `tools/test_balance_drive_v2_static.ps1`

**Interfaces:**
- Consumes: `chassis_output_struct.race_balance_limit_rpm`, race motion states, and existing safety snapshots.
- Produces: a dynamic balance clamp and `balance_diag_struct.balance_output_limit_rpm`.

- [ ] **Step 1: Add failing dynamic-cap assertions**

```powershell
Require-Pattern $config 'APP_BALANCE_RPM_LIMIT\s+\(460\.0f\)' 'Hard balance ceiling must leave 400 RPM correction reserve.'
Require-Pattern $balance 'race_balance_limit_rpm' 'Balance must consume the runtime race cap.'
Require-Pattern $balance 'LEG_MOTION_RACE_FAULT_HOLD' 'Race fault hold must preserve balance while the target ramps down.'
Require-Pattern $types 'balance_output_limit_rpm' 'Active balance cap must be observable.'
```

- [ ] **Step 2: Run the test to verify it fails**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_low_race_leg_assist_static.ps1
```

Expected: FAIL on the missing runtime clamp.

- [ ] **Step 3: Raise only the hard ceiling and retain a 300 RPM default runtime cap**

Change:

```c
#define APP_BALANCE_RPM_LIMIT (460.0f)
#define APP_BALANCE_DEFAULT_RUNTIME_RPM_LIMIT (300.0f)
```

The chassis output must publish 300 RPM whenever assist is disabled, unhealthy, stopped, or uninitialized.

- [ ] **Step 4: Clamp by the smaller valid limit**

Before the final balance clamp:

```c
balance_output_limit_rpm = chassis->race_balance_limit_rpm;
if((APP_FALSE == control_balance_is_finite(balance_output_limit_rpm)) ||
   (0.0f >= balance_output_limit_rpm))
{
    balance_output_limit_rpm = APP_BALANCE_DEFAULT_RUNTIME_RPM_LIMIT;
}
if(APP_BALANCE_RPM_LIMIT < balance_output_limit_rpm)
{
    balance_output_limit_rpm = APP_BALANCE_RPM_LIMIT;
}
balance_rpm = control_balance_limit_abs(balance_rpm,
                                        balance_output_limit_rpm);
```

Publish the chosen value in `balance_diag_struct.balance_output_limit_rpm`.

- [ ] **Step 5: Allow only explicit race states to keep balance active**

Where the balance loop accepts leg motion, include `LEG_MOTION_RACE_ASSIST` and `LEG_MOTION_RACE_FAULT_HOLD`. Do not accept generic `LEG_MOTION_FAULT`, direct legacy step, disabled servo output, stale IMU, or stale wheel feedback.

For both race motion states, set the gain-scheduling stance normalization to `0.0f` before interpolating pitch, rate, wheel-speed, and base-pitch gains. The physical low-race pose was entered through IK, so the stale legacy 55-unit scalar must not select mid-height gains.

- [ ] **Step 6: Run balance and safety regressions**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_low_race_leg_assist_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v1_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_timing_noise_regressions.ps1
```

Expected: all exit 0; non-assisted modes still use a 300 RPM runtime cap.

- [ ] **Step 7: Commit runtime balance authority**

```powershell
git add project/code/app_config.h project/code/app_types.h project/code/control_balance.c tools/test_low_race_leg_assist_static.ps1 tools/test_balance_drive_v2_static.ps1
git commit -m "Add staged race balance authority"
```

---

### Task 5: Add fail-closed host commands and reset semantics

**Files:**
- Modify: `project/code/host_command.c:310-530`
- Modify: `project/code/control_chassis.h`
- Modify: `project/code/control_chassis.c`
- Modify: `tools/test_low_race_leg_assist_static.ps1`

**Interfaces:**
- Consumes: runtime setter APIs from Task 3.
- Produces: `BRA,<level>` and `BRG,<Ka>,<Ke>,<hold>` over the existing UART0 downlink.

- [ ] **Step 1: Add failing command assertions**

```powershell
Require-Pattern $host "'B' == line\[0\].*'R' == line\[1\].*'A' == line\[2\]" 'BRA command parser missing.'
Require-Pattern $host 'control_chassis_set_race_assist_level' 'BRA must call the level setter.'
Require-Pattern $host "'B' == line\[0\].*'R' == line\[1\].*'G' == line\[2\]" 'BRG command parser missing.'
Require-Pattern $host 'control_chassis_set_race_assist_gains' 'BRG must call the bounded gain setter.'
Require-Pattern $host 'host_command_match_stop\(line\)[\s\S]*control_chassis_set_race_assist_level\(0U' 'STOP must disarm race assist.'
```

- [ ] **Step 2: Run the static test to verify it fails**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_low_race_leg_assist_static.ps1
```

Expected: FAIL on missing host commands.

- [ ] **Step 3: Parse `BRA,<level>` before the generic `B,<mode>` handler**

Use the existing single-number parser and accept only exact integer values 0--4:

```c
if(('B' == line[0]) && ('R' == line[1]) && ('A' == line[2]) &&
   (',' == line[3]) &&
   (APP_TRUE == host_command_parse_number(&line[4], &value)) &&
   (value == (float)((uint8)value)) &&
   (4.0f >= value))
{
    if(APP_TRUE == control_chassis_set_race_assist_level((uint8)value, now_ms))
    {
        actuator_motor_record_command_error(APP_FALSE);
        return;
    }
}
```

Level 0 disables. Level 1 enables the only initially validated level. Levels 2--4 are syntactically valid but rejected by the compiled maximum-level gate.

- [ ] **Step 4: Parse bounded `BRG,<Ka>,<Ke>,<hold>`**

Use `host_command_parse_three_numbers()` and delegate all finite/range checks to the setter:

```c
if(('B' == line[0]) && ('R' == line[1]) && ('G' == line[2]) &&
   (',' == line[3]) &&
   (APP_TRUE == host_command_parse_three_numbers(&line[4], &kp, &ki,
                                                   &drive_turn_kp)))
{
    if(APP_TRUE == control_chassis_set_race_assist_gains(kp, ki,
                                                          drive_turn_kp))
    {
        actuator_motor_record_command_error(APP_FALSE);
        return;
    }
}
```

The setter accepts only `0<=Ka<=0.02`, `0<=Ke<=0.05`, and `0<=hold<=0.50`. Defaults remain zero after reset.

- [ ] **Step 5: Preserve emergency-stop priority**

`STOP`, `B,0`, `B,1`, and `B,2` must call the race level setter with zero before their existing chassis/balance actions. `STOP` still disables balance and motors immediately; it does not wait for a leg trajectory. `B,3` arms fast balance but does not silently enable race assist.

- [ ] **Step 6: Run host and existing command tests**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_low_race_leg_assist_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_ik_height_control_static.ps1
```

Expected: all exit 0; `LXY` still stops chassis and balance before changing pose.

- [ ] **Step 7: Commit the command contract**

```powershell
git add project/code/host_command.c project/code/control_chassis.h project/code/control_chassis.c tools/test_low_race_leg_assist_static.ps1
git commit -m "Add race assist runtime commands"
```

---

### Task 6: Append race telemetry within a bounded UART budget

**Files:**
- Modify: `project/code/app_config.h:27`
- Modify: `project/code/telemetry.c:1-170`
- Modify: `tools/collect_balance_data.ps1`
- Modify: `tools/calib_ik_servo.ps1`
- Modify: `tools/test_collect_balance_data.ps1`
- Modify: `tools/test_balance_drive_v2_static.ps1`
- Modify: `tools/test_ik_height_control_static.ps1`
- Modify: `tools/test_servo_300hz_integration_static.ps1`
- Modify: `tools/test_timing_noise_regressions.ps1`
- Modify: `tools/test_leg_coordinate_contract_static.ps1`
- Modify: `tools/test_low_race_leg_assist_static.ps1`

**Interfaces:**
- Consumes: chassis, balance, and leg diagnostics from Tasks 2--4.
- Produces: a 72-float VOFA frame and matching CSV/calibration parsers.

- [ ] **Step 1: Change tests first to require the exact appended frame**

Update count assertions from 55 to 72 and retain every existing index assertion from 0 through 54. Add assertions for indices 55--71. In the timing tests calculate:

```powershell
$frameBytes = (72 * 4) + 4
$txMs = $frameBytes * 10.0 * 1000.0 / 460800.0
$occupancy = $txMs / 20.0
Assert-True ($occupancy -lt 0.5) "72-float 20 ms frame must stay below 50% UART occupancy"
```

- [ ] **Step 2: Run telemetry tests to verify they fail**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_collect_balance_data.ps1
powershell -ExecutionPolicy Bypass -File tools/test_timing_noise_regressions.ps1
powershell -ExecutionPolicy Bypass -File tools/test_leg_coordinate_contract_static.ps1
```

Expected: FAIL because firmware and collectors still use 55 floats at 10 ms.

- [ ] **Step 3: Change only the telemetry period**

Set:

```c
#define APP_TELEMETRY_PERIOD_MS (20U)
```

Keep chassis at 5 ms, balance at 5 ms, leg control at 10 ms, motor at 1 ms, and servo PWM at 300 Hz.

- [ ] **Step 4: Append fields 55--71 without moving existing channels**

Use `static float vofa_data[72];` and assign:

```c
vofa_data[55] = (float)chassis->race_assist_enable;
vofa_data[56] = (float)chassis->race_assist_level;
vofa_data[57] = (float)chassis->race_assist_state;
vofa_data[58] = (float)chassis->race_assist_fault_reason;
vofa_data[59] = chassis->race_u_request;
vofa_data[60] = leg->race_assist_actual;
vofa_data[61] = chassis->requested_accel_rpm_s;
vofa_data[62] = chassis->forward_target_rpm;
vofa_data[63] = chassis->forward_actual_rpm;
vofa_data[64] = chassis->wheel_speed_measured_rpm;
vofa_data[65] = chassis->speed_error_rpm;
vofa_data[66] = balance->pitch_setpoint_deg;
vofa_data[67] = balance->balance_output_limit_rpm;
vofa_data[68] = chassis->race_turn_scale;
vofa_data[69] = leg->left_ik_margin;
vofa_data[70] = leg->right_ik_margin;
vofa_data[71] = (float)leg->ik_branch_flags;
```

Fields already published at indices 3, 5, 6, 31, 33--39 are not duplicated.

- [ ] **Step 5: Update both host parsers and CSV schema**

Set `$FloatCount = 72` and map the same indices in `tools/collect_balance_data.ps1` and `tools/calib_ik_servo.ps1`. Append CSV column names in index order:

```text
race_assist_enable,race_assist_level,race_assist_state,race_assist_fault_reason,
race_u_request,race_u_actual,requested_accel_rpm_s,forward_target_rpm,
forward_ramped_rpm,wheel_speed_measured_rpm,speed_error_rpm,pitch_setpoint_deg,
balance_output_limit_rpm,race_turn_scale,left_ik_margin,right_ik_margin,
ik_branch_flags
```

- [ ] **Step 6: Run all telemetry tests**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_collect_balance_data.ps1
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_ik_height_control_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_servo_300hz_integration_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_timing_noise_regressions.ps1
powershell -ExecutionPolicy Bypass -File tools/test_leg_coordinate_contract_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_low_race_leg_assist_static.ps1
```

Expected: all exit 0; the calculated transmission time is about 6.34 ms per 20 ms frame and occupancy is about 31.7%.

- [ ] **Step 7: Commit telemetry and tooling together**

```powershell
git add project/code/app_config.h project/code/telemetry.c tools/collect_balance_data.ps1 tools/calib_ik_servo.ps1 tools/test_collect_balance_data.ps1 tools/test_balance_drive_v2_static.ps1 tools/test_ik_height_control_static.ps1 tools/test_servo_300hz_integration_static.ps1 tools/test_timing_noise_regressions.ps1 tools/test_leg_coordinate_contract_static.ps1 tools/test_low_race_leg_assist_static.ps1
git commit -m "Add bounded race assist telemetry"
```

---

### Task 7: Document hardware gates and run the complete software verification

**Files:**
- Create: `docs/low-race-leg-assist-hardware-test.md`
- Modify: `tools/test_low_race_leg_assist_static.ps1`

**Interfaces:**
- Consumes: complete feature and telemetry contract.
- Produces: exact supported-chassis commands, level-1 A/B procedure, promotion evidence, and final software verification record.

- [ ] **Step 1: Add failing documentation assertions**

Require these exact searchable strings:

```powershell
Require-Pattern $hardwareDoc 'BRA,0' 'Procedure must start from assist disabled.'
Require-Pattern $hardwareDoc 'LXY,-18\.83,25\.08' 'Procedure must establish the approved low-race zero.'
Require-Pattern $hardwareDoc 'BRG,0\.005,0\.005,0\.15' 'Level-1 test must set explicit conservative gains.'
Require-Pattern $hardwareDoc 'BRA,1' 'Level-1 assist must be enabled explicitly.'
Require-Pattern $hardwareDoc '250 RPM' 'Level-1 ground gate missing.'
Require-Pattern $hardwareDoc '390--410 RPM' 'Final 400 RPM acceptance band missing.'
Require-Pattern $hardwareDoc 'three seconds' 'Final dwell requirement missing.'
Require-Pattern $hardwareDoc 'command estimate' 'Open-loop pose meaning must be explicit.'
```

- [ ] **Step 2: Run the documentation test to verify it fails**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_low_race_leg_assist_static.ps1
```

Expected: FAIL because the hardware procedure does not exist.

- [ ] **Step 3: Write the supported-chassis path gate**

Document wheel-power-disconnected commands one at a time:

```text
STOP
LIKREF
LXY,-18.83,25.08
BRA,0
BRG,0.005,0.005,0.15
BRA,1
```

Require observed target progression through zero, `(-20.83,27.08)`, zero, and `(-16.83,27.08)` using a test-only supervisor sequence, with no branch-bit change, IK margin below 0.02, mechanical contact, or leg fault. State explicitly that `servo_settled` and X/Y are command estimates.

- [ ] **Step 4: Write the level-1 ground A/B gate**

Require a clear straight lane, catch tether/support operator, charged battery, and immediate physical power cutoff. Record two otherwise identical runs:

```text
A: BRG,0,0,0; BRA,1; B,3; C,250,0; C,0,0
B: BRG,0.005,0.005,0.15; BRA,1; B,3; C,250,0; C,0,0
```

For both runs record 230--250 RPM time, peak absolute pitch, peak pitch rate, output-cap occupancy, `u_request/u_actual`, command X/Y, branch flags, margins, and stopping time. Pass only if assist direction is correct, no safety/fault field changes, output is not persistently saturated, and run B improves acceleration or controlled braking without a larger unsafe pitch excursion.

- [ ] **Step 5: Write promotion and final acceptance gates**

State that changing `APP_RACE_ASSIST_MAX_VALIDATED_LEVEL` from 1 to 2, 2 to 3, or 3 to 4 requires the previous level's recorded acceptance and a fresh build. Do not expand `dx_mm/dy_mm` in the same run as a speed-level increase. Final 400 RPM acceptance is measured `390--410 RPM` for at least three seconds, fresh feedback, no persistent balance saturation, no IK/leg fault, and controllable straight-line behavior.

- [ ] **Step 6: Run the complete software suite**

Run:

```powershell
$tests = @(
  'tools/test_race_assist_numeric.ps1',
  'tools/test_low_race_leg_assist_static.ps1',
  'tools/test_leg_transition_numeric.ps1',
  'tools/test_leg_ik_zero_calibration_static.ps1',
  'tools/test_leg_physical_ik_static.ps1',
  'tools/test_servo_motion_numeric.ps1',
  'tools/test_servo_300hz_integration_static.ps1',
  'tools/test_balance_drive_v1_static.ps1',
  'tools/test_balance_drive_v2_static.ps1',
  'tools/test_tune_drive_loops_static.ps1',
  'tools/test_collect_balance_data.ps1',
  'tools/test_timing_noise_regressions.ps1',
  'tools/test_ik_height_control_static.ps1',
  'tools/test_leg_coordinate_contract_static.ps1'
)
foreach($test in $tests)
{
    powershell -ExecutionPolicy Bypass -File $test
    if(0 -ne $LASTEXITCODE)
    {
        throw "Failed: $test"
    }
}
```

Expected: every script exits 0.

- [ ] **Step 7: Build the affected target core**

Open `project/iar/cyt4bb7.eww` in IAR 9.40.1 and build `cyt4bb7_cm_7_0` from a clean target selection. Expected: zero compile errors and zero linker errors. Do not claim CM0+ or CM7_1 validation because this feature changes only the CM7_0 project.

- [ ] **Step 8: Commit the hardware handoff**

```powershell
git add docs/low-race-leg-assist-hardware-test.md tools/test_low_race_leg_assist_static.ps1
git commit -m "Document low-race assist hardware gates"
```

## Final completion boundary

Software implementation is complete only after Tasks 1--7 pass their fresh tests and CM7_0 builds. Hardware status remains `level 1 pending` until the supported path and 250 RPM A/B gates are performed. The feature must not be described as reaching 400 RPM until the level-4 measured acceptance succeeds on the vehicle.

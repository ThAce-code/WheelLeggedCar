# Phase 1 Leg-Height, Balance, and Low-Speed Drive Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Safely execute calibrated 35--120 mm open-loop leg-height transitions while the robot balances and moves at conservative low speed.

**Architecture:** Keep the five-bar analytical IK. `control_leg` owns an acceleration-limited trajectory and publishes its motion state; `control_chassis` and `control_balance` consume that state instead of inferring a transition from height error. Leg height remains a command estimate because the four servos have PWM only.

**Tech Stack:** CYT4BB7 embedded C, IAR Embedded Workbench 9.40.1, existing 1 ms cooperative scheduler, PowerShell checks, VOFA telemetry.

## Global Constraints

- Work in `D:\smartcar\WheelLeggedCar_cyt4bb7_v1\.worktree\leg-control-speed-assist` only.
- Do not modify `libraries`.
- Do not add MPC, LQR, numerical real-time IK, X-axis wheel-center control, or active roll/pitch leg compensation.
- The verified 90 degree command is the soft-fault support pose; an `app_safety` IMU fault still disables PWM.
- Never describe a PWM-command trajectory as measured height or measured servo angle.
- Preserve unrelated dirty files and stage only each task's listed files.

## File Structure

| File | Change |
|---|---|
| `project/code/app_types.h` | Motion states, fault reasons, and shared diagnostics. |
| `project/code/leg_config.h/.c` | Transition limits, settle policy, singularity margin, safe-pose height. |
| `project/code/leg_kinematics.h/.c` | Candidate continuity, normalized margin, forward kinematics. |
| `project/code/control_leg.h/.c` | Supervisor, trajectory, soft-fault latch, valid PWM targets. |
| `project/code/control_chassis.c` | Transition speed cap and fast-drive effective interlock. |
| `project/code/control_balance.c` | State-aware scheduling and leg-fault wheel-output block. |
| `project/code/telemetry.c` | Transition state and command-estimate telemetry. |
| `tools/test_leg_transition_numeric.ps1` | Numerical geometry and trajectory regression checks. |
| `tools/test_ik_height_control_static.ps1` | Source integration contracts. |
| `docs/leg-height-phase1-hardware-test.md` | Hardware-gate evidence record. |

## Task 1: Add the transition data contract and numeric baseline

**Files:**

- Modify: `project/code/app_types.h:234-255`
- Modify: `project/code/leg_config.h:57-87`
- Modify: `project/code/leg_config.c:33-55`
- Create: `tools/test_leg_transition_numeric.ps1`
- Modify: `tools/test_ik_height_control_static.ps1`

**Consumes:** The calibrated 35--120 mm interval, existing 10 ms leg period, and verified 90 degree support command.

**Produces:** The type/configuration API consumed by every later task.

- [ ] **Step 1: Write a failing numeric configuration test.** Create `tools/test_leg_transition_numeric.ps1`. Its `Get-LegTransitionConfig` function parses named settings from `project/code/leg_config.c`; it must not duplicate production values. Assert 35 mm and 120 mm calibrated limits, 90 degree safe servo command, positive acceleration, and an IK margin strictly between 0 and 1. Run `powershell -ExecutionPolicy Bypass -File .\tools\test_leg_transition_numeric.ps1`; it must fail because the transition configuration is absent.

- [ ] **Step 2: Add shared types and calibrated settings.** Add these declarations before `leg_diag_struct`:

```c
typedef enum
{
    LEG_MOTION_LOCKED = 0,
    LEG_MOTION_STABLE,
    LEG_MOTION_TRANSITION,
    LEG_MOTION_FAULT
}leg_motion_state_enum;

typedef enum
{
    LEG_FAULT_NONE = 0,
    LEG_FAULT_IK_INVALID,
    LEG_FAULT_IK_MARGIN,
    LEG_FAULT_SERVO_LIMIT
}leg_fault_reason_enum;
```

Extend `leg_diag_struct` with `height_ref_mm`, `height_rate_mm_s`, `ik_margin`, `drive_forward_limit_rpm`, `motion_state`, `fault_reason`, and `drive_allowed`. Extend `leg_height_profile_struct` with `max_height_accel_mm_s2`, `height_settle_error_mm`, `height_settle_ms`, `ik_min_margin`, and `safe_support_height_mm`. Set the initial profile to 20 mm/s maximum height speed, 40 mm/s2 acceleration, 1 mm settle error, 300 ms settle time, 0.20 normalized IK margin, and 80 mm provisional safe-pose height.

- [ ] **Step 3: Pass and commit the contract checks.** Before hardware enablement, measure the wheel-center height at four 90 degree commands, replace the provisional `safe_support_height_mm` with the measured average, and record it in Task 6's evidence file. Require the numeric check to assert exactly 20 mm/s, 40 mm/s2, 300 ms, and 0.20. Extend static checks for every new enum and configuration field. Run both PowerShell checks, run a whitespace diff check, then stage only Task-1 files and commit `Add leg transition state contract`.

## Task 2: Implement continuous IK and forward kinematics

**Files:**

- Modify: `project/code/leg_kinematics.h:12-26`
- Modify: `project/code/leg_kinematics.c:49-238`
- Modify: `tools/test_leg_transition_numeric.ps1`
- Modify: `tools/test_ik_height_control_static.ps1`

**Consumes:** Task 1 `ik_min_margin`.

**Produces:** Continuous valid IK candidates, a normalized singularity margin, and FK validation.

- [ ] **Step 1: Write a failing full-range IK sweep.** Sweep 35--120 mm in 1 mm increments for both sides. Assert finite output, a normalized margin at least 0.20, a wrapped joint delta no greater than 8 degrees per adjacent sample, and FK height within 0.5 mm. Include an out-of-workspace point and tangent-circle point, both expected to return invalid. The initial run must fail because FK is still a stub and no margin or previous solution exists.

- [ ] **Step 2: Change the IK data contract.** Extend the result and solver declaration in `leg_kinematics.h`:

```c
typedef struct
{
    float servo_deg[2];
    float alpha_rad;
    float beta_rad;
    float singularity_margin;
    uint8 valid;
}leg_ik_result_struct;

uint8 leg_kinematics_solve(uint8 right_side,
                            float x_mm,
                            float y_mm,
                            const leg_ik_result_struct *previous,
                            leg_ik_result_struct *result);
```

- [ ] **Step 3: Implement continuity and margin.** For both circular constraints calculate `sqrtf(disc) / sqrtf((a * a) + (b * b))`, publish the smaller value, and reject values below `cfg->ik_min_margin`. Evaluate both half-angle candidates. If a valid previous result exists, pick the within-limit candidate having the smallest wrapped angular distance to that previous angle; otherwise use the configured branch. Never cross a calibrated servo limit to preserve continuity.

- [ ] **Step 4: Replace FK stub with circle intersections.** Compute driven endpoints C and D, then calculate:

```c
projection = ((l2 * l2) - (l3 * l3) + (distance * distance)) / (2.0f * distance);
height = sqrtf((l2 * l2) - (projection * projection));
base_x = c_x + (projection * dx / distance);
base_y = c_y + (projection * dy / distance);
```

Generate both perpendicular circle intersections, return the positive-Y candidate inside the configured workspace, and reject a zero center distance, nonfinite values, or a negative root term.

- [ ] **Step 5: Verify and commit Task 2.** Run numeric and static checks, run a whitespace diff check, stage only the four Task-2 files, and commit `Harden leg IK continuity and validation`.

## Task 3: Implement the authoritative height-transition supervisor

**Files:**

- Modify: `project/code/control_leg.h:12-31`
- Modify: `project/code/control_leg.c:12-465`
- Modify: `project/code/actuator_servo.h:11-18`
- Modify: `project/code/actuator_servo.c:102-173`
- Modify: `tools/test_leg_transition_numeric.ps1`
- Modify: `tools/test_ik_height_control_static.ps1`

**Consumes:** Tasks 1 and 2.

**Produces:** Bounded trajectory, `LOCKED/STABLE/TRANSITION/FAULT` behavior, and soft-fault recovery to 90 degrees.

- [ ] **Step 1: Write failing trajectory and fault checks.** Model 10 ms updates for 35→120, 120→35, and 80→110→50 retargets. Require speed no higher than 20 mm/s, each rate change no higher than 0.4 mm/s per step, and only `TRANSITION` or `STABLE` in a valid run. Inject an insufficient IK margin and require `FAULT`, denied drive, and safe targets approaching 90 degrees no faster than 4.5 degrees per 10 ms.

- [ ] **Step 2: Add supervisor state.** Add private state for `control_leg_height_ref_mm`, `control_leg_height_rate_mm_s`, `control_leg_motion_state`, `control_leg_fault_reason`, and `control_leg_settle_start_ms`. Use braking-distance deceleration:

```c
error_mm = target_mm - reference_mm;
brake_rate_mm_s = sqrtf(2.0f * accel_mm_s2 * control_leg_absf(error_mm));
desired_rate_mm_s = control_leg_clamp(brake_rate_mm_s, 0.0f, max_speed_mm_s);
if(0.0f > error_mm) desired_rate_mm_s = -desired_rate_mm_s;
rate_mm_s = control_leg_ramp_toward(rate_mm_s, desired_rate_mm_s, accel_mm_s2 * dt_s);
reference_mm += rate_mm_s * dt_s;
```

Clamp overshoot to target and zero rate when target is crossed.

- [ ] **Step 3: Apply valid IK and publish state.** Solve both legs at `x=0.0f, y=height_ref_mm`, pass the prior valid result, and publish the lower left/right margin. Keep `actual_height_mm` assigned from `height_ref_mm` for protocol compatibility, with a comment that it is an open-loop command estimate. Enter `STABLE` only after 1 mm error, zero rate, and 300 ms settle time.

- [ ] **Step 4: Implement soft fault and truthful servo output diagnostics.** Add `control_leg_enter_fault(reason)` that latches `LEG_MOTION_FAULT`, zeros rate, commands verified safe angles through the existing speed limiter, and denies drive without disabling PWM. A fresh valid `LH` command clears only this soft fault and starts at `safe_support_height_mm`; `app_safety` remains higher priority. Add `actuator_servo_get_current_angle()` and use it for `servo_actual_deg`, documenting it as output command rather than encoder feedback.

- [ ] **Step 5: Verify and commit Task 3.** Run numeric and static checks, run a whitespace diff check, stage only Task-3 files, and commit `Add bounded leg height transition control`.

## Task 4: Coordinate chassis and balance with leg motion state

**Files:**

- Modify: `project/code/control_chassis.c:217-305,425-460`
- Modify: `project/code/control_balance.c:305-395`
- Modify: `tools/test_leg_transition_numeric.ps1`
- Modify: `tools/test_ik_height_control_static.ps1`

**Consumes:** Task 3 state and diagnostics.

**Produces:** 30 RPM transition cap, fast-drive interlock, and zero wheel output on a leg soft fault.

- [ ] **Step 1: Write failing motion-policy checks.** Require a `TRANSITION` with a fast request to have 30 RPM forward limit and no fast blend; require `FAULT` to have 0 RPM forward limit and no balance output. Add static checks requiring both `LEG_MOTION_TRANSITION` and `LEG_MOTION_FAULT` in chassis and balance code.

- [ ] **Step 2: Replace chassis height-error inference.** Replace both `target_height_mm - actual_height_mm` transition checks with motion-state policy:

```c
if((LEG_MOTION_FAULT == leg->motion_state) || (APP_FALSE == leg->drive_allowed))
{
    forward_limit_rpm = 0.0f;
    effective_fast_enable = APP_FALSE;
}
else if(LEG_MOTION_TRANSITION == leg->motion_state)
{
    forward_limit_rpm = height_profile->transition_forward_limit_rpm;
    effective_fast_enable = APP_FALSE;
}
else
{
    forward_limit_rpm = configured_height_limit_rpm;
    effective_fast_enable = control_chassis_cmd.fast_enable;
}
```

Use `effective_fast_enable` for blend/feed-forward calculations without clearing the operator fast-mode request.

- [ ] **Step 3: Gate balance output.** In `control_balance_update()`, when state is `FAULT`, reset motion state, call `control_balance_stop_output()`, set `safety_blocked`, and return before computing wheel output. For valid `TRANSITION` and `STABLE`, schedule gains from `height_ref_mm`; otherwise retain manual configured gains. The Task-3 trajectory supplies the continuous scheduling input.

- [ ] **Step 4: Verify and commit Task 4.** Run numeric/static checks and a whitespace diff check, stage only Task-4 files, and commit `Coordinate wheel control with leg transitions`.

## Task 5: Expose transition telemetry and update capture tools

**Files:**

- Modify: `project/code/telemetry.c:75-94`
- Modify: `tools/collect_balance_data.ps1`
- Modify: `tools/calib_ik_servo.ps1`
- Modify: `tools/test_ik_height_control_static.ps1`

**Consumes:** Tasks 1, 3, and 4.

**Produces:** Observable state/fault data and 65-float capture tooling.

- [ ] **Step 1: Write failing telemetry contracts.** Require telemetry and the collector to expose `height_ref_mm`, `height_rate_mm_s`, `ik_margin`, `drive_forward_limit_rpm`, `motion_state`, `fault_reason`, and `drive_allowed`. Require CSV labels `leg_height_ref_mm` and `servoN_output_deg`, rejecting labels that falsely claim measured actual height or encoder angles.

- [ ] **Step 2: Append telemetry; do not reorder the existing 58 floats.** Add exactly:

```c
vofa_data[58] = leg->height_ref_mm;
vofa_data[59] = leg->height_rate_mm_s;
vofa_data[60] = leg->ik_margin;
vofa_data[61] = leg->drive_forward_limit_rpm;
vofa_data[62] = (float)leg->motion_state;
vofa_data[63] = (float)leg->fault_reason;
vofa_data[64] = (float)leg->drive_allowed;
```

Set every affected tool frame count to 65 and add the new calibration CSV columns.

- [ ] **Step 3: Verify and commit Task 5.** Run static checks and a whitespace diff check, stage only Task-5 files, and commit `Add leg transition telemetry`.

## Task 6: Record build and staged hardware evidence

**Files:**

- Create: `docs/leg-height-phase1-hardware-test.md`
- Modify: `tools/test_leg_transition_numeric.ps1`
- Modify: `tools/test_ik_height_control_static.ps1`

**Consumes:** Tasks 1--5 and the IAR CM7_0 project.

**Produces:** Offline, build, and hardware-gate evidence.

- [ ] **Step 1: Complete final offline cases.** Add out-of-range 34 and 121 mm, a tangent-circle failure, and injected-margin fault. Require no nonfinite output, no adjacent-sample joint jump greater than 8 degrees, maximum 20 mm/s trajectory speed, and maximum 40 mm/s2 trajectory acceleration.

- [ ] **Step 2: Create the hardware record before bench work.** Create this exact table:

```markdown
| Gate | Build SHA | Height start/end (mm) | Safe-pose measured height (mm) | Max pitch (deg) | Max wheel RPM | IK margin min | IK faults | Safety trips | Result | Notes |
|---|---|---:|---:|---:|---:|---:|---:|---:|---|---|
```

List gates in immutable order: bench/no wheel output; supported stationary at low/default/high heights; balance-in-place transition; low-speed straight transition; low-speed turn and stop. State that a failure blocks all later gates.

- [ ] **Step 3: Build and execute in order.** Run numeric/static checks and a whitespace diff check. Build `cyt4bb7_cm_7_0` from `project/iar/cyt4bb7.eww` in IAR 9.40.1 before hardware Gate 1. Record build SHA, telemetry filename, every attempted gate, and any failure factually; do not label unrun gates as passed.

- [ ] **Step 4: Verify and commit Task 6.** Rerun both PowerShell checks, run a whitespace diff check, stage only the Task-6 evidence/test assets, and commit `Document leg transition validation`.

## Plan Self-Review

- Tasks 1 and 3 cover bounded open-loop trajectory, authority, diagnostics, and soft fault to 90 degrees.
- Task 2 covers branch continuity, normalized singularity margin, and forward kinematics.
- Task 4 covers low-speed drive and balance coupling.
- Task 5 covers telemetry/capture observability.
- Task 6 covers offline sweep, IAR build, and staged hardware validation.
- The plan deliberately excludes active leg attitude control, leg sensor hardware, and whole-body optimization.

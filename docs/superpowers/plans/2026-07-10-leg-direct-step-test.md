# Leg Direct-Step Bench Test Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a compile-guarded `LJ,<height_mm>` bench command that immediately applies the empirical four-servo pose while denying chassis drive.

**Architecture:** Add `LEG_MODE_DIRECT_STEP` and `control_leg_set_direct_step_height()`. The mode maps a valid 45--65 mm height with the existing empirical mapping, marks the leg as a transition with drive disabled, and calls a new actuator immediate-write API after its command is set. Normal `LH`, `LIK`, LOCK, and `actuator_servo_update()` retain their existing paths.

**Tech Stack:** C99 firmware, PowerShell static/numeric checks.

## Global Constraints

- `LJ` is guarded by `APP_LEG_DIRECT_STEP_TEST_ENABLE`; default value is `0U`.
- The test is bench-only: vehicle supported, wheel motors stopped, hands clear.
- Valid `LJ` height is exactly 45--65 mm.
- `LJ` always sets `drive_allowed = APP_FALSE`; `STOP` exits it.
- No pin, scheduler, motor limit, normal `LH`, or normal PWM-slew change.

---

### Task 1: Lock the direct-step safety contract in tests

**Files:**
- Modify: `tools/test_ik_height_control_static.ps1`
- Modify: `tools/test_leg_transition_numeric.ps1`

**Interfaces:**
- Consumes: `APP_LEG_DIRECT_STEP_TEST_ENABLE`, `control_leg_set_direct_step_height(float, uint32)`, and `actuator_servo_apply_immediate()`.
- Produces: checks for parser guard, exact height range, immediate output, and drive denial.

- [ ] **Step 1: Write failing assertions**

Add assertions requiring these exact source contracts:

```powershell
Assert-Contains "project/code/app_config.h" "APP_LEG_DIRECT_STEP_TEST_ENABLE\s+\(0U\)" "Direct-step mode must default off."
Assert-Contains "project/code/control_leg.h" "LEG_MODE_DIRECT_STEP" "Missing direct-step leg mode."
Assert-Contains "project/code/control_leg.h" "control_leg_set_direct_step_height" "Missing direct-step API."
Assert-Contains "project/code/actuator_servo.h" "actuator_servo_apply_immediate" "Missing immediate PWM API."
Assert-Contains "project/code/host_command.c" "APP_LEG_DIRECT_STEP_TEST_ENABLE[\s\S]*'L' == line\[0\].*'J' == line\[1\]" "LJ parser must be compile guarded."
Assert-Contains "project/code/control_leg.c" "LEG_MODE_DIRECT_STEP[\s\S]*drive_allowed = APP_FALSE" "Direct step must deny drive."
```

Extend the numeric empirical mapping test with 45, 55, and 65 mm direct poses.

- [ ] **Step 2: Verify RED**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\test_ik_height_control_static.ps1
```

Expected: fail because direct-step declarations and parser do not exist.

### Task 2: Implement the guarded direct-step path

**Files:**
- Modify: `project/code/app_config.h`
- Modify: `project/code/control_leg.h`
- Modify: `project/code/control_leg.c`
- Modify: `project/code/actuator_servo.h`
- Modify: `project/code/actuator_servo.c`
- Modify: `project/code/host_command.c`

**Interfaces:**
- Produces: `uint8 control_leg_set_direct_step_height(float height_mm, uint32 now_ms)` and `void actuator_servo_apply_immediate(void)`.

- [ ] **Step 1: Add guarded interfaces**

Add in `app_config.h`:

```c
#define APP_LEG_DIRECT_STEP_TEST_ENABLE (0U)
```

Append `LEG_MODE_DIRECT_STEP` after `LEG_MODE_HEIGHT`, add the direct-step API declaration, and change the range check in `control_leg_set_mode()` to accept it.

- [ ] **Step 2: Add immediate actuator output**

Add this actuator function:

```c
void actuator_servo_apply_immediate(void)
{
    uint8 i;
    for(i = 0U; i < APP_SERVO_COUNT; i++)
    {
        if(APP_TRUE == actuator_servo_cmd.enable[i])
        {
            actuator_servo_current_angle[i] = actuator_servo_cmd.angle_deg[i];
            actuator_servo_write(i, actuator_servo_current_angle[i]);
        }
    }
}
```

- [ ] **Step 3: Add direct leg mode and parser**

`control_leg_set_direct_step_height()` rejects non-finite/out-of-range inputs, sets target/reference directly to the request, clears rate/acceleration, selects `LEG_MODE_DIRECT_STEP`, and returns `APP_TRUE`.

The direct-step switch case calls `control_leg_apply_empirical_height()`, writes all four angles, sets `control_leg_motion_state = LEG_MOTION_TRANSITION`, and publishes `drive_allowed = APP_FALSE`. Add a local `direct_step_active` path in `control_leg_publish_diag()` that always denies drive. After `actuator_servo_set_cmd()` at the end of `control_leg_update()`, call `actuator_servo_apply_immediate()` only for direct-step mode.

In `host_command.c`, under `#if (APP_LEG_DIRECT_STEP_TEST_ENABLE == 1U)`, parse `LJ,<number>` with the same number parser as `LH`; call `control_chassis_stop(now_ms)`, `control_balance_set_mode(BALANCE_MODE_OFF)`, `actuator_motor_set_mode_stop()`, then call the direct-step API. If disabled or invalid, preserve the existing command-error path.

- [ ] **Step 4: Verify GREEN**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\test_leg_transition_numeric.ps1
powershell -ExecutionPolicy Bypass -File .\tools\test_ik_height_control_static.ps1
powershell -ExecutionPolicy Bypass -File .\tools\test_fit_leg_ik_calibration.ps1
git diff --check
```

Expected: all scripts pass with no whitespace errors.

### Task 3: Bench enablement and procedure

**Files:**
- Modify: `docs/leg-height-phase1-hardware-test.md`

- [ ] **Step 1: Add procedure**

Document that a bench binary changes `APP_LEG_DIRECT_STEP_TEST_ENABLE` to `1U`, is not a drive binary, and is tested in this order:

```text
STOP
LJ,55
LJ,65
STOP
```

Record servo heating, linkage interference, supply sag, and any movement of the wheels. Restore the macro to `0U` before any supported drive test.

- [ ] **Step 2: Commit direct-step test feature only**

Stage only Task 1--3 files. Do not stage the uncommitted 10 mm/s A/B files or raw CSVs. Commit with:

```powershell
git commit -m "Add guarded leg direct-step bench test"
```

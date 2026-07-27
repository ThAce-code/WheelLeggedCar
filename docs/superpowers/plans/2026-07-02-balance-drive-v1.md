# Balance Drive v1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add conservative chassis command shaping so the already-stable balancing robot can move slowly forward, backward, and turn in place through serial `C,forward,turn` commands without falling.

**Architecture:** Keep the standing balance controller unchanged. `control_chassis` becomes the command-shaping layer: `C` commands set target forward/turn RPM, `control_chassis_update()` ramps actual forward/turn RPM toward those targets, applies command timeout, and publishes ramped left/right base RPM for `control_balance` to add to `balance_rpm`.

**Tech Stack:** Embedded C for CYT4BB/Traveo II, IAR Embedded Workbench project files, PowerShell static validation scripts, VOFA/UART telemetry collection.

---

## Mandatory DeepSeek Setup

Before implementation, DeepSeek must:

1. Open **teammates mode**.
2. State that it is executing `docs/superpowers/plans/2026-07-02-balance-drive-v1.md`.
3. Work task-by-task, with one commit per task unless a task is explicitly verification-only.
4. Report static validation results and any IAR/hardware tests that were not run.

Do not skip teammates mode.

## Scope

Implement only Balance Drive v1:

- Add conservative chassis drive/turn limits.
- Add forward/turn ramp limiting.
- Add `C` command timeout that ramps back to zero.
- Keep `STOP` and `B,0` as immediate hard stops.
- Preserve the current balance law and frozen standing parameters.

Do not implement:

- speed PI control;
- yaw-rate control;
- heading hold;
- joystick/PWM/remote input;
- persistent parameter storage;
- leg coordination.

## File Structure

Modify:

- `project/code/app_config.h`: add chassis drive/turn limits, ramp rates, and command timeout constants.
- `project/code/app_types.h`: extend `chassis_cmd_struct` with target and actual forward/turn state plus `last_update_ms`.
- `project/code/control_chassis.c`: implement finite checks, target clamping, ramping, timeout, mixed base output, and hard-stop clearing.
- `project/code/host_command.c`: verify existing `STOP`, `B,0`, and `C` behavior; only change if the implementation reveals a mismatch.

Create:

- `tools/test_balance_drive_v1_static.ps1`: static checks for constants, data model, timeout/ramp implementation, hard-stop behavior, and dependency boundaries.

Do not modify:

- `project/code/control_balance.c` balance law.
- `project/code/actuator_motor.c` motor output logic.
- `project/code/bldc_foc_uart.c` or `libraries/*`.

---

### Task 1: Add Static Validation For Balance Drive v1

**Files:**
- Create: `tools/test_balance_drive_v1_static.ps1`

- [ ] **Step 1: Create the static validation script**

Create `tools/test_balance_drive_v1_static.ps1` with this content:

```powershell
$ErrorActionPreference = "Stop"

function Assert-Contains {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if(-not (Select-String -Path $Path -Pattern $Pattern -Quiet)) {
        throw $Message
    }
}

function Assert-NotContains {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if(Select-String -Path $Path -Pattern $Pattern -Quiet) {
        throw $Message
    }
}

Assert-Contains "project/code/app_config.h" "APP_CHASSIS_DRIVE_RPM_LIMIT" "Missing drive RPM limit."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_TURN_RPM_LIMIT" "Missing turn RPM limit."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_FORWARD_RAMP_RPM_S" "Missing forward ramp limit."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_TURN_RAMP_RPM_S" "Missing turn ramp limit."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_CMD_TIMEOUT_MS" "Missing chassis command timeout."

Assert-Contains "project/code/app_types.h" "target_forward_rpm" "Missing target forward field."
Assert-Contains "project/code/app_types.h" "target_turn_rpm" "Missing target turn field."
Assert-Contains "project/code/app_types.h" "actual_forward_rpm" "Missing actual forward field."
Assert-Contains "project/code/app_types.h" "actual_turn_rpm" "Missing actual turn field."
Assert-Contains "project/code/app_types.h" "last_update_ms" "Missing chassis last_update_ms field."

Assert-Contains "project/code/control_chassis.c" "control_chassis_ramp_toward" "Missing chassis ramp helper."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_CMD_TIMEOUT_MS" "Missing command timeout handling."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_FORWARD_RAMP_RPM_S" "Missing forward ramp use."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_TURN_RAMP_RPM_S" "Missing turn ramp use."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_DRIVE_RPM_LIMIT" "Missing drive target clamp."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_TURN_RPM_LIMIT" "Missing turn target clamp."

Assert-Contains "project/code/host_command.c" "control_chassis_stop\(now_ms\)" "STOP/B,0 must still clear chassis immediately."

Assert-NotContains "project/code/control_chassis.c" "sensor_imu" "control_chassis must not depend on IMU."
Assert-NotContains "project/code/control_chassis.c" "actuator_motor" "control_chassis must not call motor actuator."
Assert-NotContains "project/code/control_chassis.c" "bldc_foc_uart" "control_chassis must not call BLDC UART."
Assert-NotContains "project/code/control_balance.c" "bldc_foc_uart" "control_balance must not call BLDC UART."

Write-Host "balance drive v1 static checks passed"
```

- [ ] **Step 2: Run the new validation and confirm it fails before implementation**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\test_balance_drive_v1_static.ps1
```

Expected: FAIL, because the new constants and fields do not exist yet.

- [ ] **Step 3: Commit the failing validation script**

Run:

```powershell
git add tools/test_balance_drive_v1_static.ps1
git commit -m "Add balance drive static validation"
```

---

### Task 2: Add Balance Drive Configuration And Chassis State Fields

**Files:**
- Modify: `project/code/app_config.h`
- Modify: `project/code/app_types.h`
- Test: `tools/test_balance_drive_v1_static.ps1`

- [ ] **Step 1: Add conservative chassis drive constants**

In `project/code/app_config.h`, near the existing chassis/balance constants:

```c
#define APP_CHASSIS_RPM_LIMIT           (200.0f)
#define APP_CHASSIS_DRIVE_RPM_LIMIT     (30.0f)
#define APP_CHASSIS_TURN_RPM_LIMIT      (20.0f)
#define APP_CHASSIS_FORWARD_RAMP_RPM_S  (30.0f)
#define APP_CHASSIS_TURN_RAMP_RPM_S     (40.0f)
#define APP_CHASSIS_CMD_TIMEOUT_MS      (1000U)
#define APP_BALANCE_RPM_LIMIT           (300.0f)
```

Do not change the frozen balance parameters from `balance-stand-v1`.

- [ ] **Step 2: Replace `chassis_cmd_struct` fields**

In `project/code/app_types.h`, replace the current `chassis_cmd_struct` with:

```c
typedef struct
{
    float target_forward_rpm;
    float target_turn_rpm;
    float actual_forward_rpm;
    float actual_turn_rpm;
    uint8 enable;
    uint32 last_cmd_ms;
    uint32 last_update_ms;
}chassis_cmd_struct;
```

Keep `chassis_output_struct` unchanged.

- [ ] **Step 3: Run validation and confirm it still fails on implementation-specific checks**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\test_balance_drive_v1_static.ps1
```

Expected: FAIL, because `control_chassis.c` does not yet contain ramp and timeout logic.

- [ ] **Step 4: Run whitespace check**

Run:

```powershell
git diff --check -- project/code/app_config.h project/code/app_types.h
```

Expected: no output.

- [ ] **Step 5: Commit**

Run:

```powershell
git add project/code/app_config.h project/code/app_types.h
git commit -m "Add chassis drive limits and ramp state"
```

---

### Task 3: Implement Chassis Ramp, Limits, Timeout, And Hard Stop

**Files:**
- Modify: `project/code/control_chassis.c`
- Test: `tools/test_balance_drive_v1_static.ps1`

- [ ] **Step 1: Replace `control_chassis.c` implementation**

Replace `project/code/control_chassis.c` with this implementation:

```c
/*********************************************************************************************************************
* File: control_chassis.c
* Description: Chassis forward/turn command shaper and mixer.
********************************************************************************************************************/

#include "control_chassis.h"
#include "app_config.h"

static chassis_cmd_struct control_chassis_cmd;
static chassis_output_struct control_chassis_output;

static float control_chassis_absf(float value)
{
    return (0.0f > value) ? -value : value;
}

static float control_chassis_limit_abs(float value, float limit)
{
    float abs_limit;

    abs_limit = control_chassis_absf(limit);
    if(abs_limit < value)
    {
        return abs_limit;
    }
    if((-abs_limit) > value)
    {
        return -abs_limit;
    }
    return value;
}

static uint8 control_chassis_is_finite(float value)
{
    if(value != value)
    {
        return APP_FALSE;
    }
    if(APP_BALANCE_FINITE_ABS_LIMIT < value)
    {
        return APP_FALSE;
    }
    if((-APP_BALANCE_FINITE_ABS_LIMIT) > value)
    {
        return APP_FALSE;
    }
    return APP_TRUE;
}

static float control_chassis_ramp_toward(float current, float target, float max_delta)
{
    float delta;

    delta = target - current;
    if(max_delta >= control_chassis_absf(delta))
    {
        return target;
    }
    if(0.0f < delta)
    {
        return current + max_delta;
    }
    return current - max_delta;
}

static void control_chassis_clear_output(void)
{
    control_chassis_output.left_base_rpm = 0.0f;
    control_chassis_output.right_base_rpm = 0.0f;
    control_chassis_output.enable = APP_FALSE;
}

void control_chassis_init(void)
{
    control_chassis_cmd.target_forward_rpm = 0.0f;
    control_chassis_cmd.target_turn_rpm = 0.0f;
    control_chassis_cmd.actual_forward_rpm = 0.0f;
    control_chassis_cmd.actual_turn_rpm = 0.0f;
    control_chassis_cmd.enable = APP_FALSE;
    control_chassis_cmd.last_cmd_ms = 0;
    control_chassis_cmd.last_update_ms = 0;

    control_chassis_clear_output();
}

void control_chassis_update(uint32 now_ms)
{
    float target_forward_rpm;
    float target_turn_rpm;
    float left_rpm;
    float right_rpm;
    float dt_s;
    float forward_max_delta;
    float turn_max_delta;

    if(0U == control_chassis_cmd.last_update_ms)
    {
        dt_s = (float)APP_CHASSIS_PERIOD_MS / 1000.0f;
    }
    else
    {
        dt_s = (float)(now_ms - control_chassis_cmd.last_update_ms) / 1000.0f;
    }
    control_chassis_cmd.last_update_ms = now_ms;

    if((0.0f >= dt_s) ||
       (1.0f < dt_s) ||
       (APP_FALSE == control_chassis_is_finite(dt_s)))
    {
        dt_s = (float)APP_CHASSIS_PERIOD_MS / 1000.0f;
    }

    if((APP_FALSE == control_chassis_cmd.enable) ||
       (APP_CHASSIS_CMD_TIMEOUT_MS < (now_ms - control_chassis_cmd.last_cmd_ms)))
    {
        control_chassis_cmd.target_forward_rpm = 0.0f;
        control_chassis_cmd.target_turn_rpm = 0.0f;
    }

    target_forward_rpm = control_chassis_limit_abs(control_chassis_cmd.target_forward_rpm,
                                                   APP_CHASSIS_DRIVE_RPM_LIMIT);
    target_turn_rpm = control_chassis_limit_abs(control_chassis_cmd.target_turn_rpm,
                                                APP_CHASSIS_TURN_RPM_LIMIT);

    forward_max_delta = APP_CHASSIS_FORWARD_RAMP_RPM_S * dt_s;
    turn_max_delta = APP_CHASSIS_TURN_RAMP_RPM_S * dt_s;

    control_chassis_cmd.actual_forward_rpm =
        control_chassis_ramp_toward(control_chassis_cmd.actual_forward_rpm,
                                    target_forward_rpm,
                                    forward_max_delta);
    control_chassis_cmd.actual_turn_rpm =
        control_chassis_ramp_toward(control_chassis_cmd.actual_turn_rpm,
                                    target_turn_rpm,
                                    turn_max_delta);

    control_chassis_cmd.actual_forward_rpm =
        control_chassis_limit_abs(control_chassis_cmd.actual_forward_rpm,
                                  APP_CHASSIS_DRIVE_RPM_LIMIT);
    control_chassis_cmd.actual_turn_rpm =
        control_chassis_limit_abs(control_chassis_cmd.actual_turn_rpm,
                                  APP_CHASSIS_TURN_RPM_LIMIT);

    if((APP_FALSE == control_chassis_is_finite(control_chassis_cmd.actual_forward_rpm)) ||
       (APP_FALSE == control_chassis_is_finite(control_chassis_cmd.actual_turn_rpm)))
    {
        control_chassis_stop(now_ms);
        return;
    }

    left_rpm = control_chassis_cmd.actual_forward_rpm - control_chassis_cmd.actual_turn_rpm;
    right_rpm = control_chassis_cmd.actual_forward_rpm + control_chassis_cmd.actual_turn_rpm;

    control_chassis_output.left_base_rpm = control_chassis_limit_abs(left_rpm, APP_CHASSIS_RPM_LIMIT);
    control_chassis_output.right_base_rpm = control_chassis_limit_abs(right_rpm, APP_CHASSIS_RPM_LIMIT);
    control_chassis_output.enable = APP_TRUE;

    if((0.0f == control_chassis_cmd.actual_forward_rpm) &&
       (0.0f == control_chassis_cmd.actual_turn_rpm) &&
       (0.0f == control_chassis_cmd.target_forward_rpm) &&
       (0.0f == control_chassis_cmd.target_turn_rpm) &&
       (APP_FALSE == control_chassis_cmd.enable))
    {
        control_chassis_clear_output();
    }
}

void control_chassis_set_cmd(float forward_rpm, float turn_rpm, uint8 enable, uint32 now_ms)
{
    if((APP_FALSE == control_chassis_is_finite(forward_rpm)) ||
       (APP_FALSE == control_chassis_is_finite(turn_rpm)))
    {
        control_chassis_stop(now_ms);
        return;
    }

    control_chassis_cmd.target_forward_rpm = control_chassis_limit_abs(forward_rpm,
                                                                       APP_CHASSIS_DRIVE_RPM_LIMIT);
    control_chassis_cmd.target_turn_rpm = control_chassis_limit_abs(turn_rpm,
                                                                    APP_CHASSIS_TURN_RPM_LIMIT);
    control_chassis_cmd.enable = (APP_TRUE == enable) ? APP_TRUE : APP_FALSE;
    control_chassis_cmd.last_cmd_ms = now_ms;
}

void control_chassis_stop(uint32 now_ms)
{
    control_chassis_cmd.target_forward_rpm = 0.0f;
    control_chassis_cmd.target_turn_rpm = 0.0f;
    control_chassis_cmd.actual_forward_rpm = 0.0f;
    control_chassis_cmd.actual_turn_rpm = 0.0f;
    control_chassis_cmd.enable = APP_FALSE;
    control_chassis_cmd.last_cmd_ms = now_ms;
    control_chassis_cmd.last_update_ms = now_ms;
    control_chassis_clear_output();
}

const chassis_cmd_struct *control_chassis_get_cmd(void)
{
    return &control_chassis_cmd;
}

const chassis_output_struct *control_chassis_get_output(void)
{
    return &control_chassis_output;
}
```

- [ ] **Step 2: Review two intended behaviors before running tests**

Confirm by inspection:

- `C,0,0` sets targets to zero and ramps actual values down.
- `STOP` and `B,0` call `control_chassis_stop(now_ms)`, clearing targets, actual values, and output immediately.

- [ ] **Step 3: Run static validation**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\test_balance_drive_v1_static.ps1
```

Expected:

```text
balance drive v1 static checks passed
```

- [ ] **Step 4: Run dependency checks**

Run:

```powershell
rg -n "sensor_imu|actuator_motor|bldc_foc_uart|debug_" project/code/control_chassis.c
rg -n "bldc_foc_uart|debug_read|host_command" project/code/control_balance.c
```

Expected: no output.

- [ ] **Step 5: Run whitespace check**

Run:

```powershell
git diff --check -- project/code/control_chassis.c
```

Expected: no output.

- [ ] **Step 6: Commit**

Run:

```powershell
git add project/code/control_chassis.c
git commit -m "Add chassis command ramp and timeout"
```

---

### Task 4: Validate Integration And Prepare Hardware Test Commands

**Files:**
- Inspect: `project/code/host_command.c`
- Inspect: `project/code/telemetry.c`
- Test: `tools/test_balance_drive_v1_static.ps1`

- [ ] **Step 1: Verify host command hard-stop paths**

Check `project/code/host_command.c` and confirm:

```c
STOP -> control_balance_set_ident_excitation(0.0f, 0U, now_ms)
     -> control_chassis_stop(now_ms)
     -> control_balance_set_mode(BALANCE_MODE_OFF)
     -> actuator_motor_set_mode_stop()
```

and:

```c
B,0 -> control_balance_set_ident_excitation(0.0f, 0U, now_ms)
    -> control_chassis_stop(now_ms)
    -> control_balance_set_mode(BALANCE_MODE_OFF)
```

If either path is missing `control_chassis_stop(now_ms)`, add it and commit:

```powershell
git add project/code/host_command.c
git commit -m "Preserve hard stop chassis clearing"
```

If both paths are already correct, do not create a no-op commit.

- [ ] **Step 2: Verify telemetry meaning**

Check `project/code/telemetry.c` and confirm the balance frame fields for `chassis_left_rpm` and `chassis_right_rpm` come from `control_balance_get_diag()`.

Expected behavior after Task 3:

- telemetry `chassis_left_rpm` and `chassis_right_rpm` report ramped mixed base RPM;
- no telemetry layout change is required for this version.

- [ ] **Step 3: Run all available static checks**

Run:

```powershell
git diff --check
powershell -ExecutionPolicy Bypass -File tools\test_balance_drive_v1_static.ps1
powershell -ExecutionPolicy Bypass -File tools\test_collect_balance_data.ps1
powershell -ExecutionPolicy Bypass -File tools\test_analyze_balance_pd.ps1
powershell -ExecutionPolicy Bypass -File tools\test_identify_balance_model.ps1
```

Expected:

- `git diff --check`: no output.
- all PowerShell scripts pass.

- [ ] **Step 4: Build in IAR**

Open `project/iar/cyt4bb7.eww` and build:

```text
cyt4bb7_cm_7_0
```

If the workspace requires full multi-core validation, also build:

```text
cyt4bb7_cm_0_plus
cyt4bb7_cm_7_1
```

Record whether IAR was available. Do not claim build pass unless the IAR build actually ran.

- [ ] **Step 5: Hardware smoke test sequence**

Use the frozen standing parameters already compiled in. After boot, run:

```text
IMU_ZERO
STOP
B,1
B,2
C,0,0
```

Confirm the robot stands before motion tests.

- [ ] **Step 6: Capture low-speed forward**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 `
  -Duration 20 `
  -Commands "0:STOP;0.5:IMU_ZERO;1:B,1;2:B,2;2.2:C,0,0;5:C,20,0;12:C,0,0" `
  -Note "drive_v1_forward_20"
```

Pass criteria:

- does not fall;
- `chassis_left_rpm` and `chassis_right_rpm` ramp toward +20 instead of stepping;
- `C,0,0` returns to stable standing.

- [ ] **Step 7: Capture low-speed reverse**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 `
  -Duration 20 `
  -Commands "0:STOP;0.5:IMU_ZERO;1:B,1;2:B,2;2.2:C,0,0;5:C,-20,0;12:C,0,0" `
  -Note "drive_v1_reverse_20"
```

Pass criteria:

- does not fall;
- `chassis_left_rpm` and `chassis_right_rpm` ramp toward -20 instead of stepping;
- `C,0,0` returns to stable standing.

- [ ] **Step 8: Capture in-place left turn**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 `
  -Duration 20 `
  -Commands "0:STOP;0.5:IMU_ZERO;1:B,1;2:B,2;2.2:C,0,0;5:C,0,15;12:C,0,0" `
  -Note "drive_v1_left_turn_15"
```

Pass criteria:

- does not fall;
- `chassis_left_rpm` ramps toward -15;
- `chassis_right_rpm` ramps toward +15;
- `C,0,0` returns to stable standing.

- [ ] **Step 9: Capture in-place right turn**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 `
  -Duration 20 `
  -Commands "0:STOP;0.5:IMU_ZERO;1:B,1;2:B,2;2.2:C,0,0;5:C,0,-15;12:C,0,0" `
  -Note "drive_v1_right_turn_15"
```

Pass criteria:

- does not fall;
- `chassis_left_rpm` ramps toward +15;
- `chassis_right_rpm` ramps toward -15;
- `C,0,0` returns to stable standing.

- [ ] **Step 10: Capture command timeout behavior**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 `
  -Duration 12 `
  -Commands "0:STOP;0.5:IMU_ZERO;1:B,1;2:B,2;2.2:C,0,0;4:C,20,0" `
  -Note "drive_v1_cmd_timeout"
```

Pass criteria:

- after the last `C` command ages past `APP_CHASSIS_CMD_TIMEOUT_MS`, `chassis_left_rpm` and `chassis_right_rpm` ramp back toward 0;
- the robot remains balanced if it has room to coast safely.

- [ ] **Step 11: Capture hard stop behavior**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 `
  -Duration 12 `
  -Commands "0:STOP;0.5:IMU_ZERO;1:B,1;2:B,2;2.2:C,0,0;4:C,20,0;7:STOP" `
  -Note "drive_v1_hard_stop"
```

Pass criteria:

- `STOP` immediately clears chassis output and motor output;
- no delayed ramp is used for `STOP`.

- [ ] **Step 12: Final summary**

Report:

- commits created;
- files changed;
- static checks;
- IAR build result;
- hardware test result;
- any failed motion direction or safety behavior;
- whether the next stage can move to speed/yaw closed-loop work.

Do not claim Balance Drive v1 is complete unless low-speed forward, reverse, left turn, right turn, return-to-zero, timeout, and hard-stop tests have passed on hardware.

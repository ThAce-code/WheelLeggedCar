# Balance Drive v2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Upgrade `C,forward,turn` into closed-loop balanced driving: forward is target average wheel speed, turn is target yaw rate, and the existing four-state balance controller remains the inner stabilizing loop.

**Architecture:** `control_chassis` owns operator motion targets and the outer speed/yaw-rate loops. `control_balance` consumes the chassis drive request, applies the speed-loop pitch offset to the balance pitch setpoint, applies the yaw-rate turn differential to final left/right motor targets, and remains the only balance module that commands the motor actuator. Telemetry expands so hardware tests can see targets, measured speed/yaw, pitch offset, pitch setpoint, and turn output.

**Tech Stack:** Embedded C for CYT4BB/Traveo II, IAR Embedded Workbench, PowerShell static validation and telemetry collection scripts.

---

## Mandatory DeepSeek Setup

Before implementation, DeepSeek must:

1. Open **teammates mode**.
2. State that it is executing `docs/superpowers/plans/2026-07-02-balance-drive-v2.md`.
3. Work task-by-task, with one commit per task unless a task is explicitly verification-only.
4. Report static validation results, IAR build result, and hardware tests that were not run.

Do not skip teammates mode.

## Scope

Implement Balance Drive v2 only:

- `C,forward,turn` means target average wheel speed RPM and target yaw-rate deg/s.
- Forward target is converted to a clamped pitch-setpoint offset by a speed P/PI loop.
- Turn target is converted to a clamped differential RPM by a yaw-rate P loop.
- Default `turn_kp` is `0.0f`, so yaw-rate loop exists but is disabled until tuned.
- `C,0,0` ramps targets to zero and keeps balance alive.
- `STOP` and `B,0` remain hard-stop/clear paths.
- Do not reintroduce any `C` command timeout.

Do not implement:

- position hold;
- yaw angle hold;
- persistent gain storage;
- joystick/PWM/wireless input;
- leg coordination;
- MATLAB gain redesign;
- current/torque mode for the external BLDC driver.

## File Structure

Modify:

- `project/code/app_config.h`: add V2 speed/yaw loop constants and update old V1 chassis limits.
- `project/code/app_types.h`: extend `chassis_cmd_struct`, reinterpret `chassis_output_struct`, and extend `balance_diag_struct`.
- `project/code/control_chassis.h`: add `control_chassis_set_drive_gain()`.
- `project/code/control_chassis.c`: compute forward speed-loop pitch offset and yaw-rate turn output.
- `project/code/control_balance.c`: use chassis pitch offset and turn output in final motor command.
- `project/code/control_balance.h`: add pitch-setpoint getter if needed for telemetry/debug consistency.
- `project/code/host_command.c`: parse `BD,speed_kp,speed_ki,turn_kp`.
- `project/code/telemetry.c`: expand balance frame to 21 floats.
- `tools/collect_balance_data.ps1`: parse and write 21-channel V2 frames.
- `tools/test_collect_balance_data.ps1`: update static expectations for new telemetry columns.
- `tools/test_balance_drive_v1_static.ps1`: update to V2 semantics or replace with V2 static script.

Create:

- `tools/test_balance_drive_v2_static.ps1`: static checks for V2 semantics.

Do not modify:

- `project/code/bldc_foc_uart.c` for drive behavior.
- `project/code/actuator_motor.c` motor output logic.
- `libraries/*`.

---

### Task 1: Add V2 Static Validation

**Files:**
- Create: `tools/test_balance_drive_v2_static.ps1`

- [ ] **Step 1: Create the failing V2 static validation script**

Create `tools/test_balance_drive_v2_static.ps1`:

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

Assert-Contains "project/code/app_config.h" "APP_CHASSIS_FORWARD_RPM_LIMIT" "Missing forward speed limit."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_TURN_RATE_LIMIT_DPS" "Missing turn-rate limit."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_TURN_RATE_RAMP_DPS_S" "Missing turn-rate ramp."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_SPEED_KP" "Missing speed loop Kp."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_SPEED_KI" "Missing speed loop Ki."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_SPEED_INTEGRAL_LIMIT" "Missing speed integral limit."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_SPEED_PITCH_LIMIT_DEG" "Missing speed pitch limit."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_TURN_KP" "Missing turn loop Kp."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_TURN_RPM_LIMIT" "Missing turn output limit."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_DRIVE_GAIN_ABS_LIMIT" "Missing drive gain limit."
Assert-NotContains "project/code/app_config.h" "APP_CHASSIS_CMD_TIMEOUT_MS" "C command timeout must not be reintroduced."

Assert-Contains "project/code/app_types.h" "target_turn_dps" "chassis_cmd_struct must store target yaw-rate."
Assert-Contains "project/code/app_types.h" "actual_turn_dps" "chassis_cmd_struct must store ramped yaw-rate."
Assert-Contains "project/code/app_types.h" "speed_pitch_offset_deg" "chassis_cmd_struct must store speed pitch offset."
Assert-Contains "project/code/app_types.h" "turn_rpm" "chassis structs must store turn output RPM."
Assert-Contains "project/code/app_types.h" "speed_integral" "chassis_cmd_struct must store speed integral."
Assert-Contains "project/code/app_types.h" "forward_actual_rpm" "chassis_output_struct must expose measured forward speed."
Assert-Contains "project/code/app_types.h" "gyro_z_dps" "chassis_output_struct must expose yaw rate."
Assert-Contains "project/code/app_types.h" "pitch_setpoint_deg" "balance_diag_struct must expose effective pitch setpoint."

Assert-Contains "project/code/control_chassis.h" "control_chassis_set_drive_gain" "Missing runtime drive gain API."
Assert-Contains "project/code/control_chassis.c" "actuator_motor_get_motor_rpm_loop_diag" "control_chassis must read wheel speed feedback."
Assert-Contains "project/code/control_chassis.c" "sensor_imu_get_state" "control_chassis must read gyro_z yaw-rate."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_SPEED_KP" "control_chassis must use speed Kp."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_SPEED_KI" "control_chassis must use speed Ki."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_TURN_KP" "control_chassis must use turn Kp."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_SPEED_PITCH_LIMIT_DEG" "control_chassis must clamp pitch offset."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_TURN_RPM_LIMIT" "control_chassis must clamp turn output."
Assert-NotContains "project/code/control_chassis.c" "APP_CHASSIS_CMD_TIMEOUT_MS" "control_chassis must not timeout C commands."
Assert-NotContains "project/code/control_chassis.c" "actuator_motor_set_" "control_chassis must not command motor output."
Assert-NotContains "project/code/control_chassis.c" "bldc_foc_uart" "control_chassis must not call BLDC UART."
Assert-NotContains "project/code/control_chassis.c" "debug_read" "control_chassis must not parse host commands."

Assert-Contains "project/code/control_balance.c" "pitch_offset_deg" "control_balance must apply chassis pitch offset."
Assert-Contains "project/code/control_balance.c" "chassis->turn_rpm" "control_balance must apply chassis turn output."
Assert-Contains "project/code/control_balance.c" "pitch_setpoint_deg" "control_balance must publish effective pitch setpoint."
Assert-NotContains "project/code/control_balance.c" "bldc_foc_uart" "control_balance must not call BLDC UART."
Assert-NotContains "project/code/control_balance.c" "debug_read" "control_balance must not parse host commands."

Assert-Contains "project/code/host_command.c" "'D' == line\[1\]" "host_command must parse BD command."
Assert-Contains "project/code/host_command.c" "control_chassis_set_drive_gain" "BD must call chassis drive gain API."

Assert-Contains "project/code/telemetry.c" "float vofa_data\\[21\\]" "Telemetry must emit 21-channel V2 frame."
Assert-Contains "tools/collect_balance_data.ps1" "\\$FloatCount = 21" "Collector must parse 21 floats."
Assert-Contains "tools/collect_balance_data.ps1" "forward_target_rpm" "Collector must write forward target."
Assert-Contains "tools/collect_balance_data.ps1" "speed_pitch_offset_deg" "Collector must write speed pitch offset."
Assert-Contains "tools/collect_balance_data.ps1" "turn_target_dps" "Collector must write turn target."
Assert-Contains "tools/collect_balance_data.ps1" "gyro_z_dps" "Collector must write gyro_z."
Assert-Contains "tools/collect_balance_data.ps1" "turn_rpm" "Collector must write turn output."

Write-Host "balance drive v2 static checks passed"
```

- [ ] **Step 2: Run the new validation and confirm it fails before implementation**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\test_balance_drive_v2_static.ps1
```

Expected: FAIL, with the first missing V2 symbol such as `APP_CHASSIS_FORWARD_RPM_LIMIT`.

- [ ] **Step 3: Commit the failing validation**

Run:

```powershell
git add tools/test_balance_drive_v2_static.ps1
git commit -m "Add balance drive v2 static validation"
```

---

### Task 2: Add V2 Configuration And Shared Types

**Files:**
- Modify: `project/code/app_config.h`
- Modify: `project/code/app_types.h`
- Test: `tools/test_balance_drive_v2_static.ps1`

- [ ] **Step 1: Update drive configuration constants**

In `project/code/app_config.h`, replace the V1 drive constants:

```c
#define APP_CHASSIS_RPM_LIMIT           (200.0f)
#define APP_CHASSIS_DRIVE_RPM_LIMIT     (30.0f)
#define APP_CHASSIS_TURN_RPM_LIMIT      (20.0f)
#define APP_CHASSIS_FORWARD_RAMP_RPM_S  (30.0f)
#define APP_CHASSIS_TURN_RAMP_RPM_S     (40.0f)
```

with:

```c
#define APP_CHASSIS_RPM_LIMIT                (200.0f)
#define APP_CHASSIS_FORWARD_RPM_LIMIT        (60.0f)
#define APP_CHASSIS_TURN_RATE_LIMIT_DPS      (60.0f)
#define APP_CHASSIS_FORWARD_RAMP_RPM_S       (60.0f)
#define APP_CHASSIS_TURN_RATE_RAMP_DPS_S     (120.0f)
#define APP_CHASSIS_SPEED_KP                 (0.03f)
#define APP_CHASSIS_SPEED_KI                 (0.0f)
#define APP_CHASSIS_SPEED_INTEGRAL_LIMIT     (100.0f)
#define APP_CHASSIS_SPEED_PITCH_LIMIT_DEG    (3.0f)
#define APP_CHASSIS_TURN_KP                  (0.0f)
#define APP_CHASSIS_TURN_RPM_LIMIT           (60.0f)
#define APP_CHASSIS_DRIVE_GAIN_ABS_LIMIT     (100.0f)
```

Do not add `APP_CHASSIS_CMD_TIMEOUT_MS`.

- [ ] **Step 2: Replace chassis shared structs**

In `project/code/app_types.h`, replace `chassis_cmd_struct` and `chassis_output_struct` with:

```c
typedef struct
{
    float target_forward_rpm;
    float target_turn_dps;
    float actual_forward_rpm;
    float actual_turn_dps;
    float speed_pitch_offset_deg;
    float turn_rpm;
    float speed_kp;
    float speed_ki;
    float turn_kp;
    float speed_integral;
    uint8 enable;
    uint32 last_cmd_ms;
    uint32 last_update_ms;
}chassis_cmd_struct;

typedef struct
{
    float pitch_offset_deg;
    float turn_rpm;
    float forward_target_rpm;
    float forward_actual_rpm;
    float turn_target_dps;
    float gyro_z_dps;
    uint8 enable;
}chassis_output_struct;
```

- [ ] **Step 3: Extend balance diagnostics**

In `project/code/app_types.h`, add these fields to `balance_diag_struct` after `wheel_pos_kp` and before `output_enable`:

```c
    float pitch_setpoint_deg;
    float drive_forward_target_rpm;
    float drive_forward_actual_rpm;
    float drive_speed_pitch_offset_deg;
    float drive_turn_target_dps;
    float drive_gyro_z_dps;
    float drive_turn_rpm;
```

- [ ] **Step 4: Run V2 static validation and confirm it fails on implementation-specific checks**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\test_balance_drive_v2_static.ps1
```

Expected: FAIL, because `control_chassis.c`, `control_balance.c`, `host_command.c`, telemetry, and collector are not updated yet.

- [ ] **Step 5: Run whitespace check**

Run:

```powershell
git diff --check -- project/code/app_config.h project/code/app_types.h
```

Expected: no whitespace errors.

- [ ] **Step 6: Commit**

Run:

```powershell
git add project/code/app_config.h project/code/app_types.h
git commit -m "Add balance drive v2 config and types"
```

---

### Task 3: Implement Chassis Speed And Yaw-Rate Outer Loops

**Files:**
- Modify: `project/code/control_chassis.h`
- Modify: `project/code/control_chassis.c`
- Test: `tools/test_balance_drive_v2_static.ps1`

- [ ] **Step 1: Update the chassis public API**

In `project/code/control_chassis.h`, add:

```c
uint8 control_chassis_set_drive_gain(float speed_kp, float speed_ki, float turn_kp);
```

Keep existing APIs unchanged.

- [ ] **Step 2: Add required includes**

In `project/code/control_chassis.c`, add:

```c
#include "actuator_motor.h"
#include "sensor_imu.h"
```

`control_chassis` may read feedback and IMU state for outer loops. It must not call `actuator_motor_set_*` or BLDC APIs.

- [ ] **Step 3: Update initialization and hard-stop clearing**

In `control_chassis_init()`, initialize all new fields:

```c
control_chassis_cmd.target_forward_rpm = 0.0f;
control_chassis_cmd.target_turn_dps = 0.0f;
control_chassis_cmd.actual_forward_rpm = 0.0f;
control_chassis_cmd.actual_turn_dps = 0.0f;
control_chassis_cmd.speed_pitch_offset_deg = 0.0f;
control_chassis_cmd.turn_rpm = 0.0f;
control_chassis_cmd.speed_kp = APP_CHASSIS_SPEED_KP;
control_chassis_cmd.speed_ki = APP_CHASSIS_SPEED_KI;
control_chassis_cmd.turn_kp = APP_CHASSIS_TURN_KP;
control_chassis_cmd.speed_integral = 0.0f;
control_chassis_cmd.enable = APP_FALSE;
control_chassis_cmd.last_cmd_ms = 0;
control_chassis_cmd.last_update_ms = 0;
```

In `control_chassis_stop()`, clear targets, ramped targets, speed integral, pitch offset, turn output, and output struct:

```c
control_chassis_cmd.target_forward_rpm = 0.0f;
control_chassis_cmd.target_turn_dps = 0.0f;
control_chassis_cmd.actual_forward_rpm = 0.0f;
control_chassis_cmd.actual_turn_dps = 0.0f;
control_chassis_cmd.speed_pitch_offset_deg = 0.0f;
control_chassis_cmd.turn_rpm = 0.0f;
control_chassis_cmd.speed_integral = 0.0f;
control_chassis_cmd.enable = APP_FALSE;
control_chassis_cmd.last_cmd_ms = now_ms;
control_chassis_cmd.last_update_ms = now_ms;
```

- [ ] **Step 4: Update output clearing**

Replace `control_chassis_clear_output()` with:

```c
static void control_chassis_clear_output(void)
{
    control_chassis_output.pitch_offset_deg = 0.0f;
    control_chassis_output.turn_rpm = 0.0f;
    control_chassis_output.forward_target_rpm = 0.0f;
    control_chassis_output.forward_actual_rpm = 0.0f;
    control_chassis_output.turn_target_dps = 0.0f;
    control_chassis_output.gyro_z_dps = 0.0f;
    control_chassis_output.enable = APP_FALSE;
}
```

- [ ] **Step 5: Update `control_chassis_set_cmd()` semantics**

Replace V1 target clamping with V2 target clamping:

```c
control_chassis_cmd.target_forward_rpm = control_chassis_limit_abs(forward_rpm,
                                                                   APP_CHASSIS_FORWARD_RPM_LIMIT);
control_chassis_cmd.target_turn_dps = control_chassis_limit_abs(turn_rpm,
                                                                APP_CHASSIS_TURN_RATE_LIMIT_DPS);
control_chassis_cmd.enable = (APP_TRUE == enable) ? APP_TRUE : APP_FALSE;
control_chassis_cmd.last_cmd_ms = now_ms;
```

Keep the function signature unchanged so `host_command.c` still calls `control_chassis_set_cmd(kp, ki, APP_TRUE, now_ms)`.

- [ ] **Step 6: Implement `control_chassis_set_drive_gain()`**

Add this function to `project/code/control_chassis.c`:

```c
uint8 control_chassis_set_drive_gain(float speed_kp, float speed_ki, float turn_kp)
{
    if((APP_FALSE == control_chassis_is_finite(speed_kp)) ||
       (APP_FALSE == control_chassis_is_finite(speed_ki)) ||
       (APP_FALSE == control_chassis_is_finite(turn_kp)) ||
       (APP_CHASSIS_DRIVE_GAIN_ABS_LIMIT < control_chassis_absf(speed_kp)) ||
       (APP_CHASSIS_DRIVE_GAIN_ABS_LIMIT < control_chassis_absf(speed_ki)) ||
       (APP_CHASSIS_DRIVE_GAIN_ABS_LIMIT < control_chassis_absf(turn_kp)))
    {
        return APP_FALSE;
    }

    control_chassis_cmd.speed_kp = speed_kp;
    control_chassis_cmd.speed_ki = speed_ki;
    control_chassis_cmd.turn_kp = turn_kp;
    control_chassis_cmd.speed_integral = 0.0f;
    return APP_TRUE;
}
```

- [ ] **Step 7: Implement the V2 `control_chassis_update()` body**

Inside `control_chassis_update()`, after dt validation:

```c
const motor_rpm_loop_diag_struct *rpm_diag;
const imu_state_struct *imu;
float target_forward_rpm;
float target_turn_dps;
float forward_max_delta;
float turn_max_delta;
float speed_error_rpm;
float speed_pitch_offset_deg;
float avg_wheel_speed_rpm;
float turn_error_dps;
float turn_rpm;

rpm_diag = actuator_motor_get_motor_rpm_loop_diag();
imu = sensor_imu_get_state();
```

If disabled:

```c
if(APP_FALSE == control_chassis_cmd.enable)
{
    control_chassis_cmd.target_forward_rpm = 0.0f;
    control_chassis_cmd.target_turn_dps = 0.0f;
}
```

Ramp targets:

```c
target_forward_rpm = control_chassis_limit_abs(control_chassis_cmd.target_forward_rpm,
                                               APP_CHASSIS_FORWARD_RPM_LIMIT);
target_turn_dps = control_chassis_limit_abs(control_chassis_cmd.target_turn_dps,
                                            APP_CHASSIS_TURN_RATE_LIMIT_DPS);

forward_max_delta = APP_CHASSIS_FORWARD_RAMP_RPM_S * dt_s;
turn_max_delta = APP_CHASSIS_TURN_RATE_RAMP_DPS_S * dt_s;

control_chassis_cmd.actual_forward_rpm =
    control_chassis_ramp_toward(control_chassis_cmd.actual_forward_rpm,
                                target_forward_rpm,
                                forward_max_delta);
control_chassis_cmd.actual_turn_dps =
    control_chassis_ramp_toward(control_chassis_cmd.actual_turn_dps,
                                target_turn_dps,
                                turn_max_delta);
```

Compute measured speed and speed-loop output:

```c
avg_wheel_speed_rpm = 0.5f * (rpm_diag->left_motor_rpm + rpm_diag->right_motor_rpm);
speed_error_rpm = control_chassis_cmd.actual_forward_rpm - avg_wheel_speed_rpm;
control_chassis_cmd.speed_integral += speed_error_rpm * dt_s;
control_chassis_cmd.speed_integral =
    control_chassis_limit_abs(control_chassis_cmd.speed_integral,
                              APP_CHASSIS_SPEED_INTEGRAL_LIMIT);

speed_pitch_offset_deg =
    (control_chassis_cmd.speed_kp * speed_error_rpm) +
    (control_chassis_cmd.speed_ki * control_chassis_cmd.speed_integral);
speed_pitch_offset_deg =
    control_chassis_limit_abs(speed_pitch_offset_deg,
                              APP_CHASSIS_SPEED_PITCH_LIMIT_DEG);
```

Compute yaw-rate output:

```c
turn_error_dps = control_chassis_cmd.actual_turn_dps - imu->gyro_z_dps;
turn_rpm = control_chassis_cmd.turn_kp * turn_error_dps;
turn_rpm = control_chassis_limit_abs(turn_rpm, APP_CHASSIS_TURN_RPM_LIMIT);
```

Add finite guard before publishing:

```c
if((APP_FALSE == control_chassis_is_finite(avg_wheel_speed_rpm)) ||
   (APP_FALSE == control_chassis_is_finite(imu->gyro_z_dps)) ||
   (APP_FALSE == control_chassis_is_finite(speed_pitch_offset_deg)) ||
   (APP_FALSE == control_chassis_is_finite(turn_rpm)))
{
    control_chassis_stop(now_ms);
    return;
}
```

Publish command and output:

```c
control_chassis_cmd.speed_pitch_offset_deg = speed_pitch_offset_deg;
control_chassis_cmd.turn_rpm = turn_rpm;

control_chassis_output.pitch_offset_deg = speed_pitch_offset_deg;
control_chassis_output.turn_rpm = turn_rpm;
control_chassis_output.forward_target_rpm = control_chassis_cmd.actual_forward_rpm;
control_chassis_output.forward_actual_rpm = avg_wheel_speed_rpm;
control_chassis_output.turn_target_dps = control_chassis_cmd.actual_turn_dps;
control_chassis_output.gyro_z_dps = imu->gyro_z_dps;
control_chassis_output.enable = APP_TRUE;
```

If disabled and all ramped values have returned to zero, clear output:

```c
if((APP_FALSE == control_chassis_cmd.enable) &&
   (0.0f == control_chassis_cmd.actual_forward_rpm) &&
   (0.0f == control_chassis_cmd.actual_turn_dps) &&
   (0.0f == control_chassis_cmd.target_forward_rpm) &&
   (0.0f == control_chassis_cmd.target_turn_dps))
{
    control_chassis_cmd.speed_pitch_offset_deg = 0.0f;
    control_chassis_cmd.turn_rpm = 0.0f;
    control_chassis_cmd.speed_integral = 0.0f;
    control_chassis_clear_output();
}
```

- [ ] **Step 8: Run V2 static validation and dependency checks**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\test_balance_drive_v2_static.ps1
rg -n "actuator_motor_set_|bldc_foc_uart|debug_read|host_command" project/code/control_chassis.c
```

Expected:

- V2 static script still fails later on balance/host/telemetry checks.
- `rg` dependency check returns no output.

- [ ] **Step 9: Run whitespace check**

Run:

```powershell
git diff --check -- project/code/control_chassis.h project/code/control_chassis.c
```

Expected: no whitespace errors.

- [ ] **Step 10: Commit**

Run:

```powershell
git add project/code/control_chassis.h project/code/control_chassis.c
git commit -m "Add chassis speed and yaw-rate loops"
```

---

### Task 4: Integrate V2 Chassis Output Into Balance Control

**Files:**
- Modify: `project/code/control_balance.c`
- Modify: `project/code/control_balance.h`
- Test: `tools/test_balance_drive_v2_static.ps1`

- [ ] **Step 1: Initialize new diagnostics**

In `control_balance_init()`, initialize:

```c
control_balance_diag.pitch_setpoint_deg = control_balance_pitch_setpoint_deg;
control_balance_diag.drive_forward_target_rpm = 0.0f;
control_balance_diag.drive_forward_actual_rpm = 0.0f;
control_balance_diag.drive_speed_pitch_offset_deg = 0.0f;
control_balance_diag.drive_turn_target_dps = 0.0f;
control_balance_diag.drive_gyro_z_dps = 0.0f;
control_balance_diag.drive_turn_rpm = 0.0f;
```

- [ ] **Step 2: Replace chassis diag assignments**

In `control_balance_update()`, replace:

```c
control_balance_diag.chassis_left_rpm = chassis->left_base_rpm;
control_balance_diag.chassis_right_rpm = chassis->right_base_rpm;
```

with:

```c
control_balance_diag.chassis_left_rpm = -chassis->turn_rpm;
control_balance_diag.chassis_right_rpm = chassis->turn_rpm;
control_balance_diag.drive_forward_target_rpm = chassis->forward_target_rpm;
control_balance_diag.drive_forward_actual_rpm = chassis->forward_actual_rpm;
control_balance_diag.drive_speed_pitch_offset_deg = chassis->pitch_offset_deg;
control_balance_diag.drive_turn_target_dps = chassis->turn_target_dps;
control_balance_diag.drive_gyro_z_dps = chassis->gyro_z_dps;
control_balance_diag.drive_turn_rpm = chassis->turn_rpm;
```

This keeps old `chassis_left_rpm/chassis_right_rpm` fields meaningful as differential output preview until telemetry fully switches to explicit V2 fields.

- [ ] **Step 3: Use effective pitch setpoint**

Before computing `balance_rpm`, add:

```c
float pitch_setpoint_deg;
```

near the other local float declarations.

Then before the balance law:

```c
pitch_setpoint_deg = control_balance_pitch_setpoint_deg + chassis->pitch_offset_deg;
pitch_setpoint_deg = control_balance_limit_abs(pitch_setpoint_deg, APP_BALANCE_GAIN_ABS_LIMIT);
control_balance_diag.pitch_setpoint_deg = pitch_setpoint_deg;
```

Replace:

```c
balance_rpm = (control_balance_pitch_kp * (imu->pitch - control_balance_pitch_setpoint_deg)) +
```

with:

```c
balance_rpm = (control_balance_pitch_kp * (imu->pitch - pitch_setpoint_deg)) +
```

- [ ] **Step 4: Apply turn differential to final output**

Replace:

```c
output_left_rpm = chassis->left_base_rpm + balance_rpm;
output_right_rpm = chassis->right_base_rpm + balance_rpm;
```

with:

```c
output_left_rpm = balance_rpm - chassis->turn_rpm;
output_right_rpm = balance_rpm + chassis->turn_rpm;
```

- [ ] **Step 5: Extend finite checks**

In the existing final finite guard, include:

```c
(APP_FALSE == control_balance_is_finite(pitch_setpoint_deg)) ||
(APP_FALSE == control_balance_is_finite(chassis->pitch_offset_deg)) ||
(APP_FALSE == control_balance_is_finite(chassis->turn_rpm)) ||
```

- [ ] **Step 6: Add a pitch-setpoint getter**

In `project/code/control_balance.h`, add:

```c
float control_balance_get_pitch_setpoint(void);
```

In `project/code/control_balance.c`, add:

```c
float control_balance_get_pitch_setpoint(void)
{
    return control_balance_pitch_setpoint_deg;
}
```

This getter returns the base `BS` setpoint, not the drive-offset effective setpoint. Effective setpoint is exposed in diagnostics.

- [ ] **Step 7: Run V2 static validation**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\test_balance_drive_v2_static.ps1
```

Expected: FAIL later on missing `BD` parser and telemetry/collector updates.

- [ ] **Step 8: Run dependency check**

Run:

```powershell
rg -n "bldc_foc_uart|debug_read|host_command" project/code/control_balance.c
```

Expected: no output.

- [ ] **Step 9: Run whitespace check**

Run:

```powershell
git diff --check -- project/code/control_balance.h project/code/control_balance.c
```

Expected: no whitespace errors.

- [ ] **Step 10: Commit**

Run:

```powershell
git add project/code/control_balance.h project/code/control_balance.c
git commit -m "Apply chassis drive request in balance control"
```

---

### Task 5: Add Runtime Drive Gain Command

**Files:**
- Modify: `project/code/host_command.c`
- Test: `tools/test_balance_drive_v2_static.ps1`

- [ ] **Step 1: Add a three-number parser**

`host_command.c` already has `host_command_parse_pid_gain()` for nonnegative PID gains. Do not reuse it because speed and turn gains may need signed values during direction testing.

Add this helper after `host_command_parse_two_numbers()`:

```c
static uint8 host_command_parse_three_numbers(const char *text, float *first, float *second, float *third)
{
    char number_text[16];
    float values[3];
    uint8 value_index = 0;
    uint8 number_index = 0;
    uint8 read_index = 0;

    while('\0' != text[read_index])
    {
        if(',' == text[read_index])
        {
            if((0U == number_index) || (2U <= value_index))
            {
                return APP_FALSE;
            }
            number_text[number_index] = '\0';
            if(APP_FALSE == host_command_parse_number(number_text, &values[value_index]))
            {
                return APP_FALSE;
            }
            value_index++;
            number_index = 0;
        }
        else
        {
            if((sizeof(number_text) - 1U) <= number_index)
            {
                return APP_FALSE;
            }
            number_text[number_index] = text[read_index];
            number_index++;
        }
        read_index++;
    }

    if((0U == number_index) || (2U != value_index))
    {
        return APP_FALSE;
    }
    number_text[number_index] = '\0';
    if(APP_FALSE == host_command_parse_number(number_text, &values[value_index]))
    {
        return APP_FALSE;
    }

    *first = values[0];
    *second = values[1];
    *third = values[2];
    return APP_TRUE;
}
```

- [ ] **Step 2: Add `BD` parsing**

In `host_command_process_line()`, after `BI` and before `BL`, add:

```c
    if(('B' == line[0]) && ('D' == line[1]) && (',' == line[2]) &&
       (APP_TRUE == host_command_parse_three_numbers(&line[3], &kp, &ki, &kd)))
    {
        if(APP_TRUE == control_chassis_set_drive_gain(kp, ki, kd))
        {
            actuator_motor_record_command_error(APP_FALSE);
            return;
        }
    }
```

The variable names `kp`, `ki`, and `kd` are reused as `speed_kp`, `speed_ki`, and `turn_kp`.

- [ ] **Step 3: Verify `STOP` and `B,0` still clear chassis**

Confirm `STOP` and `B,0` still call:

```c
control_chassis_stop(now_ms);
```

Do not alter these paths.

- [ ] **Step 4: Run V2 static validation**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\test_balance_drive_v2_static.ps1
```

Expected: FAIL later on telemetry/collector updates.

- [ ] **Step 5: Run whitespace check**

Run:

```powershell
git diff --check -- project/code/host_command.c
```

Expected: no whitespace errors.

- [ ] **Step 6: Commit**

Run:

```powershell
git add project/code/host_command.c
git commit -m "Add balance drive runtime gain command"
```

---

### Task 6: Expand V2 Telemetry And Collection Script

**Files:**
- Modify: `project/code/telemetry.c`
- Modify: `tools/collect_balance_data.ps1`
- Modify: `tools/test_collect_balance_data.ps1`
- Test: `tools/test_balance_drive_v2_static.ps1`

- [ ] **Step 1: Expand telemetry frame to 21 floats**

In `project/code/telemetry.c`, change:

```c
float vofa_data[16];
```

to:

```c
float vofa_data[21];
```

Use this channel map:

```c
vofa_data[0] = (float)now_ms;
vofa_data[1] = (float)balance->mode;
vofa_data[2] = imu->roll;
vofa_data[3] = imu->pitch;
vofa_data[4] = imu->yaw;
vofa_data[5] = balance->pitch_rate_dps;
vofa_data[6] = balance->balance_rpm;
vofa_data[7] = (float)(wheel->online && wheel->left_online && wheel->right_online);
vofa_data[8] = rpm_diag->left_motor_rpm;
vofa_data[9] = rpm_diag->right_motor_rpm;
vofa_data[10] = rpm_diag->left_duty;
vofa_data[11] = rpm_diag->right_duty;
vofa_data[12] = balance->pitch_kp;
vofa_data[13] = balance->pitch_rate_kd;
vofa_data[14] = balance->drive_forward_target_rpm;
vofa_data[15] = balance->drive_forward_actual_rpm;
vofa_data[16] = balance->drive_speed_pitch_offset_deg;
vofa_data[17] = balance->pitch_setpoint_deg;
vofa_data[18] = balance->drive_turn_target_dps;
vofa_data[19] = balance->drive_gyro_z_dps;
vofa_data[20] = balance->drive_turn_rpm;
```

The old V1 `chassis_left_rpm/chassis_right_rpm` are no longer emitted in balance telemetry because V2 exposes explicit forward and turn signals.

- [ ] **Step 2: Update collector frame constants and header**

In `tools/collect_balance_data.ps1`, change:

```powershell
$FloatCount = 16
```

to:

```powershell
$FloatCount = 21
```

Replace `$Fields` with:

```powershell
$Fields = "pc_time_s,elapsed_s,sample_index,last_command,time_ms,balance_mode,roll_deg,pitch_deg,yaw_deg,pitch_rate_dps,balance_rpm,feedback_online,left_motor_rpm,right_motor_rpm,left_duty,right_duty,balance_kp,balance_kd,forward_target_rpm,forward_actual_rpm,speed_pitch_offset_deg,pitch_setpoint_deg,turn_target_dps,gyro_z_dps,turn_rpm,note"
```

- [ ] **Step 3: Update collector frame object fields**

In `Pop-BalanceFrames`, replace the last two V1 fields:

```powershell
chassis_left_rpm = $values[14]
chassis_right_rpm = $values[15]
```

with:

```powershell
forward_target_rpm = $values[14]
forward_actual_rpm = $values[15]
speed_pitch_offset_deg = $values[16]
pitch_setpoint_deg = $values[17]
turn_target_dps = $values[18]
gyro_z_dps = $values[19]
turn_rpm = $values[20]
```

- [ ] **Step 4: Update collector CSV row**

Replace the last two frame fields in `$row`:

```powershell
("{0:F3}" -f $frame.chassis_left_rpm),
("{0:F3}" -f $frame.chassis_right_rpm),
```

with:

```powershell
("{0:F3}" -f $frame.forward_target_rpm),
("{0:F3}" -f $frame.forward_actual_rpm),
("{0:F6}" -f $frame.speed_pitch_offset_deg),
("{0:F6}" -f $frame.pitch_setpoint_deg),
("{0:F3}" -f $frame.turn_target_dps),
("{0:F6}" -f $frame.gyro_z_dps),
("{0:F3}" -f $frame.turn_rpm),
```

- [ ] **Step 5: Update collector static tests**

Open `tools/test_collect_balance_data.ps1`.

Update assertions so they expect:

```powershell
$FloatCount = 21
forward_target_rpm
forward_actual_rpm
speed_pitch_offset_deg
pitch_setpoint_deg
turn_target_dps
gyro_z_dps
turn_rpm
```

and no longer require `chassis_left_rpm` / `chassis_right_rpm` in the collector header.

- [ ] **Step 6: Run telemetry and collector tests**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\test_balance_drive_v2_static.ps1
powershell -ExecutionPolicy Bypass -File tools\test_collect_balance_data.ps1
```

Expected:

- V2 static checks pass or fail only if another V2 task was missed.
- Collector test passes with 21-channel fields.

- [ ] **Step 7: Run whitespace check**

Run:

```powershell
git diff --check -- project/code/telemetry.c tools/collect_balance_data.ps1 tools/test_collect_balance_data.ps1
```

Expected: no whitespace errors.

- [ ] **Step 8: Commit**

Run:

```powershell
git add project/code/telemetry.c tools/collect_balance_data.ps1 tools/test_collect_balance_data.ps1
git commit -m "Expand telemetry for balance drive v2"
```

---

### Task 7: Final Validation, IAR Build, And Hardware Commands

**Files:**
- Test all modified files
- No code changes unless a validation failure reveals a missed implementation detail

- [ ] **Step 1: Run all local static checks**

Run:

```powershell
git diff --check
powershell -ExecutionPolicy Bypass -File tools\test_balance_drive_v2_static.ps1
powershell -ExecutionPolicy Bypass -File tools\test_balance_drive_v1_static.ps1
powershell -ExecutionPolicy Bypass -File tools\test_iar_warning_cleanup.ps1
powershell -ExecutionPolicy Bypass -File tools\test_collect_balance_data.ps1
powershell -ExecutionPolicy Bypass -File tools\test_analyze_balance_pd.ps1
powershell -ExecutionPolicy Bypass -File tools\test_identify_balance_model.ps1
```

Expected:

- `git diff --check`: no whitespace errors.
- all PowerShell scripts pass.

If `test_analyze_balance_pd.ps1` or `test_identify_balance_model.ps1` assumes the old CSV columns, update those static tests and scripts to tolerate both old and V2 CSVs. Do not break analysis of old 16-channel captures.

- [ ] **Step 2: Verify dependency boundaries**

Run:

```powershell
rg -n "actuator_motor_set_|bldc_foc_uart|debug_read|host_command" project/code/control_chassis.c
rg -n "bldc_foc_uart|debug_read|host_command" project/code/control_balance.c
rg -n "bldc_foc_uart_set|uart_write|uart_put|uart_send" project/code | Select-String -NotMatch "bldc_foc_uart.c"
```

Expected:

- first command: no output.
- second command: no output.
- third command: only expected low-level UART/debug files, not `control_chassis.c` or `control_balance.c`.

- [ ] **Step 3: Build in IAR**

Open:

```text
project/iar/cyt4bb7.eww
```

Build:

```text
cyt4bb7_cm_7_0 - Debug
```

Expected:

- 0 errors.
- No new warnings from changed files.

If IAR reports warnings, fix only warnings introduced by V2 or touched files. Existing unrelated warnings should be reported separately if they remain.

- [ ] **Step 4: Hardware standing regression**

After flashing, run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 `
  -Duration 20 `
  -Commands "0:STOP;0.5:B,1;1:B,2;2:C,0,0" `
  -Note "drive_v2_stand_baseline"
```

Pass criteria:

- Robot stands as before.
- `forward_target_rpm` and `turn_target_dps` are 0.
- `speed_pitch_offset_deg` and `turn_rpm` are near 0.
- `pitch_setpoint_deg` is near the compiled/BS setpoint.

- [ ] **Step 5: Hardware forward direction test**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 `
  -Duration 20 `
  -Commands "0:STOP;0.5:B,1;1:B,2;2:C,0,0;4:BD,0.03,0,0;5:C,20,0;12:C,0,0" `
  -Note "drive_v2_forward_20_p003"
```

Pass criteria:

- Positive `C,20,0` moves in the same direction as `BS` increase.
- `forward_target_rpm` ramps toward +20.
- `speed_pitch_offset_deg` becomes positive while speed error is positive.
- `turn_rpm` remains near 0 because `turn_kp=0`.
- `C,0,0` returns to standing.

- [ ] **Step 6: Hardware reverse direction test**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 `
  -Duration 20 `
  -Commands "0:STOP;0.5:B,1;1:B,2;2:C,0,0;4:BD,0.03,0,0;5:C,-20,0;12:C,0,0" `
  -Note "drive_v2_reverse_20_p003"
```

Pass criteria:

- Negative `C,-20,0` moves in the same direction as `BS` decrease.
- `forward_target_rpm` ramps toward -20.
- `speed_pitch_offset_deg` becomes negative while speed error is negative.
- `C,0,0` returns to standing.

- [ ] **Step 7: Hardware speed gain sweep**

Only if Steps 5 and 6 are stable, test:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 `
  -Duration 20 `
  -Commands "0:STOP;0.5:B,1;1:B,2;2:C,0,0;4:BD,0.05,0,0;5:C,30,0;12:C,0,0" `
  -Note "drive_v2_forward_30_p005"
```

Pass criteria:

- Faster than `BD,0.03` without pitching beyond safe range.
- No RPM saturation.
- `C,0,0` recovers.

- [ ] **Step 8: Hardware turn loop disabled check**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 `
  -Duration 20 `
  -Commands "0:STOP;0.5:B,1;1:B,2;2:C,0,0;4:BD,0.03,0,0;5:C,20,0;12:C,0,0" `
  -Note "drive_v2_turn_disabled_forward_20"
```

Pass criteria:

- `turn_rpm` remains 0.
- Any yaw drift is measured through `gyro_z_dps`, not corrected.

- [ ] **Step 9: Hardware turn sign probe**

Start with a very small turn gain:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 `
  -Duration 16 `
  -Commands "0:STOP;0.5:B,1;1:B,2;2:C,0,0;4:BD,0.03,0,0.2;5:C,0,10;10:C,0,0" `
  -Note "drive_v2_turn_left_k020"
```

Then:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 `
  -Duration 16 `
  -Commands "0:STOP;0.5:B,1;1:B,2;2:C,0,0;4:BD,0.03,0,0.2;5:C,0,-10;10:C,0,0" `
  -Note "drive_v2_turn_right_k020"
```

Pass criteria:

- `turn_target_dps` has the commanded sign.
- `gyro_z_dps` moves toward the command instead of away from it.
- If it moves away, do not increase gain. Report data and flip the sign in one explicit code location after review.

- [ ] **Step 10: Final report**

Report:

- commits created;
- files changed;
- static checks;
- IAR build result;
- hardware test results and CSV paths;
- whether forward speed loop direction is correct;
- whether turn loop sign is correct;
- recommended next tuning step.

Do not claim Balance Drive v2 is hardware-complete unless standing, forward, reverse, return-to-zero, and at least one turn sign probe pass on hardware.

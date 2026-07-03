# Balance Fast Blend Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a safe high-speed blended balance-drive mode (`B,3`) that preserves current `B,2` low-speed behavior while allowing staged 40/70/100 RPM validation.

**Architecture:** `control_chassis` owns operator targets, fast-mode arming, target ramping, speed-loop pitch offset, and `fast_blend`. `control_balance` consumes the chassis output, interpolates only the effective wheel-speed damping term, adds optional signed speed feedforward, and publishes term-level diagnostics. Telemetry and collection scripts expand from 28 to 38 balance floats.

**Tech Stack:** Embedded C for CYT4BB/Traveo II, IAR Embedded Workbench, PowerShell static checks and telemetry capture scripts.

---

## Mandatory DeepSeek Setup

Before implementation, DeepSeek must:

1. Work in `D:\smartcar\WheelLeggedCar_cyt4bb7_v1\.worktree\balance-fast-mode-spec`.
2. Confirm the branch is `codex/balance-fast-mode-spec`.
3. State that it is executing `docs/superpowers/plans/2026-07-03-balance-fast-blend.md`.
4. Work task-by-task.
5. Make one commit after each task unless the task is verification-only.
6. Report IAR build and hardware smoke-test status at the end.

Do not modify `libraries/*` or generated IAR output directories.

## Scope

Implement only:

- `B,3` high-speed armed balance mode.
- Smooth `fast_blend` based on ramped forward target.
- Interpolated pitch-offset limit.
- Interpolated effective `wheel_speed_ks`.
- Signed speed feedforward infrastructure with default `0.0f`.
- 38-float telemetry and collector support.
- Static validation updates.

Do not implement:

- Leg X assist.
- Position hold.
- 300 RPM limits.
- Persistent profile storage.
- Motor inner-loop changes.

## File Structure

Modify:

- `project/code/app_config.h`: add fast-mode limits, blend thresholds, fast pitch limit, fast `Ks`, and feedforward gain.
- `project/code/app_types.h`: add `BALANCE_MODE_BALANCE_FAST`, chassis fast fields, and balance term diagnostics.
- `project/code/control_chassis.h`: add `control_chassis_set_fast_enable()`.
- `project/code/control_chassis.c`: add smoothstep/lerp helpers, fast-mode state, blend ramp, dynamic forward limit, dynamic pitch limit, and feedforward request.
- `project/code/control_balance.c`: allow fast mode, compute effective `Ks`, compute term diagnostics, add feedforward term, and clear diagnostics in safe paths.
- `project/code/host_command.c`: parse `B,3` and reset fast enable on `STOP`, `B,0`, `B,1`, and `B,2`.
- `project/code/telemetry.c`: expand balance telemetry to 38 floats.
- `tools/collect_balance_data.ps1`: parse/write 38-float balance telemetry.
- `tools/test_collect_balance_data.ps1`: validate new columns.
- `tools/test_balance_drive_v2_static.ps1`: add static checks for fast mode.
- `tools/test_tune_drive_loops_static.ps1`: update telemetry-count expectations if present.
- `tools/tune_drive_loops.m`: update comments/column handling from 28 to 38 floats, and keep old analysis behavior intact.

Create:

- No new source modules. Keep the change local to the existing balance/chassis split.

---

### Task 1: Add Fast-Mode Configuration and Types

**Files:**
- Modify: `project/code/app_config.h`
- Modify: `project/code/app_types.h`
- Modify: `tools/test_balance_drive_v2_static.ps1`

- [ ] **Step 1: Add failing static checks**

Append these checks to `tools/test_balance_drive_v2_static.ps1` near the existing config/type checks:

```powershell
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_FAST_FORWARD_RPM_LIMIT" "Missing fast forward RPM limit."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_FAST_BLEND_START_RPM" "Missing fast blend start."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_FAST_BLEND_FULL_RPM" "Missing fast blend full threshold."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_FAST_BLEND_RAMP_S" "Missing fast blend ramp rate."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_FAST_SPEED_PITCH_LIMIT_DEG" "Missing fast pitch offset limit."
Assert-Contains "project/code/app_config.h" "APP_BALANCE_FAST_WHEEL_SPEED_KS" "Missing fast wheel speed damping."
Assert-Contains "project/code/app_config.h" "APP_BALANCE_FAST_SPEED_FF_GAIN" "Missing fast speed feedforward gain."
Assert-Contains "project/code/app_types.h" "BALANCE_MODE_BALANCE_FAST" "Missing fast balance mode enum."
Assert-Contains "project/code/app_types.h" "fast_enable" "Chassis command must store fast enable."
Assert-Contains "project/code/app_types.h" "fast_blend" "Chassis diagnostics must expose fast blend."
Assert-Contains "project/code/app_types.h" "speed_pitch_limit_deg" "Chassis diagnostics must expose active pitch limit."
Assert-Contains "project/code/app_types.h" "speed_ff_rpm" "Chassis diagnostics must expose speed feedforward request."
Assert-Contains "project/code/app_types.h" "pitch_term_rpm" "Balance diagnostics must expose pitch term."
Assert-Contains "project/code/app_types.h" "rate_term_rpm" "Balance diagnostics must expose rate term."
Assert-Contains "project/code/app_types.h" "speed_term_rpm" "Balance diagnostics must expose speed term."
Assert-Contains "project/code/app_types.h" "pos_term_rpm" "Balance diagnostics must expose position term."
Assert-Contains "project/code/app_types.h" "ff_term_rpm" "Balance diagnostics must expose feedforward term."
```

- [ ] **Step 2: Run static check and verify it fails**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
```

Expected: FAIL with the first missing fast-mode constant.

- [ ] **Step 3: Add config constants**

In `project/code/app_config.h`, add these constants immediately after `APP_CHASSIS_SPEED_PITCH_LIMIT_DEG` and near the balance constants:

```c
#define APP_CHASSIS_FAST_FORWARD_RPM_LIMIT       (180.0f)
#define APP_CHASSIS_FAST_BLEND_START_RPM         (40.0f)
#define APP_CHASSIS_FAST_BLEND_FULL_RPM          (90.0f)
#define APP_CHASSIS_FAST_BLEND_RAMP_S            (1.0f)
#define APP_CHASSIS_FAST_SPEED_PITCH_LIMIT_DEG   (12.0f)

#define APP_BALANCE_FAST_WHEEL_SPEED_KS          (0.8f)
#define APP_BALANCE_FAST_SPEED_FF_GAIN           (0.0f)
```

Keep `APP_CHASSIS_FORWARD_RPM_LIMIT (60.0f)` unchanged for `B,2`.

- [ ] **Step 4: Add type fields**

In `project/code/app_types.h`, update the balance mode enum:

```c
typedef enum
{
    BALANCE_MODE_OFF = 0,
    BALANCE_MODE_STANDBY,
    BALANCE_MODE_BALANCE_TEST,
    BALANCE_MODE_BALANCE_FAST
}balance_mode_enum;
```

Add to `chassis_cmd_struct`:

```c
    float fast_blend;
    float speed_pitch_limit_deg;
    float speed_ff_rpm;
    uint8 fast_enable;
```

Add to `chassis_output_struct`:

```c
    float fast_blend;
    float speed_integral;
    float speed_pitch_limit_deg;
    float speed_ff_rpm;
```

Add to `balance_diag_struct`:

```c
    float drive_fast_blend;
    float drive_speed_integral;
    float drive_speed_pitch_limit_deg;
    float drive_speed_ff_rpm;
    float pitch_term_rpm;
    float rate_term_rpm;
    float speed_term_rpm;
    float pos_term_rpm;
    float ff_term_rpm;
```

- [ ] **Step 5: Run static check and verify it passes this task**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
```

Expected: PASS or fail on checks that belong to later tasks.

- [ ] **Step 6: Commit**

```powershell
git add project/code/app_config.h project/code/app_types.h tools/test_balance_drive_v2_static.ps1
git commit -m "Add fast blend configuration and diagnostics types"
```

---

### Task 2: Implement Chassis Fast Blend

**Files:**
- Modify: `project/code/control_chassis.h`
- Modify: `project/code/control_chassis.c`
- Modify: `tools/test_balance_drive_v2_static.ps1`

- [ ] **Step 1: Add failing static checks**

Append these checks:

```powershell
Assert-Contains "project/code/control_chassis.h" "control_chassis_set_fast_enable" "Missing fast mode enable API."
Assert-Contains "project/code/control_chassis.c" "control_chassis_smoothstep" "Missing fast blend smoothstep helper."
Assert-Contains "project/code/control_chassis.c" "control_chassis_lerp" "Missing fast blend lerp helper."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_FAST_FORWARD_RPM_LIMIT" "Fast mode must use fast forward limit."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_FAST_BLEND_START_RPM" "Fast mode must use blend start threshold."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_FAST_BLEND_FULL_RPM" "Fast mode must use blend full threshold."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_FAST_BLEND_RAMP_S" "Fast blend must be ramp limited."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_FAST_SPEED_PITCH_LIMIT_DEG" "Fast mode must interpolate pitch limit."
Assert-Contains "project/code/control_chassis.c" "APP_BALANCE_FAST_SPEED_FF_GAIN" "Chassis must compute feedforward request."
Assert-Contains "project/code/control_chassis.c" "control_chassis_cmd.fast_blend = 0.0f" "Stop/reset paths must clear fast blend."
```

- [ ] **Step 2: Run static check and verify it fails**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
```

Expected: FAIL on missing `control_chassis_set_fast_enable`.

- [ ] **Step 3: Add public API**

In `project/code/control_chassis.h`, add:

```c
void control_chassis_set_fast_enable(uint8 enable);
```

- [ ] **Step 4: Add helpers**

In `project/code/control_chassis.c`, add these static helpers near the existing limit/ramp helpers:

```c
static float control_chassis_clamp01(float value)
{
    if(0.0f > value)
    {
        return 0.0f;
    }
    if(1.0f < value)
    {
        return 1.0f;
    }
    return value;
}

static float control_chassis_lerp(float low, float high, float blend)
{
    blend = control_chassis_clamp01(blend);
    return low + ((high - low) * blend);
}

static float control_chassis_smoothstep(float edge0, float edge1, float value)
{
    float t;

    if(edge1 <= edge0)
    {
        return 0.0f;
    }

    t = control_chassis_clamp01((value - edge0) / (edge1 - edge0));
    return t * t * (3.0f - (2.0f * t));
}
```

- [ ] **Step 5: Initialize and clear fast state**

In `control_chassis_clear_output()`, set:

```c
    control_chassis_output.fast_blend = 0.0f;
    control_chassis_output.speed_integral = 0.0f;
    control_chassis_output.speed_pitch_limit_deg = APP_CHASSIS_SPEED_PITCH_LIMIT_DEG;
    control_chassis_output.speed_ff_rpm = 0.0f;
```

In `control_chassis_init()` and `control_chassis_stop()`, set:

```c
    control_chassis_cmd.fast_blend = 0.0f;
    control_chassis_cmd.speed_pitch_limit_deg = APP_CHASSIS_SPEED_PITCH_LIMIT_DEG;
    control_chassis_cmd.speed_ff_rpm = 0.0f;
    control_chassis_cmd.fast_enable = APP_FALSE;
```

In stale/offline input handling inside `control_chassis_update()`, clear:

```c
        control_chassis_cmd.fast_blend = 0.0f;
        control_chassis_cmd.speed_ff_rpm = 0.0f;
        control_chassis_cmd.speed_pitch_limit_deg = APP_CHASSIS_SPEED_PITCH_LIMIT_DEG;
```

- [ ] **Step 6: Apply dynamic forward target limit**

Replace both uses that clamp forward command with `APP_CHASSIS_FORWARD_RPM_LIMIT` with a local `forward_limit_rpm`:

```c
float forward_limit_rpm;

forward_limit_rpm = (APP_TRUE == control_chassis_cmd.fast_enable) ?
                    APP_CHASSIS_FAST_FORWARD_RPM_LIMIT :
                    APP_CHASSIS_FORWARD_RPM_LIMIT;

target_forward_rpm = control_chassis_limit_abs(control_chassis_cmd.target_forward_rpm,
                                               forward_limit_rpm);
```

In `control_chassis_set_cmd()`, use the same `forward_limit_rpm` before storing `target_forward_rpm`.

- [ ] **Step 7: Compute fast blend before speed-loop pitch limit**

After `actual_forward_rpm` is ramped and before computing `speed_pitch_offset_deg`, add:

```c
float raw_fast_blend;
float speed_pitch_limit_deg;

raw_fast_blend = control_chassis_smoothstep(APP_CHASSIS_FAST_BLEND_START_RPM,
                                            APP_CHASSIS_FAST_BLEND_FULL_RPM,
                                            control_chassis_absf(control_chassis_cmd.actual_forward_rpm));
if(APP_FALSE == control_chassis_cmd.fast_enable)
{
    raw_fast_blend = 0.0f;
}

control_chassis_cmd.fast_blend =
    control_chassis_ramp_toward(control_chassis_cmd.fast_blend,
                                raw_fast_blend,
                                APP_CHASSIS_FAST_BLEND_RAMP_S * dt_s);

speed_pitch_limit_deg =
    control_chassis_lerp(APP_CHASSIS_SPEED_PITCH_LIMIT_DEG,
                         APP_CHASSIS_FAST_SPEED_PITCH_LIMIT_DEG,
                         control_chassis_cmd.fast_blend);
control_chassis_cmd.speed_pitch_limit_deg = speed_pitch_limit_deg;

control_chassis_cmd.speed_ff_rpm =
    APP_BALANCE_FAST_SPEED_FF_GAIN *
    control_chassis_cmd.actual_forward_rpm *
    control_chassis_cmd.fast_blend;
```

Then clamp `speed_pitch_offset_deg` with `speed_pitch_limit_deg` instead of `APP_CHASSIS_SPEED_PITCH_LIMIT_DEG`.

- [ ] **Step 8: Publish fast diagnostics**

When filling `control_chassis_output`, add:

```c
    control_chassis_output.fast_blend = control_chassis_cmd.fast_blend;
    control_chassis_output.speed_integral = control_chassis_cmd.speed_integral;
    control_chassis_output.speed_pitch_limit_deg = control_chassis_cmd.speed_pitch_limit_deg;
    control_chassis_output.speed_ff_rpm = control_chassis_cmd.speed_ff_rpm;
```

- [ ] **Step 9: Add setter**

Add this function near the other setters:

```c
void control_chassis_set_fast_enable(uint8 enable)
{
    control_chassis_cmd.fast_enable = (APP_TRUE == enable) ? APP_TRUE : APP_FALSE;
    if(APP_FALSE == control_chassis_cmd.fast_enable)
    {
        control_chassis_cmd.fast_blend = 0.0f;
        control_chassis_cmd.speed_ff_rpm = 0.0f;
        control_chassis_cmd.speed_pitch_limit_deg = APP_CHASSIS_SPEED_PITCH_LIMIT_DEG;
    }
}
```

- [ ] **Step 10: Run static check**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
```

Expected: PASS or fail on checks from later tasks.

- [ ] **Step 11: Commit**

```powershell
git add project/code/control_chassis.h project/code/control_chassis.c tools/test_balance_drive_v2_static.ps1
git commit -m "Add ramped chassis fast blend"
```

---

### Task 3: Implement Balance Fast Terms

**Files:**
- Modify: `project/code/control_balance.c`
- Modify: `tools/test_balance_drive_v2_static.ps1`

- [ ] **Step 1: Add failing static checks**

Append:

```powershell
Assert-Contains "project/code/control_balance.c" "BALANCE_MODE_BALANCE_FAST" "Balance loop must allow fast mode."
Assert-Contains "project/code/control_balance.c" "APP_BALANCE_FAST_WHEEL_SPEED_KS" "Balance loop must interpolate fast Ks."
Assert-Contains "project/code/control_balance.c" "pitch_term_rpm" "Balance loop must compute pitch term diagnostics."
Assert-Contains "project/code/control_balance.c" "rate_term_rpm" "Balance loop must compute rate term diagnostics."
Assert-Contains "project/code/control_balance.c" "speed_term_rpm" "Balance loop must compute speed term diagnostics."
Assert-Contains "project/code/control_balance.c" "pos_term_rpm" "Balance loop must compute position term diagnostics."
Assert-Contains "project/code/control_balance.c" "ff_term_rpm" "Balance loop must compute feedforward term diagnostics."
Assert-Contains "project/code/control_balance.c" "chassis->speed_ff_rpm" "Balance loop must consume chassis speed feedforward."
Assert-Contains "project/code/control_balance.c" "drive_fast_blend" "Balance diagnostics must copy fast blend."
```

- [ ] **Step 2: Run static check and verify it fails**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
```

Expected: FAIL on missing fast-mode balance logic.

- [ ] **Step 3: Add local lerp helper**

In `project/code/control_balance.c`, add:

```c
static float control_balance_clamp01(float value)
{
    if(0.0f > value)
    {
        return 0.0f;
    }
    if(1.0f < value)
    {
        return 1.0f;
    }
    return value;
}

static float control_balance_lerp(float low, float high, float blend)
{
    blend = control_balance_clamp01(blend);
    return low + ((high - low) * blend);
}
```

- [ ] **Step 4: Initialize and clear diagnostics**

In `control_balance_init()` and all safe clear paths, set these fields to zero except `wheel_speed_ks`, which remains the active/effective low-speed value:

```c
    control_balance_diag.drive_fast_blend = 0.0f;
    control_balance_diag.drive_speed_integral = 0.0f;
    control_balance_diag.drive_speed_pitch_limit_deg = APP_CHASSIS_SPEED_PITCH_LIMIT_DEG;
    control_balance_diag.drive_speed_ff_rpm = 0.0f;
    control_balance_diag.pitch_term_rpm = 0.0f;
    control_balance_diag.rate_term_rpm = 0.0f;
    control_balance_diag.speed_term_rpm = 0.0f;
    control_balance_diag.pos_term_rpm = 0.0f;
    control_balance_diag.ff_term_rpm = 0.0f;
```

- [ ] **Step 5: Allow fast mode as an active balance mode**

Replace the active-mode check:

```c
if(BALANCE_MODE_BALANCE_TEST != control_balance_mode)
```

with:

```c
if((BALANCE_MODE_BALANCE_TEST != control_balance_mode) &&
   (BALANCE_MODE_BALANCE_FAST != control_balance_mode))
```

- [ ] **Step 6: Copy chassis fast diagnostics**

Near the existing drive diagnostic copies, add:

```c
    control_balance_diag.drive_fast_blend = chassis->fast_blend;
    control_balance_diag.drive_speed_integral = chassis->speed_integral;
    control_balance_diag.drive_speed_pitch_limit_deg = chassis->speed_pitch_limit_deg;
    control_balance_diag.drive_speed_ff_rpm = chassis->speed_ff_rpm;
```

- [ ] **Step 7: Replace monolithic balance formula with term diagnostics**

Declare:

```c
float effective_wheel_speed_ks;
float pitch_term_rpm;
float rate_term_rpm;
float speed_term_rpm;
float pos_term_rpm;
float ff_term_rpm;
```

Replace the current `balance_rpm = ...` block with:

```c
effective_wheel_speed_ks =
    control_balance_lerp(control_balance_wheel_speed_ks,
                         APP_BALANCE_FAST_WHEEL_SPEED_KS,
                         chassis->fast_blend);

pitch_term_rpm = control_balance_pitch_kp * (imu->pitch - pitch_setpoint_deg);
rate_term_rpm = control_balance_pitch_rate_kd * pitch_rate_dps;
speed_term_rpm = effective_wheel_speed_ks * wheel_speed_rpm;
pos_term_rpm = control_balance_wheel_pos_kp * control_balance_wheel_pos_rev;
ff_term_rpm = chassis->speed_ff_rpm;

balance_rpm = pitch_term_rpm +
              rate_term_rpm +
              speed_term_rpm +
              pos_term_rpm +
              ff_term_rpm;
```

Then keep the existing ident excitation and output clamp.

- [ ] **Step 8: Publish term diagnostics**

After finite checks and before final output publish, add:

```c
    control_balance_diag.wheel_speed_ks = effective_wheel_speed_ks;
    control_balance_diag.pitch_term_rpm = pitch_term_rpm;
    control_balance_diag.rate_term_rpm = rate_term_rpm;
    control_balance_diag.speed_term_rpm = speed_term_rpm;
    control_balance_diag.pos_term_rpm = pos_term_rpm;
    control_balance_diag.ff_term_rpm = ff_term_rpm;
```

- [ ] **Step 9: Extend finite checks**

Add all new terms to the finite guard:

```c
       (APP_FALSE == control_balance_is_finite(effective_wheel_speed_ks)) ||
       (APP_FALSE == control_balance_is_finite(pitch_term_rpm)) ||
       (APP_FALSE == control_balance_is_finite(rate_term_rpm)) ||
       (APP_FALSE == control_balance_is_finite(speed_term_rpm)) ||
       (APP_FALSE == control_balance_is_finite(pos_term_rpm)) ||
       (APP_FALSE == control_balance_is_finite(ff_term_rpm)) ||
```

- [ ] **Step 10: Run static check**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
```

Expected: PASS or fail on later host/telemetry checks.

- [ ] **Step 11: Commit**

```powershell
git add project/code/control_balance.c tools/test_balance_drive_v2_static.ps1
git commit -m "Add balance fast blend terms"
```

---

### Task 4: Add B,3 Host Command Semantics

**Files:**
- Modify: `project/code/host_command.c`
- Modify: `tools/test_balance_drive_v2_static.ps1`

- [ ] **Step 1: Add failing static checks**

Append:

```powershell
Assert-Contains "project/code/host_command.c" "BALANCE_MODE_BALANCE_FAST" "B,3 must enter fast balance mode."
Assert-Contains "project/code/host_command.c" "control_chassis_set_fast_enable\\(APP_TRUE\\)" "B,3 must enable chassis fast mode."
Assert-Contains "project/code/host_command.c" "control_chassis_set_fast_enable\\(APP_FALSE\\)" "Stop/low-speed paths must disable fast mode."
```

- [ ] **Step 2: Run static check and verify it fails**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
```

Expected: FAIL on missing fast host command.

- [ ] **Step 3: Disable fast mode on hard stop**

In `STOP` handling, before or after `control_chassis_stop(now_ms)`, add:

```c
        control_chassis_set_fast_enable(APP_FALSE);
```

- [ ] **Step 4: Disable fast mode for B,0/B,1/B,2**

In each existing mode branch, add:

```c
            control_chassis_set_fast_enable(APP_FALSE);
```

Use it for:

- `B,0`
- `B,1`
- `B,2`

- [ ] **Step 5: Add B,3**

After the `B,2` branch, add:

```c
        if(3.0f == value)
        {
            control_chassis_set_fast_enable(APP_TRUE);
            control_balance_set_mode(BALANCE_MODE_BALANCE_FAST);
            actuator_motor_record_command_error(APP_FALSE);
            return;
        }
```

- [ ] **Step 6: Run static check**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
```

Expected: PASS or fail on telemetry checks from later tasks.

- [ ] **Step 7: Commit**

```powershell
git add project/code/host_command.c tools/test_balance_drive_v2_static.ps1
git commit -m "Add fast balance host mode"
```

---

### Task 5: Expand Telemetry and Collector to 38 Floats

**Files:**
- Modify: `project/code/telemetry.c`
- Modify: `tools/collect_balance_data.ps1`
- Modify: `tools/test_collect_balance_data.ps1`
- Modify: `tools/test_balance_drive_v2_static.ps1`
- Modify: `tools/test_tune_drive_loops_static.ps1`
- Modify: `tools/tune_drive_loops.m`

- [ ] **Step 1: Add failing static checks**

Update existing checks:

```powershell
Assert-Contains "project/code/telemetry.c" 'float vofa_data\\[38\\]' "Telemetry must emit 38-channel fast diagnostics frame."
Assert-Contains "tools/collect_balance_data.ps1" '\\$FloatCount = 38' "Collector must parse 38-channel telemetry."
Assert-Contains "tools/collect_balance_data.ps1" "fast_blend" "Collector must write fast blend."
Assert-Contains "tools/collect_balance_data.ps1" "speed_ff_rpm" "Collector must write speed feedforward."
Assert-Contains "tools/collect_balance_data.ps1" "pitch_term_rpm" "Collector must write pitch term."
Assert-Contains "tools/collect_balance_data.ps1" "ff_term_rpm" "Collector must write feedforward term."
```

Remove or update older checks that require 28 floats.

- [ ] **Step 2: Run static checks and verify they fail**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_collect_balance_data.ps1
```

Expected: FAIL on 38-float expectations.

- [ ] **Step 3: Update firmware telemetry**

In `project/code/telemetry.c`, change:

```c
float vofa_data[28];
```

to:

```c
float vofa_data[38];
```

Append:

```c
    vofa_data[28] = balance->drive_fast_blend;
    vofa_data[29] = balance->drive_speed_integral;
    vofa_data[30] = balance->drive_speed_pitch_limit_deg;
    vofa_data[31] = balance->drive_speed_ff_rpm;
    vofa_data[32] = balance->wheel_speed_ks;
    vofa_data[33] = balance->pitch_term_rpm;
    vofa_data[34] = balance->rate_term_rpm;
    vofa_data[35] = balance->speed_term_rpm;
    vofa_data[36] = balance->pos_term_rpm;
    vofa_data[37] = balance->ff_term_rpm;
```

- [ ] **Step 4: Update collector**

In `tools/collect_balance_data.ps1`, set:

```powershell
$FloatCount = 38
```

Add frame fields with indexes 28-37:

```powershell
fast_blend = $values[28]
speed_integral = $values[29]
speed_pitch_limit_deg = $values[30]
speed_ff_rpm = $values[31]
wheel_speed_ks = $values[32]
pitch_term_rpm = $values[33]
rate_term_rpm = $values[34]
speed_term_rpm = $values[35]
pos_term_rpm = $values[36]
ff_term_rpm = $values[37]
```

Add matching CSV columns in the same order after `wheel_age_ms`.

- [ ] **Step 5: Update collector test**

In `tools/test_collect_balance_data.ps1`, build a 38-float test frame and assert the new fields:

```powershell
Assert-Near $frames[0].fast_blend 0.25 0.001 "fast_blend"
Assert-Near $frames[0].speed_integral 12.0 0.001 "speed_integral"
Assert-Near $frames[0].speed_pitch_limit_deg 9.0 0.001 "speed_pitch_limit_deg"
Assert-Near $frames[0].speed_ff_rpm 1.5 0.001 "speed_ff_rpm"
Assert-Near $frames[0].wheel_speed_ks 2.4 0.001 "wheel_speed_ks"
Assert-Near $frames[0].pitch_term_rpm -8.0 0.001 "pitch_term_rpm"
Assert-Near $frames[0].rate_term_rpm 3.0 0.001 "rate_term_rpm"
Assert-Near $frames[0].speed_term_rpm 20.0 0.001 "speed_term_rpm"
Assert-Near $frames[0].pos_term_rpm 0.0 0.001 "pos_term_rpm"
Assert-Near $frames[0].ff_term_rpm 1.5 0.001 "ff_term_rpm"
```

- [ ] **Step 6: Update MATLAB script metadata**

In `tools/tune_drive_loops.m`, update comments and any fixed telemetry count references from 28 to 38. Keep existing speed/turn analysis working by using column names, not fixed indexes.

- [ ] **Step 7: Run tests**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_collect_balance_data.ps1
powershell -ExecutionPolicy Bypass -File tools/test_tune_drive_loops_static.ps1
```

Expected: all PASS.

- [ ] **Step 8: Commit**

```powershell
git add project/code/telemetry.c tools/collect_balance_data.ps1 tools/test_collect_balance_data.ps1 tools/test_balance_drive_v2_static.ps1 tools/test_tune_drive_loops_static.ps1 tools/tune_drive_loops.m
git commit -m "Expose fast balance telemetry"
```

---

### Task 6: Final Verification and Hardware Handoff

**Files:**
- No source changes unless earlier tests expose issues.

- [ ] **Step 1: Run full static verification**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_collect_balance_data.ps1
powershell -ExecutionPolicy Bypass -File tools/test_tune_drive_loops_static.ps1
git diff --check
```

Expected:

```text
balance drive v2 static checks passed
collect_balance_data tests passed
tune_drive_loops static checks passed
```

`git diff --check` must report no whitespace errors.

- [ ] **Step 2: IAR build**

Build these projects in IAR Embedded Workbench:

```text
project/iar/cyt4bb7.eww
cyt4bb7_cm_0_plus
cyt4bb7_cm_7_0
cyt4bb7_cm_7_1
```

Expected: all compile and link. If this environment cannot run IAR, report `IAR build not run in this environment`.

- [ ] **Step 3: Hardware smoke test without IMU_ZERO**

Use staged captures only after the IAR build succeeds. Do not add `IMU_ZERO`.

40 RPM:

```powershell
powershell -ExecutionPolicy Bypass -File tools/collect_balance_data.ps1 -Port COM6 -Duration 12 -Commands "0:STOP;0.3:BD,0.3,0,1.5;0.6:B,3;1.0:C,0,0;2.0:C,40,0;7.0:C,0,0;11:STOP" -Note "fast_b3_40_ff0"
```

70 RPM:

```powershell
powershell -ExecutionPolicy Bypass -File tools/collect_balance_data.ps1 -Port COM6 -Duration 12 -Commands "0:STOP;0.3:BD,0.3,0,1.5;0.6:B,3;1.0:C,0,0;2.0:C,70,0;7.0:C,0,0;11:STOP" -Note "fast_b3_70_ff0"
```

100 RPM:

```powershell
powershell -ExecutionPolicy Bypass -File tools/collect_balance_data.ps1 -Port COM6 -Duration 12 -Commands "0:STOP;0.3:BD,0.3,0,1.5;0.6:B,3;1.0:C,0,0;2.0:C,100,0;7.0:C,0,0;11:STOP" -Note "fast_b3_100_ff0"
```

- [ ] **Step 4: Acceptance review**

Inspect each CSV for:

```text
feedback_online remains 1 during active step
imu_age_ms <= 15
wheel_age_ms <= 30
fast_blend ramps smoothly
speed_pitch_limit_deg rises smoothly from 8 toward 12
wheel_speed_ks decreases smoothly from 3 toward 0.8
speed_ff_rpm remains 0 with default config
pitch_term_rpm, rate_term_rpm, speed_term_rpm are finite
balance_rpm does not sit at +/-300 saturation
pitch_deg stays below the existing safety threshold
```

- [ ] **Step 5: Commit final test notes if documentation was updated**

If a hardware note file is added, commit it:

```powershell
git add docs/superpowers/specs/2026-07-03-balance-fast-blend-design.md docs/superpowers/plans/2026-07-03-balance-fast-blend.md
git commit -m "Document fast balance validation"
```

If no files changed in this task, do not create an empty commit.

## Merge Gate

Do not merge back to `main` until:

- all static checks pass;
- IAR build status is known;
- at least 40 RPM and 70 RPM captures have been reviewed;
- 100 RPM is either validated or explicitly deferred;
- `B,2` low-speed behavior is confirmed unchanged with `fast_blend = 0`.

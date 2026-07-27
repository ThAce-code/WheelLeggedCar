# IK Height Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add direct five-bar inverse-kinematics height control so the robot can command continuous body height and keep balance/chassis behavior height-aware without changing `C,forward_rpm,turn_dps` semantics.

**Architecture:** `leg_kinematics` owns five-bar IK math and validation. `control_leg` owns height command state, ramping, IK invocation, servo targets, and diagnostics. `control_balance` and `control_chassis` read leg diagnostics to interpolate effective gains and speed limits while `actuator_servo` remains the only PWM output owner.

**Tech Stack:** Embedded C for CYT4BB7/Traveo II, IAR Embedded Workbench, PowerShell static tests and telemetry collector tests.

---

## Mandatory DeepSeek Setup

Before implementation, DeepSeek must:

1. Work in:

   ```text
   D:\smartcar\WheelLeggedCar_cyt4bb7_v1\.worktree\leg-control-speed-assist
   ```

2. Confirm the branch is:

   ```text
   codex/leg-control-speed-assist
   ```

3. State that it is executing:

   ```text
   docs/superpowers/plans/2026-07-04-ik-height-control.md
   ```

4. Work task-by-task.
5. Run the task's listed checks before each commit.
6. Make one commit after each implementation task.
7. Report IAR build and hardware test status at the end.

Do not modify `libraries/*` or generated IAR output directories.

## Scope

Implement:

- five-bar IK module for `(x_mm, y_mm) -> servo angles`;
- `LIK,a0,a1,a2,a3` direct servo calibration command;
- `LH,height_mm` continuous height command;
- `LEG_MODE_IK_CALIB` and `LEG_MODE_HEIGHT`;
- leg diagnostics and 58-float telemetry;
- height-based balance gain/setpoint scheduling;
- height-based chassis forward-limit scheduling;
- scheduler order change so leg diagnostics are updated before chassis and balance.

Do not implement:

- roll terrain adaptation;
- P9 fore-aft `x` speed assist;
- jump control;
- persistent flash parameter storage;
- any change to `C,forward_rpm,turn_dps`;
- any BLDC protocol or motor PID rewrite.

## File Structure

Create:

- `project/code/leg_kinematics.h`: public IK result type and solve/forward-check APIs.
- `project/code/leg_kinematics.c`: five-bar IK math, branch selection, workspace and servo limit validation.
- `tools/test_ik_height_control_static.ps1`: static checks for IK height control.

Modify:

- `project/code/app_config.h`: add leg height timing, transition speed limit, and telemetry count constants if local style supports them.
- `project/code/app_types.h`: add `leg_diag_struct`.
- `project/code/leg_config.h`: add IK branch enum, kinematics config, height profile, and accessors.
- `project/code/leg_config.c`: add conservative measured-default kinematics/profile constants.
- `project/code/control_leg.h`: add height/calibration APIs and `control_leg_get_diag()`.
- `project/code/control_leg.c`: replace attitude height path with IK-backed height mode and diagnostics.
- `project/code/host_command.c`: parse `LH` and `LIK`.
- `project/code/app_scheduler.c`: run `control_leg_update()` before `control_chassis_update()` and `control_balance_update()`.
- `project/code/control_balance.c`: read leg diagnostics and interpolate effective balance terms.
- `project/code/control_chassis.c`: read leg diagnostics and interpolate forward limits.
- `project/code/telemetry.c`: expand balance telemetry from 38 to 58 floats.
- `tools/collect_balance_data.ps1`: parse/write 58-float telemetry.
- `tools/test_collect_balance_data.ps1`: validate the 58-float frame and leg columns.
- `tools/test_balance_drive_v2_static.ps1`: preserve existing checks and optionally assert unchanged `C` command semantics.

---

### Task 1: Add IK Height Static Validation

**Files:**
- Create: `tools/test_ik_height_control_static.ps1`

- [ ] **Step 1: Create the failing static test**

Create `tools/test_ik_height_control_static.ps1`:

```powershell
$ErrorActionPreference = "Stop"

function Assert-Contains {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if(-not (Test-Path $Path)) {
        throw ("Missing file: {0}" -f $Path)
    }
    $text = Get-Content $Path -Raw
    if($text -notmatch $Pattern) {
        throw $Message
    }
}

function Assert-NotContains {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if((Test-Path $Path) -and ((Get-Content $Path -Raw) -match $Pattern)) {
        throw $Message
    }
}

Assert-Contains "project/code/leg_kinematics.h" "leg_kinematics_solve" "Missing IK solve API."
Assert-Contains "project/code/leg_kinematics.h" "leg_kinematics_forward" "Missing IK forward-check API."
Assert-Contains "project/code/leg_kinematics.c" "sqrt" "IK implementation must solve five-bar geometry."
Assert-Contains "project/code/leg_kinematics.c" "LEG_IK_BRANCH_PLUS" "IK implementation must use configured branches."
Assert-Contains "project/code/leg_kinematics.c" "x_min_mm" "IK must validate x workspace."
Assert-Contains "project/code/leg_kinematics.c" "y_min_mm" "IK must validate y workspace."

Assert-Contains "project/code/leg_config.h" "leg_kinematics_config_struct" "Missing kinematics config struct."
Assert-Contains "project/code/leg_config.h" "leg_height_profile_struct" "Missing height profile struct."
Assert-Contains "project/code/leg_config.h" "LEG_IK_BRANCH_PLUS" "Missing IK branch enum."
Assert-Contains "project/code/leg_config.c" "l1_mm" "Missing configured link length L1."
Assert-Contains "project/code/leg_config.c" "default_height_mm" "Missing default height config."

Assert-Contains "project/code/app_types.h" "leg_diag_struct" "Missing leg diagnostics."
Assert-Contains "project/code/app_types.h" "target_height_mm" "Leg diagnostics must expose target height."
Assert-Contains "project/code/app_types.h" "actual_height_mm" "Leg diagnostics must expose actual height."
Assert-Contains "project/code/app_types.h" "height_norm" "Leg diagnostics must expose normalized height."
Assert-Contains "project/code/app_types.h" "ik_valid" "Leg diagnostics must expose IK validity."

Assert-Contains "project/code/control_leg.h" "control_leg_set_height" "Missing height command API."
Assert-Contains "project/code/control_leg.h" "control_leg_set_calib_angles" "Missing direct calibration API."
Assert-Contains "project/code/control_leg.h" "control_leg_get_diag" "Missing leg diagnostics getter."
Assert-Contains "project/code/control_leg.c" "LEG_MODE_HEIGHT" "Control leg must implement height mode."
Assert-Contains "project/code/control_leg.c" "LEG_MODE_IK_CALIB" "Control leg must implement IK calibration mode."
Assert-Contains "project/code/control_leg.c" "leg_kinematics_solve" "Control leg must call IK solver."
Assert-NotContains "project/code/control_leg.c" "height \\+ \\(pitch \\* servo_cfg->mount_x\\)" "Height mode must not use old direct servo mixer."

Assert-Contains "project/code/host_command.c" "'L' == line\\[0\\].*'H' == line\\[1\\]" "Host command must parse LH."
Assert-Contains "project/code/host_command.c" "'L' == line\\[0\\].*'I' == line\\[1\\].*'K' == line\\[2\\]" "Host command must parse LIK."
Assert-Contains "project/code/app_scheduler.c" "control_leg_update\\(now_ms\\).*control_chassis_update\\(now_ms\\).*control_balance_update\\(now_ms\\)" "Scheduler must update leg before chassis and balance."

Assert-Contains "project/code/control_balance.c" "control_leg_get_diag" "Balance must read leg height diagnostics."
Assert-Contains "project/code/control_balance.c" "balance_pitch_kp_low" "Balance must use height profile low gain."
Assert-Contains "project/code/control_balance.c" "balance_pitch_kp_high" "Balance must use height profile high gain."
Assert-Contains "project/code/control_chassis.c" "control_leg_get_diag" "Chassis must read leg height diagnostics."
Assert-Contains "project/code/control_chassis.c" "chassis_forward_limit_low_rpm" "Chassis must use height profile forward limits."

Assert-Contains "project/code/telemetry.c" "float vofa_data\\[58\\]" "Telemetry must emit 58 floats."
Assert-Contains "tools/collect_balance_data.ps1" "\\$FloatCount = 58" "Collector must parse 58 floats."
Assert-Contains "tools/collect_balance_data.ps1" "leg_actual_height_mm" "Collector must write leg height."
Assert-Contains "tools/collect_balance_data.ps1" "balance_pitch_kp_eff" "Collector must write effective balance gain."
Assert-Contains "tools/collect_balance_data.ps1" "chassis_forward_limit_eff_rpm" "Collector must write effective chassis limit."

Write-Host "ik height control static checks passed"
```

- [ ] **Step 2: Run the static test and verify it fails**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_ik_height_control_static.ps1
```

Expected: FAIL with `Missing file: project/code/leg_kinematics.h`.

- [ ] **Step 3: Commit**

```powershell
git add tools/test_ik_height_control_static.ps1
git commit -m "Add IK height control static validation"
```

---

### Task 2: Add IK and Height Types

**Files:**
- Modify: `project/code/app_types.h`
- Modify: `project/code/leg_config.h`
- Modify: `project/code/leg_config.c`
- Test: `tools/test_ik_height_control_static.ps1`

- [ ] **Step 1: Add leg diagnostics to `app_types.h`**

Add after `servo_cmd_struct`:

```c
typedef struct
{
    float target_height_mm;
    float actual_height_mm;
    float height_norm;
    float left_x_mm;
    float left_y_mm;
    float right_x_mm;
    float right_y_mm;
    float servo_target_deg[4];
    float servo_actual_deg[4];
    uint8 mode;
    uint8 ik_valid;
    uint8 output_enable;
    uint32 ik_error_count;
}leg_diag_struct;
```

- [ ] **Step 2: Extend `leg_config.h`**

Replace the current file content after the existing `leg_servo_config_struct` with these added types and accessors while preserving the existing servo fields:

```c
typedef enum
{
    LEG_IK_BRANCH_PLUS = 0,
    LEG_IK_BRANCH_MINUS = 1
}leg_ik_branch_enum;

typedef struct
{
    float l1_mm;
    float l2_mm;
    float l3_mm;
    float l4_mm;
    float l5_mm;
    float x_min_mm;
    float x_max_mm;
    float y_min_mm;
    float y_max_mm;
    float x_offset_mm;
    float y_offset_mm;
    leg_ik_branch_enum left_alpha_branch;
    leg_ik_branch_enum left_beta_branch;
    leg_ik_branch_enum right_alpha_branch;
    leg_ik_branch_enum right_beta_branch;
}leg_kinematics_config_struct;

typedef struct
{
    float low_height_mm;
    float high_height_mm;
    float default_height_mm;
    float max_height_speed_mm_s;
    float transition_forward_limit_rpm;
    float balance_pitch_kp_low;
    float balance_pitch_kp_high;
    float balance_pitch_rate_kd_low;
    float balance_pitch_rate_kd_high;
    float balance_wheel_speed_ks_low;
    float balance_wheel_speed_ks_high;
    float balance_pitch_setpoint_low_deg;
    float balance_pitch_setpoint_high_deg;
    float chassis_forward_limit_low_rpm;
    float chassis_forward_limit_high_rpm;
    float chassis_fast_forward_limit_low_rpm;
    float chassis_fast_forward_limit_high_rpm;
}leg_height_profile_struct;
```

Update `leg_config_struct`:

```c
typedef struct
{
    leg_servo_config_struct servo[LEG_SERVO_COUNT];
    leg_kinematics_config_struct kinematics;
    leg_height_profile_struct height_profile;
    float  height_min;
    float  height_max;
    float  pitch_limit;
    float  roll_limit;
}leg_config_struct;
```

Add accessors:

```c
const leg_kinematics_config_struct *leg_config_get_kinematics(void);
const leg_height_profile_struct *leg_config_get_height_profile(void);
```

- [ ] **Step 3: Add conservative config in `leg_config.c`**

Use measured values if available before implementation. If measured values are not available when DeepSeek executes, use these safe initial values and mark the hardware validation as calibration-required in the final report:

```c
static const leg_config_struct leg_config_default =
{
    {
        {0,  90.0f,  90.0f, 15.0f, 165.0f,  1.0f,  1.0f,  1.0f},
        {1,  90.0f,  90.0f, 15.0f, 165.0f, -1.0f,  1.0f, -1.0f},
        {2,  90.0f,  90.0f, 15.0f, 165.0f, -1.0f, -1.0f,  1.0f},
        {3,  90.0f,  90.0f, 15.0f, 165.0f,  1.0f, -1.0f, -1.0f}
    },
    {
        55.0f,
        90.0f,
        90.0f,
        55.0f,
        70.0f,
        -35.0f,
        35.0f,
        70.0f,
        145.0f,
        0.0f,
        0.0f,
        LEG_IK_BRANCH_PLUS,
        LEG_IK_BRANCH_MINUS,
        LEG_IK_BRANCH_PLUS,
        LEG_IK_BRANCH_MINUS
    },
    {
        80.0f,
        130.0f,
        100.0f,
        60.0f,
        30.0f,
        18.0f,
        22.0f,
        8.0f,
        10.0f,
        3.0f,
        2.0f,
        -1.35f,
        -1.35f,
        80.0f,
        40.0f,
        220.0f,
        120.0f
    },
    80.0f,
    130.0f,
    30.0f,
    30.0f
};
```

Add accessors:

```c
const leg_kinematics_config_struct *leg_config_get_kinematics(void)
{
    return &leg_config_default.kinematics;
}

const leg_height_profile_struct *leg_config_get_height_profile(void)
{
    return &leg_config_default.height_profile;
}
```

- [ ] **Step 4: Run the static test**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_ik_height_control_static.ps1
```

Expected: FAIL on missing `leg_kinematics.h`.

- [ ] **Step 5: Run whitespace check**

```powershell
git diff --check -- project/code/app_types.h project/code/leg_config.h project/code/leg_config.c
```

Expected: no output.

- [ ] **Step 6: Commit**

```powershell
git add project/code/app_types.h project/code/leg_config.h project/code/leg_config.c
git commit -m "Add IK height configuration types"
```

---

### Task 3: Add Five-Bar IK Module

**Files:**
- Create: `project/code/leg_kinematics.h`
- Create: `project/code/leg_kinematics.c`
- Test: `tools/test_ik_height_control_static.ps1`

- [ ] **Step 1: Create `leg_kinematics.h`**

```c
/*********************************************************************************************************************
* File: leg_kinematics.h
* Description: Five-bar wheel-leg inverse kinematics.
********************************************************************************************************************/

#ifndef _leg_kinematics_h_
#define _leg_kinematics_h_

#include "app_types.h"
#include "leg_config.h"

typedef struct
{
    float servo_deg[2];
    float alpha_rad;
    float beta_rad;
    uint8 valid;
}leg_ik_result_struct;

uint8 leg_kinematics_solve(uint8 right_side,
                           float x_mm,
                           float y_mm,
                           leg_ik_result_struct *result);
uint8 leg_kinematics_forward(uint8 right_side,
                             float servo_a_deg,
                             float servo_b_deg,
                             float *x_mm,
                             float *y_mm);

#endif
```

- [ ] **Step 2: Create `leg_kinematics.c`**

Implement these exact helpers and APIs:

```c
/*********************************************************************************************************************
* File: leg_kinematics.c
* Description: Five-bar wheel-leg inverse kinematics.
********************************************************************************************************************/

#include "leg_kinematics.h"
#include "app_config.h"
#include <math.h>

#define LEG_KINEMATICS_PI        (3.14159265358979323846f)
#define LEG_KINEMATICS_TWO_PI    (6.28318530717958647692f)
#define LEG_KINEMATICS_EPS       (0.000001f)

static float leg_kinematics_absf(float value)
{
    return (0.0f > value) ? -value : value;
}

static uint8 leg_kinematics_is_finite(float value)
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

static float leg_kinematics_wrap_positive(float angle_rad)
{
    while(0.0f > angle_rad)
    {
        angle_rad += LEG_KINEMATICS_TWO_PI;
    }
    while(LEG_KINEMATICS_TWO_PI <= angle_rad)
    {
        angle_rad -= LEG_KINEMATICS_TWO_PI;
    }
    return angle_rad;
}

static uint8 leg_kinematics_pick_branch(float plus_value,
                                        float minus_value,
                                        leg_ik_branch_enum branch,
                                        float *selected)
{
    *selected = (LEG_IK_BRANCH_PLUS == branch) ? plus_value : minus_value;
    return leg_kinematics_is_finite(*selected);
}

static uint8 leg_kinematics_solve_angle(float a,
                                        float b,
                                        float c,
                                        leg_ik_branch_enum branch,
                                        float *angle_rad)
{
    float disc;
    float root;
    float denom;
    float plus_value;
    float minus_value;
    float selected;

    disc = (a * a) + (b * b) - (c * c);
    if(0.0f > disc)
    {
        return APP_FALSE;
    }

    root = sqrtf(disc);
    denom = a + c;
    if(LEG_KINEMATICS_EPS > leg_kinematics_absf(denom))
    {
        return APP_FALSE;
    }

    plus_value = 2.0f * atanf((b + root) / denom);
    minus_value = 2.0f * atanf((b - root) / denom);

    if(APP_FALSE == leg_kinematics_pick_branch(plus_value, minus_value, branch, &selected))
    {
        return APP_FALSE;
    }

    *angle_rad = leg_kinematics_wrap_positive(selected);
    return APP_TRUE;
}

static float leg_kinematics_rad_to_deg(float angle_rad)
{
    return angle_rad * 180.0f / LEG_KINEMATICS_PI;
}

static uint8 leg_kinematics_servo_valid(uint8 servo_index, float angle_deg)
{
    const leg_servo_config_struct *servo_cfg;

    servo_cfg = leg_config_get_servo(servo_index);
    if(NULL == servo_cfg)
    {
        return APP_FALSE;
    }
    if((servo_cfg->min_deg > angle_deg) || (servo_cfg->max_deg < angle_deg))
    {
        return APP_FALSE;
    }
    return leg_kinematics_is_finite(angle_deg);
}

static uint8 leg_kinematics_workspace_valid(const leg_kinematics_config_struct *cfg,
                                            float x_mm,
                                            float y_mm)
{
    if((cfg->x_min_mm > x_mm) || (cfg->x_max_mm < x_mm))
    {
        return APP_FALSE;
    }
    if((cfg->y_min_mm > y_mm) || (cfg->y_max_mm < y_mm))
    {
        return APP_FALSE;
    }
    return APP_TRUE;
}

uint8 leg_kinematics_solve(uint8 right_side,
                           float x_mm,
                           float y_mm,
                           leg_ik_result_struct *result)
{
    const leg_kinematics_config_struct *cfg;
    uint8 servo_a;
    uint8 servo_b;
    float x;
    float y;
    float a;
    float b;
    float c;
    float d;
    float e;
    float f;
    float alpha_rad;
    float beta_rad;
    float alpha_deg;
    float beta_deg;
    leg_ik_branch_enum alpha_branch;
    leg_ik_branch_enum beta_branch;

    if(NULL == result)
    {
        return APP_FALSE;
    }

    result->servo_deg[0] = 0.0f;
    result->servo_deg[1] = 0.0f;
    result->alpha_rad = 0.0f;
    result->beta_rad = 0.0f;
    result->valid = APP_FALSE;

    cfg = leg_config_get_kinematics();
    x = x_mm + cfg->x_offset_mm;
    y = y_mm + cfg->y_offset_mm;

    if((APP_FALSE == leg_kinematics_workspace_valid(cfg, x, y)) ||
       (APP_FALSE == leg_kinematics_is_finite(x)) ||
       (APP_FALSE == leg_kinematics_is_finite(y)))
    {
        return APP_FALSE;
    }

    a = 2.0f * x * cfg->l1_mm;
    b = 2.0f * y * cfg->l1_mm;
    c = (x * x) + (y * y) + (cfg->l1_mm * cfg->l1_mm) - (cfg->l2_mm * cfg->l2_mm);
    d = 2.0f * (x - cfg->l5_mm) * cfg->l4_mm;
    e = 2.0f * y * cfg->l4_mm;
    f = ((x - cfg->l5_mm) * (x - cfg->l5_mm)) + (y * y) +
        (cfg->l4_mm * cfg->l4_mm) - (cfg->l3_mm * cfg->l3_mm);

    alpha_branch = (APP_TRUE == right_side) ? cfg->right_alpha_branch : cfg->left_alpha_branch;
    beta_branch = (APP_TRUE == right_side) ? cfg->right_beta_branch : cfg->left_beta_branch;

    if((APP_FALSE == leg_kinematics_solve_angle(a, b, c, alpha_branch, &alpha_rad)) ||
       (APP_FALSE == leg_kinematics_solve_angle(d, e, f, beta_branch, &beta_rad)))
    {
        return APP_FALSE;
    }

    alpha_deg = leg_kinematics_rad_to_deg(alpha_rad);
    beta_deg = leg_kinematics_rad_to_deg(beta_rad);

    if(APP_TRUE == right_side)
    {
        servo_a = LEG_SERVO_FR;
        servo_b = LEG_SERVO_RR;
    }
    else
    {
        servo_a = LEG_SERVO_FL;
        servo_b = LEG_SERVO_RL;
    }

    if((APP_FALSE == leg_kinematics_servo_valid(servo_a, alpha_deg)) ||
       (APP_FALSE == leg_kinematics_servo_valid(servo_b, beta_deg)))
    {
        return APP_FALSE;
    }

    result->servo_deg[0] = alpha_deg;
    result->servo_deg[1] = beta_deg;
    result->alpha_rad = alpha_rad;
    result->beta_rad = beta_rad;
    result->valid = APP_TRUE;
    return APP_TRUE;
}

uint8 leg_kinematics_forward(uint8 right_side,
                             float servo_a_deg,
                             float servo_b_deg,
                             float *x_mm,
                             float *y_mm)
{
    (void)right_side;
    (void)servo_a_deg;
    (void)servo_b_deg;
    if((NULL == x_mm) || (NULL == y_mm))
    {
        return APP_FALSE;
    }
    *x_mm = 0.0f;
    *y_mm = 0.0f;
    return APP_TRUE;
}
```

The forward function is deliberately a safe stub in this task because the first firmware path only needs inverse solving. Calibration data collection uses measured `(x, y)` outside firmware. A later plan can replace this with full FK when needed.

- [ ] **Step 3: Run static test**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_ik_height_control_static.ps1
```

Expected: FAIL on missing `control_leg` height integration.

- [ ] **Step 4: Run whitespace check**

```powershell
git diff --check -- project/code/leg_kinematics.h project/code/leg_kinematics.c
```

Expected: no output.

- [ ] **Step 5: Commit**

```powershell
git add project/code/leg_kinematics.h project/code/leg_kinematics.c
git commit -m "Add five-bar leg IK module"
```

---

### Task 4: Implement IK Height Mode in `control_leg`

**Files:**
- Modify: `project/code/control_leg.h`
- Modify: `project/code/control_leg.c`
- Test: `tools/test_ik_height_control_static.ps1`

- [ ] **Step 1: Update `control_leg.h`**

Replace the mode enum with:

```c
typedef enum
{
    LEG_MODE_LOCK = 0,
    LEG_MODE_MANUAL,
    LEG_MODE_IK_CALIB,
    LEG_MODE_HEIGHT
}leg_mode_enum;
```

Add APIs:

```c
uint8 control_leg_set_height(float height_mm, uint32 now_ms);
uint8 control_leg_set_calib_angles(float servo0_deg,
                                   float servo1_deg,
                                   float servo2_deg,
                                   float servo3_deg);
const leg_diag_struct *control_leg_get_diag(void);
```

Keep `control_leg_set_body_cmd()` as a compatibility wrapper that maps `height_cmd` to `control_leg_set_height()` when possible.

- [ ] **Step 2: Update `control_leg.c` includes and state**

Add include:

```c
#include "leg_kinematics.h"
```

Add file-scope state:

```c
static leg_diag_struct control_leg_diag;
static float control_leg_target_height_mm;
static float control_leg_actual_height_mm;
static uint32 control_leg_last_update_ms;
```

- [ ] **Step 3: Add finite and ramp helpers**

```c
static uint8 control_leg_is_finite(float value)
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

static float control_leg_absf(float value)
{
    return (0.0f > value) ? -value : value;
}

static float control_leg_ramp_toward(float current, float target, float max_delta)
{
    float delta;

    delta = target - current;
    if(max_delta >= control_leg_absf(delta))
    {
        return target;
    }
    return current + ((0.0f < delta) ? max_delta : -max_delta);
}
```

- [ ] **Step 4: Add diagnostics publisher**

```c
static void control_leg_publish_diag(uint8 ik_valid, uint8 output_enable)
{
    uint8 i;
    const leg_height_profile_struct *profile;

    profile = leg_config_get_height_profile();
    control_leg_diag.target_height_mm = control_leg_target_height_mm;
    control_leg_diag.actual_height_mm = control_leg_actual_height_mm;
    if(profile->high_height_mm > profile->low_height_mm)
    {
        control_leg_diag.height_norm =
            (control_leg_actual_height_mm - profile->low_height_mm) /
            (profile->high_height_mm - profile->low_height_mm);
    }
    else
    {
        control_leg_diag.height_norm = 0.0f;
    }
    control_leg_diag.height_norm = control_leg_clamp(control_leg_diag.height_norm, 0.0f, 1.0f);
    control_leg_diag.mode = (uint8)control_leg_mode;
    control_leg_diag.ik_valid = ik_valid;
    control_leg_diag.output_enable = output_enable;
    for(i = 0; i < APP_SERVO_COUNT; i++)
    {
        control_leg_diag.servo_target_deg[i] = control_leg_servo_cmd.angle_deg[i];
        control_leg_diag.servo_actual_deg[i] = control_leg_servo_cmd.angle_deg[i];
    }
}
```

- [ ] **Step 5: Initialize height state**

In `control_leg_init()`, after loading `config`:

```c
const leg_height_profile_struct *profile;
profile = leg_config_get_height_profile();
control_leg_target_height_mm = profile->default_height_mm;
control_leg_actual_height_mm = profile->default_height_mm;
control_leg_last_update_ms = 0U;
control_leg_diag.ik_error_count = 0U;
control_leg_diag.left_x_mm = 0.0f;
control_leg_diag.right_x_mm = 0.0f;
control_leg_diag.left_y_mm = control_leg_actual_height_mm;
control_leg_diag.right_y_mm = control_leg_actual_height_mm;
```

Call:

```c
control_leg_publish_diag(APP_FALSE, APP_FALSE);
```

- [ ] **Step 6: Replace height-mode body of `control_leg_update()`**

In `control_leg_update()`, compute `dt_s` from `control_leg_last_update_ms`. Then implement height mode:

```c
case LEG_MODE_HEIGHT:
{
    leg_ik_result_struct left_ik;
    leg_ik_result_struct right_ik;
    const leg_height_profile_struct *profile;
    float max_delta;
    uint8 output_enable;

    profile = leg_config_get_height_profile();
    max_delta = profile->max_height_speed_mm_s * dt_s;
    control_leg_actual_height_mm =
        control_leg_ramp_toward(control_leg_actual_height_mm,
                                control_leg_target_height_mm,
                                max_delta);

    control_leg_diag.left_x_mm = 0.0f;
    control_leg_diag.right_x_mm = 0.0f;
    control_leg_diag.left_y_mm = control_leg_actual_height_mm;
    control_leg_diag.right_y_mm = control_leg_actual_height_mm;

    if((APP_TRUE == leg_kinematics_solve(APP_FALSE, 0.0f, control_leg_actual_height_mm, &left_ik)) &&
       (APP_TRUE == leg_kinematics_solve(APP_TRUE, 0.0f, control_leg_actual_height_mm, &right_ik)))
    {
        control_leg_servo_cmd.angle_deg[LEG_SERVO_FL] = left_ik.servo_deg[0];
        control_leg_servo_cmd.angle_deg[LEG_SERVO_RL] = left_ik.servo_deg[1];
        control_leg_servo_cmd.angle_deg[LEG_SERVO_FR] = right_ik.servo_deg[0];
        control_leg_servo_cmd.angle_deg[LEG_SERVO_RR] = right_ik.servo_deg[1];
        output_enable = control_leg_run_enabled();
        control_leg_publish_diag(APP_TRUE, output_enable);
    }
    else
    {
        control_leg_diag.ik_error_count++;
        control_leg_mode = LEG_MODE_LOCK;
        output_enable = APP_FALSE;
        control_leg_publish_diag(APP_FALSE, output_enable);
    }
    break;
}
```

Remove the old direct target expression from the height mode path:

```c
mix = height + (pitch * servo_cfg->mount_x) + (roll * servo_cfg->mount_y);
```

`LEG_MODE_MANUAL` and `LEG_MODE_LOCK` remain available.

- [ ] **Step 7: Add setters and getter**

```c
uint8 control_leg_set_height(float height_mm, uint32 now_ms)
{
    const leg_height_profile_struct *profile;

    (void)now_ms;
    profile = leg_config_get_height_profile();
    if((APP_FALSE == control_leg_is_finite(height_mm)) ||
       (profile->low_height_mm > height_mm) ||
       (profile->high_height_mm < height_mm))
    {
        return APP_FALSE;
    }

    control_leg_target_height_mm = height_mm;
    control_leg_mode = LEG_MODE_HEIGHT;
    return APP_TRUE;
}

uint8 control_leg_set_calib_angles(float servo0_deg,
                                   float servo1_deg,
                                   float servo2_deg,
                                   float servo3_deg)
{
    control_leg_set_manual_angle(0U, servo0_deg);
    control_leg_set_manual_angle(1U, servo1_deg);
    control_leg_set_manual_angle(2U, servo2_deg);
    control_leg_set_manual_angle(3U, servo3_deg);
    control_leg_servo_cmd.angle_deg[0] = servo0_deg;
    control_leg_servo_cmd.angle_deg[1] = servo1_deg;
    control_leg_servo_cmd.angle_deg[2] = servo2_deg;
    control_leg_servo_cmd.angle_deg[3] = servo3_deg;
    control_leg_mode = LEG_MODE_IK_CALIB;
    control_leg_publish_diag(APP_FALSE, control_leg_run_enabled());
    return APP_TRUE;
}

const leg_diag_struct *control_leg_get_diag(void)
{
    return &control_leg_diag;
}
```

- [ ] **Step 8: Run static test**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_ik_height_control_static.ps1
```

Expected: FAIL on missing host command and scheduler integration.

- [ ] **Step 9: Run whitespace check**

```powershell
git diff --check -- project/code/control_leg.h project/code/control_leg.c
```

Expected: no output.

- [ ] **Step 10: Commit**

```powershell
git add project/code/control_leg.h project/code/control_leg.c
git commit -m "Add IK-backed leg height mode"
```

---

### Task 5: Add Height Commands and Scheduler Order

**Files:**
- Modify: `project/code/host_command.c`
- Modify: `project/code/app_scheduler.c`
- Test: `tools/test_ik_height_control_static.ps1`

- [ ] **Step 1: Add four-number parser if not reusable**

`host_command_parse_four_numbers()` already exists. Reuse it for `LIK`.

- [ ] **Step 2: Add `LH,height_mm` branch**

In `host_command_process_line()`, before balance `B` parsing:

```c
    if(('L' == line[0]) && ('H' == line[1]) && (',' == line[2]) &&
       (APP_TRUE == host_command_parse_number(&line[3], &value)))
    {
        if(APP_TRUE == control_leg_set_height(value, now_ms))
        {
            actuator_motor_record_command_error(APP_FALSE);
            return;
        }
    }
```

- [ ] **Step 3: Add `LIK,a0,a1,a2,a3` branch**

Declare a fourth float near existing locals:

```c
float fourth;
```

Add:

```c
    if(('L' == line[0]) && ('I' == line[1]) && ('K' == line[2]) && (',' == line[3]) &&
       (APP_TRUE == host_command_parse_four_numbers(&line[4], &kp, &ki, &kd, &fourth)))
    {
        if(APP_TRUE == control_leg_set_calib_angles(kp, ki, kd, fourth))
        {
            actuator_motor_record_command_error(APP_FALSE);
            return;
        }
    }
```

- [ ] **Step 4: Lock legs on `STOP`**

In the `STOP` branch, add:

```c
        control_leg_set_mode(LEG_MODE_LOCK);
```

- [ ] **Step 5: Reorder scheduler**

Move the leg update block before chassis and balance:

```c
    if(APP_TRUE == app_task_elapsed(now_ms, &leg_last_ms, APP_LEG_CONTROL_PERIOD_MS))
    {
        control_leg_update(now_ms);
    }

    if(APP_TRUE == app_task_elapsed(now_ms, &chassis_last_ms, APP_CHASSIS_PERIOD_MS))
    {
        control_chassis_update(now_ms);
    }

    if(APP_TRUE == app_task_elapsed(now_ms, &balance_last_ms, APP_BALANCE_PERIOD_MS))
    {
        control_balance_update(now_ms);
    }
```

Keep `actuator_servo_update()` after telemetry and motor updates.

- [ ] **Step 6: Run static test**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_ik_height_control_static.ps1
```

Expected: FAIL on missing balance/chassis integration.

- [ ] **Step 7: Run whitespace check**

```powershell
git diff --check -- project/code/host_command.c project/code/app_scheduler.c
```

Expected: no output.

- [ ] **Step 8: Commit**

```powershell
git add project/code/host_command.c project/code/app_scheduler.c
git commit -m "Add IK height host commands"
```

---

### Task 6: Add Height-Aware Balance and Chassis Scheduling

**Files:**
- Modify: `project/code/control_balance.c`
- Modify: `project/code/control_chassis.c`
- Modify: `project/code/app_types.h`
- Test: `tools/test_ik_height_control_static.ps1`

- [ ] **Step 1: Add effective fields to diagnostics**

In `balance_diag_struct`, add:

```c
    float leg_height_norm;
    float balance_pitch_kp_eff;
    float balance_pitch_rate_kd_eff;
    float balance_wheel_speed_ks_eff;
    float balance_pitch_setpoint_base_eff_deg;
    float chassis_forward_limit_eff_rpm;
    float chassis_fast_forward_limit_eff_rpm;
```

- [ ] **Step 2: Add includes**

In `control_balance.c` and `control_chassis.c`, add:

```c
#include "control_leg.h"
```

- [ ] **Step 3: Add lerp helper in `control_chassis.c` if available helper is static**

`control_chassis_lerp()` already exists for fast blend. Reuse it.

- [ ] **Step 4: Apply height-aware forward limit**

In `control_chassis_update()` and `control_chassis_set_cmd()`, replace the forward-limit decision with:

```c
const leg_diag_struct *leg;
const leg_height_profile_struct *height_profile;
float height_forward_limit_rpm;
float height_fast_forward_limit_rpm;
float height_norm;

leg = control_leg_get_diag();
height_profile = leg_config_get_height_profile();
height_norm = control_chassis_limit_abs(leg->height_norm, 1.0f);
height_forward_limit_rpm =
    control_chassis_lerp(height_profile->chassis_forward_limit_low_rpm,
                         height_profile->chassis_forward_limit_high_rpm,
                         height_norm);
height_fast_forward_limit_rpm =
    control_chassis_lerp(height_profile->chassis_fast_forward_limit_low_rpm,
                         height_profile->chassis_fast_forward_limit_high_rpm,
                         height_norm);

if(APP_CHASSIS_FORWARD_ZERO_TARGET_RPM <
   control_chassis_absf(leg->target_height_mm - leg->actual_height_mm))
{
    height_forward_limit_rpm = height_profile->transition_forward_limit_rpm;
    height_fast_forward_limit_rpm = height_profile->transition_forward_limit_rpm;
}

forward_limit_rpm = (APP_TRUE == control_chassis_cmd.fast_enable) ?
                    height_fast_forward_limit_rpm :
                    height_forward_limit_rpm;
```

This keeps `C` semantics unchanged and only changes internal limits.

- [ ] **Step 5: Add height scheduling in `control_balance.c`**

Near gain usage in `control_balance_update()`, declare:

```c
const leg_diag_struct *leg;
const leg_height_profile_struct *height_profile;
float height_norm;
float pitch_kp_eff;
float pitch_rate_kd_eff;
float wheel_speed_ks_base_eff;
float pitch_setpoint_base_eff;
```

Before computing `pitch_setpoint_deg`:

```c
leg = control_leg_get_diag();
height_profile = leg_config_get_height_profile();
height_norm = control_balance_limit_abs(leg->height_norm, 1.0f);

if((APP_TRUE == leg->ik_valid) && (APP_TRUE == leg->output_enable))
{
    pitch_kp_eff = control_balance_lerp(height_profile->balance_pitch_kp_low,
                                        height_profile->balance_pitch_kp_high,
                                        height_norm);
    pitch_rate_kd_eff = control_balance_lerp(height_profile->balance_pitch_rate_kd_low,
                                             height_profile->balance_pitch_rate_kd_high,
                                             height_norm);
    wheel_speed_ks_base_eff = control_balance_lerp(height_profile->balance_wheel_speed_ks_low,
                                                   height_profile->balance_wheel_speed_ks_high,
                                                   height_norm);
    pitch_setpoint_base_eff = control_balance_lerp(height_profile->balance_pitch_setpoint_low_deg,
                                                   height_profile->balance_pitch_setpoint_high_deg,
                                                   height_norm);
}
else
{
    pitch_kp_eff = control_balance_pitch_kp;
    pitch_rate_kd_eff = control_balance_pitch_rate_kd;
    wheel_speed_ks_base_eff = control_balance_wheel_speed_ks;
    pitch_setpoint_base_eff = control_balance_pitch_setpoint_deg;
}
```

Use `pitch_kp_eff`, `pitch_rate_kd_eff`, and `wheel_speed_ks_base_eff` in the existing balance term calculation. Use:

```c
pitch_setpoint_deg = pitch_setpoint_base_eff + chassis->pitch_offset_deg;
```

Publish diagnostics:

```c
control_balance_diag.leg_height_norm = height_norm;
control_balance_diag.balance_pitch_kp_eff = pitch_kp_eff;
control_balance_diag.balance_pitch_rate_kd_eff = pitch_rate_kd_eff;
control_balance_diag.balance_wheel_speed_ks_eff = wheel_speed_ks_base_eff;
control_balance_diag.balance_pitch_setpoint_base_eff_deg = pitch_setpoint_base_eff;
```

- [ ] **Step 6: Run static test**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_ik_height_control_static.ps1
```

Expected: FAIL on telemetry integration.

- [ ] **Step 7: Run existing balance tests**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
```

Expected: PASS.

- [ ] **Step 8: Run whitespace check**

```powershell
git diff --check -- project/code/control_balance.c project/code/control_chassis.c project/code/app_types.h
```

Expected: no output.

- [ ] **Step 9: Commit**

```powershell
git add project/code/control_balance.c project/code/control_chassis.c project/code/app_types.h
git commit -m "Schedule balance and chassis by leg height"
```

---

### Task 7: Expand Telemetry and Collector to 58 Floats

**Files:**
- Modify: `project/code/telemetry.c`
- Modify: `tools/collect_balance_data.ps1`
- Modify: `tools/test_collect_balance_data.ps1`
- Test: `tools/test_collect_balance_data.ps1`
- Test: `tools/test_ik_height_control_static.ps1`

- [ ] **Step 1: Expand firmware telemetry**

In `telemetry.c`, add:

```c
#include "control_leg.h"
```

Change:

```c
float vofa_data[38];
```

to:

```c
float vofa_data[58];
```

After index 37, append:

```c
const leg_diag_struct *leg;
leg = control_leg_get_diag();

vofa_data[38] = (float)leg->mode;
vofa_data[39] = leg->target_height_mm;
vofa_data[40] = leg->actual_height_mm;
vofa_data[41] = leg->height_norm;
vofa_data[42] = leg->left_x_mm;
vofa_data[43] = leg->left_y_mm;
vofa_data[44] = leg->right_x_mm;
vofa_data[45] = leg->right_y_mm;
vofa_data[46] = (float)leg->ik_valid;
vofa_data[47] = (float)leg->output_enable;
vofa_data[48] = leg->servo_target_deg[0];
vofa_data[49] = leg->servo_target_deg[1];
vofa_data[50] = leg->servo_target_deg[2];
vofa_data[51] = leg->servo_target_deg[3];
vofa_data[52] = balance->balance_pitch_kp_eff;
vofa_data[53] = balance->balance_pitch_rate_kd_eff;
vofa_data[54] = balance->balance_wheel_speed_ks_eff;
vofa_data[55] = balance->balance_pitch_setpoint_base_eff_deg;
vofa_data[56] = balance->chassis_forward_limit_eff_rpm;
vofa_data[57] = balance->chassis_fast_forward_limit_eff_rpm;
```

- [ ] **Step 2: Update collector fields**

In `tools/collect_balance_data.ps1`, set:

```powershell
$FloatCount = 58
```

Append field names after `ff_term_rpm`:

```text
leg_mode,leg_target_height_mm,leg_actual_height_mm,leg_height_norm,leg_left_x_mm,leg_left_y_mm,leg_right_x_mm,leg_right_y_mm,leg_ik_valid,leg_output_enable,leg_servo0_target_deg,leg_servo1_target_deg,leg_servo2_target_deg,leg_servo3_target_deg,balance_pitch_kp_eff,balance_pitch_rate_kd_eff,balance_wheel_speed_ks_eff,balance_pitch_setpoint_base_eff_deg,chassis_forward_limit_eff_rpm,chassis_fast_forward_limit_eff_rpm
```

In `Pop-BalanceFrames`, map indexes:

```powershell
leg_mode = $values[38]
leg_target_height_mm = $values[39]
leg_actual_height_mm = $values[40]
leg_height_norm = $values[41]
leg_left_x_mm = $values[42]
leg_left_y_mm = $values[43]
leg_right_x_mm = $values[44]
leg_right_y_mm = $values[45]
leg_ik_valid = $values[46]
leg_output_enable = $values[47]
leg_servo0_target_deg = $values[48]
leg_servo1_target_deg = $values[49]
leg_servo2_target_deg = $values[50]
leg_servo3_target_deg = $values[51]
balance_pitch_kp_eff = $values[52]
balance_pitch_rate_kd_eff = $values[53]
balance_wheel_speed_ks_eff = $values[54]
balance_pitch_setpoint_base_eff_deg = $values[55]
chassis_forward_limit_eff_rpm = $values[56]
chassis_fast_forward_limit_eff_rpm = $values[57]
```

Add the same fields to the CSV row writer.

- [ ] **Step 3: Update collector test**

In `tools/test_collect_balance_data.ps1`, expand `$values` from 38 to 58 values by appending:

```powershell
3.0, 100.0, 95.0, 0.3, 0.0, 95.0, 0.0, 95.0, 1.0, 1.0, 88.0, 92.0, 89.0, 91.0, 19.2, 8.4, 2.7, -1.35, 64.0, 190.0
```

Add asserts:

```powershell
Assert-Near $frames[0].leg_mode 3.0 0.001 "leg_mode"
Assert-Near $frames[0].leg_target_height_mm 100.0 0.001 "leg_target_height_mm"
Assert-Near $frames[0].leg_actual_height_mm 95.0 0.001 "leg_actual_height_mm"
Assert-Near $frames[0].leg_height_norm 0.3 0.001 "leg_height_norm"
Assert-Near $frames[0].leg_ik_valid 1.0 0.001 "leg_ik_valid"
Assert-Near $frames[0].leg_output_enable 1.0 0.001 "leg_output_enable"
Assert-Near $frames[0].leg_servo0_target_deg 88.0 0.001 "leg_servo0_target_deg"
Assert-Near $frames[0].balance_pitch_kp_eff 19.2 0.001 "balance_pitch_kp_eff"
Assert-Near $frames[0].chassis_forward_limit_eff_rpm 64.0 0.001 "chassis_forward_limit_eff_rpm"
Assert-Near $frames[0].chassis_fast_forward_limit_eff_rpm 190.0 0.001 "chassis_fast_forward_limit_eff_rpm"
```

- [ ] **Step 4: Run tests**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_collect_balance_data.ps1
powershell -ExecutionPolicy Bypass -File tools/test_ik_height_control_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
```

Expected: all PASS.

- [ ] **Step 5: Run whitespace check**

```powershell
git diff --check -- project/code/telemetry.c tools/collect_balance_data.ps1 tools/test_collect_balance_data.ps1
```

Expected: no output.

- [ ] **Step 6: Commit**

```powershell
git add project/code/telemetry.c tools/collect_balance_data.ps1 tools/test_collect_balance_data.ps1
git commit -m "Expose IK height telemetry"
```

---

### Task 8: Final Verification and Hardware Handoff

**Files:**
- No source changes unless validation exposes a missed implementation detail.

- [ ] **Step 1: Run all local static checks**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_ik_height_control_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_collect_balance_data.ps1
powershell -ExecutionPolicy Bypass -File tools/test_tune_drive_loops_static.ps1
git diff --check
```

Expected:

```text
ik height control static checks passed
balance drive v2 static checks passed
collect_balance_data tests passed
tune_drive_loops static checks passed
```

`git diff --check` must produce no output.

- [ ] **Step 2: IAR build**

Open:

```text
project/iar/cyt4bb7.eww
```

Build:

```text
cyt4bb7_cm_0_plus
cyt4bb7_cm_7_0
cyt4bb7_cm_7_1
```

Expected:

- all selected projects compile and link;
- no new warnings in modified project files.

- [ ] **Step 3: Servo-only calibration command test**

With robot supported and wheels off the ground, send:

```text
STOP
LIK,90,90,90,90
LIK,85,95,85,95
LIK,95,85,95,85
STOP
```

Acceptance:

- servos move only within configured limits;
- telemetry shows `leg_mode=2` for `LEG_MODE_IK_CALIB`;
- no balance or chassis motion is started by `LIK`.

- [ ] **Step 4: IK height supported sweep**

With robot supported:

```powershell
powershell -ExecutionPolicy Bypass -File tools/collect_balance_data.ps1 -Port COM6 -Duration 18 -Commands "0:STOP;1:LH,80;5:LH,100;9:LH,120;13:LH,100;17:STOP" -Note "ik_height_supported_sweep"
```

Acceptance:

- `leg_ik_valid` stays `1` for reachable heights;
- `leg_actual_height_mm` ramps smoothly;
- servo targets do not jump;
- no target angle touches configured servo min/max.

- [ ] **Step 5: Standing at low/mid/high**

Run three supported tests after calibration values are trusted:

```powershell
powershell -ExecutionPolicy Bypass -File tools/collect_balance_data.ps1 -Port COM6 -Duration 12 -Commands "0:STOP;0.5:LH,80;2:B,2;2.5:C,0,0;11:STOP" -Note "ik_height_low_stand"
powershell -ExecutionPolicy Bypass -File tools/collect_balance_data.ps1 -Port COM6 -Duration 12 -Commands "0:STOP;0.5:LH,100;2:B,2;2.5:C,0,0;11:STOP" -Note "ik_height_mid_stand"
powershell -ExecutionPolicy Bypass -File tools/collect_balance_data.ps1 -Port COM6 -Duration 12 -Commands "0:STOP;0.5:LH,120;2:B,2;2.5:C,0,0;11:STOP" -Note "ik_height_high_stand"
```

Acceptance:

- robot stands at each tested height;
- `feedback_online=1`;
- `imu_age_ms <= 15`;
- `wheel_age_ms <= 30`;
- `balance_rpm` does not sit at +/-300 saturation.

- [ ] **Step 6: Low-speed drive at low/mid/high**

Run only after standing passes:

```powershell
powershell -ExecutionPolicy Bypass -File tools/collect_balance_data.ps1 -Port COM6 -Duration 14 -Commands "0:STOP;0.5:LH,80;2:B,2;2.5:C,0,0;4:C,20,0;9:C,0,0;13:STOP" -Note "ik_height_low_drive20"
powershell -ExecutionPolicy Bypass -File tools/collect_balance_data.ps1 -Port COM6 -Duration 14 -Commands "0:STOP;0.5:LH,100;2:B,2;2.5:C,0,0;4:C,20,0;9:C,0,0;13:STOP" -Note "ik_height_mid_drive20"
powershell -ExecutionPolicy Bypass -File tools/collect_balance_data.ps1 -Port COM6 -Duration 14 -Commands "0:STOP;0.5:LH,120;2:B,2;2.5:C,0,0;4:C,20,0;9:C,0,0;13:STOP" -Note "ik_height_high_drive20"
```

Acceptance:

- `C,20,0` keeps existing command semantics;
- `chassis_forward_limit_eff_rpm` reflects height profile;
- `C,0,0` returns to standing;
- no IK invalid event occurs during drive.

- [ ] **Step 7: Final report**

Report:

- commits created;
- static checks run;
- IAR build result;
- hardware tests run and CSV paths;
- measured `L1-L5`, servo zero/direction values used;
- whether `LH` low/mid/high standing passed;
- whether low-speed driving at low/mid/high passed;
- whether fast mode was retested or deferred.

Do not claim "any height" support until low, mid, high, and transition tests pass on hardware.

## Merge Gate

Do not merge this branch until:

- `tools/test_ik_height_control_static.ps1` passes;
- existing balance drive static and collector tests still pass;
- IAR build status is known;
- `LIK` is verified on supported hardware;
- at least low/mid/high standing tests pass;
- `C,forward_rpm,turn_dps` semantics are unchanged.

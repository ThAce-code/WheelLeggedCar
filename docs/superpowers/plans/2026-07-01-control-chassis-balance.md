# Control Chassis And Balance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **For DeepSeek:** 必须先开启 teammates 模式再执行本计划。执行时按任务顺序推进，每完成一个任务后先自检、提交，再进入下一个任务。不要一次性跨任务大改。

**Goal:** Add a safe first-version `control_chassis` and `control_balance` layer so the firmware can enter low-power balance tuning without coupling balance logic to BLDC UART, VOFA parsing, or actuator PID internals.

**Architecture:** `control_chassis` owns forward/turn motion intent and left/right base RPM mixing. `control_balance` owns balance mode, safety gates, pitch derivative, low-power PD output, and the final handoff to `actuator_motor`. Existing `actuator_motor` remains the only module that sends motor output.

**Tech Stack:** Embedded C for CYT4BB7, IAR Embedded Workbench, existing `project/code` modules, cooperative scheduler, VOFA/UART0 debug commands, CYT2BL3 motor actuator interface.

---

## DeepSeek Execution Rules

- 开始执行前，显式开启 DeepSeek teammates 模式。
- 不修改 `libraries/*`。
- 不修改 `bldc_foc_uart.c` 来实现平衡行为。
- 不把 `PROJECT_MEMORY.md`、`data/` 或无关未跟踪文件加入提交。
- 每个任务完成后运行该任务列出的静态检查。
- 如果 IAR 不可在当前环境命令行构建，记录“未运行 IAR 构建”，但仍执行 `git diff --check` 和 `rg` 静态检查。
- 保留现有 `STOP`、`M`、`D`、`P`、`PL`、`PR` 调试命令语义。
- 第一版默认平衡增益为 `0.0f`，避免启用模式后车轮意外动作。

## Scope Check

This plan implements one subsystem: the first chassis/balance control skeleton with low-power balance test output. It does not implement full self-balancing performance, LQR, servo-leg coordination, runtime balance gain tuning, persistent storage, or torque/current control.

## File Structure

Create:

- `project/code/control_chassis.h`: public chassis command/output API.
- `project/code/control_chassis.c`: chassis command storage, limit, and forward/turn mixing.
- `project/code/control_balance.h`: public balance mode/diagnostic API.
- `project/code/control_balance.c`: balance mode state, safety gates, pitch derivative, low-power PD output, motor actuator handoff.

Modify:

- `project/code/app_types.h`: add shared chassis/balance structs and `balance_mode_enum`.
- `project/code/app_config.h`: add chassis/balance periods, limits, gains, and telemetry mode switch.
- `project/code/app.c`: initialize chassis and balance modules.
- `project/code/app_scheduler.c`: schedule chassis and balance before motor actuator update.
- `project/code/host_command.c`: parse `B,mode` and `C,forward,turn`.
- `project/code/telemetry.c`: support balance-focused VOFA telemetry through a config switch.

Do not modify:

- `project/code/bldc_foc_uart.c`
- `libraries/*`

---

### Task 1: Add Shared Types And Config

**Files:**

- Modify: `project/code/app_types.h`
- Modify: `project/code/app_config.h`

- [ ] **Step 1: Add balance/chassis types in `app_types.h`**

Add the following after `motor_rpm_loop_diag_struct` and before `servo_cmd_struct`:

```c
typedef enum
{
    BALANCE_MODE_OFF = 0,
    BALANCE_MODE_STANDBY,
    BALANCE_MODE_BALANCE_TEST
}balance_mode_enum;

typedef struct
{
    float forward_rpm;
    float turn_rpm;
    uint8 enable;
    uint32 last_cmd_ms;
}chassis_cmd_struct;

typedef struct
{
    float left_base_rpm;
    float right_base_rpm;
    uint8 enable;
}chassis_output_struct;

typedef struct
{
    balance_mode_enum mode;
    float pitch_deg;
    float pitch_rate_dps;
    float chassis_left_rpm;
    float chassis_right_rpm;
    float balance_rpm;
    float output_left_rpm;
    float output_right_rpm;
    uint8 output_enable;
    uint8 safety_blocked;
}balance_diag_struct;
```

- [ ] **Step 2: Add config constants in `app_config.h`**

Add these near the scheduler period constants:

```c
#define APP_CHASSIS_PERIOD_MS           (5U)
#define APP_BALANCE_PERIOD_MS           (5U)
```

Add these near the motor/leg control constants:

```c
#define APP_CHASSIS_RPM_LIMIT           (200.0f)
#define APP_BALANCE_RPM_LIMIT           (150.0f)
#define APP_BALANCE_TEST_PITCH_LIMIT_DEG (20.0f)
#define APP_BALANCE_PITCH_KP            (0.0f)
#define APP_BALANCE_PITCH_RATE_KD       (0.0f)
#define APP_TELEMETRY_BALANCE_ENABLE    (1U)
```

- [ ] **Step 3: Run static checks**

Run:

```powershell
git diff --check -- project/code/app_types.h project/code/app_config.h
rg -n "BALANCE_MODE|chassis_cmd_struct|balance_diag_struct|APP_CHASSIS_PERIOD_MS|APP_BALANCE_RPM_LIMIT|APP_TELEMETRY_BALANCE_ENABLE" project/code/app_types.h project/code/app_config.h
```

Expected:

```text
No whitespace errors.
All added type and config names are found.
```

- [ ] **Step 4: Commit**

```powershell
git add project/code/app_types.h project/code/app_config.h
git commit -m "Add chassis balance shared types"
```

---

### Task 2: Implement `control_chassis`

**Files:**

- Create: `project/code/control_chassis.h`
- Create: `project/code/control_chassis.c`

- [ ] **Step 1: Create `control_chassis.h`**

```c
/*********************************************************************************************************************
* File: control_chassis.h
* Description: Chassis command mixing interface.
********************************************************************************************************************/

#ifndef _control_chassis_h_
#define _control_chassis_h_

#include "app_types.h"

void control_chassis_init(void);
void control_chassis_update(uint32 now_ms);
void control_chassis_set_cmd(float forward_rpm, float turn_rpm, uint8 enable, uint32 now_ms);
void control_chassis_stop(uint32 now_ms);
const chassis_cmd_struct *control_chassis_get_cmd(void);
const chassis_output_struct *control_chassis_get_output(void);

#endif
```

- [ ] **Step 2: Create `control_chassis.c`**

```c
/*********************************************************************************************************************
* File: control_chassis.c
* Description: Chassis forward/turn command mixer.
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

void control_chassis_init(void)
{
    control_chassis_cmd.forward_rpm = 0.0f;
    control_chassis_cmd.turn_rpm = 0.0f;
    control_chassis_cmd.enable = APP_FALSE;
    control_chassis_cmd.last_cmd_ms = 0;

    control_chassis_output.left_base_rpm = 0.0f;
    control_chassis_output.right_base_rpm = 0.0f;
    control_chassis_output.enable = APP_FALSE;
}

void control_chassis_update(uint32 now_ms)
{
    float forward_rpm;
    float turn_rpm;
    float left_rpm;
    float right_rpm;

    (void)now_ms;

    if(APP_FALSE == control_chassis_cmd.enable)
    {
        control_chassis_output.left_base_rpm = 0.0f;
        control_chassis_output.right_base_rpm = 0.0f;
        control_chassis_output.enable = APP_FALSE;
        return;
    }

    forward_rpm = control_chassis_limit_abs(control_chassis_cmd.forward_rpm, APP_CHASSIS_RPM_LIMIT);
    turn_rpm = control_chassis_limit_abs(control_chassis_cmd.turn_rpm, APP_CHASSIS_RPM_LIMIT);

    left_rpm = forward_rpm - turn_rpm;
    right_rpm = forward_rpm + turn_rpm;

    control_chassis_output.left_base_rpm = control_chassis_limit_abs(left_rpm, APP_CHASSIS_RPM_LIMIT);
    control_chassis_output.right_base_rpm = control_chassis_limit_abs(right_rpm, APP_CHASSIS_RPM_LIMIT);
    control_chassis_output.enable = APP_TRUE;
}

void control_chassis_set_cmd(float forward_rpm, float turn_rpm, uint8 enable, uint32 now_ms)
{
    control_chassis_cmd.forward_rpm = control_chassis_limit_abs(forward_rpm, APP_CHASSIS_RPM_LIMIT);
    control_chassis_cmd.turn_rpm = control_chassis_limit_abs(turn_rpm, APP_CHASSIS_RPM_LIMIT);
    control_chassis_cmd.enable = (APP_TRUE == enable) ? APP_TRUE : APP_FALSE;
    control_chassis_cmd.last_cmd_ms = now_ms;
}

void control_chassis_stop(uint32 now_ms)
{
    control_chassis_set_cmd(0.0f, 0.0f, APP_FALSE, now_ms);
    control_chassis_update(now_ms);
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

- [ ] **Step 3: Run static checks**

Run:

```powershell
git diff --check -- project/code/control_chassis.h project/code/control_chassis.c
rg -n "control_chassis_(init|update|set_cmd|stop|get_output)" project/code/control_chassis.h project/code/control_chassis.c
```

Expected:

```text
No whitespace errors.
All public chassis functions are declared and defined.
```

- [ ] **Step 4: Commit**

```powershell
git add project/code/control_chassis.h project/code/control_chassis.c
git commit -m "Add chassis command mixer"
```

---

### Task 3: Implement `control_balance`

**Files:**

- Create: `project/code/control_balance.h`
- Create: `project/code/control_balance.c`

- [ ] **Step 1: Create `control_balance.h`**

```c
/*********************************************************************************************************************
* File: control_balance.h
* Description: Low-power balance control interface.
********************************************************************************************************************/

#ifndef _control_balance_h_
#define _control_balance_h_

#include "app_types.h"

void control_balance_init(void);
void control_balance_update(uint32 now_ms);
void control_balance_set_mode(balance_mode_enum mode);
balance_mode_enum control_balance_get_mode(void);
const balance_diag_struct *control_balance_get_diag(void);

#endif
```

- [ ] **Step 2: Create `control_balance.c`**

```c
/*********************************************************************************************************************
* File: control_balance.c
* Description: Low-power balance mode and pitch feedback controller.
********************************************************************************************************************/

#include "control_balance.h"
#include "app_config.h"
#include "app_state.h"
#include "sensor_imu.h"
#include "control_chassis.h"
#include "actuator_motor.h"

static balance_mode_enum control_balance_mode;
static balance_diag_struct control_balance_diag;
static float control_balance_last_pitch_deg;
static uint32 control_balance_last_update_ms;
static uint8 control_balance_derivative_valid;

static float control_balance_absf(float value)
{
    return (0.0f > value) ? -value : value;
}

static float control_balance_limit_abs(float value, float limit)
{
    float abs_limit;

    abs_limit = control_balance_absf(limit);
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

static void control_balance_reset_derivative(void)
{
    control_balance_last_pitch_deg = 0.0f;
    control_balance_last_update_ms = 0;
    control_balance_derivative_valid = APP_FALSE;
}

static void control_balance_stop_output(void)
{
    control_balance_diag.balance_rpm = 0.0f;
    control_balance_diag.output_left_rpm = 0.0f;
    control_balance_diag.output_right_rpm = 0.0f;
    control_balance_diag.output_enable = APP_FALSE;
    actuator_motor_set_mode_stop();
}

void control_balance_init(void)
{
    control_balance_mode = BALANCE_MODE_STANDBY;
    control_balance_diag.mode = BALANCE_MODE_STANDBY;
    control_balance_diag.pitch_deg = 0.0f;
    control_balance_diag.pitch_rate_dps = 0.0f;
    control_balance_diag.chassis_left_rpm = 0.0f;
    control_balance_diag.chassis_right_rpm = 0.0f;
    control_balance_diag.balance_rpm = 0.0f;
    control_balance_diag.output_left_rpm = 0.0f;
    control_balance_diag.output_right_rpm = 0.0f;
    control_balance_diag.output_enable = APP_FALSE;
    control_balance_diag.safety_blocked = APP_TRUE;
    control_balance_reset_derivative();
}

void control_balance_update(uint32 now_ms)
{
    const imu_state_struct *imu;
    const wheel_feedback_struct *wheel;
    const chassis_output_struct *chassis;
    float dt_s;
    float pitch_rate_dps;
    float balance_rpm;
    float output_left_rpm;
    float output_right_rpm;
    uint8 dt_valid;

    imu = sensor_imu_get_state();
    wheel = actuator_motor_get_feedback();
    chassis = control_chassis_get_output();

    control_balance_diag.mode = control_balance_mode;
    control_balance_diag.pitch_deg = imu->pitch;
    control_balance_diag.chassis_left_rpm = chassis->left_base_rpm;
    control_balance_diag.chassis_right_rpm = chassis->right_base_rpm;

    pitch_rate_dps = 0.0f;
    dt_valid = APP_FALSE;
    if((APP_TRUE == control_balance_derivative_valid) && (now_ms > control_balance_last_update_ms))
    {
        dt_s = (float)(now_ms - control_balance_last_update_ms) / 1000.0f;
        pitch_rate_dps = (imu->pitch - control_balance_last_pitch_deg) / dt_s;
        dt_valid = APP_TRUE;
    }
    else
    {
        control_balance_derivative_valid = APP_TRUE;
    }
    control_balance_last_pitch_deg = imu->pitch;
    control_balance_last_update_ms = now_ms;
    control_balance_diag.pitch_rate_dps = pitch_rate_dps;

    control_balance_diag.safety_blocked = APP_FALSE;
    if((BALANCE_MODE_BALANCE_TEST != control_balance_mode) ||
       (APP_STATE_FAULT == app_state_get()) ||
       (APP_FALSE == imu->healthy) ||
       (APP_FALSE == wheel->online) ||
       (APP_FALSE == chassis->enable) ||
       (APP_FALSE == dt_valid) ||
       (APP_BALANCE_TEST_PITCH_LIMIT_DEG < control_balance_absf(imu->pitch)))
    {
        control_balance_diag.safety_blocked = APP_TRUE;
        if(APP_TRUE == dt_valid)
        {
            control_balance_reset_derivative();
        }
        control_balance_stop_output();
        return;
    }

    balance_rpm = (APP_BALANCE_PITCH_KP * imu->pitch) +
                  (APP_BALANCE_PITCH_RATE_KD * pitch_rate_dps);
    balance_rpm = control_balance_limit_abs(balance_rpm, APP_BALANCE_RPM_LIMIT);

    output_left_rpm = chassis->left_base_rpm + balance_rpm;
    output_right_rpm = chassis->right_base_rpm + balance_rpm;

    control_balance_diag.balance_rpm = balance_rpm;
    control_balance_diag.output_left_rpm = output_left_rpm;
    control_balance_diag.output_right_rpm = output_right_rpm;
    control_balance_diag.output_enable = APP_TRUE;

    actuator_motor_set_mode_motor_rpm(output_left_rpm, output_right_rpm);
}

void control_balance_set_mode(balance_mode_enum mode)
{
    if(mode > BALANCE_MODE_BALANCE_TEST)
    {
        mode = BALANCE_MODE_OFF;
    }

    if(mode != control_balance_mode)
    {
        control_balance_reset_derivative();
    }

    control_balance_mode = mode;
    control_balance_diag.mode = mode;

    if(BALANCE_MODE_BALANCE_TEST != mode)
    {
        control_balance_stop_output();
    }
}

balance_mode_enum control_balance_get_mode(void)
{
    return control_balance_mode;
}

const balance_diag_struct *control_balance_get_diag(void)
{
    return &control_balance_diag;
}
```

- [ ] **Step 3: Run static checks**

Run:

```powershell
git diff --check -- project/code/control_balance.h project/code/control_balance.c
rg -n "bldc_foc|debug_read|debug_send" project/code/control_balance.c
rg -n "control_balance_(init|update|set_mode|get_diag)" project/code/control_balance.h project/code/control_balance.c
```

Expected:

```text
No whitespace errors.
The bldc_foc/debug search returns no matches.
All public balance functions are declared and defined.
```

- [ ] **Step 4: Commit**

```powershell
git add project/code/control_balance.h project/code/control_balance.c
git commit -m "Add low power balance controller"
```

---

### Task 4: Wire Init And Scheduler

**Files:**

- Modify: `project/code/app.c`
- Modify: `project/code/app_scheduler.c`

- [ ] **Step 1: Include new headers in `app.c`**

Add:

```c
#include "control_chassis.h"
#include "control_balance.h"
```

- [ ] **Step 2: Initialize new modules in `app_init()`**

After `host_command_init();`, add:

```c
    control_chassis_init();
    control_balance_init();
```

- [ ] **Step 3: Include new headers in `app_scheduler.c`**

Add:

```c
#include "control_chassis.h"
#include "control_balance.h"
```

- [ ] **Step 4: Add scheduler state variables**

Inside `app_scheduler_run_pending()`, near the other `static uint32` task timestamps, add:

```c
    static uint32 chassis_last_ms = 0;
    static uint32 balance_last_ms = 0;
```

- [ ] **Step 5: Schedule chassis and balance before motor actuator update**

Place this block after `app_safety_update(now_ms);` and before `actuator_motor_update(now_ms);`:

```c
    if(APP_TRUE == app_task_elapsed(now_ms, &chassis_last_ms, APP_CHASSIS_PERIOD_MS))
    {
        control_chassis_update(now_ms);
    }

    if(APP_TRUE == app_task_elapsed(now_ms, &balance_last_ms, APP_BALANCE_PERIOD_MS))
    {
        control_balance_update(now_ms);
    }
```

- [ ] **Step 6: Run static checks**

Run:

```powershell
git diff --check -- project/code/app.c project/code/app_scheduler.c
rg -n "control_chassis|control_balance|APP_CHASSIS_PERIOD_MS|APP_BALANCE_PERIOD_MS" project/code/app.c project/code/app_scheduler.c
```

Expected:

```text
No whitespace errors.
Both modules are included, initialized, and scheduled.
```

- [ ] **Step 7: Commit**

```powershell
git add project/code/app.c project/code/app_scheduler.c
git commit -m "Schedule chassis balance control"
```

---

### Task 5: Add Host Debug Commands

**Files:**

- Modify: `project/code/host_command.c`

- [ ] **Step 1: Include control headers**

Add:

```c
#include "control_chassis.h"
#include "control_balance.h"
```

- [ ] **Step 2: Add two-number parser**

Add this helper after `host_command_parse_number()`:

```c
static uint8 host_command_parse_two_numbers(const char *text, float *first, float *second)
{
    char number_text[16];
    float values[2];
    uint8 value_index = 0;
    uint8 number_index = 0;
    uint8 read_index = 0;

    while('\0' != text[read_index])
    {
        if(',' == text[read_index])
        {
            if((0U == number_index) || (1U <= value_index))
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

    if((0U == number_index) || (1U != value_index))
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
    return APP_TRUE;
}
```

- [ ] **Step 3: Update STOP handling**

Inside `host_command_match_stop(line)` handling, before recording command success, add:

```c
        control_chassis_stop(now_ms);
        control_balance_set_mode(BALANCE_MODE_OFF);
```

The block should call `actuator_motor_set_mode_stop()` as it does today.

- [ ] **Step 4: Parse `B,mode`**

In `host_command_process_line()`, after STOP handling and before `M` handling, add:

```c
    if(('B' == line[0]) && (',' == line[1]) &&
       (APP_TRUE == host_command_parse_number(&line[2], &value)))
    {
        if(0.0f == value)
        {
            control_balance_set_mode(BALANCE_MODE_OFF);
            actuator_motor_record_command_error(APP_FALSE);
            return;
        }
        if(1.0f == value)
        {
            control_balance_set_mode(BALANCE_MODE_STANDBY);
            actuator_motor_record_command_error(APP_FALSE);
            return;
        }
        if(2.0f == value)
        {
            control_balance_set_mode(BALANCE_MODE_BALANCE_TEST);
            actuator_motor_record_command_error(APP_FALSE);
            return;
        }
    }
```

- [ ] **Step 5: Parse `C,forward,turn`**

In `host_command_process_line()`, after `B` handling and before `M` handling, add:

```c
    if(('C' == line[0]) && (',' == line[1]) &&
       (APP_TRUE == host_command_parse_two_numbers(&line[2], &kp, &ki)))
    {
        control_chassis_set_cmd(kp, ki, APP_TRUE, now_ms);
        actuator_motor_record_command_error(APP_FALSE);
        return;
    }
```

Use existing local variables `kp` and `ki` as parsed `forward` and `turn` values to avoid adding more stack variables.

- [ ] **Step 6: Run static checks**

Run:

```powershell
git diff --check -- project/code/host_command.c
rg -n "control_chassis|control_balance|host_command_parse_two_numbers|B'|C'" project/code/host_command.c
```

Expected:

```text
No whitespace errors.
New includes, parser, STOP integration, B command, and C command are present.
```

- [ ] **Step 7: Commit**

```powershell
git add project/code/host_command.c
git commit -m "Add chassis balance debug commands"
```

---

### Task 6: Add Balance Telemetry Switch

**Files:**

- Modify: `project/code/telemetry.c`

- [ ] **Step 1: Include balance header**

Add:

```c
#include "control_balance.h"
```

- [ ] **Step 2: Emit balance telemetry when enabled**

In `telemetry_update()`, after declaring `float vofa_data[8];`, wrap the existing motor telemetry assignments with a config branch:

```c
#if APP_TELEMETRY_BALANCE_ENABLE
    const balance_diag_struct *balance;

    balance = control_balance_get_diag();
    vofa_data[0] = (float)now_ms;
    vofa_data[1] = (float)balance->mode;
    vofa_data[2] = balance->pitch_deg;
    vofa_data[3] = balance->pitch_rate_dps;
    vofa_data[4] = balance->chassis_left_rpm;
    vofa_data[5] = balance->chassis_right_rpm;
    vofa_data[6] = balance->balance_rpm;
    vofa_data[7] = (float)wheel->online;
#else
    vofa_data[0] = (float)now_ms;
    vofa_data[1] = (float)rpm_diag->mode;
    vofa_data[2] = rpm_diag->target_motor_rpm;
    vofa_data[3] = rpm_diag->left_motor_rpm;
    vofa_data[4] = rpm_diag->right_motor_rpm;
    vofa_data[5] = rpm_diag->left_duty;
    vofa_data[6] = rpm_diag->right_duty;
    vofa_data[7] = (float)wheel->online;
#endif
```

Keep the existing `debug_send_buffer()` calls after the branch.

- [ ] **Step 3: Run static checks**

Run:

```powershell
git diff --check -- project/code/telemetry.c
rg -n "APP_TELEMETRY_BALANCE_ENABLE|control_balance_get_diag|balance->pitch_deg" project/code/telemetry.c
```

Expected:

```text
No whitespace errors.
Telemetry can switch between balance and motor layouts.
```

- [ ] **Step 4: Commit**

```powershell
git add project/code/telemetry.c
git commit -m "Add balance telemetry layout"
```

---

### Task 7: Final Static Validation

**Files:**

- Inspect all modified `project/code/*` files.

- [ ] **Step 1: Verify dependency boundaries**

Run:

```powershell
rg -n "bldc_foc|debug_read|debug_send" project/code/control_chassis.c project/code/control_balance.c
rg -n "sensor_imu_get_state|actuator_motor_set" project/code/control_chassis.c
rg -n "bldc_foc_uart" project/code/control_balance.c project/code/control_chassis.c project/code/host_command.c
```

Expected:

```text
First command: no matches.
Second command: no matches.
Third command: no matches.
```

- [ ] **Step 2: Verify public API usage**

Run:

```powershell
rg -n "control_chassis_|control_balance_" project/code
rg -n "BALANCE_MODE|APP_BALANCE|APP_CHASSIS" project/code
```

Expected:

```text
New APIs are used in app init, scheduler, host command, telemetry, and balance control.
Config constants and balance modes are present.
```

- [ ] **Step 3: Verify whitespace**

Run:

```powershell
git diff --check
```

Expected:

```text
No output.
```

- [ ] **Step 4: Build in IAR**

Open:

```text
project/iar/cyt4bb7.eww
```

Build these projects:

```text
cyt4bb7_cm_0_plus
cyt4bb7_cm_7_0
cyt4bb7_cm_7_1
```

Expected:

```text
All affected core projects compile.
```

If IAR is unavailable in the execution environment, record this exact note in the final handoff:

```text
IAR build was not run in this environment; static checks completed.
```

- [ ] **Step 5: Hardware smoke test**

With wheels suspended:

```text
STOP
B,1
B,2
C,0,0
C,50,0
C,0,50
STOP
```

Expected:

```text
STOP stops motor output immediately.
B,1 keeps no motor output.
B,2 with zero gains keeps no balance motion.
C,50,0 changes chassis_left_rpm and chassis_right_rpm equally in telemetry.
C,0,50 changes chassis_left_rpm and chassis_right_rpm in opposite directions.
Tilting the chassis changes pitch_deg and pitch_rate_dps telemetry.
Feedback offline, IMU unhealthy, fault state, or pitch over limit forces STOP.
```

- [ ] **Step 6: Commit validation notes if a project convention exists**

This repository does not currently require a validation log file. Do not create one unless the user requests it. Report build and hardware results in the final response or PR description.

---

## Final Handoff To User

DeepSeek should report:

- commits created;
- files changed;
- static checks run and results;
- IAR build result or the exact reason it was not run;
- hardware smoke result or that hardware smoke was not run;
- any behavior that differs from this plan.

Do not claim the robot can self-balance unless a supported hardware test proves it.

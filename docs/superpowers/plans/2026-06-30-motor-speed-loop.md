# Motor Speed Loop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a VOFA-commanded wheel speed loop so both BLDC wheels track the same `speed_count` target and respond promptly to `M,xxx` and `STOP` commands.

**Architecture:** Add a reusable PID module, a small UART0/VOFA command parser, and a speed-loop mode inside `actuator_motor`. `bldc_foc_uart` remains the CYT2BL3 binary protocol adapter; speed control converts target speed feedback units into duty outputs.

**Tech Stack:** Embedded C for CYT4BB7, IAR Embedded Workbench project files, Seekfree UART/debug ring buffer, CYT2BL3 binary UART protocol, VOFA JustFloat telemetry.

---

## Scope Check

This plan implements one focused subsystem: motor speed-loop bring-up. It does not implement balance control, chassis kinematics, or current/torque control. The resulting firmware can be tested independently by sending VOFA commands on `COM6/UART0` and watching wheel speed response plus duty output.

The worktree currently may contain local hardware-test edits in:

```text
project/code/actuator_motor.c
project/code/app_config.h
project/code/sensor_imu.c
project/code/telemetry.c
```

Before executing each task, inspect the real diff and preserve useful user changes. Do not revert unrelated local edits.

## File Structure

Create:

- `project/code/control_pid.h`: reusable PID type and function declarations.
- `project/code/control_pid.c`: reusable bounded PID implementation.
- `project/code/host_command.h`: VOFA/UART0 command parser interface.
- `project/code/host_command.c`: reads `debug_read_ring_buffer()`, parses `M,xxx` and `STOP`, calls motor speed target API.

Modify:

- `project/code/app_types.h`: rename motor command semantics to speed targets and add `motor_speed_loop_diag_struct`.
- `project/code/app_config.h`: add host command and speed-loop configuration.
- `project/code/actuator_motor.h`: add speed-target and speed-loop diagnostic APIs.
- `project/code/actuator_motor.c`: replace duty-test path with feedback-based speed PI loop.
- `project/code/app.c`: initialize host command parser.
- `project/code/app_scheduler.c`: run host command parser before motor update.
- `project/code/telemetry.c`: publish speed-loop diagnostics.
- `project/iar/project_config/cyt4bb7_cm_7_0.ewp`: add the two new modules to the CM7_0 project.

Do not modify:

- `libraries/zf_driver/*`: keep the restored Seekfree driver untouched.
- `project/code/bldc_foc_uart.c` unless a compile error reveals an API mismatch.

---

### Task 1: Add Reusable PID Module

**Files:**

- Create: `project/code/control_pid.h`
- Create: `project/code/control_pid.c`
- Modify later in Task 6: `project/iar/project_config/cyt4bb7_cm_7_0.ewp`

- [ ] **Step 1: Create `control_pid.h`**

Use this complete header:

```c
/*********************************************************************************************************************
* File: control_pid.h
* Description: Reusable bounded PID controller.
********************************************************************************************************************/

#ifndef _control_pid_h_
#define _control_pid_h_

#include "app_types.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float integral_limit;
    float output_limit;
    uint8 first_update;
}pid_controller_struct;

void control_pid_init(pid_controller_struct *pid);
void control_pid_set_gain(pid_controller_struct *pid, float kp, float ki, float kd);
void control_pid_set_limit(pid_controller_struct *pid, float integral_limit, float output_limit);
void control_pid_reset(pid_controller_struct *pid);
float control_pid_update(pid_controller_struct *pid, float target, float feedback, float dt_s);

#endif
```

- [ ] **Step 2: Create `control_pid.c`**

Use this complete implementation:

```c
/*********************************************************************************************************************
* File: control_pid.c
* Description: Reusable bounded PID controller.
********************************************************************************************************************/

#include "control_pid.h"

static float control_pid_absf(float value)
{
    return (0.0f > value) ? -value : value;
}

static float control_pid_limit(float value, float limit)
{
    float abs_limit;

    abs_limit = control_pid_absf(limit);
    if(abs_limit <= 0.0f)
    {
        return value;
    }
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

void control_pid_init(pid_controller_struct *pid)
{
    pid->kp = 0.0f;
    pid->ki = 0.0f;
    pid->kd = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_limit = 0.0f;
    pid->output_limit = 0.0f;
    pid->first_update = APP_TRUE;
}

void control_pid_set_gain(pid_controller_struct *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void control_pid_set_limit(pid_controller_struct *pid, float integral_limit, float output_limit)
{
    pid->integral_limit = control_pid_absf(integral_limit);
    pid->output_limit = control_pid_absf(output_limit);
}

void control_pid_reset(pid_controller_struct *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->first_update = APP_TRUE;
}

float control_pid_update(pid_controller_struct *pid, float target, float feedback, float dt_s)
{
    float error;
    float derivative;
    float output;

    error = target - feedback;
    derivative = 0.0f;

    if(0.0f < dt_s)
    {
        pid->integral += error * dt_s;
        pid->integral = control_pid_limit(pid->integral, pid->integral_limit);

        if(APP_FALSE == pid->first_update)
        {
            derivative = (error - pid->prev_error) / dt_s;
        }
    }

    output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    output = control_pid_limit(output, pid->output_limit);

    pid->prev_error = error;
    pid->first_update = APP_FALSE;

    return output;
}
```

- [ ] **Step 3: Run static checks**

Run:

```powershell
git diff --check -- project/code/control_pid.h project/code/control_pid.c
rg -n "control_pid_update|pid_controller_struct" project/code/control_pid.*
```

Expected:

```text
No whitespace errors.
Both files contain the expected symbols.
```

- [ ] **Step 4: Commit PID module**

```powershell
git add project/code/control_pid.h project/code/control_pid.c
git commit -m "Add reusable PID controller"
```

---

### Task 2: Add Speed-Loop Types And Configuration

**Files:**

- Modify: `project/code/app_types.h`
- Modify: `project/code/app_config.h`

- [ ] **Step 1: Update motor command semantics in `app_types.h`**

Replace the existing `motor_cmd_struct` with this speed-target version:

```c
typedef struct
{
    float left_target_speed;
    float right_target_speed;
    uint8 enable;
}motor_cmd_struct;
```

Add this type after `motor_diag_struct`:

```c
typedef struct
{
    uint8 enable;
    float target_speed;
    float left_target_speed;
    float right_target_speed;
    float left_speed;
    float right_speed;
    float left_error;
    float right_error;
    float left_duty;
    float right_duty;
    float left_integral;
    float right_integral;
    uint32 command_error_count;
}motor_speed_loop_diag_struct;
```

- [ ] **Step 2: Add config constants to `app_config.h`**

Keep existing BLDC UART config. Set feedback on for speed loop and disable the old fixed-duty test:

```c
#define APP_BLDC_START_FEEDBACK         (1U)
#define APP_BLDC_TEST_ENABLE            (0U)
```

Add these constants near the motor/BLDC configuration:

```c
#define APP_HOST_COMMAND_PERIOD_MS       (5U)

#define APP_MOTOR_SPEED_LOOP_ENABLE      (1U)
#define APP_MOTOR_SPEED_TARGET_LIMIT     (1000.0f)
#define APP_MOTOR_SPEED_DUTY_LIMIT       (2000.0f)
#define APP_MOTOR_SPEED_INTEGRAL_LIMIT   (1000.0f)
#define APP_MOTOR_SPEED_KP               (1.0f)
#define APP_MOTOR_SPEED_KI               (0.0f)
#define APP_MOTOR_SPEED_KD               (0.0f)

#define APP_MOTOR_LEFT_DUTY_SIGN         (-1.0f)
#define APP_MOTOR_RIGHT_DUTY_SIGN        (-1.0f)
#define APP_MOTOR_LEFT_SPEED_SIGN        (1.0f)
#define APP_MOTOR_RIGHT_SPEED_SIGN       (1.0f)
```

If hardware testing has already proven different signs, use the proven values in this task.

- [ ] **Step 3: Update old field references**

Search:

```powershell
rg -n "left_target|right_target" project/code
```

Expected references requiring update:

```text
project/code/actuator_motor.c
project/code/telemetry.c
```

Do not leave any `motor_cmd->left_target` or `motor_cmd->right_target` references after Task 4 and Task 5.

- [ ] **Step 4: Run static checks**

Run:

```powershell
git diff --check -- project/code/app_types.h project/code/app_config.h
rg -n "left_target_speed|right_target_speed|APP_MOTOR_SPEED_LOOP_ENABLE" project/code/app_types.h project/code/app_config.h
```

Expected:

```text
No whitespace errors.
New speed target fields and config symbols are present.
```

- [ ] **Step 5: Commit type/config changes**

```powershell
git add project/code/app_types.h project/code/app_config.h
git commit -m "Define motor speed loop configuration"
```

---

### Task 3: Add VOFA Host Command Parser

**Files:**

- Create: `project/code/host_command.h`
- Create: `project/code/host_command.c`
- Modify later in Task 6: `project/iar/project_config/cyt4bb7_cm_7_0.ewp`

- [ ] **Step 1: Create `host_command.h`**

Use this complete header:

```c
/*********************************************************************************************************************
* File: host_command.h
* Description: UART0/VOFA downlink command parser.
********************************************************************************************************************/

#ifndef _host_command_h_
#define _host_command_h_

#include "app_types.h"

void host_command_init(void);
void host_command_update(uint32 now_ms);

#endif
```

- [ ] **Step 2: Create `host_command.c`**

Use this complete implementation:

```c
/*********************************************************************************************************************
* File: host_command.c
* Description: UART0/VOFA downlink command parser.
********************************************************************************************************************/

#include "host_command.h"
#include "actuator_motor.h"
#include "zf_common_debug.h"

#define HOST_COMMAND_RX_BUFFER_LEN       (32U)
#define HOST_COMMAND_LINE_MAX            (32U)

static char host_command_line[HOST_COMMAND_LINE_MAX];
static uint8 host_command_index = 0;

static uint8 host_command_is_space(uint8 ch)
{
    return ((' ' == ch) || ('\t' == ch)) ? APP_TRUE : APP_FALSE;
}

static uint8 host_command_match_stop(const char *line)
{
    return (('S' == line[0]) &&
            ('T' == line[1]) &&
            ('O' == line[2]) &&
            ('P' == line[3]) &&
            ('\0' == line[4])) ? APP_TRUE : APP_FALSE;
}

static uint8 host_command_parse_float(const char *text, float *value)
{
    uint8 index = 0;
    uint8 digit_found = APP_FALSE;
    float sign = 1.0f;
    float result = 0.0f;

    if('-' == text[index])
    {
        sign = -1.0f;
        index++;
    }
    else if('+' == text[index])
    {
        index++;
    }

    while(('0' <= text[index]) && ('9' >= text[index]))
    {
        digit_found = APP_TRUE;
        result = (result * 10.0f) + (float)(text[index] - '0');
        index++;
    }

    if('\0' != text[index])
    {
        return APP_FALSE;
    }
    if(APP_FALSE == digit_found)
    {
        return APP_FALSE;
    }

    *value = sign * result;
    return APP_TRUE;
}

static void host_command_process_line(char *line)
{
    float target;
    uint8 read_index = 0;
    uint8 write_index = 0;

    while('\0' != line[read_index])
    {
        if(APP_FALSE == host_command_is_space((uint8)line[read_index]))
        {
            line[write_index] = line[read_index];
            write_index++;
        }
        read_index++;
    }
    line[write_index] = '\0';

    if(APP_TRUE == host_command_match_stop(line))
    {
        actuator_motor_stop();
        actuator_motor_record_command_error(APP_FALSE);
        return;
    }

    if(('M' == line[0]) && (',' == line[1]) &&
       (APP_TRUE == host_command_parse_float(&line[2], &target)))
    {
        actuator_motor_set_speed_target(target, target, APP_TRUE);
        actuator_motor_record_command_error(APP_FALSE);
        return;
    }

    actuator_motor_record_command_error(APP_TRUE);
}

static void host_command_push_byte(uint8 ch)
{
    if(('\r' == ch) || ('\n' == ch))
    {
        if(0U < host_command_index)
        {
            host_command_line[host_command_index] = '\0';
            host_command_process_line(host_command_line);
            host_command_index = 0;
        }
        return;
    }

    if((0x20U <= ch) && ((HOST_COMMAND_LINE_MAX - 1U) > host_command_index))
    {
        host_command_line[host_command_index] = (char)ch;
        host_command_index++;
    }
    else if((HOST_COMMAND_LINE_MAX - 1U) <= host_command_index)
    {
        host_command_index = 0;
        actuator_motor_record_command_error(APP_TRUE);
    }
}

void host_command_init(void)
{
    host_command_index = 0;
    host_command_line[0] = '\0';
}

void host_command_update(uint32 now_ms)
{
    uint8 buffer[HOST_COMMAND_RX_BUFFER_LEN];
    uint32 count;
    uint32 index;

    (void)now_ms;

    count = debug_read_ring_buffer(buffer, HOST_COMMAND_RX_BUFFER_LEN);
    for(index = 0; index < count; index++)
    {
        host_command_push_byte(buffer[index]);
    }
}
```

- [ ] **Step 3: Add required actuator API declarations before compiling**

Task 4 implements these APIs. Add declarations in `project/code/actuator_motor.h` during Task 4:

```c
void actuator_motor_set_speed_target(float left_speed, float right_speed, uint8 enable);
void actuator_motor_record_command_error(uint8 is_error);
const motor_speed_loop_diag_struct *actuator_motor_get_speed_loop_diag(void);
```

- [ ] **Step 4: Run static checks**

Run:

```powershell
git diff --check -- project/code/host_command.h project/code/host_command.c
rg -n "M,|STOP|debug_read_ring_buffer|actuator_motor_set_speed_target" project/code/host_command.c
```

Expected:

```text
No whitespace errors.
Parser recognizes M,xxx and STOP and reads the debug ring buffer.
```

- [ ] **Step 5: Commit host command parser**

```powershell
git add project/code/host_command.h project/code/host_command.c
git commit -m "Add VOFA motor command parser"
```

---

### Task 4: Implement Actuator Motor Speed Loop

**Files:**

- Modify: `project/code/actuator_motor.h`
- Modify: `project/code/actuator_motor.c`

- [ ] **Step 1: Update `actuator_motor.h` API**

Replace the header contents after includes with these declarations:

```c
void actuator_motor_init(void);
void actuator_motor_set_cmd(const motor_cmd_struct *cmd);
void actuator_motor_set_speed_target(float left_speed, float right_speed, uint8 enable);
void actuator_motor_record_command_error(uint8 is_error);
void actuator_motor_update(uint32 now_ms);
void actuator_motor_stop(void);
const motor_cmd_struct *actuator_motor_get_cmd(void);
const wheel_feedback_struct *actuator_motor_get_feedback(void);
const motor_diag_struct *actuator_motor_get_diag(void);
const motor_speed_loop_diag_struct *actuator_motor_get_speed_loop_diag(void);
```

- [ ] **Step 2: Add includes and static state in `actuator_motor.c`**

Add includes:

```c
#include "app_state.h"
#include "control_pid.h"
```

Add static state near existing globals:

```c
static motor_speed_loop_diag_struct actuator_motor_speed_diag;
static pid_controller_struct actuator_motor_left_pid;
static pid_controller_struct actuator_motor_right_pid;
static uint32 actuator_motor_last_loop_ms = 0;
```

- [ ] **Step 3: Add helper functions**

Add these helpers before `actuator_motor_clear_snapshot()`:

```c
static float actuator_motor_absf(float value)
{
    return (0.0f > value) ? -value : value;
}

static float actuator_motor_limit_abs(float value, float limit)
{
    float abs_limit;

    abs_limit = actuator_motor_absf(limit);
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

static int16 actuator_motor_float_to_duty(float value)
{
    if(0.0f <= value)
    {
        return (int16)(value + 0.5f);
    }
    return (int16)(value - 0.5f);
}

static void actuator_motor_reset_speed_loop(void)
{
    control_pid_reset(&actuator_motor_left_pid);
    control_pid_reset(&actuator_motor_right_pid);
    actuator_motor_speed_diag.left_error = 0.0f;
    actuator_motor_speed_diag.right_error = 0.0f;
    actuator_motor_speed_diag.left_duty = 0.0f;
    actuator_motor_speed_diag.right_duty = 0.0f;
    actuator_motor_speed_diag.left_integral = 0.0f;
    actuator_motor_speed_diag.right_integral = 0.0f;
    actuator_motor_last_loop_ms = 0;
}
```

Remove the older duplicate `actuator_motor_float_to_duty()` if present.

- [ ] **Step 4: Initialize speed-loop diagnostics and PIDs**

In `actuator_motor_clear_snapshot()`, set all `actuator_motor_speed_diag` fields:

```c
actuator_motor_speed_diag.enable = APP_FALSE;
actuator_motor_speed_diag.target_speed = 0.0f;
actuator_motor_speed_diag.left_target_speed = 0.0f;
actuator_motor_speed_diag.right_target_speed = 0.0f;
actuator_motor_speed_diag.left_speed = 0.0f;
actuator_motor_speed_diag.right_speed = 0.0f;
actuator_motor_speed_diag.left_error = 0.0f;
actuator_motor_speed_diag.right_error = 0.0f;
actuator_motor_speed_diag.left_duty = 0.0f;
actuator_motor_speed_diag.right_duty = 0.0f;
actuator_motor_speed_diag.left_integral = 0.0f;
actuator_motor_speed_diag.right_integral = 0.0f;
actuator_motor_speed_diag.command_error_count = 0;
```

In `actuator_motor_init()` after `actuator_motor_clear_snapshot()`:

```c
control_pid_init(&actuator_motor_left_pid);
control_pid_init(&actuator_motor_right_pid);
control_pid_set_gain(&actuator_motor_left_pid, APP_MOTOR_SPEED_KP, APP_MOTOR_SPEED_KI, APP_MOTOR_SPEED_KD);
control_pid_set_gain(&actuator_motor_right_pid, APP_MOTOR_SPEED_KP, APP_MOTOR_SPEED_KI, APP_MOTOR_SPEED_KD);
control_pid_set_limit(&actuator_motor_left_pid, APP_MOTOR_SPEED_INTEGRAL_LIMIT, APP_MOTOR_SPEED_DUTY_LIMIT);
control_pid_set_limit(&actuator_motor_right_pid, APP_MOTOR_SPEED_INTEGRAL_LIMIT, APP_MOTOR_SPEED_DUTY_LIMIT);
```

- [ ] **Step 5: Implement speed target API**

Replace `actuator_motor_set_cmd()` implementation with speed semantics:

```c
void actuator_motor_set_cmd(const motor_cmd_struct *cmd)
{
    actuator_motor_set_speed_target(cmd->left_target_speed, cmd->right_target_speed, cmd->enable);
}
```

Add:

```c
void actuator_motor_set_speed_target(float left_speed, float right_speed, uint8 enable)
{
    float limited_left;
    float limited_right;

    limited_left = actuator_motor_limit_abs(left_speed, APP_MOTOR_SPEED_TARGET_LIMIT);
    limited_right = actuator_motor_limit_abs(right_speed, APP_MOTOR_SPEED_TARGET_LIMIT);

    actuator_motor_cmd.left_target_speed = limited_left;
    actuator_motor_cmd.right_target_speed = limited_right;
    actuator_motor_cmd.enable = enable;

    actuator_motor_speed_diag.enable = enable;
    actuator_motor_speed_diag.left_target_speed = limited_left;
    actuator_motor_speed_diag.right_target_speed = limited_right;
    if(limited_left == limited_right)
    {
        actuator_motor_speed_diag.target_speed = limited_left;
    }
    else
    {
        actuator_motor_speed_diag.target_speed = 0.5f * (limited_left + limited_right);
    }
}

void actuator_motor_record_command_error(uint8 is_error)
{
    if(APP_TRUE == is_error)
    {
        actuator_motor_speed_diag.command_error_count++;
    }
}
```

- [ ] **Step 6: Implement speed-loop update**

Add this function before `actuator_motor_update()`:

```c
static void actuator_motor_update_speed_loop(uint32 now_ms)
{
    float dt_s;
    float left_speed;
    float right_speed;
    float left_duty;
    float right_duty;
    int16 left_duty_i;
    int16 right_duty_i;

    if((APP_STATE_FAULT == app_state_get()) ||
       (APP_FALSE == actuator_motor_cmd.enable) ||
       (APP_FALSE == actuator_motor_feedback.online))
    {
        actuator_motor_send_duty_periodic(now_ms, 0, 0);
        actuator_motor_reset_speed_loop();
        return;
    }

    if(0U == actuator_motor_last_loop_ms)
    {
        dt_s = (float)APP_MOTOR_PERIOD_MS / 1000.0f;
    }
    else
    {
        dt_s = (float)(now_ms - actuator_motor_last_loop_ms) / 1000.0f;
    }
    actuator_motor_last_loop_ms = now_ms;

    left_speed = APP_MOTOR_LEFT_SPEED_SIGN * (float)actuator_motor_feedback.left_speed;
    right_speed = APP_MOTOR_RIGHT_SPEED_SIGN * (float)actuator_motor_feedback.right_speed;

    left_duty = control_pid_update(&actuator_motor_left_pid,
                                   actuator_motor_cmd.left_target_speed,
                                   left_speed,
                                   dt_s);
    right_duty = control_pid_update(&actuator_motor_right_pid,
                                    actuator_motor_cmd.right_target_speed,
                                    right_speed,
                                    dt_s);

    left_duty = APP_MOTOR_LEFT_DUTY_SIGN * left_duty;
    right_duty = APP_MOTOR_RIGHT_DUTY_SIGN * right_duty;
    left_duty = actuator_motor_limit_abs(left_duty, APP_MOTOR_SPEED_DUTY_LIMIT);
    right_duty = actuator_motor_limit_abs(right_duty, APP_MOTOR_SPEED_DUTY_LIMIT);

    left_duty_i = actuator_motor_float_to_duty(left_duty);
    right_duty_i = actuator_motor_float_to_duty(right_duty);

    actuator_motor_speed_diag.left_speed = left_speed;
    actuator_motor_speed_diag.right_speed = right_speed;
    actuator_motor_speed_diag.left_error = actuator_motor_cmd.left_target_speed - left_speed;
    actuator_motor_speed_diag.right_error = actuator_motor_cmd.right_target_speed - right_speed;
    actuator_motor_speed_diag.left_duty = left_duty;
    actuator_motor_speed_diag.right_duty = right_duty;
    actuator_motor_speed_diag.left_integral = actuator_motor_left_pid.integral;
    actuator_motor_speed_diag.right_integral = actuator_motor_right_pid.integral;

    actuator_motor_send_duty_periodic(now_ms, left_duty_i, right_duty_i);
}
```

- [ ] **Step 7: Replace `actuator_motor_update()` body**

Use this body:

```c
void actuator_motor_update(uint32 now_ms)
{
    actuator_motor_refresh_feedback(now_ms);

#if APP_MOTOR_SPEED_LOOP_ENABLE
    actuator_motor_update_speed_loop(now_ms);
    return;
#endif

    if(APP_FALSE == actuator_motor_cmd.enable)
    {
        if(APP_TRUE == actuator_motor_output_active)
        {
            actuator_motor_stop();
        }
        return;
    }

    if(APP_BLDC_SEND_PERIOD_MS <= (now_ms - actuator_motor_last_send_ms))
    {
        actuator_motor_last_send_ms = now_ms;
        actuator_motor_send_duty(actuator_motor_float_to_duty(actuator_motor_cmd.left_target_speed),
                                 actuator_motor_float_to_duty(actuator_motor_cmd.right_target_speed));
    }
}
```

- [ ] **Step 8: Update stop and getter**

Replace `actuator_motor_stop()` target fields:

```c
actuator_motor_cmd.left_target_speed = 0.0f;
actuator_motor_cmd.right_target_speed = 0.0f;
actuator_motor_cmd.enable = APP_FALSE;
actuator_motor_speed_diag.enable = APP_FALSE;
actuator_motor_speed_diag.target_speed = 0.0f;
actuator_motor_speed_diag.left_target_speed = 0.0f;
actuator_motor_speed_diag.right_target_speed = 0.0f;
bldc_foc_uart_stop();
actuator_motor_output_active = APP_FALSE;
actuator_motor_last_send_ms = 0;
actuator_motor_reset_speed_loop();
```

Add getter:

```c
const motor_speed_loop_diag_struct *actuator_motor_get_speed_loop_diag(void)
{
    return &actuator_motor_speed_diag;
}
```

- [ ] **Step 9: Run static checks**

Run:

```powershell
git diff --check -- project/code/actuator_motor.h project/code/actuator_motor.c
rg -n "left_target\\b|right_target\\b" project/code/actuator_motor.c project/code/actuator_motor.h
rg -n "actuator_motor_update_speed_loop|control_pid_update|actuator_motor_set_speed_target" project/code/actuator_motor.c project/code/actuator_motor.h
```

Expected:

```text
No whitespace errors.
No old left_target/right_target field references.
Speed-loop functions are present.
```

- [ ] **Step 10: Commit actuator speed loop**

```powershell
git add project/code/actuator_motor.h project/code/actuator_motor.c
git commit -m "Add motor speed loop actuator mode"
```

---

### Task 5: Wire Host Command Into App Scheduler

**Files:**

- Modify: `project/code/app.c`
- Modify: `project/code/app_scheduler.c`

- [ ] **Step 1: Initialize host command parser**

In `project/code/app.c`, add include:

```c
#include "host_command.h"
```

In `app_init()`, after `app_scheduler_init();`:

```c
host_command_init();
```

- [ ] **Step 2: Run host command parser before motor update**

In `project/code/app_scheduler.c`, add include:

```c
#include "host_command.h"
```

Add local static timer:

```c
static uint32 host_command_last_ms = 0;
```

Run commands after `now_ms = app_tick_ms;` and before IMU or motor tasks:

```c
if(APP_TRUE == app_task_elapsed(now_ms, &host_command_last_ms, APP_HOST_COMMAND_PERIOD_MS))
{
    host_command_update(now_ms);
}
```

This placement ensures VOFA commands are consumed promptly and motor update sees the latest target.

- [ ] **Step 3: Run static checks**

Run:

```powershell
git diff --check -- project/code/app.c project/code/app_scheduler.c
rg -n "host_command_init|host_command_update|APP_HOST_COMMAND_PERIOD_MS" project/code/app.c project/code/app_scheduler.c project/code/app_config.h
```

Expected:

```text
No whitespace errors.
Host command init and update are wired.
```

- [ ] **Step 4: Commit scheduler wiring**

```powershell
git add project/code/app.c project/code/app_scheduler.c
git commit -m "Wire VOFA motor commands into scheduler"
```

---

### Task 6: Register New Modules In IAR CM7_0 Project

**Files:**

- Modify: `project/iar/project_config/cyt4bb7_cm_7_0.ewp`

- [ ] **Step 1: Add new files to the `code` group**

Open `project/iar/project_config/cyt4bb7_cm_7_0.ewp` and add these entries near other `project/code` files:

```xml
<file>
    <name>$PROJ_DIR$\..\..\code\control_pid.c</name>
</file>
<file>
    <name>$PROJ_DIR$\..\..\code\control_pid.h</name>
</file>
<file>
    <name>$PROJ_DIR$\..\..\code\host_command.c</name>
</file>
<file>
    <name>$PROJ_DIR$\..\..\code\host_command.h</name>
</file>
```

Do not add these modules to CM0+ unless that project currently builds all app code and fails with missing files. The intended runtime owner is CM7_0.

- [ ] **Step 2: Verify project references**

Run:

```powershell
rg -n "control_pid|host_command" project/iar/project_config/cyt4bb7_cm_7_0.ewp
```

Expected:

```text
Four entries: control_pid.c, control_pid.h, host_command.c, host_command.h.
```

- [ ] **Step 3: Commit IAR project update**

```powershell
git add project/iar/project_config/cyt4bb7_cm_7_0.ewp
git commit -m "Register speed loop modules in CM7_0 project"
```

---

### Task 7: Extend Telemetry For Speed Loop

**Files:**

- Modify: `project/code/telemetry.c`

- [ ] **Step 1: Read speed-loop diagnostics**

In `telemetry_update()`, add:

```c
const motor_speed_loop_diag_struct *speed_diag;
```

After existing `motor_diag = actuator_motor_get_diag();`:

```c
speed_diag = actuator_motor_get_speed_loop_diag();
```

- [ ] **Step 2: Expand VOFA data array**

Change:

```c
float vofa_data[32];
```

to:

```c
float vofa_data[44];
```

Keep `I0` through `I31` unchanged except renamed motor command fields:

```c
vofa_data[21] = motor_cmd->left_target_speed;
vofa_data[22] = motor_cmd->right_target_speed;
```

Append:

```c
vofa_data[32] = (float)speed_diag->enable;
vofa_data[33] = speed_diag->target_speed;
vofa_data[34] = speed_diag->left_speed;
vofa_data[35] = speed_diag->right_speed;
vofa_data[36] = speed_diag->left_error;
vofa_data[37] = speed_diag->right_error;
vofa_data[38] = speed_diag->left_duty;
vofa_data[39] = speed_diag->right_duty;
vofa_data[40] = speed_diag->left_integral;
vofa_data[41] = speed_diag->right_integral;
vofa_data[42] = (float)wheel->online;
vofa_data[43] = (float)speed_diag->command_error_count;
```

- [ ] **Step 3: Run static checks**

Run:

```powershell
git diff --check -- project/code/telemetry.c
rg -n "vofa_data\\[43\\]|motor_speed_loop_diag_struct|left_target_speed|right_target_speed" project/code/telemetry.c
```

Expected:

```text
No whitespace errors.
Telemetry contains speed-loop channels through index 43.
```

- [ ] **Step 4: Commit telemetry update**

```powershell
git add project/code/telemetry.c
git commit -m "Expose motor speed loop telemetry"
```

---

### Task 8: Build And Hardware Smoke Test

**Files:**

- No code changes expected.
- Use IAR Embedded Workbench for build.
- Use VOFA for runtime test.

- [ ] **Step 1: Verify repository state before build**

Run:

```powershell
git status --short --branch
```

Expected:

```text
Only intentional local hardware-tuning edits, or a clean worktree.
No unstaged changes in files touched by completed tasks.
```

- [ ] **Step 2: Build CM7_0 in IAR**

Open:

```text
project/iar/cyt4bb7.eww
```

Build:

```text
cyt4bb7_cm_7_0
```

Expected:

```text
No compile errors.
No linker errors for control_pid, host_command, actuator_motor, or telemetry symbols.
```

- [ ] **Step 3: Confirm telemetry baseline**

Flash target. In VOFA on `COM6`, confirm:

```text
I27 != 4
I28 = 0
I29 = 1
I32 = 0 before command
I42 = 1 after BLDC speed feedback is online
```

If `I42 = 0`, do not test closed-loop motion. Confirm UART1 RX/TX/GND and that `APP_BLDC_START_FEEDBACK = 1U`.

- [ ] **Step 4: Test STOP command**

Send in VOFA:

```text
STOP
```

Expected:

```text
I32 = 0
I38 = 0
I39 = 0
BLDC duty frame sends 0 duty
```

- [ ] **Step 5: Test positive target**

Suspend wheels. Send:

```text
M,100
```

Expected:

```text
I32 = 1
I33 = 100
I34 and I35 move toward 100
I38 and I39 become nonzero
left and right wheel speeds approach each other
```

If wheels move opposite expected direction, do not change PID gains. Adjust `APP_MOTOR_LEFT_DUTY_SIGN`, `APP_MOTOR_RIGHT_DUTY_SIGN`, `APP_MOTOR_LEFT_SPEED_SIGN`, or `APP_MOTOR_RIGHT_SPEED_SIGN`.

- [ ] **Step 6: Test negative and zero target**

Send:

```text
M,-100
M,0
STOP
```

Expected:

```text
Negative target reverses feedback sign after sign configuration is correct.
M,0 keeps loop enabled and drives speed toward zero.
STOP disables loop and clears duty.
```

- [ ] **Step 7: Commit verified tuning changes**

If signs or initial gains changed during hardware testing:

```powershell
git add project/code/app_config.h
git commit -m "Tune initial motor speed loop signs"
```

If no code changed during test, no commit is needed.

---

## Self-Review

Spec coverage:

- Generic PID module: Task 1.
- VOFA/UART0 command parser: Task 3 and Task 5.
- Motor speed loop: Task 4.
- Direction sign configuration: Task 2 and Task 8.
- Feedback timeout/online protection: Task 4 uses `actuator_motor_feedback.online`, which is already derived from `APP_BLDC_FEEDBACK_TIMEOUT_MS`.
- Telemetry: Task 7.
- Hardware test: Task 8.

Placeholder scan:

- This plan intentionally contains no `TODO`, `TBD`, or unspecified implementation step.
- All new public APIs and types are named before use.

Type consistency:

- `motor_cmd_struct.left_target_speed/right_target_speed` is introduced in Task 2 and used in Task 4 and Task 7.
- `motor_speed_loop_diag_struct` is introduced in Task 2 and exposed by `actuator_motor_get_speed_loop_diag()` in Task 4.
- `host_command.c` uses APIs declared and implemented in Task 4.


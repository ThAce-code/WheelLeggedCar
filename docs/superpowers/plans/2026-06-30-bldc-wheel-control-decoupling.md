# BLDC Wheel Control Decoupling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Expose CYT2BL3 BLDC wheel command, feedback, and diagnostics through generic actuator-level interfaces so future wheel-leg control does not depend on the UART protocol module.

**Architecture:** Keep `bldc_foc_uart.c/.h` as the private CYT2BL3 binary UART adapter. Move generic wheel feedback and motor diagnostics into `app_types.h`, make `actuator_motor.c` translate BLDC feedback into those generic snapshots, and let telemetry read actuator-level data only.

**Tech Stack:** Embedded C for CYT4BB7/Traveo II, IAR Embedded Workbench project files, existing SeekFree `zf_driver_uart` APIs, PowerShell for static verification.

---

## File Structure

- Modify `project/code/app_types.h`: add generic `wheel_feedback_struct` and `motor_diag_struct`.
- Modify `project/code/app_config.h`: add `APP_BLDC_FEEDBACK_TIMEOUT_MS`.
- Modify `project/code/actuator_motor.h`: remove public dependency on `bldc_foc_uart.h`; expose generic feedback and diagnostics getters.
- Modify `project/code/actuator_motor.c`: include `bldc_foc_uart.h` privately; maintain generic wheel feedback and diagnostics snapshots; update feedback age and online state.
- Modify `project/code/telemetry.c`: publish compact IMU plus motor feedback diagnostics through VOFA without including `bldc_foc_uart.h`.
- Verify `project/user/cm7_0_isr.c`: UART1 ISR still calls `bldc_foc_uart_rx_isr()`.
- Verify `project/iar/project_config/cyt4bb7_cm_7_0.ewp`: existing BLDC and actuator files remain included; no new source files are needed.

## Task 1: Add Generic Wheel Feedback and Diagnostics Types

**Files:**
- Modify: `project/code/app_types.h`

- [ ] **Step 1: Inspect current shared types**

Run:

```powershell
Get-Content -Raw 'project\code\app_types.h'
```

Expected: `motor_cmd_struct` exists and there is no wheel feedback type yet.

- [ ] **Step 2: Add generic feedback and diagnostics types**

In `project/code/app_types.h`, insert this constant and these structs after `motor_cmd_struct`:

```c
#define MOTOR_DIAG_ASCII_LINE_MAX       (64U)

typedef struct
{
    int16 left_speed;
    int16 right_speed;
    int16 left_reduced_angle;
    int16 right_reduced_angle;
    uint32 last_rx_ms;
    uint32 age_ms;
    uint8 online;
}wheel_feedback_struct;

typedef struct
{
    int16 left_raw_angle;
    int16 right_raw_angle;
    uint32 checksum_error_count;
    uint32 unknown_frame_count;
    char last_unknown_ascii[MOTOR_DIAG_ASCII_LINE_MAX];
}motor_diag_struct;
```

- [ ] **Step 3: Verify type names are discoverable**

Run:

```powershell
rg -n "MOTOR_DIAG_ASCII_LINE_MAX|wheel_feedback_struct|motor_diag_struct" project/code/app_types.h
```

Expected: one match for each new public type or constant.

- [ ] **Step 4: Commit this task**

Run:

```powershell
git add -- project/code/app_types.h
git commit -m "Add generic wheel feedback types"
```

Expected: commit succeeds and only `app_types.h` is included in this commit.

## Task 2: Add BLDC Feedback Timeout Configuration

**Files:**
- Modify: `project/code/app_config.h`

- [ ] **Step 1: Locate motor actuator configuration**

Run:

```powershell
rg -n "Motor actuator|APP_BLDC_SEND_PERIOD_MS|APP_BLDC_START_FEEDBACK" project/code/app_config.h
```

Expected: matches inside the motor actuator configuration block.

- [ ] **Step 2: Add timeout constant**

In `project/code/app_config.h`, add this line directly after `APP_BLDC_SEND_PERIOD_MS`:

```c
#define APP_BLDC_FEEDBACK_TIMEOUT_MS    (100U)
```

- [ ] **Step 3: Verify timeout constant**

Run:

```powershell
rg -n "APP_BLDC_FEEDBACK_TIMEOUT_MS" project/code/app_config.h
```

Expected: one match in `app_config.h`.

- [ ] **Step 4: Commit this task**

Run:

```powershell
git add -- project/code/app_config.h
git commit -m "Add BLDC feedback timeout config"
```

Expected: commit succeeds and only `app_config.h` is included in this commit.

## Task 3: Hide BLDC Protocol Types Behind Actuator Interface

**Files:**
- Modify: `project/code/actuator_motor.h`
- Modify: `project/code/actuator_motor.c`

- [ ] **Step 1: Confirm current public BLDC dependency**

Run:

```powershell
rg -n "bldc_foc_uart|bldc_foc_feedback_struct|actuator_motor_get_feedback" project/code/actuator_motor.h project/code/actuator_motor.c
```

Expected: `actuator_motor.h` includes `bldc_foc_uart.h` and returns `bldc_foc_feedback_struct`.

- [ ] **Step 2: Update `actuator_motor.h` public API**

Replace the whole header body with:

```c
/*********************************************************************************************************************
* File: actuator_motor.h
* Description: Brushless motor actuator interface.
********************************************************************************************************************/

#ifndef _actuator_motor_h_
#define _actuator_motor_h_

#include "app_types.h"

void actuator_motor_init(void);
void actuator_motor_set_cmd(const motor_cmd_struct *cmd);
void actuator_motor_update(uint32 now_ms);
void actuator_motor_stop(void);
const motor_cmd_struct *actuator_motor_get_cmd(void);
const wheel_feedback_struct *actuator_motor_get_feedback(void);
const motor_diag_struct *actuator_motor_get_diag(void);

#endif
```

- [ ] **Step 3: Add private BLDC include and snapshots in `actuator_motor.c`**

At the top of `project/code/actuator_motor.c`, make the includes and static state look like this:

```c
#include "actuator_motor.h"
#include "app_config.h"
#include "bldc_foc_uart.h"

static motor_cmd_struct actuator_motor_cmd;
static wheel_feedback_struct actuator_motor_feedback;
static motor_diag_struct actuator_motor_diag;
static uint8 actuator_motor_output_active = APP_FALSE;
static uint32 actuator_motor_last_send_ms = 0;
```

- [ ] **Step 4: Add snapshot helper functions**

Add these helpers before `actuator_motor_limit()`:

```c
static void actuator_motor_clear_snapshot(void)
{
    uint8 i;

    actuator_motor_feedback.left_speed = 0;
    actuator_motor_feedback.right_speed = 0;
    actuator_motor_feedback.left_reduced_angle = 0;
    actuator_motor_feedback.right_reduced_angle = 0;
    actuator_motor_feedback.last_rx_ms = 0;
    actuator_motor_feedback.age_ms = 0;
    actuator_motor_feedback.online = APP_FALSE;

    actuator_motor_diag.left_raw_angle = 0;
    actuator_motor_diag.right_raw_angle = 0;
    actuator_motor_diag.checksum_error_count = 0;
    actuator_motor_diag.unknown_frame_count = 0;
    for(i = 0; i < MOTOR_DIAG_ASCII_LINE_MAX; i++)
    {
        actuator_motor_diag.last_unknown_ascii[i] = '\0';
    }
}

static void actuator_motor_copy_ascii_diag(const char *src)
{
    uint8 i;

    for(i = 0; i < (MOTOR_DIAG_ASCII_LINE_MAX - 1U); i++)
    {
        actuator_motor_diag.last_unknown_ascii[i] = src[i];
        if('\0' == src[i])
        {
            return;
        }
    }
    actuator_motor_diag.last_unknown_ascii[MOTOR_DIAG_ASCII_LINE_MAX - 1U] = '\0';
}

static void actuator_motor_refresh_feedback(uint32 now_ms)
{
    const bldc_foc_feedback_struct *raw;

    raw = bldc_foc_uart_get_feedback();
    actuator_motor_feedback.left_speed = raw->left_speed;
    actuator_motor_feedback.right_speed = raw->right_speed;
    actuator_motor_feedback.left_reduced_angle = raw->left_reduced_angle;
    actuator_motor_feedback.right_reduced_angle = raw->right_reduced_angle;
    actuator_motor_feedback.last_rx_ms = raw->last_rx_ms;

    if((APP_TRUE == raw->online) && (now_ms >= raw->last_rx_ms))
    {
        actuator_motor_feedback.age_ms = now_ms - raw->last_rx_ms;
        actuator_motor_feedback.online = (APP_BLDC_FEEDBACK_TIMEOUT_MS >= actuator_motor_feedback.age_ms) ? APP_TRUE : APP_FALSE;
    }
    else
    {
        actuator_motor_feedback.age_ms = 0;
        actuator_motor_feedback.online = APP_FALSE;
    }

    actuator_motor_diag.left_raw_angle = raw->left_angle;
    actuator_motor_diag.right_raw_angle = raw->right_angle;
    actuator_motor_diag.checksum_error_count = raw->checksum_error_count;
    actuator_motor_diag.unknown_frame_count = raw->unknown_frame_count;
    actuator_motor_copy_ascii_diag(raw->last_unknown_ascii);
}
```

- [ ] **Step 5: Initialize and refresh snapshots**

In `actuator_motor_init()`, add `actuator_motor_clear_snapshot();` before `bldc_foc_uart_init();`.

At the start of `actuator_motor_update(uint32 now_ms)`, before the `#if APP_BLDC_TEST_ENABLE` block, add:

```c
    actuator_motor_refresh_feedback(now_ms);
```

- [ ] **Step 6: Update getters**

Replace the existing feedback getter and add diagnostics:

```c
const wheel_feedback_struct *actuator_motor_get_feedback(void)
{
    return &actuator_motor_feedback;
}

const motor_diag_struct *actuator_motor_get_diag(void)
{
    return &actuator_motor_diag;
}
```

- [ ] **Step 7: Verify public BLDC dependency is gone**

Run:

```powershell
rg -n "bldc_foc_uart|bldc_foc_feedback_struct" project/code/actuator_motor.h
```

Expected: no output.

- [ ] **Step 8: Verify private BLDC dependency remains in implementation**

Run:

```powershell
rg -n "bldc_foc_uart|bldc_foc_feedback_struct|actuator_motor_refresh_feedback|actuator_motor_get_diag" project/code/actuator_motor.c
```

Expected: matches in `actuator_motor.c`.

- [ ] **Step 9: Commit this task**

Run:

```powershell
git add -- project/code/actuator_motor.h project/code/actuator_motor.c
git commit -m "Decouple motor actuator feedback API"
```

Expected: commit succeeds with only actuator motor files.

## Task 4: Publish Compact Motor Diagnostics in Telemetry

**Files:**
- Modify: `project/code/telemetry.c`

- [ ] **Step 1: Inspect current telemetry**

Run:

```powershell
Get-Content -Raw 'project\code\telemetry.c'
```

Expected: telemetry sends only three IMU floats.

- [ ] **Step 2: Replace telemetry implementation**

Use this implementation:

```c
/*********************************************************************************************************************
* File: telemetry.c
* Description: VOFA+ telemetry.
********************************************************************************************************************/

#include "telemetry.h"
#include "app_config.h"
#include "sensor_imu.h"
#include "actuator_motor.h"

void telemetry_init(void)
{
}

void telemetry_update(uint32 now_ms)
{
    static const uint8 tail[4] = {0x00, 0x00, 0x80, 0x7F};
    const imu_state_struct *imu;
    const wheel_feedback_struct *wheel;
    const motor_diag_struct *motor_diag;
    float vofa_data[12];

    (void)now_ms;
    imu = sensor_imu_get_state();
    wheel = actuator_motor_get_feedback();
    motor_diag = actuator_motor_get_diag();

    vofa_data[0] = imu->roll;
    vofa_data[1] = imu->pitch;
    vofa_data[2] = imu->yaw;
    vofa_data[3] = (float)wheel->online;
    vofa_data[4] = (float)wheel->age_ms;
    vofa_data[5] = (float)wheel->left_speed;
    vofa_data[6] = (float)wheel->right_speed;
    vofa_data[7] = (float)wheel->left_reduced_angle;
    vofa_data[8] = (float)wheel->right_reduced_angle;
    vofa_data[9] = (float)motor_diag->left_raw_angle;
    vofa_data[10] = (float)motor_diag->checksum_error_count;
    vofa_data[11] = (float)motor_diag->unknown_frame_count;

    debug_send_buffer((const uint8 *)vofa_data, sizeof(vofa_data));
    debug_send_buffer(tail, sizeof(tail));
}
```

- [ ] **Step 3: Verify telemetry uses only actuator-level motor API**

Run:

```powershell
rg -n "bldc_foc|actuator_motor_get_feedback|actuator_motor_get_diag|vofa_data\\[11\\]" project/code/telemetry.c
```

Expected: no `bldc_foc` match; actuator getter matches exist.

- [ ] **Step 4: Commit this task**

Run:

```powershell
git add -- project/code/telemetry.c
git commit -m "Add motor feedback telemetry"
```

Expected: commit succeeds with only `telemetry.c`.

## Task 5: Verify ISR and IAR Project Wiring

**Files:**
- Verify: `project/user/cm7_0_isr.c`
- Verify: `project/iar/project_config/cyt4bb7_cm_7_0.ewp`

- [ ] **Step 1: Verify UART1 RX calls BLDC parser**

Run:

```powershell
rg -n "void uart1_isr|bldc_foc_uart_rx_isr" project/user/cm7_0_isr.c
```

Expected: `uart1_isr` exists and calls `bldc_foc_uart_rx_isr()`.

- [ ] **Step 2: Verify CM7_0 IAR project includes BLDC and actuator sources**

Run:

```powershell
Select-String -Path 'project\iar\project_config\cyt4bb7_cm_7_0.ewp' -Pattern 'actuator_motor|bldc_foc_uart|telemetry' -Context 0,1
```

Expected: `.c` and `.h` entries for `actuator_motor`, `bldc_foc_uart`, and `telemetry` are present.

- [ ] **Step 3: Commit only if wiring files changed**

If the checks pass without edits, do not commit. If an IAR file must be updated manually, commit exactly that file:

```powershell
git add -- project/iar/project_config/cyt4bb7_cm_7_0.ewp
git commit -m "Include BLDC actuator files in CM7_0 project"
```

Expected when no edits are needed: no commit for this task.

## Task 6: Static Verification and Handoff

**Files:**
- Verify: `project/code/*.h`
- Verify: `project/code/*.c`

- [ ] **Step 1: Check no public actuator header exposes BLDC protocol types**

Run:

```powershell
rg -n "bldc_foc_uart|bldc_foc_feedback_struct" project/code/*.h
```

Expected: matches only in `project/code/bldc_foc_uart.h`; no match in `actuator_motor.h`.

- [ ] **Step 2: Check new generic API call sites**

Run:

```powershell
rg -n "wheel_feedback_struct|motor_diag_struct|actuator_motor_get_feedback|actuator_motor_get_diag|APP_BLDC_FEEDBACK_TIMEOUT_MS" project/code
```

Expected: definitions in `app_types.h`, config in `app_config.h`, implementation in `actuator_motor.c`, declarations in `actuator_motor.h`, and telemetry call sites.

- [ ] **Step 3: Check for unresolved old feedback type use**

Run:

```powershell
rg -n "const bldc_foc_feedback_struct \\*actuator_motor_get_feedback|actuator_motor_get_feedback\\(void\\).*bldc" project/code
```

Expected: no output.

- [ ] **Step 4: Review git diff**

Run:

```powershell
git diff -- project/code/app_types.h project/code/app_config.h project/code/actuator_motor.h project/code/actuator_motor.c project/code/telemetry.c
```

Expected: diff matches this plan and does not include unrelated refactors.

- [ ] **Step 5: Build in IAR**

Open `project/iar/cyt4bb7.eww` in IAR Embedded Workbench 9.40.1 or compatible and build `cyt4bb7_cm_7_0`.

Expected: build succeeds. If it fails, fix only compile errors caused by this change and rerun the build.

- [ ] **Step 6: Hardware smoke test**

Flash the target and verify:

```text
Power-on default: wheels do not spin.
VOFA ch0..ch2: roll, pitch, yaw still update.
VOFA ch3: BLDC online, 0 until valid feedback is received.
VOFA ch4: BLDC feedback age in ms.
VOFA ch5..ch6: left/right speed.
VOFA ch7..ch8: left/right reduced angle.
VOFA ch10..ch11: checksum and unknown frame counters do not increase during clean communication.
```

Expected: no startup spin, scheduler remains responsive, and feedback fields update only when the driver sends valid frames.

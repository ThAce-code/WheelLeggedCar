# Four State Balance Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **For DeepSeek:** 必须先开启 teammates 模式再执行本计划。按任务顺序执行，每个任务完成后运行检查并单独提交。不要一次性跨任务大改。不要提交 `data/` 采集文件。

**Goal:** Upgrade the balance controller from pitch-only PD to a safer four-state standing controller using pitch, pitch rate, average wheel speed, and integrated wheel position, with telemetry and tooling updated for data-driven tuning.

**Architecture:** Keep `actuator_motor` as the only motor output path. `control_balance` computes a common balance RPM correction from body attitude and wheel motion, then adds it to `control_chassis` base RPM. Telemetry expands from 8 to 14 float channels so MATLAB can analyze controller command, motor response, duty effort, and active gains from the same capture.

**Tech Stack:** Embedded C for CYT4BB7 in IAR Embedded Workbench, existing VOFA JustFloat UART telemetry, PowerShell serial capture scripts, MATLAB analysis scripts using relative `data/` paths.

---

## Scope And Engineering Estimate

工程量：中等。

Expected code impact:

- Firmware: `project/code/app_types.h`, `project/code/control_balance.h`, `project/code/control_balance.c`, `project/code/host_command.c`, `project/code/telemetry.c`, `project/code/app_config.h`.
- Tools: `tools/collect_balance_data.ps1`, `tools/test_collect_balance_data.ps1`, `tools/analyze_balance_pd.m`, `tools/test_analyze_balance_pd.ps1`.
- IAR project files should not need changes because `control_balance.c` and existing tool scripts are already present.

Do not change:

- `project/code/bldc_foc_uart.c`
- `libraries/*`
- `project/iar/project_config/*.ewp` unless a build proves a missing project entry.

## Behavior Contract

Existing commands remain valid:

```text
STOP
B,0
B,1
B,2
C,forward,turn
BP,kp,kd
M,rpm
D,duty
P,kp,ki,kd
PL,kp,ki,kd
PR,kp,ki,kd
```

Add:

```text
BL,angle,rate,speed,pos
BZ
```

Meanings:

- `BL,angle,rate,speed,pos`: set four-state balance gains.
- `BZ`: reset wheel position integrator and pitch derivative state while keeping current mode/gains.
- `BP,kp,kd`: remains a compatibility shortcut equivalent to `BL,kp,kd,0,0`.

The four-state controller output is:

```text
balance_rpm =
    K_angle * pitch_deg
  + K_rate  * pitch_rate_dps
  + K_speed * wheel_speed_rpm
  + K_pos   * wheel_pos_rev
```

Sign convention:

- Start with `K_angle` and `K_rate` positive because prior tests confirmed positive `Kp` saves in the correct direction.
- `K_speed` and `K_pos` must start at `0.0f` and be tuned carefully. Their signs may need hardware validation.

## Telemetry 14-Channel Layout

Use this layout when `APP_TELEMETRY_BALANCE_ENABLE == 1U`:

```text
I0  time_ms
I1  balance_mode
I2  pitch_deg
I3  pitch_rate_dps
I4  chassis_left_rpm
I5  chassis_right_rpm
I6  balance_rpm
I7  feedback_online
I8  left_motor_rpm
I9  right_motor_rpm
I10 left_duty
I11 right_duty
I12 balance_kp
I13 balance_kd
```

`I12/I13` stay pitch gains for compatibility with existing `BP` analysis. Four-state gain values are also stored in `balance_diag_struct` for future telemetry expansion if needed.

## Task 1: Expand Balance Telemetry To 14 Floats

**Files:**

- Modify: `project/code/telemetry.c`
- Modify: `tools/collect_balance_data.ps1`
- Modify: `tools/test_collect_balance_data.ps1`

- [ ] **Step 1: Change firmware balance telemetry payload**

In `project/code/telemetry.c`, change:

```c
    float vofa_data[8];
```

to:

```c
#if APP_TELEMETRY_BALANCE_ENABLE
    float vofa_data[14];
#else
    float vofa_data[8];
#endif
```

In the `APP_TELEMETRY_BALANCE_ENABLE` branch, assign all 14 channels:

```c
    vofa_data[0] = (float)now_ms;
    vofa_data[1] = (float)balance->mode;
    vofa_data[2] = balance->pitch_deg;
    vofa_data[3] = balance->pitch_rate_dps;
    vofa_data[4] = balance->chassis_left_rpm;
    vofa_data[5] = balance->chassis_right_rpm;
    vofa_data[6] = balance->balance_rpm;
    vofa_data[7] = (float)wheel->online;
    vofa_data[8] = rpm_diag->left_motor_rpm;
    vofa_data[9] = rpm_diag->right_motor_rpm;
    vofa_data[10] = rpm_diag->left_duty;
    vofa_data[11] = rpm_diag->right_duty;
    vofa_data[12] = balance->pitch_kp;
    vofa_data[13] = balance->pitch_rate_kd;
```

Keep the existing 8-channel motor telemetry branch unchanged.

- [ ] **Step 2: Update PowerShell capture constants**

In `tools/collect_balance_data.ps1`, change:

```powershell
$FloatCount = 8
$Fields = "pc_time_s,elapsed_s,sample_index,last_command,time_ms,balance_mode,pitch_deg,pitch_rate_dps,chassis_left_rpm,chassis_right_rpm,balance_rpm,feedback_online,note"
```

to:

```powershell
$FloatCount = 14
$Fields = "pc_time_s,elapsed_s,sample_index,last_command,time_ms,balance_mode,pitch_deg,pitch_rate_dps,chassis_left_rpm,chassis_right_rpm,balance_rpm,feedback_online,left_motor_rpm,right_motor_rpm,left_duty,right_duty,balance_kp,balance_kd,note"
```

- [ ] **Step 3: Update PowerShell frame object**

In `Pop-BalanceFrames`, return these fields:

```powershell
$frames.Add([pscustomobject]@{
    time_ms = $values[0]
    balance_mode = $values[1]
    pitch_deg = $values[2]
    pitch_rate_dps = $values[3]
    chassis_left_rpm = $values[4]
    chassis_right_rpm = $values[5]
    balance_rpm = $values[6]
    feedback_online = $values[7]
    left_motor_rpm = $values[8]
    right_motor_rpm = $values[9]
    left_duty = $values[10]
    right_duty = $values[11]
    balance_kp = $values[12]
    balance_kd = $values[13]
})
```

- [ ] **Step 4: Update CSV row writer**

Add the six new values before `note`:

```powershell
("{0:F3}" -f $frame.left_motor_rpm),
("{0:F3}" -f $frame.right_motor_rpm),
("{0:F3}" -f $frame.left_duty),
("{0:F3}" -f $frame.right_duty),
("{0:F6}" -f $frame.balance_kp),
("{0:F6}" -f $frame.balance_kd),
(Convert-CsvField $Note)
```

- [ ] **Step 5: Update collection test**

In `tools/test_collect_balance_data.ps1`, change the test frame values to 14 floats:

```powershell
$values = [single[]](1234.0, 2.0, 4.5, -12.25, 50.0, 50.0, 9.75, 1.0, 48.0, 47.0, -120.0, -118.0, 4.0, 0.2)
```

Add assertions:

```powershell
Assert-Near $frames[0].left_motor_rpm 48.0 0.001 "left_motor_rpm"
Assert-Near $frames[0].right_motor_rpm 47.0 0.001 "right_motor_rpm"
Assert-Near $frames[0].left_duty -120.0 0.001 "left_duty"
Assert-Near $frames[0].right_duty -118.0 0.001 "right_duty"
Assert-Near $frames[0].balance_kp 4.0 0.001 "balance_kp"
Assert-Near $frames[0].balance_kd 0.2 0.001 "balance_kd"
```

- [ ] **Step 6: Run checks**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\test_collect_balance_data.ps1
git diff --check -- project/code/telemetry.c tools/collect_balance_data.ps1 tools/test_collect_balance_data.ps1
rg -n "FloatCount = 14|left_motor_rpm|balance_kp|vofa_data\\[13\\]" project/code/telemetry.c tools/collect_balance_data.ps1 tools/test_collect_balance_data.ps1
```

Expected:

```text
collect_balance_data tests passed
No whitespace errors.
The 14-channel fields are present in firmware and capture script.
```

- [ ] **Step 7: Commit**

```powershell
git add project/code/telemetry.c tools/collect_balance_data.ps1 tools/test_collect_balance_data.ps1
git commit -m "Expand balance telemetry capture channels"
```

---

## Task 2: Extend Balance Diagnostics And Default Four-State Gains

**Files:**

- Modify: `project/code/app_types.h`
- Modify: `project/code/app_config.h`

- [ ] **Step 1: Add four-state diagnostics**

In `balance_diag_struct`, after `pitch_rate_kd`, add:

```c
    float wheel_speed_rpm;
    float wheel_pos_rev;
    float wheel_speed_ks;
    float wheel_pos_kp;
```

The field names intentionally avoid `kp` for speed gain because `pitch_kp` already means angle proportional gain. `wheel_speed_ks` means speed feedback gain.

- [ ] **Step 2: Add default gain and integrator constants**

In `project/code/app_config.h`, near the balance constants, add:

```c
#define APP_BALANCE_WHEEL_SPEED_KS      (0.0f)
#define APP_BALANCE_WHEEL_POS_KP        (0.0f)
#define APP_BALANCE_WHEEL_POS_LIMIT_REV (2.0f)
#define APP_BALANCE_WHEEL_POS_DECAY     (0.999f)
```

Meaning:

- `APP_BALANCE_WHEEL_SPEED_KS`: default wheel speed feedback gain.
- `APP_BALANCE_WHEEL_POS_KP`: default wheel position feedback gain.
- `APP_BALANCE_WHEEL_POS_LIMIT_REV`: integrator clamp in wheel motor revolutions.
- `APP_BALANCE_WHEEL_POS_DECAY`: very slow leak to keep the position term from staying biased forever.

- [ ] **Step 3: Run checks**

Run:

```powershell
git diff --check -- project/code/app_types.h project/code/app_config.h
rg -n "wheel_speed_rpm|wheel_pos_rev|APP_BALANCE_WHEEL_SPEED_KS|APP_BALANCE_WHEEL_POS_LIMIT_REV" project/code/app_types.h project/code/app_config.h
```

Expected:

```text
No whitespace errors.
Four-state fields and defaults are present.
```

- [ ] **Step 4: Commit**

```powershell
git add project/code/app_types.h project/code/app_config.h
git commit -m "Add four state balance diagnostics"
```

---

## Task 3: Implement Wheel Speed And Position State

**Files:**

- Modify: `project/code/control_balance.c`
- Modify: `project/code/control_balance.h`

- [ ] **Step 1: Add static gain and state variables**

In `control_balance.c`, after current gain variables, add:

```c
static float control_balance_wheel_speed_ks;
static float control_balance_wheel_pos_kp;
static float control_balance_wheel_pos_rev;
```

- [ ] **Step 2: Add reset helper**

After `control_balance_reset_derivative()`, add:

```c
static void control_balance_reset_motion_state(void)
{
    control_balance_wheel_pos_rev = 0.0f;
    control_balance_reset_derivative();
    control_balance_diag.wheel_speed_rpm = 0.0f;
    control_balance_diag.wheel_pos_rev = 0.0f;
}
```

- [ ] **Step 3: Initialize gains and state**

In `control_balance_init()`, after pitch gain initialization, add:

```c
    control_balance_wheel_speed_ks = APP_BALANCE_WHEEL_SPEED_KS;
    control_balance_wheel_pos_kp = APP_BALANCE_WHEEL_POS_KP;
    control_balance_wheel_pos_rev = 0.0f;
    control_balance_diag.wheel_speed_rpm = 0.0f;
    control_balance_diag.wheel_pos_rev = 0.0f;
    control_balance_diag.wheel_speed_ks = control_balance_wheel_speed_ks;
    control_balance_diag.wheel_pos_kp = control_balance_wheel_pos_kp;
```

- [ ] **Step 4: Compute signed average wheel speed**

In `control_balance_update()`, add local variables:

```c
    const motor_rpm_loop_diag_struct *rpm_diag;
    float wheel_speed_rpm;
    float wheel_pos_delta_rev;
```

After existing sensor/chassis snapshots are read, add:

```c
    rpm_diag = actuator_motor_get_motor_rpm_loop_diag();
```

After `dt_valid` is computed and before mode checks, add:

```c
    wheel_speed_rpm = 0.5f * (rpm_diag->left_motor_rpm + rpm_diag->right_motor_rpm);
    control_balance_diag.wheel_speed_rpm = wheel_speed_rpm;
```

- [ ] **Step 5: Integrate wheel position only when active and valid**

After the safety gate passes, before computing `balance_rpm`, add:

```c
    wheel_pos_delta_rev = wheel_speed_rpm * dt_s / 60.0f;
    control_balance_wheel_pos_rev += wheel_pos_delta_rev;
    control_balance_wheel_pos_rev *= APP_BALANCE_WHEEL_POS_DECAY;
    control_balance_wheel_pos_rev = control_balance_limit_abs(control_balance_wheel_pos_rev,
                                                              APP_BALANCE_WHEEL_POS_LIMIT_REV);
    control_balance_diag.wheel_pos_rev = control_balance_wheel_pos_rev;
```

- [ ] **Step 6: Extend control law**

Replace:

```c
    balance_rpm = (control_balance_pitch_kp * imu->pitch) +
                  (control_balance_pitch_rate_kd * pitch_rate_dps);
```

with:

```c
    balance_rpm = (control_balance_pitch_kp * imu->pitch) +
                  (control_balance_pitch_rate_kd * pitch_rate_dps) +
                  (control_balance_wheel_speed_ks * wheel_speed_rpm) +
                  (control_balance_wheel_pos_kp * control_balance_wheel_pos_rev);
```

- [ ] **Step 7: Reset wheel position on inactive/blocking paths**

In `control_balance_set_mode()`, replace `control_balance_reset_derivative();` with:

```c
        control_balance_reset_motion_state();
```

In the safety blocked branch, call:

```c
        control_balance_reset_motion_state();
```

only when the output must stop due to fault/feedback/pitch/chassis/dt gate. Do not reset position in the non-BALANCE telemetry-only path on every update unless the mode just changed.

- [ ] **Step 8: Add public reset and gain API**

In `control_balance.h`, add:

```c
void control_balance_set_full_gain(float pitch_kp, float pitch_rate_kd, float wheel_speed_ks, float wheel_pos_kp);
void control_balance_reset_motion_state_public(void);
```

In `control_balance.c`, add:

```c
void control_balance_set_full_gain(float pitch_kp, float pitch_rate_kd, float wheel_speed_ks, float wheel_pos_kp)
{
    if((0.0f > pitch_kp) || (0.0f > pitch_rate_kd))
    {
        return;
    }

    control_balance_pitch_kp = pitch_kp;
    control_balance_pitch_rate_kd = pitch_rate_kd;
    control_balance_wheel_speed_ks = wheel_speed_ks;
    control_balance_wheel_pos_kp = wheel_pos_kp;
    control_balance_diag.pitch_kp = control_balance_pitch_kp;
    control_balance_diag.pitch_rate_kd = control_balance_pitch_rate_kd;
    control_balance_diag.wheel_speed_ks = control_balance_wheel_speed_ks;
    control_balance_diag.wheel_pos_kp = control_balance_wheel_pos_kp;
    control_balance_reset_motion_state();
}

void control_balance_reset_motion_state_public(void)
{
    control_balance_reset_motion_state();
}
```

Update existing `control_balance_set_gain()` to call the full gain setter:

```c
void control_balance_set_gain(float pitch_kp, float pitch_rate_kd)
{
    control_balance_set_full_gain(pitch_kp, pitch_rate_kd, 0.0f, 0.0f);
}
```

- [ ] **Step 9: Run checks**

Run:

```powershell
git diff --check -- project/code/control_balance.c project/code/control_balance.h
rg -n "control_balance_set_full_gain|control_balance_reset_motion_state|wheel_speed_rpm|wheel_pos_rev|WHEEL_POS" project/code/control_balance.c project/code/control_balance.h
rg -n "bldc_foc|debug_read|debug_send" project/code/control_balance.c
```

Expected:

```text
No whitespace errors.
New full gain and reset APIs exist.
No BLDC/debug dependency appears in control_balance.c.
```

- [ ] **Step 10: Commit**

```powershell
git add project/code/control_balance.c project/code/control_balance.h
git commit -m "Add four state balance controller"
```

---

## Task 4: Add BL And BZ Host Commands

**Files:**

- Modify: `project/code/host_command.c`

- [ ] **Step 1: Add four-number parser**

After `host_command_parse_two_numbers()`, add:

```c
static uint8 host_command_parse_four_numbers(const char *text, float *first, float *second, float *third, float *fourth)
{
    char number_text[16];
    float values[4];
    uint8 value_index = 0;
    uint8 number_index = 0;
    uint8 read_index = 0;

    while('\0' != text[read_index])
    {
        if(',' == text[read_index])
        {
            if((0U == number_index) || (3U <= value_index))
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

    if((0U == number_index) || (3U != value_index))
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
    *fourth = values[3];
    return APP_TRUE;
}
```

- [ ] **Step 2: Add locals**

In `host_command_process_line()`, add:

```c
    float ks;
    float pos_kp;
```

- [ ] **Step 3: Add BZ command**

After STOP handling, add:

```c
    if(('B' == line[0]) && ('Z' == line[1]) && ('\0' == line[2]))
    {
        control_balance_reset_motion_state_public();
        actuator_motor_record_command_error(APP_FALSE);
        return;
    }
```

- [ ] **Step 4: Add BL command**

Before the existing `BP` command block, add:

```c
    if(('B' == line[0]) && ('L' == line[1]) && (',' == line[2]) &&
       (APP_TRUE == host_command_parse_four_numbers(&line[3], &kp, &ki, &ks, &pos_kp)))
    {
        if((0.0f <= kp) && (0.0f <= ki))
        {
            control_balance_set_full_gain(kp, ki, ks, pos_kp);
            actuator_motor_record_command_error(APP_FALSE);
            return;
        }
    }
```

Allow `ks` and `pos_kp` to be negative because the correct signs depend on the normalized wheel speed convention.

- [ ] **Step 5: Keep BP compatibility**

Update the `BP` block so it still calls:

```c
            control_balance_set_gain(kp, ki);
```

This sets wheel speed and position gains to zero.

- [ ] **Step 6: Run checks**

Run:

```powershell
git diff --check -- project/code/host_command.c
rg -n "host_command_parse_four_numbers|B'\\) && \\('L|B'\\) && \\('Z|control_balance_set_full_gain|control_balance_reset_motion_state_public" project/code/host_command.c
```

Expected:

```text
No whitespace errors.
BL and BZ commands are present.
```

- [ ] **Step 7: Commit**

```powershell
git add project/code/host_command.c
git commit -m "Add four state balance tuning commands"
```

---

## Task 5: Update MATLAB Analysis For 14 Channels

**Files:**

- Modify: `tools/analyze_balance_pd.m`
- Modify: `tools/test_analyze_balance_pd.ps1`

- [ ] **Step 1: Update static test expectations**

In `tools/test_analyze_balance_pd.ps1`, add required strings:

```powershell
Assert-Contains $text 'left_motor_rpm' "script should analyze left motor RPM"
Assert-Contains $text 'right_motor_rpm' "script should analyze right motor RPM"
Assert-Contains $text 'left_duty' "script should analyze left duty"
Assert-Contains $text 'right_duty' "script should analyze right duty"
Assert-Contains $text 'balance_kp' "script should retain balance Kp"
Assert-Contains $text 'balance_kd' "script should retain balance Kd"
Assert-Contains $text 'balance_pd_motor_response\.png' "script should save motor response figure"
```

- [ ] **Step 2: Add optional-column handling in MATLAB**

After reading `active`, add:

```matlab
hasMotorColumns = all(ismember(["left_motor_rpm", "right_motor_rpm", "left_duty", "right_duty"], string(active.Properties.VariableNames)));
hasGainColumns = all(ismember(["balance_kp", "balance_kd"], string(active.Properties.VariableNames)));
```

For older 8-channel CSV files, set missing metrics to `NaN`:

```matlab
if hasMotorColumns
    avgMotorRpm = (active.left_motor_rpm + active.right_motor_rpm) / 2.0;
    avgDuty = (active.left_duty + active.right_duty) / 2.0;
else
    avgMotorRpm = NaN(height(active), 1);
    avgDuty = NaN(height(active), 1);
end

if hasGainColumns
    balanceKp = median(active.balance_kp);
    balanceKd = median(active.balance_kd);
else
    balanceKp = NaN;
    balanceKd = NaN;
end
```

- [ ] **Step 3: Add summary fields**

Extend the summary table with:

```matlab
balanceKp, balanceKd, ...
local_percentile(abs(avgMotorRpm), 95), local_percentile(abs(avgDuty), 95), ...
```

Use variable names:

```matlab
"balance_kp", "balance_kd", "avg_motor_rpm_abs_p95", "avg_duty_abs_p95"
```

- [ ] **Step 4: Add motor response figure**

After the existing time-series figure, add:

```matlab
figure("Name", "Balance motor response", "Color", "w");
tiledlayout(2, 1, "TileSpacing", "compact");
nexttile;
hold on; grid on;
for i = 1:numel(series)
    if isfield(series(i), "avgMotorRpm")
        plot(series(i).t, series(i).avgMotorRpm, "DisplayName", series(i).note);
    end
end
yline(0, "k:");
ylabel("avg motor rpm");
title("Average motor response by PD setting");
legend("Location", "eastoutside", "Interpreter", "none");

nexttile;
hold on; grid on;
for i = 1:numel(series)
    if isfield(series(i), "avgDuty")
        plot(series(i).t, series(i).avgDuty, "DisplayName", series(i).note);
    end
end
yline(0, "k:");
ylabel("avg duty");
xlabel("elapsed in BALANCE_TEST (s)");
title("Average duty effort by PD setting");
saveas(gcf, fullfile(dataDir, "balance_pd_motor_response.png"));
```

- [ ] **Step 5: Store series motor fields**

When saving `series(end)`, add:

```matlab
    series(end).avgMotorRpm = avgMotorRpm;
    series(end).avgDuty = avgDuty;
```

- [ ] **Step 6: Run checks**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\test_analyze_balance_pd.ps1
git diff --check -- tools/analyze_balance_pd.m tools/test_analyze_balance_pd.ps1
rg -n "left_motor_rpm|avg_motor_rpm_abs_p95|balance_pd_motor_response|balance_kp|balance_kd" tools/analyze_balance_pd.m tools/test_analyze_balance_pd.ps1
```

Expected:

```text
analyze_balance_pd static checks passed
No whitespace errors.
MATLAB script handles 14-channel captures while tolerating older 8-channel files.
```

- [ ] **Step 7: Commit**

```powershell
git add tools/analyze_balance_pd.m tools/test_analyze_balance_pd.ps1
git commit -m "Analyze balance motor response telemetry"
```

---

## Task 6: Final Firmware And Hardware Validation

**Files:**

- Inspect modified files only.

- [ ] **Step 1: Run static boundary checks**

Run:

```powershell
rg -n "bldc_foc|debug_read|debug_send" project/code/control_balance.c project/code/control_chassis.c
rg -n "BL,|BZ|BP," project/code/host_command.c
rg -n "vofa_data\\[13\\]|FloatCount = 14|left_motor_rpm|balance_kp" project/code/telemetry.c tools/collect_balance_data.ps1
git diff --check
```

Expected:

```text
No BLDC/debug matches in control_balance/control_chassis.
BL/BZ/BP command paths are present.
14-channel telemetry and capture are present.
No whitespace errors.
```

- [ ] **Step 2: Build in IAR**

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
cyt4bb7_cm_7_0 builds without compile/link errors.
```

If IAR is unavailable, report exactly:

```text
IAR build was not run in this environment; static checks completed.
```

- [ ] **Step 3: Test telemetry decode without motion**

After flashing, run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 -Duration 5 -Commands "0:STOP;1:B,1" -Note "telemetry14_standby"
```

Open the CSV and confirm the header includes:

```text
left_motor_rpm,right_motor_rpm,left_duty,right_duty,balance_kp,balance_kd
```

Expected:

```text
Rows are collected.
balance_mode is 1 during standby.
No parse errors.
```

- [ ] **Step 4: Test BP compatibility**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 -Duration 8 -Commands "0:STOP;0.5:BP,4,0.2;1:B,1;2:B,2;2.2:C,0,0" -Note "bp_compat_kp4_kd020"
```

Expected:

```text
balance_kp is 4.0.
balance_kd is 0.2.
wheel speed and wheel position gains remain zero internally.
Robot behavior matches prior BP,4,0.2 tests.
```

- [ ] **Step 5: Test BL with zero wheel terms**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 -Duration 8 -Commands "0:STOP;0.5:BL,4,0.2,0,0;1:B,1;2:B,2;2.2:C,0,0" -Note "bl_zero_wheel_terms"
```

Expected:

```text
Behavior is equivalent to BP,4,0.2.
No stale wheel position output appears after mode transition.
```

- [ ] **Step 6: Wheel speed and position tuning guardrail**

Do not start with large wheel terms. First hardware candidates:

```text
BL,4,0.2,0,0
BL,4,0.2,-0.05,0
BL,4,0.2,-0.10,0
BL,4,0.2,-0.10,-0.5
BL,4,0.2,-0.15,-0.5
```

If speed/position terms make the robot run away faster, reverse their signs:

```text
BL,4,0.2,0.05,0
BL,4,0.2,0.10,0
BL,4,0.2,0.10,0.5
```

Keep `APP_BALANCE_RPM_LIMIT` unchanged for this validation.

- [ ] **Step 7: MATLAB analysis**

On a machine with MATLAB, run from repository root:

```matlab
tools/analyze_balance_pd
```

Expected outputs:

```text
data/balance_pd_summary.csv
data/balance_pd_timeseries.png
data/balance_pd_score.png
data/balance_pd_phase.png
data/balance_pd_motor_response.png
```

- [ ] **Step 8: Final handoff**

DeepSeek should report:

```text
Commits created
Files changed
Static checks run
IAR build result
Telemetry14 standby result
BP compatibility result
BL zero-wheel result
Hardware tuning notes
MATLAB run result or reason it was not run
```

Do not claim stable standing unless the robot can stand unsupported in repeated tests.

# Balance Model Identification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **For DeepSeek:** 必须先开启 teammates 模式再执行本计划。按任务顺序执行，每个任务完成后运行检查并单独提交。不要提交 `data/` 采集文件。不要继续经验扫参；本计划目标是让 MATLAB 从建模采集数据中辨识模型并给出候选 `BL` 参数。

**Goal:** Add safe model-identification support for the balance controller: block non-finite values in firmware, inject a bounded identification signal, and generate MATLAB-derived `BL,angle,rate,speed,pos` candidates from captured data.

**Architecture:** Keep `actuator_motor` as the only motor output path. `control_balance` remains responsible for computing the total balance RPM command, but adds finite-value gates and an optional bounded identification excitation term. MATLAB reads the existing 14-channel CSV captures, reconstructs four-state samples, identifies a discrete model with least squares, solves a small LQR problem without toolbox dependencies, and prints clipped firmware gains.

**Tech Stack:** Embedded C for CYT4BB7 in IAR Embedded Workbench, existing VOFA JustFloat UART telemetry, PowerShell serial capture scripts, MATLAB scripts using relative `data/` paths.

---

## Scope And Files

Engineering effort: medium-small firmware change plus one MATLAB modeling script.

Files to modify:

- `project/code/app_config.h`: finite/gain/excitation safety constants.
- `project/code/control_balance.h`: public identification excitation API.
- `project/code/control_balance.c`: finite gates, gain validation, bounded identification excitation.
- `project/code/host_command.c`: `BI,amp,period_ms` host command and STOP/B,0 excitation clear.
- `tools/collect_balance_data.ps1`: no required schema change; use it for new collection commands.
- `tools/analyze_balance_pd.m`: no required change for model identification.

Files to create:

- `tools/identify_balance_model.m`: MATLAB model identification and LQR gain calculation.
- `tools/test_identify_balance_model.ps1`: static checks for the MATLAB script.

Do not change:

- `project/code/bldc_foc_uart.c`
- `libraries/*`
- IAR project files, unless a build proves a missing project entry.
- CSV data under `data/`

## Behavior Contract

Existing commands remain valid:

```text
STOP
B,0
B,1
B,2
BP,kp,kd
BL,angle,rate,speed,pos
BZ
C,forward,turn
M,rpm
D,duty
P,kp,ki,kd
PL,kp,ki,kd
PR,kp,ki,kd
```

Add one command:

```text
BI,amp,period_ms
```

Meaning:

- `BI,amp,period_ms`: enable a square-wave identification excitation added to `balance_rpm`.
- `amp` is RPM and may be positive or negative. Firmware clamps absolute value.
- `period_ms` is the half-period for sign toggling.
- `BI,0,0`: disable identification excitation.
- `STOP` and `B,0` must disable identification excitation.
- `B,1` must not disable identification excitation, so scheduled collection can set `BI` before entering `B,2`.

Total balance command after this plan:

```text
feedback_rpm =
    K_angle * pitch_deg
  + K_rate  * pitch_rate_dps
  + K_speed * wheel_speed_rpm
  + K_pos   * wheel_pos_rev

balance_rpm = limit(feedback_rpm + ident_rpm, APP_BALANCE_RPM_LIMIT)
```

The telemetry `balance_rpm` remains the total motor command input used by MATLAB identification.

## Task 1: Add Finite And Gain Safety Constants

**Files:**

- Modify: `project/code/app_config.h`

- [ ] **Step 1: Add constants near existing balance constants**

In `project/code/app_config.h`, after `APP_BALANCE_WHEEL_POS_DECAY`, add:

```c
#define APP_BALANCE_FINITE_ABS_LIMIT     (100000.0f)
#define APP_BALANCE_GAIN_ABS_LIMIT       (1000.0f)
#define APP_BALANCE_IDENT_RPM_LIMIT      (20.0f)
#define APP_BALANCE_IDENT_MIN_PERIOD_MS  (100U)
#define APP_BALANCE_IDENT_MAX_PERIOD_MS  (2000U)
```

Meanings:

- `APP_BALANCE_FINITE_ABS_LIMIT`: rejects NaN, Inf, and impossible large float values before motor output.
- `APP_BALANCE_GAIN_ABS_LIMIT`: rejects accidental extreme gains from host commands.
- `APP_BALANCE_IDENT_RPM_LIMIT`: caps the modeling excitation to a small command.
- `APP_BALANCE_IDENT_MIN_PERIOD_MS` and `APP_BALANCE_IDENT_MAX_PERIOD_MS`: keep excitation slow enough for hardware and fast enough for identification.

- [ ] **Step 2: Run checks**

Run:

```powershell
git diff --check -- project/code/app_config.h
rg -n "APP_BALANCE_FINITE_ABS_LIMIT|APP_BALANCE_GAIN_ABS_LIMIT|APP_BALANCE_IDENT_RPM_LIMIT|APP_BALANCE_IDENT_MIN_PERIOD_MS|APP_BALANCE_IDENT_MAX_PERIOD_MS" project/code/app_config.h
```

Expected:

```text
No whitespace errors.
All five new constants are present.
```

- [ ] **Step 3: Commit**

```powershell
git add project/code/app_config.h
git commit -m "Add balance identification safety limits"
```

## Task 2: Block Non-Finite Balance Values Before Motor Output

**Files:**

- Modify: `project/code/control_balance.c`

- [ ] **Step 1: Add finite helper**

After `control_balance_limit_abs()`, add:

```c
static uint8 control_balance_is_finite(float value)
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
```

This avoids a dependency on `<math.h>` and works for NaN because `NaN != NaN`.

- [ ] **Step 2: Add input finite checks to the BALANCE_TEST safety gate**

In `control_balance_update()`, extend the existing safety gate:

```c
    if((APP_STATE_FAULT == app_state_get()) ||
       (APP_FALSE == imu->healthy) ||
       (APP_FALSE == wheel->online) ||
       (APP_FALSE == wheel->left_online) ||
       (APP_FALSE == wheel->right_online) ||
       (APP_FALSE == chassis->enable) ||
       (APP_FALSE == dt_valid) ||
       (APP_FALSE == control_balance_is_finite(imu->pitch)) ||
       (APP_FALSE == control_balance_is_finite(pitch_rate_dps)) ||
       (APP_FALSE == control_balance_is_finite(wheel_speed_rpm)) ||
       (APP_BALANCE_TEST_PITCH_LIMIT_DEG < control_balance_absf(imu->pitch)))
```

Keep the existing blocked branch behavior:

```c
        control_balance_diag.safety_blocked = APP_TRUE;
        if(APP_TRUE == dt_valid)
        {
            control_balance_reset_motion_state();
        }
        control_balance_stop_output();
        return;
```

- [ ] **Step 3: Add state and output finite checks before actuator call**

After computing and limiting `balance_rpm`, and after computing `output_left_rpm` and `output_right_rpm`, add:

```c
    if((APP_FALSE == control_balance_is_finite(control_balance_wheel_pos_rev)) ||
       (APP_FALSE == control_balance_is_finite(balance_rpm)) ||
       (APP_FALSE == control_balance_is_finite(output_left_rpm)) ||
       (APP_FALSE == control_balance_is_finite(output_right_rpm)))
    {
        control_balance_diag.safety_blocked = APP_TRUE;
        control_balance_reset_motion_state();
        control_balance_stop_output();
        return;
    }
```

Place this block before:

```c
    control_balance_diag.balance_rpm = balance_rpm;
```

- [ ] **Step 4: Reject non-finite and extreme four-state gains**

Replace the validation at the top of `control_balance_set_full_gain()` with:

```c
    if((0.0f > pitch_kp) ||
       (0.0f > pitch_rate_kd) ||
       (APP_FALSE == control_balance_is_finite(pitch_kp)) ||
       (APP_FALSE == control_balance_is_finite(pitch_rate_kd)) ||
       (APP_FALSE == control_balance_is_finite(wheel_speed_ks)) ||
       (APP_FALSE == control_balance_is_finite(wheel_pos_kp)) ||
       (APP_BALANCE_GAIN_ABS_LIMIT < control_balance_absf(pitch_kp)) ||
       (APP_BALANCE_GAIN_ABS_LIMIT < control_balance_absf(pitch_rate_kd)) ||
       (APP_BALANCE_GAIN_ABS_LIMIT < control_balance_absf(wheel_speed_ks)) ||
       (APP_BALANCE_GAIN_ABS_LIMIT < control_balance_absf(wheel_pos_kp)))
    {
        return;
    }
```

- [ ] **Step 5: Run checks**

Run:

```powershell
git diff --check -- project/code/control_balance.c
rg -n "control_balance_is_finite|APP_BALANCE_FINITE_ABS_LIMIT|APP_BALANCE_GAIN_ABS_LIMIT" project/code/control_balance.c
rg -n "bldc_foc|debug_read|debug_send" project/code/control_balance.c
```

Expected:

```text
No whitespace errors.
Finite checks are present.
No BLDC/debug dependency appears in control_balance.c.
```

- [ ] **Step 6: Commit**

```powershell
git add project/code/control_balance.c
git commit -m "Block non-finite balance outputs"
```

## Task 3: Add Bounded Identification Excitation

**Files:**

- Modify: `project/code/control_balance.h`
- Modify: `project/code/control_balance.c`

- [ ] **Step 1: Add public API**

In `project/code/control_balance.h`, after `control_balance_reset_motion_state_public()`, add:

```c
void control_balance_set_ident_excitation(float amp_rpm, uint32 period_ms, uint32 now_ms);
```

- [ ] **Step 2: Add static excitation state**

In `project/code/control_balance.c`, after `control_balance_wheel_pos_rev`, add:

```c
static float control_balance_ident_amp_rpm;
static uint32 control_balance_ident_period_ms;
static uint32 control_balance_ident_start_ms;
```

- [ ] **Step 3: Add excitation helper**

After `control_balance_reset_motion_state()`, add:

```c
static float control_balance_get_ident_rpm(uint32 now_ms)
{
    uint32 elapsed_ms;
    uint32 phase;

    if((0.0f == control_balance_ident_amp_rpm) ||
       (0U == control_balance_ident_period_ms))
    {
        return 0.0f;
    }

    elapsed_ms = now_ms - control_balance_ident_start_ms;
    phase = (elapsed_ms / control_balance_ident_period_ms) & 0x01U;
    return (0U == phase) ? control_balance_ident_amp_rpm : -control_balance_ident_amp_rpm;
}
```

- [ ] **Step 4: Initialize excitation state**

In `control_balance_init()`, before `control_balance_reset_derivative();`, add:

```c
    control_balance_ident_amp_rpm = 0.0f;
    control_balance_ident_period_ms = 0U;
    control_balance_ident_start_ms = 0U;
```

- [ ] **Step 5: Add excitation to the total balance command**

In `control_balance_update()`, add a local variable near the other floats:

```c
    float ident_rpm;
```

After the four-state control law and before limiting `balance_rpm`, insert:

```c
    ident_rpm = control_balance_get_ident_rpm(now_ms);
    balance_rpm += ident_rpm;
```

The resulting total `balance_rpm` is what telemetry already reports.

- [ ] **Step 6: Add public setter**

After `control_balance_reset_motion_state_public()`, add:

```c
void control_balance_set_ident_excitation(float amp_rpm, uint32 period_ms, uint32 now_ms)
{
    if((APP_FALSE == control_balance_is_finite(amp_rpm)) ||
       (APP_BALANCE_IDENT_RPM_LIMIT < control_balance_absf(amp_rpm)))
    {
        return;
    }

    if(0.0f == amp_rpm)
    {
        control_balance_ident_amp_rpm = 0.0f;
        control_balance_ident_period_ms = 0U;
        control_balance_ident_start_ms = now_ms;
        return;
    }

    if((APP_BALANCE_IDENT_MIN_PERIOD_MS > period_ms) ||
       (APP_BALANCE_IDENT_MAX_PERIOD_MS < period_ms))
    {
        return;
    }

    control_balance_ident_amp_rpm = amp_rpm;
    control_balance_ident_period_ms = period_ms;
    control_balance_ident_start_ms = now_ms;
}
```

- [ ] **Step 7: Run checks**

Run:

```powershell
git diff --check -- project/code/control_balance.c project/code/control_balance.h
rg -n "control_balance_set_ident_excitation|control_balance_get_ident_rpm|control_balance_ident_amp_rpm|APP_BALANCE_IDENT" project/code/control_balance.c project/code/control_balance.h
```

Expected:

```text
No whitespace errors.
Identification excitation API and helper are present.
```

- [ ] **Step 8: Commit**

```powershell
git add project/code/control_balance.c project/code/control_balance.h
git commit -m "Add balance identification excitation"
```

## Task 4: Add BI Host Command

**Files:**

- Modify: `project/code/host_command.c`

- [ ] **Step 1: Add local for period**

In `host_command_process_line()`, after `float pos_kp;`, add:

```c
    float period_ms_f;
```

- [ ] **Step 2: Clear excitation on STOP**

In the STOP command block, before `control_chassis_stop(now_ms);`, add:

```c
        control_balance_set_ident_excitation(0.0f, 0U, now_ms);
```

The full STOP block should begin:

```c
    if(APP_TRUE == host_command_match_stop(line))
    {
        control_balance_set_ident_excitation(0.0f, 0U, now_ms);
        control_chassis_stop(now_ms);
        control_balance_set_mode(BALANCE_MODE_OFF);
        actuator_motor_set_mode_stop();
        actuator_motor_record_command_error(APP_FALSE);
        return;
    }
```

- [ ] **Step 3: Add BI command block**

After the `BZ` command block and before the `BL` command block, add:

```c
    if(('B' == line[0]) && ('I' == line[1]) && (',' == line[2]) &&
       (APP_TRUE == host_command_parse_two_numbers(&line[3], &value, &period_ms_f)))
    {
        if(0.0f == value)
        {
            control_balance_set_ident_excitation(0.0f, 0U, now_ms);
            actuator_motor_record_command_error(APP_FALSE);
            return;
        }
        if((0.0f <= period_ms_f) && (4294967295.0f >= period_ms_f))
        {
            control_balance_set_ident_excitation(value, (uint32)period_ms_f, now_ms);
            actuator_motor_record_command_error(APP_FALSE);
            return;
        }
    }
```

This reuses the existing two-number parser and lets the controller setter enforce amplitude and period limits.

- [ ] **Step 4: Clear excitation on B,0**

In the `B,0` block, before `control_chassis_stop(now_ms);`, add:

```c
            control_balance_set_ident_excitation(0.0f, 0U, now_ms);
```

The `B,0` block should become:

```c
        if(0.0f == value)
        {
            control_balance_set_ident_excitation(0.0f, 0U, now_ms);
            control_chassis_stop(now_ms);
            control_balance_set_mode(BALANCE_MODE_OFF);
            actuator_motor_record_command_error(APP_FALSE);
            return;
        }
```

- [ ] **Step 5: Run checks**

Run:

```powershell
git diff --check -- project/code/host_command.c
rg -n "B'\\) && \\('I|period_ms_f|control_balance_set_ident_excitation" project/code/host_command.c
```

Expected:

```text
No whitespace errors.
BI command is present.
STOP and B,0 clear identification excitation.
```

- [ ] **Step 6: Commit**

```powershell
git add project/code/host_command.c
git commit -m "Add balance identification host command"
```

## Task 5: Add MATLAB Model Identification Script

**Files:**

- Create: `tools/identify_balance_model.m`
- Create: `tools/test_identify_balance_model.ps1`

- [ ] **Step 1: Create `tools/identify_balance_model.m`**

Create this complete MATLAB script:

```matlab
% Identify a local four-state balance model and compute candidate BL gains.
%
% Run from repository root in MATLAB:
%   tools/identify_balance_model
%
% Inputs:
%   data/balance_capture_*.csv
%
% Outputs:
%   data/balance_model_summary.csv
%   data/balance_model_fit.png

clear; clc;

dataDir = fullfile("data");
files = dir(fullfile(dataDir, "balance_capture_*.csv"));
if isempty(files)
    error("No balance capture CSV files found under data/.");
end

notes = strings(numel(files), 1);
for i = 1:numel(files)
    one = readtable(fullfile(dataDir, files(i).name));
    if ismember("note", string(one.Properties.VariableNames)) && height(one) > 0
        notes(i) = string(one.note(1));
    end
    if strlength(notes(i)) == 0
        notes(i) = string(files(i).name);
    end
end

uniqueNotes = unique(notes, "stable");
keepFiles = repmat(files(1), numel(uniqueNotes), 1);
for i = 1:numel(uniqueNotes)
    idx = find(notes == uniqueNotes(i));
    [~, newestLocal] = max([files(idx).datenum]);
    keepFiles(i) = files(idx(newestLocal));
end

allX = [];
allY = [];
allU = [];
summary = table();
series = struct([]);

for i = 1:numel(keepFiles)
    file = keepFiles(i);
    raw = readtable(fullfile(dataDir, file.name));
    required = ["elapsed_s", "time_ms", "balance_mode", "feedback_online", ...
        "pitch_deg", "pitch_rate_dps", "balance_rpm", "left_motor_rpm", "right_motor_rpm"];
    if ~all(ismember(required, string(raw.Properties.VariableNames)))
        warning("Skipping %s: missing required model columns.", file.name);
        continue;
    end

    note = string(raw.note(1));
    if strlength(note) == 0
        note = string(file.name);
    end

    active = raw(abs(raw.balance_mode - 2.0) < 0.01, :);
    active = active(active.feedback_online >= 0.5, :);
    finiteMask = isfinite(active.pitch_deg) & ...
        isfinite(active.pitch_rate_dps) & ...
        isfinite(active.balance_rpm) & ...
        isfinite(active.left_motor_rpm) & ...
        isfinite(active.right_motor_rpm) & ...
        isfinite(active.time_ms);
    active = active(finiteMask, :);

    localMask = (abs(active.pitch_deg) <= 18.0) & (abs(active.balance_rpm) <= 140.0);
    active = active(localMask, :);

    if height(active) < 200
        warning("Skipping %s: only %d usable local rows.", file.name, height(active));
        continue;
    end

    dt = [median(diff(active.time_ms)); diff(active.time_ms)] / 1000.0;
    dt(dt <= 0.0) = median(dt(dt > 0.0));
    avgMotorRpm = (active.left_motor_rpm + active.right_motor_rpm) / 2.0;
    wheelPos = cumsum(avgMotorRpm .* dt / 60.0);
    wheelPos = wheelPos - wheelPos(1);

    x = [active.pitch_deg, active.pitch_rate_dps, avgMotorRpm, wheelPos]';
    u = active.balance_rpm';

    x0 = x(:, 1:end-1);
    x1 = x(:, 2:end);
    u0 = u(:, 1:end-1);

    allX = [allX, x0]; %#ok<AGROW>
    allY = [allY, x1]; %#ok<AGROW>
    allU = [allU, u0]; %#ok<AGROW>

    series(end + 1).note = note; %#ok<SAGROW>
    series(end).file = string(file.name);
    series(end).t = active.elapsed_s - active.elapsed_s(1);
    series(end).x = x;
    series(end).u = u;

    summary = [summary; table(note, string(file.name), height(raw), height(active), ...
        min(active.elapsed_s), max(active.elapsed_s), median(diff(active.time_ms)), ...
        local_percentile(abs(active.pitch_deg), 95), ...
        local_percentile(abs(active.pitch_rate_dps), 95), ...
        local_percentile(abs(active.balance_rpm), 95), ...
        "VariableNames", ["note", "file", "rows_total", "rows_model", ...
        "time_start_s", "time_end_s", "dt_median_ms", ...
        "pitch_abs_p95", "pitch_rate_abs_p95", "balance_rpm_abs_p95"])]; %#ok<AGROW>
end

if size(allX, 2) < 500
    error("Not enough usable model samples. Need at least 500; got %d.", size(allX, 2));
end

phi = [allX; allU];
theta = allY * pinv(phi);
A = theta(:, 1:4);
B = theta(:, 5);

rankPhi = rank(phi);
condPhi = cond(phi * phi');
pred = theta * phi;
err = allY - pred;
rmse = sqrt(mean(err .^ 2, 2));

Q = diag([10.0, 0.2, 0.02, 2.0]);
R = 0.08;
[K_lqr, P, dareIters] = local_dlqr(A, B, Q, R); %#ok<ASGLU>
firmwareGain = -K_lqr;
clippedGain = firmwareGain;
clippedGain(1) = min(max(clippedGain(1), 0.0), 6.0);
clippedGain(2) = min(max(clippedGain(2), 0.0), 0.4);
clippedGain(3) = min(max(clippedGain(3), -0.2), 0.2);
clippedGain(4) = min(max(clippedGain(4), -1.0), 1.0);

modelSummary = table(rankPhi, condPhi, rmse(1), rmse(2), rmse(3), rmse(4), ...
    firmwareGain(1), firmwareGain(2), firmwareGain(3), firmwareGain(4), ...
    clippedGain(1), clippedGain(2), clippedGain(3), clippedGain(4), dareIters, ...
    "VariableNames", ["rank_phi", "cond_phi", "rmse_pitch_deg", ...
    "rmse_pitch_rate_dps", "rmse_wheel_rpm", "rmse_wheel_pos_rev", ...
    "raw_k_angle", "raw_k_rate", "raw_k_speed", "raw_k_pos", ...
    "clip_k_angle", "clip_k_rate", "clip_k_speed", "clip_k_pos", "dare_iters"]);
writetable(modelSummary, fullfile(dataDir, "balance_model_summary.csv"));
writetable(summary, fullfile(dataDir, "balance_model_input_summary.csv"));

disp("Input captures used for model:");
disp(summary(:, ["note", "file", "rows_model", "pitch_abs_p95", "balance_rpm_abs_p95"]));
disp("Identified A:");
disp(A);
disp("Identified B:");
disp(B);
disp("One-step RMSE [pitch, pitch_rate, wheel_rpm, wheel_pos]:");
disp(rmse');
disp("Raw firmware BL gains:");
disp(firmwareGain);
disp("Clipped firmware BL gains:");
disp(clippedGain);
fprintf("Suggested command: BL,%.3f,%.3f,%.3f,%.3f\n", ...
    clippedGain(1), clippedGain(2), clippedGain(3), clippedGain(4));

figure("Name", "Balance model one-step fit", "Color", "w");
tiledlayout(4, 1, "TileSpacing", "compact");
labels = ["pitch deg", "pitch rate dps", "avg motor rpm", "wheel pos rev"];
for stateIndex = 1:4
    nexttile;
    hold on; grid on;
    plot(allY(stateIndex, 1:min(2000, size(allY, 2))), "DisplayName", "measured");
    plot(pred(stateIndex, 1:min(2000, size(pred, 2))), "DisplayName", "one-step fit");
    ylabel(labels(stateIndex));
    if stateIndex == 1
        legend("Location", "best");
    end
end
xlabel("sample");
saveas(gcf, fullfile(dataDir, "balance_model_fit.png"));

if rankPhi < 5
    warning("Regressor rank is %d, expected 5. Collect BI excitation data before trusting gains.", rankPhi);
end
if condPhi > 1.0e8
    warning("Regressor condition is high: %.3g. Gains may be unreliable.", condPhi);
end
if clippedGain(1) ~= firmwareGain(1) || clippedGain(2) ~= firmwareGain(2)
    warning("Angle/rate gains were clipped. Treat the command as a cautious first test, not final tuning.");
end

function [K, P, iterations] = local_dlqr(A, B, Q, R)
    P = Q;
    for iterations = 1:1000
        den = R + (B' * P * B);
        K = den \ (B' * P * A);
        nextP = (A' * P * A) - (A' * P * B * K) + Q;
        if norm(nextP - P, "fro") < 1.0e-9
            P = nextP;
            return;
        end
        P = nextP;
    end
end

function value = local_percentile(values, percent)
    values = sort(values(:));
    if isempty(values)
        value = NaN;
        return;
    end

    index = 1 + ((numel(values) - 1) * percent / 100.0);
    low = floor(index);
    high = ceil(index);
    if low == high
        value = values(low);
    else
        weight = index - low;
        value = (values(low) * (1.0 - weight)) + (values(high) * weight);
    end
end
```

- [ ] **Step 2: Create `tools/test_identify_balance_model.ps1`**

Create this complete static test:

```powershell
$ErrorActionPreference = "Stop"

$scriptPath = Join-Path $PSScriptRoot "identify_balance_model.m"
if(-not (Test-Path -LiteralPath $scriptPath)) {
    throw "Missing tools/identify_balance_model.m"
}

$text = Get-Content -LiteralPath $scriptPath -Raw

function Assert-Contains {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )

    if($Text -notmatch $Pattern) {
        throw $Message
    }
}

function Assert-NotContains {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )

    if($Text -match $Pattern) {
        throw $Message
    }
}

Assert-NotContains $text '[A-Za-z]:\\' "script must not contain Windows absolute paths"
Assert-Contains $text 'fullfile\("data"' "script should use relative data/ paths"
Assert-Contains $text 'balance_capture_\*\.csv' "script should read balance capture CSV files"
Assert-Contains $text 'unique\(notes' "script should group by note"
Assert-Contains $text 'max\(\[files\(idx\)\.datenum\]\)' "script should keep newest capture per note"
Assert-Contains $text 'pinv\(phi\)' "script should identify model with least squares"
Assert-Contains $text 'local_dlqr' "script should solve LQR locally"
Assert-NotContains $text '\bdlqr\(' "script should not require Control System Toolbox dlqr"
Assert-Contains $text 'balance_model_summary\.csv' "script should save model summary"
Assert-Contains $text 'balance_model_fit\.png' "script should save fit figure"
Assert-Contains $text 'Suggested command: BL' "script should print BL command"
Assert-Contains $text 'isfinite' "script should filter non-finite rows"

Write-Host "identify_balance_model static checks passed"
```

- [ ] **Step 3: Run checks**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\test_identify_balance_model.ps1
git diff --check -- tools/identify_balance_model.m tools/test_identify_balance_model.ps1
rg -n "local_dlqr|pinv\\(phi\\)|balance_model_summary|Suggested command: BL|isfinite" tools/identify_balance_model.m tools/test_identify_balance_model.ps1
```

Expected:

```text
identify_balance_model static checks passed
No whitespace errors.
Model identification script contains local LQR and finite filtering.
```

- [ ] **Step 4: Commit**

```powershell
git add tools/identify_balance_model.m tools/test_identify_balance_model.ps1
git commit -m "Add MATLAB balance model identification"
```

## Task 6: Final Static And IAR Validation

**Files:**

- Inspect modified files only.

- [ ] **Step 1: Run full static checks**

Run:

```powershell
git diff --check
powershell -NoProfile -ExecutionPolicy Bypass -File tools\test_collect_balance_data.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tools\test_analyze_balance_pd.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tools\test_identify_balance_model.ps1
rg -n "bldc_foc|debug_read|debug_send" project/code/control_balance.c project/code/control_chassis.c
rg -n "BI,|control_balance_set_ident_excitation|control_balance_is_finite|APP_BALANCE_IDENT" project/code tools
```

Expected:

```text
No whitespace errors.
All PowerShell tests pass.
No BLDC/debug dependency appears in control_balance/control_chassis.
BI command and finite gates are present.
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

- [ ] **Step 3: Commit any final build-only fix**

If the IAR build requires a syntax or include fix, make the smallest change and commit:

```powershell
git add project/code
git commit -m "Fix balance model identification build issues"
```

If no fix is needed, do not create an empty commit.

## Task 7: Model-Identification Collection And MATLAB Run

**Files:**

- No source files should be modified in this task.
- Do not commit CSV or PNG files under `data/`.

- [ ] **Step 1: Flash firmware**

Flash the `cyt4bb7_cm_7_0` build that includes `BI` and finite gates.

- [ ] **Step 2: Verify BI can be disabled safely**

Run this 5-second standby capture:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 `
  -Duration 5 `
  -Commands "0:STOP;0.5:BI,0,0;1:B,1" `
  -Note "id_bi_off_standby"
```

Expected:

```text
Rows are collected.
No NaN appears in the CSV.
balance_mode is 1 during standby after B,1.
```

- [ ] **Step 3: Collect small-excitation model data**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 `
  -Duration 30 `
  -Commands "0:STOP;0.5:BL,4,0.2,0,0;0.8:BI,8,500;1:B,1;2:B,2;2.2:C,0,0;28:BI,0,0" `
  -Note "id_bl4_d020_bi8_p500"
```

Expected:

```text
The robot remains controllable.
feedback_online stays 1.
No NaN appears in the CSV.
balance_rpm shows a small square-wave component on top of feedback.
```

If the robot becomes more unstable than the previous `BL,4,0.2,0,0` baseline, stop and collect a smaller excitation:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 `
  -Duration 30 `
  -Commands "0:STOP;0.5:BL,4,0.2,0,0;0.8:BI,5,700;1:B,1;2:B,2;2.2:C,0,0;28:BI,0,0" `
  -Note "id_bl4_d020_bi5_p700"
```

- [ ] **Step 4: Collect a second excitation period if the first capture is stable**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 `
  -Duration 30 `
  -Commands "0:STOP;0.5:BL,4,0.2,0,0;0.8:BI,8,900;1:B,1;2:B,2;2.2:C,0,0;28:BI,0,0" `
  -Note "id_bl4_d020_bi8_p900"
```

Expected:

```text
The robot remains controllable.
No NaN appears in the CSV.
The two excitation captures have different BI periods for better model rank.
```

- [ ] **Step 5: Run MATLAB identification**

On a machine with MATLAB, run from repository root:

```matlab
tools/identify_balance_model
```

Expected outputs:

```text
data/balance_model_summary.csv
data/balance_model_input_summary.csv
data/balance_model_fit.png
Console output containing "Suggested command: BL,..."
```

- [ ] **Step 6: Judge whether the model is usable**

Use the script output:

```text
rank_phi must be 5.
cond_phi should be below 1.0e8.
rmse_pitch_deg should be materially smaller than the current pitch p95.
The suggested clipped gain must not exceed the clipping limits printed by the script.
```

If `rank_phi < 5` or `cond_phi > 1.0e8`, do not trust the suggested gains. Collect another excitation capture:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 `
  -Duration 30 `
  -Commands "0:STOP;0.5:BL,4,0.2,0,0;0.8:BI,10,650;1:B,1;2:B,2;2.2:C,0,0;28:BI,0,0" `
  -Note "id_bl4_d020_bi10_p650"
```

- [ ] **Step 7: Hardware-test the MATLAB candidate cautiously**

Use the `Suggested command: BL,...` from MATLAB, but test for only 10 seconds first:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 `
  -Duration 10 `
  -Commands "0:STOP;0.5:<PASTE_MATLAB_BL_COMMAND>;1:B,1;2:B,2;2.2:C,0,0" `
  -Note "model_candidate_short"
```

Replace `<PASTE_MATLAB_BL_COMMAND>` with the exact `BL,...` string printed by MATLAB.

Expected:

```text
No NaN appears.
feedback_online stays 1.
No row exceeds abs(pitch_deg) > 20.
```

Only after the 10-second test passes, collect 30 seconds:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 `
  -Duration 30 `
  -Commands "0:STOP;0.5:<PASTE_MATLAB_BL_COMMAND>;1:B,1;2:B,2;2.2:C,0,0" `
  -Note "model_candidate_30s"
```

- [ ] **Step 8: Final handoff**

DeepSeek should report:

```text
Commits created
Files changed
Static checks run
IAR build result
BI standby result
Model excitation capture notes and files
MATLAB rank_phi and cond_phi
MATLAB one-step RMSE
MATLAB suggested BL command
10-second candidate test result
30-second candidate test result, if run
```

Do not claim a final stable standing parameter unless the robot stands unsupported in repeated hardware tests.

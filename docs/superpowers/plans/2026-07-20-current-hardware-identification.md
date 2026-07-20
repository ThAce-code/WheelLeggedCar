# Current-Hardware Identification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add safe, reproducible PowerShell collectors and MATLAB fitters for the current BLDC installation and the four-state balance LQR model.

**Architecture:** Two collectors wrap the existing 72-float UART collectors and expose preview-only safety contracts. Two MATLAB scripts consume only newly prefixed captures, fit signed motor dynamics first, then fit and validate the balance `A`/`B` matrices without writing firmware.

**Tech Stack:** Windows PowerShell 5.1, MATLAB with optional System Identification and Control System toolboxes, existing 72-float CSV telemetry.

## Global Constraints

- Work only in `D:\smartcar\WheelLeggedCar_cyt4bb7_v1\.worktree\balance-fast-mode-spec`.
- Preserve I8 as physical right-wheel RPM and I9 as physical left-wheel RPM through the existing CSV column names.
- Never read legacy `data/balance_capture_*.csv` files in the new balance fitter.
- Never open the serial port in `-Preview` mode.
- Require `-AllowMotion` before every real motion capture and retain the underlying collector's final `STOP`.
- Do not write fitted gains into firmware.
- MATLAB is unavailable locally; PowerShell tests and source-contract tests are the local gate, while numerical MATLAB validation happens on the other computer.

---

### Task 1: Signed Open-Duty Motor Collector

**Files:**
- Create: `tools/test_collect_motor_identification.ps1`
- Create: `tools/collect_motor_identification.ps1`

**Interfaces:**
- Consumes: `tools/collect_bldc_diagnostics.ps1 -Port <string> -Baud <int> -Duration <double> -Commands <string> -Out <string> -Note <string> -AllowMotion`.
- Produces: `data/motor_ident_<yyyyMMdd_HHmmss>.csv` plus the existing `.summary.json`; public switches are `-AllowMotion` and `-Preview`.

- [ ] **Step 1: Write the failing collector contract test**

Create `tools/test_collect_motor_identification.ps1` with an `Assert-True` helper and these executable checks:

```powershell
$preview = & powershell -ExecutionPolicy Bypass -File tools/collect_motor_identification.ps1 -Preview -Out data/motor_preview.csv 2>&1
Assert-True ($LASTEXITCODE -eq 0) "Preview must not open COM."
$text = $preview -join "`n"
Assert-True ($text -match 'duration: 14(\.0+)? s') "Preview must print 14 seconds."
Assert-True ($text -match '0:STOP;0\.5:D,300;2:STOP;2\.75:D,600;4\.25:STOP;5:D,900;6\.5:STOP;7\.25:D,-300;8\.75:STOP;9\.5:D,-600;11:STOP;11\.75:D,-900;13\.25:STOP') "Preview schedule differs from the design."
Assert-True ($text -match 'data[\\/]motor_preview\.csv') "Preview must print the requested output."

$saved = $ErrorActionPreference
$ErrorActionPreference = "Continue"
$blocked = & powershell -ExecutionPolicy Bypass -File tools/collect_motor_identification.ps1 -Out data/blocked.csv 2>&1
$blockedExit = $LASTEXITCODE
$ErrorActionPreference = $saved
Assert-True ($blockedExit -ne 0) "Real collection must require AllowMotion."
Assert-True (($blocked -join "`n") -match 'AllowMotion') "The motion gate must explain authorization."

$source = Get-Content tools/collect_motor_identification.ps1 -Raw
Assert-True ($source -match 'collect_bldc_diagnostics\.ps1') "Use the 72-float BLDC collector."
Assert-True ($source -notmatch '-NoStopOnExit') "Do not suppress final STOP."
Assert-True ($source -match 'motor_ident_\$stamp\.csv') "Use the new capture prefix."
```

- [ ] **Step 2: Run the test and verify the RED state**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_collect_motor_identification.ps1
```

Expected: nonzero exit with `The argument 'tools/collect_motor_identification.ps1' ... does not exist`.

- [ ] **Step 3: Implement the fixed collector wrapper**

Create a PowerShell script with this exact public contract and schedule:

```powershell
param(
    [string]$Port = "COM6",
    [int]$Baud = 460800,
    [string]$Out = "",
    [string]$Note = "motor_signed_duty_ident",
    [switch]$AllowMotion,
    [switch]$Preview
)
$ErrorActionPreference = "Stop"
$durationSeconds = 14.0
$commands = "0:STOP;0.5:D,300;2:STOP;2.75:D,600;4.25:STOP;5:D,900;6.5:STOP;7.25:D,-300;8.75:STOP;9.5:D,-600;11:STOP;11.75:D,-900;13.25:STOP"
if([string]::IsNullOrWhiteSpace($Out)) {
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $Out = "data/motor_ident_$stamp.csv"
}
Write-Host ("commands: {0}" -f $commands)
Write-Host ("duration: {0:F1} s" -f $durationSeconds)
Write-Host ("output: {0}" -f $Out)
if($Preview) { return }
if(-not $AllowMotion) {
    throw "Motion is blocked. Suspend both wheels, keep a physical cutoff ready, then add -AllowMotion."
}
$collector = Join-Path $PSScriptRoot "collect_bldc_diagnostics.ps1"
if(-not (Test-Path -LiteralPath $collector)) { throw "Missing collector: $collector" }
Write-Warning "Open-duty steps are enabled. Suspend both wheels and keep a physical cutoff ready."
& $collector -Port $Port -Baud $Baud -Duration $durationSeconds -Commands $commands -Out $Out -Note $Note -AllowMotion
if($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
```

- [ ] **Step 4: Run the contract test and preview**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_collect_motor_identification.ps1
powershell -ExecutionPolicy Bypass -File tools/collect_motor_identification.ps1 -Preview
```

Expected: `motor identification collector checks passed`; preview lists all six signed steps and never touches COM6.

- [ ] **Step 5: Commit Task 1**

```powershell
git add tools/test_collect_motor_identification.ps1 tools/collect_motor_identification.ps1
git commit -m "Add signed motor identification collector"
```

---

### Task 2: Multi-Period Balance Dataset Collector

**Files:**
- Create: `tools/test_collect_balance_lqr_dataset.ps1`
- Create: `tools/collect_balance_lqr_dataset.ps1`

**Interfaces:**
- Consumes: explicit left/right `P` gains, explicit `BL` gains, one of `short|medium|long`, and `tools/collect_balance_data.ps1`.
- Produces: one `data/balance_lqr_id_<profile>_<timestamp>.csv` capture per invocation.

- [ ] **Step 1: Write the failing profile and safety tests**

The test invokes all profiles with the same explicit gains:

```powershell
$common = @('-LeftMotorKp','2','-LeftMotorKi','40','-LeftMotorKd','0',
            '-RightMotorKp','2','-RightMotorKi','40','-RightMotorKd','0',
            '-PitchKp','18','-PitchRateKd','8','-WheelSpeedKs','-3',
            '-WheelPositionKp','0','-PitchSetpointDeg','0','-Preview')
$expected = @{ short='BI,6,300'; medium='BI,8,600'; long='BI,10,1000' }
foreach($profile in $expected.Keys) {
    $preview = & powershell -ExecutionPolicy Bypass -File tools/collect_balance_lqr_dataset.ps1 -Profile $profile @common 2>&1
    Assert-True ($LASTEXITCODE -eq 0) "$profile preview failed."
    $text = $preview -join "`n"
    Assert-True ($text -match [regex]::Escape($expected[$profile])) "$profile BI mapping is wrong."
    Assert-True ($text -match 'active balance window: 20000 ms') "Active window must be 20 seconds."
    Assert-True ($text -match '2:B,2;4:BI,' -and $text -match '24:BI,0,0;24\.5:STOP') "Balance settle or shutdown schedule is missing."
}
```

Also assert that a non-preview call without `-AllowMotion` exits nonzero, an invalid profile is rejected by parameter binding, the source calls `collect_balance_data.ps1`, and the source does not contain `-NoStopOnExit`.

- [ ] **Step 2: Run the test and verify the RED state**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_collect_balance_lqr_dataset.ps1
```

Expected: nonzero exit because `tools/collect_balance_lqr_dataset.ps1` does not exist.

- [ ] **Step 3: Implement invariant formatting and exact scheduling**

Declare all eleven gain arguments with `[Parameter(Mandatory=$true)][double]`, declare `[Parameter(Mandatory=$true)][ValidateSet('short','medium','long')][string]$Profile`, and format every number through:

```powershell
function Format-Number([double]$Value) {
    return $Value.ToString("0.######", [System.Globalization.CultureInfo]::InvariantCulture)
}
```

Resolve the profile with:

```powershell
$profiles = @{
    short  = @{ Amplitude = 6; HalfPeriodMs = 300 }
    medium = @{ Amplitude = 8; HalfPeriodMs = 600 }
    long   = @{ Amplitude = 10; HalfPeriodMs = 1000 }
}
$selected = $profiles[$Profile]
```

Construct the exact schedule using invariant-formatted parameters:

```text
0:STOP;0.25:PL,LKP,LKI,LKD;0.5:PR,RKP,RKI,RKD;0.75:BL,PKP,PKD,KS,KPOS;1:BS,SETPOINT;1.25:BD,0,0,0;1.5:B,1;1.75:C,0,0;2:B,2;4:BI,AMP,HALF;24:BI,0,0;24.5:STOP
```

Set duration to `25.5`, print `active balance window: 20000 ms`, return before the motion gate when `-Preview` is present, otherwise require `-AllowMotion`, warn the operator, and call the existing collector without `-NoStopOnExit`.

- [ ] **Step 4: Run the contract tests and three previews**

Run the test, then preview short/medium/long using the explicit values from Step 1. Expected: all assertions pass and each preview differs only at the BI command and output prefix.

- [ ] **Step 5: Commit Task 2**

```powershell
git add tools/test_collect_balance_lqr_dataset.ps1 tools/collect_balance_lqr_dataset.ps1
git commit -m "Add balance LQR dataset collector"
```

---

### Task 3: Signed Motor Actuator MATLAB Fitter

**Files:**
- Create: `tools/test_fit_motor_actuator_model_static.ps1`
- Create: `tools/fit_motor_actuator_model.m`

**Interfaces:**
- Consumes: `data/motor_ident_*.csv` with `elapsed_s`, `expected_mode`, `expected_open_duty`, `feedback_online`, `left_motor_rpm`, `right_motor_rpm`, `left_duty`, and `right_duty`.
- Produces: `data/motor_actuator_model.csv`, `.mat`, and `_fit.png`.

- [ ] **Step 1: Write a failing static contract test**

Assert that the MATLAB source contains the new input glob, all required columns, both physical wheel columns, the signed regressor `sign(duty)`, `rank`, `cond`, train/validation separation, per-direction residuals, `iddata`/`tfest` guarded by availability checks, all three output names, and no reference to `app_config.h` or `Set-Content`.

Use literal assertions for these searchable contracts:

```powershell
Assert-True ($source -match 'motor_ident_\*\.csv') "Wrong input glob."
Assert-True ($source -match 'phi\s*=\s*\[rpm\(1:end-1\).*duty\(1:end-1\).*sign\(duty\(1:end-1\)\)') "Signed regressor missing."
Assert-True ($source -match 'motor_actuator_model\.csv') "CSV output missing."
Assert-True ($source -match 'motor_actuator_model\.mat') "MAT output missing."
Assert-True ($source -match 'motor_actuator_model_fit\.png') "Plot output missing."
Assert-True ($source -notmatch 'app_config\.h') "Fitter must not edit firmware."
```

- [ ] **Step 2: Run the test and verify the RED state**

Run `powershell -ExecutionPolicy Bypass -File tools/test_fit_motor_actuator_model_static.ps1` and expect failure for a missing MATLAB source.

- [ ] **Step 3: Implement signed per-wheel least squares**

For each file, retain feedback-online rows with finite duty/RPM, infer sample time from `elapsed_s`, and keep only `expected_mode == "duty"`. Split complete files into training and validation sets; when only one file exists, use the first 70 percent of contiguous signed-step segments for training and the remainder for validation.

For each physical wheel execute:

```matlab
phi = [rpm(1:end-1)'; duty(1:end-1)'; sign(duty(1:end-1))'];
y = rpm(2:end)';
if rank(phi) < 3
    error("Signed motor regressor rank is %d; expected 3.", rank(phi));
end
theta = y / phi;
a = theta(1); b = theta(2); c = theta(3);
tau_s = -dt_s / log(abs(a));
deadzone_duty = abs(c / b);
prediction = theta * phi;
rmse = sqrt(mean((y - prediction).^2));
fit_percent = 100.0 * (1.0 - norm(y - prediction) / norm(y - mean(y)));
condition_number = cond(phi * phi');
```

Calculate positive and negative residual RMSE separately, reject missing direction data, generate one summary row per wheel, save raw matrices and diagnostics to MAT, and plot measured versus predicted RPM for both wheels. Optional `iddata` and `tfest` execute only when `exist(...,'file') == 2`; toolbox failure is a warning, not a replacement for the signed fit.

Derive a transparent first-order PI starting point without changing firmware:

```matlab
dc_gain_rpm_per_duty = b / (1.0 - a);
lambda_s = max(4.0 * dt_s, tau_s / 3.0);
candidate_kp_duty_per_rpm = tau_s / (dc_gain_rpm_per_duty * lambda_s);
candidate_ki_duty_per_rpm_s = 1.0 / (dc_gain_rpm_per_duty * lambda_s);
```

Export these as candidates with the fit diagnostics and label them as bench-test starting points.

- [ ] **Step 4: Run the static test**

Run the PowerShell static test. Expected: `motor actuator MATLAB contract checks passed`. Do not claim numerical correctness until MATLAB runs on the other computer.

- [ ] **Step 5: Commit Task 3**

```powershell
git add tools/test_fit_motor_actuator_model_static.ps1 tools/fit_motor_actuator_model.m
git commit -m "Add signed motor actuator model fitter"
```

---

### Task 4: Balance A/B Identification and LQR Candidate Fitter

**Files:**
- Create: `tools/test_fit_balance_lqr_model_static.ps1`
- Create: `tools/fit_balance_lqr_model.m`

**Interfaces:**
- Consumes: only `data/balance_lqr_id_*.csv` files from all three BI profiles.
- Produces: `data/balance_lqr_ab.mat`, `data/balance_lqr_model_quality.csv`, `data/balance_lqr_candidates.csv`, and `data/balance_lqr_fit.png`.

- [ ] **Step 1: Write a failing static contract test**

Assert the new glob is present and `balance_capture_*.csv` is absent. Require literal checks for `imu_age_ms <= 15`, `abs(pitch_deg) <= 15`, `feedback_online`, active `balance_mode`, `firmware_frame_sequence`, per-file wheel-position reset, rank 5, condition number, training/validation RMSE, multi-step validation, eigenvalues, `dlqr`, local DARE fallback, all four outputs, and no firmware write target.

- [ ] **Step 2: Run the test and verify the RED state**

Run `powershell -ExecutionPolicy Bypass -File tools/test_fit_balance_lqr_model_static.ps1` and expect failure because the MATLAB file is missing.

- [ ] **Step 3: Build each capture without crossing file or sequence boundaries**

Require these columns:

```matlab
required = ["elapsed_s","time_ms","balance_mode","feedback_online", ...
    "pitch_deg","pitch_rate_dps","balance_rpm","left_motor_rpm", ...
    "right_motor_rpm","firmware_frame_sequence","imu_age_ms", ...
    "balance_output_limit_rpm","last_command"];
```

For each file, verify its BI profile from `last_command`, compute `dt_s` from adjacent `time_ms`, reset `wheel_position_rev` to zero at the first accepted row, and create a transition only when both rows are healthy and `sequence(k+1) == sequence(k)+1`. Healthy means balance mode 2, feedback at least 0.5, IMU age at most 15 ms, finite state/input, pitch at most 15 degrees absolute, and command below 90 percent of the runtime limit.

- [ ] **Step 4: Fit and gate the model**

Assign complete files to training and validation so every profile contributes and at least one complete capture is validation. Fit:

```matlab
phi_train = [x0_train; u0_train];
if rank(phi_train) ~= 5
    error("Balance regressor rank is %d; expected 5.", rank(phi_train));
end
theta = x1_train / phi_train;
A = theta(:, 1:4);
B = theta(:, 5);
condition_number = cond(phi_train * phi_train');
train_prediction = theta * phi_train;
validation_prediction = theta * phi_validation;
train_rmse = sqrt(mean((x1_train - train_prediction).^2, 2));
validation_rmse = sqrt(mean((x1_validation - validation_prediction).^2, 2));
open_loop_eigenvalues = eig(A);
```

For multi-step validation, seed each contiguous validation segment with its measured first state and propagate `xhat(:,k+1) = A*xhat(:,k) + B*u(k)` without resetting inside the segment. Report per-file total, accepted, rejected, and rejection-reason counts.

- [ ] **Step 5: Compute unclipped LQR candidates**

Sweep explicit diagonal weights:

```matlab
qPitch = [10, 30, 60];
qRate = [0.5, 2, 10, 40];
qSpeed = [0.005, 0.02, 0.08];
qPosition = [0.2, 1, 4];
rInput = [0.02, 0.05, 0.1];
```

For every combination compute `K`, closed-loop eigenvalues of `A-B*K`, spectral radius, and DARE convergence. Use `dlqr` when available and cross-check it against a local iterative DARE solver; export raw gains only. If any required metric is non-finite, any profile is absent, or a requested candidate required clipping, throw an error instead of printing a recommended `BL` command.

Write the results explicitly:

```matlab
save(fullfile("data", "balance_lqr_ab.mat"), "A", "B", "train_rmse", ...
    "validation_rmse", "multi_step_rmse", "open_loop_eigenvalues", ...
    "condition_number", "rank_phi");
writetable(model_quality, fullfile("data", "balance_lqr_model_quality.csv"));
writetable(lqr_candidates, fullfile("data", "balance_lqr_candidates.csv"));
saveas(gcf, fullfile("data", "balance_lqr_fit.png"));
```

- [ ] **Step 6: Run the static test and the full local suite**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_fit_balance_lqr_model_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_collect_motor_identification.ps1
powershell -ExecutionPolicy Bypass -File tools/test_collect_balance_lqr_dataset.ps1
powershell -ExecutionPolicy Bypass -File tools/test_fit_motor_actuator_model_static.ps1
git diff --check
```

Expected: four explicit pass messages and no whitespace errors. MATLAB numerical execution remains a hardware-computer gate.

- [ ] **Step 7: Commit Task 4**

```powershell
git add tools/test_fit_balance_lqr_model_static.ps1 tools/fit_balance_lqr_model.m
git commit -m "Add balance A B and LQR model fitter"
```

---

### Task 5: Operator Commands and Cross-Contract Verification

**Files:**
- Modify: `docs/superpowers/specs/2026-07-20-current-hardware-identification-design.md`

**Interfaces:**
- Consumes: the four implemented scripts and their exact parameters.
- Produces: copy-paste commands for preview, motor capture, three balance captures, and MATLAB execution.

- [ ] **Step 1: Add concrete operator commands**

Add one preview and one hardware command for motor identification; add short, medium, and long balance commands with explicit gain arguments; add MATLAB commands `tools/fit_motor_actuator_model` and `tools/fit_balance_lqr_model`. State that the wheels must be suspended for open-duty motor identification and the vehicle must be physically caught for BI balance capture.

- [ ] **Step 2: Verify source and documentation agree**

Run all four PowerShell tests, run each collector in preview mode, compare their printed schedule and parameter names to the documentation, then run `git diff --check`.

- [ ] **Step 3: Commit the handoff documentation**

```powershell
git add docs/superpowers/specs/2026-07-20-current-hardware-identification-design.md
git commit -m "Document motor and balance identification commands"
```

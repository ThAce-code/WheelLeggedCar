# Current-Hardware Motor and Balance Identification Design

## Goal

Create a reproducible identification workflow for the currently installed motor phase order, vehicle-left/right mapping, 72-float telemetry contract, and IMU orientation. The workflow first identifies the BLDC actuator dynamics, then collects multi-period balance excitation data and fits the discrete `A` and `B` matrices used by LQR.

Old captures made before the installed motor-direction correction must not enter the new fit.

## Architecture

The workflow has two separate identification layers:

1. Motor identification maps signed applied duty to physical left/right wheel RPM. Its result is used to choose and validate a sufficiently fast motor-speed loop before balance identification.
2. Balance identification maps the balance RPM command to the four measured balance states. It uses only captures made after the motor loop has passed its hardware response gate.

Keeping these layers separate prevents motor PI integral lag from being mistaken for body dynamics.

## Files

### `tools/collect_motor_identification.ps1`

A safety wrapper around `tools/collect_bldc_diagnostics.ps1`.

- Public parameters are `-Port`, `-Baud`, `-Out`, `-Note`, `-AllowMotion`, and `-Preview`.
- Requires `-AllowMotion` for a real capture.
- Uses the current 72-float telemetry decoder.
- Applies signed open-duty steps `+300`, `+600`, `+900`, `-300`, `-600`, and `-900`, with a `STOP` interval between steps.
- Preserves the underlying collector's final `STOP` behavior.
- Defaults to `data/motor_ident_<timestamp>.csv` and note `motor_signed_duty_ident`.
- Provides `-Preview` without opening the serial port.

The fixed 14-second schedule is:

```text
0.00:STOP
0.50:D,300
2.00:STOP
2.75:D,600
4.25:STOP
5.00:D,900
6.50:STOP
7.25:D,-300
8.75:STOP
9.50:D,-600
11.00:STOP
11.75:D,-900
13.25:STOP
```

Each duty interval lasts 1.5 seconds and every explicit recovery interval lasts 0.75 seconds. The collector remains open for 0.75 seconds after the last scheduled `STOP`.

### `tools/fit_motor_actuator_model.m`

Reads one or more `data/motor_ident_*.csv` files and fits each physical wheel independently.

The primary signed discrete model is:

```text
rpm(k+1) = a*rpm(k) + b*duty(k) + c*sign(duty(k))
```

For each wheel the script reports:

- discrete `A_motor=a` and `B_motor=b`;
- sample time and equivalent first-order time constant;
- signed steady-state duty/RPM slope and dead-zone estimate;
- regression rank, condition number, RMSE, and validation fit percentage;
- positive- and negative-direction residuals separately.

If System Identification Toolbox is available, the script also runs `iddata` and `tfest` as a cross-check. Toolbox output cannot replace the signed least-squares diagnostics. Candidate PI values are reported for bench testing, not written into firmware automatically.

Outputs:

```text
data/motor_actuator_model.csv
data/motor_actuator_model.mat
data/motor_actuator_model_fit.png
```

### `tools/collect_balance_lqr_dataset.ps1`

A safety wrapper around `tools/collect_balance_data.ps1` for one BI profile per run.

Supported profiles are exact and immutable:

| Profile | Command | Full period |
| --- | --- | --- |
| short | `BI,6,300` | 600 ms |
| medium | `BI,8,600` | 1200 ms |
| long | `BI,10,1000` | 2000 ms |

The wrapper:

- accepts `-Port`, `-Baud`, `-Profile`, `-Out`, `-Note`, `-AllowMotion`, and `-Preview`;
- requires explicit `-LeftMotorKp`, `-LeftMotorKi`, `-LeftMotorKd`, `-RightMotorKp`, `-RightMotorKi`, and `-RightMotorKd` values;
- requires explicit `-PitchKp`, `-PitchRateKd`, `-WheelSpeedKs`, `-WheelPositionKp`, and `-PitchSetpointDeg` values;
- requires `-AllowMotion`;
- requires explicit runtime motor and balance gains so unvalidated defaults are never silently used;
- enters standby before active balance;
- disables the chassis outer speed loop with `BD,0,0,0`;
- records at least 20 seconds of active excitation;
- always schedules `BI,0,0` followed by `STOP`;
- preserves the underlying collector's final `STOP` on exceptions;
- defaults to `data/balance_lqr_id_<profile>_<timestamp>.csv`;
- provides `-Preview` without opening the serial port.

The wrapper formats all numeric commands with invariant-culture decimal points and runs the following fixed 25.5-second schedule:

```text
0.00:STOP
0.25:PL,<LeftMotorKp>,<LeftMotorKi>,<LeftMotorKd>
0.50:PR,<RightMotorKp>,<RightMotorKi>,<RightMotorKd>
0.75:BL,<PitchKp>,<PitchRateKd>,<WheelSpeedKs>,<WheelPositionKp>
1.00:BS,<PitchSetpointDeg>
1.25:BD,0,0,0
1.50:B,1
1.75:C,0,0
2.00:BI,<profile amplitude>,<profile half-period ms>
2.25:B,2
24.25:BI,0,0
24.75:STOP
```

This produces exactly 22 seconds in active balance mode before excitation is disabled. The final 0.75 seconds confirm the stopped state in telemetry. `-Preview` prints the resolved schedule, duration, and output path, but does not instantiate a serial port.

The operator must keep a physical cutoff ready. A frozen IMU or offline wheel causes firmware safety gates to block output; the MATLAB fit rejects those rows and the capture if insufficient healthy data remain.

### `tools/fit_balance_lqr_model.m`

Reads only `data/balance_lqr_id_*.csv`. It never scans legacy `balance_capture_*.csv` files.

The state and input are:

```text
x = [pitch_deg; pitch_rate_dps; wheel_rpm; wheel_position_rev]
u = balance_rpm
x(k+1) = A*x(k) + B*u(k)
```

Here `wheel_rpm` is the physical left/right mean and `wheel_position_rev` is integrated separately within each capture. Samples are never joined across file boundaries or telemetry gaps.

Rows are accepted only when all of the following hold:

- active balance mode;
- feedback online;
- IMU age at most 15 ms;
- finite state and input values;
- absolute pitch at most 15 degrees;
- absolute balance command below 90 percent of its runtime limit;
- no firmware sequence gap at the one-step transition.

The script uses a file-level training/validation split, fits `A(4x4)` and `B(4x1)` by least squares, and reports:

- regressor rank, which must equal 5;
- condition number;
- one-step training and validation RMSE;
- open-loop multi-step validation error;
- eigenvalues of `A`;
- per-file accepted/rejected row counts and rejection reasons.

It then computes discrete LQR candidates from explicit `Q` and `R` sweeps. If Control System Toolbox is available, `dlqr` is cross-checked against the existing local DARE solver. Gains are exported without changing firmware.

Outputs:

```text
data/balance_lqr_ab.mat
data/balance_lqr_model_quality.csv
data/balance_lqr_candidates.csv
data/balance_lqr_fit.png
```

## Acceptance Gates

Motor identification is acceptable only when both directions and both wheels are present, the signed regression has full column rank, validation fit is reported, and no side is silently replaced by the other.

Balance identification is acceptable only when all three BI profiles contribute healthy samples, the regressor rank is 5, validation metrics are finite, and no gain clipping is hidden. Failure of any gate produces an error rather than a recommended LQR command.

## Testing

PowerShell contract tests will verify preview schedules, motion authorization, output naming, profile mapping, and final stop behavior.

Static MATLAB contract tests will verify input globs, required columns, signed motor regressors, capture-boundary splitting, health filters, matrix dimensions, rank/condition diagnostics, validation outputs, and absence of firmware writes.

MATLAB execution is not available on this computer. Static tests and PowerShell previews are the local software gate; numerical execution with System Identification Toolbox and hardware acceptance occur on the user's other computer and target vehicle.

## Operator Commands

Run these from the worktree root. Preview commands do not open a serial port.

Preview the signed motor capture:

```powershell
powershell -ExecutionPolicy Bypass -File tools/collect_motor_identification.ps1 -Preview -Out data/motor_ident_preview.csv
```

With both wheels safely suspended and a physical cutoff ready, run the real motor capture:

```powershell
powershell -ExecutionPolicy Bypass -File tools/collect_motor_identification.ps1 -Port COM6 -Baud 460800 -AllowMotion -Out data/motor_ident_current.csv -Note motor_signed_duty_ident
```

Preview all three balance profiles with the currently tested motor and balance gains:

```powershell
$gains = @('-LeftMotorKp','2.14218','-LeftMotorKi','42.2684','-LeftMotorKd','0', '-RightMotorKp','2.23353','-RightMotorKi','45.6807','-RightMotorKd','0', '-PitchKp','18','-PitchRateKd','8','-WheelSpeedKs','-3','-WheelPositionKp','0','-PitchSetpointDeg','0','-Preview')
powershell -ExecutionPolicy Bypass -File tools/collect_balance_lqr_dataset.ps1 -Profile short @gains
powershell -ExecutionPolicy Bypass -File tools/collect_balance_lqr_dataset.ps1 -Profile medium @gains
powershell -ExecutionPolicy Bypass -File tools/collect_balance_lqr_dataset.ps1 -Profile long @gains
```

After the motor actuator response gate passes, repeat those commands without `-Preview` and add `-AllowMotion`, one profile per capture. Keep the vehicle physically supported throughout the 22-second excitation window.

On the MATLAB computer, from the same repository root, run:

```matlab
run("tools/fit_motor_actuator_model.m");
run("tools/fit_balance_lqr_model.m");
```

Accept the identified matrices only when the scripts complete without an acceptance-gate error, all three profile rows are healthy, and the exported validation errors are finite. The scripts export candidate gains; they do not modify firmware.

## Out of Scope

- Automatically writing fitted gains into `app_config.h`.
- Running BI excitation before the motor response gate passes.
- Reusing old captures by renaming them to the new prefixes.
- Adding PRBS or chirp excitation in the first version.

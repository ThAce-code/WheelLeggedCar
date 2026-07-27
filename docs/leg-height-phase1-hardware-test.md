# Leg Height Phase 1 Hardware Validation

This file is the staged validation record for Phase 1: stable standing and low-speed driving during leg height transitions.

IAR build requirement: build `cyt4bb7_cm_7_0` from `project/iar/cyt4bb7.eww` with IAR Embedded Workbench 9.40.1 before hardware Gate 1.

A failure blocks all later gates. Do not mark a later gate as passed until every earlier gate has passed in order.

Current status: offline PowerShell checks are runnable in this workspace; IAR build has not been rerun after the empirical differential height-map update.

Local build-tool check: `IarBuild.exe` was not found at the common IAR 9.40 install paths under `C:\Program Files` or `C:\Program Files (x86)`, so no IAR build was executed here.

| Gate | Build SHA | Height start/end (mm) | Safe-pose measured height (mm) | Max pitch (deg) | Max wheel RPM | IK margin min | IK faults | Safety trips | Result | Notes |
|---|---|---:|---:|---:|---:|---:|---:|---:|---|---|
| bench/no wheel output | working tree | TBD | ~55 | TBD | TBD | TBD | TBD | TBD | needs rerun | Rebuild after empirical differential height-map update; confirm servo PWM behavior with wheel/motor output disabled. |
| supported stationary at low/default/high heights | working tree | 30 / 55 / 80 | ~55 | TBD | TBD | TBD | TBD | TBD | needs rerun | Validate the extended empirical range one endpoint at a time; `Y ~= 55 + 0.595 * d`, `d=a0-a1=a3-a2`. |
| LH S-curve re-target (supported) | TBD | 55 -> 65 -> 45 -> 55 | ~55 | TBD | TBD | TBD | TBD | TBD | not run | Run `collect_balance_data.ps1 -Port COM6 -Duration 18 -Commands "0:STOP;2:LH,55;4:LH,65;6:LH,45;12:LH,55" -Out data\phase1_gate1_lh_scurve_retarget.csv -Note phase1_gate1_lh_scurve_retarget`; record pitch/rate, target-versus-output mismatch, fault state, and observed jerk. A 55 mm stationary shake is a hardware/servo-hold investigation. |
| balance-in-place transition | TBD | 45 -> 65 -> 45 | TBD | TBD | TBD | TBD | TBD | TBD | not run | Confirm `TRANSITION` and `STABLE` telemetry, no leg soft faults, and acceptable pitch. |
| low-speed straight transition | TBD | 45 -> 65 -> 45 | TBD | TBD | TBD | TBD | TBD | TBD | not run | Confirm 30 RPM transition cap and fast-drive interlock during height motion. |
| low-speed turn and stop | TBD | 45 -> 65 -> 45 | TBD | TBD | TBD | TBD | TBD | TBD | not run | Confirm turn/stop commands remain bounded and leg `FAULT` stops balance output. |

Telemetry file: TBD

## Fast-height bench comparison only

`LHF,<height_mm>` is the global-height fast profile for response testing. It
uses the same 30–80 mm empirical map as `LH`, but completes the reference
move in exactly 500 ms with a quintic no-overshoot blend. It does not alter
roll, balance, chassis drive, or individual left/right leg height.

Only `LHF` raises the software servo slew cap to 180 °/s; normal `LH`, lock,
calibration, and all other paths retain the 90 °/s cap. This permits a 30→80
mm reference move to be evaluated without making normal low-speed driving
more aggressive.

Keep the vehicle supported with wheels stopped, rebuild the affected IAR core,
then collect one trace:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\collect_balance_data.ps1 `
  -Port COM6 -Duration 8 `
  -Commands "0:STOP;2:LHF,65;4:LHF,45;6:STOP" `
  -Out data\phase1_lhf_500ms_bench.csv `
  -Note phase1_lhf_500ms_bench
```

Pass only if `leg_fault_reason` remains zero, `leg_height_ref_mm` stays within
30–80 mm, each leg-height move reaches its target without reversing past it,
and no linkage interference, supply sag, or wheel movement occurs. A failed
bench result blocks any moving-chassis test.

After the two 25 mm endpoint moves pass, run the 50 mm fast-span comparison
with the vehicle still supported and wheel motors stopped:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\collect_balance_data.ps1 `
  -Port COM6 -Duration 7 `
  -Commands "0:STOP;2:LHF,30;3:LHF,80;5:STOP" `
  -Out data\phase1_lhf_30_to_80_fast.csv `
  -Note phase1_lhf_30_to_80_fast
```

Abort immediately on linkage interference, sustained servo chatter, supply
sag, or any wheel movement. The CSV only reports PWM commands, so visually
confirm the physical move completes before judging the result.

## Fresh 300 Hz validation

Before any balance or wheel-drive test, complete this sequence:

1. Support the vehicle and keep wheel output stopped.
2. Capture all four PWM channels for at least 90 seconds. Confirm approximately
   3.333 ms frame period and 1.5 ms pulse at the all-90-degree pose. Every pulse
   must remain within 0.5-2.5 ms; at the normal 90 deg/s limit, adjacent pulse
   widths must change by no more than approximately 3.33 us. Reject any isolated
   long pulse or one-channel discontinuity.
3. Hold all four servos at 90 degrees for 30 seconds; abort on sustained
   chatter, loss of holding force, supply sag, or abnormal temperature rise.
4. Run `LIKREF` before any `LXY` command.
5. Run one small command at a time: `LXY,5,55`, `LXY,-5,55`, `LXY,0,52`,
   `LXY,0,58`.
6. Run a small `LH` move before `LHF`.
7. On any failure, stop PWM and return to the known 50 Hz build; do not
   continue to terrain or balance testing.

Pass criteria: zero leg faults, valid IK, monotonic target/filter/output
traces, actuator settled before `STABLE`, no unexpected heating or chatter,
no increase in `telemetry_drop_count` or `scheduler_missed_tick_count` during
the motion, and a continuous `servo_tick_count` consistent with 300 Hz.

## Superseded PWM input-rate record

The earlier 100 Hz shaking result was produced with an incorrect servo-control
algorithm. It is not valid evidence for accepting or rejecting any PWM frame
rate and must be ignored in later design or hardware decisions.

The current target is a fresh 300 Hz validation using the S7 trajectory and
first-order LPF actuator algorithm. Judge that configuration only from its own
oscilloscope, supported-vehicle, command-trace, chatter, and temperature checks.

## Direct-step bench comparison only

`LJ,<height_mm>` is excluded from normal Phase 1 gates. To build the temporary
bench binary, change `APP_LEG_DIRECT_STEP_TEST_ENABLE` from `0U` to `1U`,
rebuild, and keep the vehicle supported with wheel motors stopped. Send only:

```text
STOP
LJ,55
LJ,65
STOP
```

The direct-step mode bypasses both software trajectory layers and forces drive
permission off. Abort on linkage interference, servo heating, supply sag, or
wheel movement. Restore `APP_LEG_DIRECT_STEP_TEST_ENABLE` to `0U` before any
supported drive or balance test.

Offline checks to record before bench work:

- `powershell -ExecutionPolicy Bypass -File .\tools\test_leg_transition_numeric.ps1`
- `powershell -ExecutionPolicy Bypass -File .\tools\test_ik_height_control_static.ps1`
- `git diff --check`

## IK zero-calibration path is separate from Phase 1 height control

The experimental `LIKREF` and restricted `LXY,x,y` commands are documented in
[`leg-ik-zero-calibration-hardware-test.md`](leg-ik-zero-calibration-hardware-test.md).
They stop drive and balance before commanding the legs and do not replace the
empirical `LH/LHF` height map. Complete the reference-pose and small-range
bench procedure before considering any wider 2D IK work.

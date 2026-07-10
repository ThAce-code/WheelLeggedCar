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

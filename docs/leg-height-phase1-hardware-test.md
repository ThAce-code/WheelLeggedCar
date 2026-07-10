# Leg Height Phase 1 Hardware Validation

This file is the staged validation record for Phase 1: stable standing and low-speed driving during leg height transitions.

IAR build requirement: build `cyt4bb7_cm_7_0` from `project/iar/cyt4bb7.eww` with IAR Embedded Workbench 9.40.1 before hardware Gate 1.

A failure blocks all later gates. Do not mark a later gate as passed until every earlier gate has passed in order.

Current status: offline PowerShell checks are runnable in this workspace; IAR build has not been rerun after the measured monotonic Phase 1 height-window update.

Local build-tool check: `IarBuild.exe` was not found at the common IAR 9.40 install paths under `C:\Program Files` or `C:\Program Files (x86)`, so no IAR build was executed here.

| Gate | Build SHA | Height start/end (mm) | Safe-pose measured height (mm) | Max pitch (deg) | Max wheel RPM | IK margin min | IK faults | Safety trips | Result | Notes |
|---|---|---:|---:|---:|---:|---:|---:|---:|---|---|
| bench/no wheel output | working tree | TBD | ~55 | TBD | TBD | TBD | TBD | TBD | needs rerun | Rebuild after mapped height-window update; confirm servo PWM behavior with wheel/motor output disabled. |
| supported stationary at low/default/high heights | working tree | 35 / 55 / 80 | ~55 | TBD | TBD | TBD | TBD | TBD | needs rerun | Measured wheel-center response was monotonic only in the `LH,35` to `LH,80` command window; do not use `LH,120` in Phase 1. |
| balance-in-place transition | TBD | 35 -> 80 -> 35 | TBD | TBD | TBD | TBD | TBD | TBD | not run | Confirm `TRANSITION` and `STABLE` telemetry, no leg soft faults, and acceptable pitch. |
| low-speed straight transition | TBD | 35 -> 80 -> 35 | TBD | TBD | TBD | TBD | TBD | TBD | not run | Confirm 30 RPM transition cap and fast-drive interlock during height motion. |
| low-speed turn and stop | TBD | 35 -> 80 -> 35 | TBD | TBD | TBD | TBD | TBD | TBD | not run | Confirm turn/stop commands remain bounded and leg `FAULT` stops balance output. |

Telemetry file: TBD

Offline checks to record before bench work:

- `powershell -ExecutionPolicy Bypass -File .\tools\test_leg_transition_numeric.ps1`
- `powershell -ExecutionPolicy Bypass -File .\tools\test_ik_height_control_static.ps1`
- `git diff --check`

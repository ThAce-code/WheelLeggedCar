# Balance PD Isolation Capture Script Design

**Date:** 2026-07-20

## Goal

Add a small PowerShell wrapper that safely reproduces the pure pitch-PD
balance probe and records the existing 72-float telemetry stream. The script
must distinguish the chassis speed loop from the pitch and pitch-rate loop
without changing firmware or duplicating the telemetry parser.

## Approach

Create `tools/collect_balance_pd_isolation.ps1`. It invokes
`tools/collect_balance_data.ps1` with a fixed command schedule and relies on
that collector's `finally` block to send an additional `STOP` before closing
the serial port. The wrapper must not pass `-NoStopOnExit`.

The wrapper accepts `-Port`, `-Baud`, `-Out`, and `-Note`. It also accepts:

- `-AllowMotion`: mandatory before opening the serial port and sending the
  active-balance sequence.
- `-Preview`: print the exact schedule and output path without opening the
  serial port. Preview does not require `-AllowMotion`.

The default output is
`data/balance_pd_isolation_<yyyyMMdd_HHmmss>.csv`, and the default note is
`balance_pd_isolation_bd0`.

## Fixed Command Schedule

The total capture duration is 2.50 seconds:

```text
0.00 s  STOP
0.25 s  BL,18,8,0,0
0.50 s  BS,0
0.75 s  BD,0,0,0
1.00 s  B,1
1.25 s  C,0,0
1.50 s  B,2
1.80 s  STOP
```

The active `B,2` window is therefore exactly 300 ms. The remaining 700 ms
records post-stop behavior. The command schedule is not user-overridable in
this diagnostic wrapper.

## Safety and Failure Behavior

Without `-AllowMotion`, a real capture fails before the collector is invoked.
The message must instruct the operator to suspend or firmly support the
vehicle and keep a physical cutoff ready. The wrapper prints a motion warning
before starting.

The scheduled `STOP` ends the active window. Normal completion, serial errors,
and PowerShell interruption are additionally covered by the existing
collector's `finally`-block `STOP`. Force-killing the PowerShell process or
disconnecting the host cannot guarantee delivery, so the physical cutoff
remains required.

## Data Contract

The wrapper preserves the existing 72-channel CSV contract. Diagnosis will
primarily use:

- `pitch_deg`, `pitch_rate_dps`, and `pitch_setpoint_deg`;
- `balance_rpm`;
- `left_motor_rpm`, `right_motor_rpm`, `left_duty`, and `right_duty`;
- `forward_target_rpm`, `forward_ramped_rpm`,
  `wheel_speed_measured_rpm`, and `speed_error_rpm`;
- `feedback_online`, `imu_age_ms`, `leg_motion_state`, and
  `leg_fault_reason`.

## Verification

Add `tools/test_collect_balance_pd_isolation.ps1` before implementation. It
must verify:

1. preview mode reports the exact fixed schedule in order;
2. `B,2` occurs at 1.50 s and scheduled `STOP` at 1.80 s;
3. a real run without `-AllowMotion` fails before serial access;
4. the wrapper invokes the existing collector and never opts out of its final
   `STOP`;
5. the default output prefix and diagnostic note are stable.

Run the new test, the existing balance collector test, and the complete
PowerShell regression suite. No IAR build is required because the change is a
host-only tool.

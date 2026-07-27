# Balance Speed-Damping Sign Correction Design

## Goal

Restore negative wheel-speed feedback after the installed-car motor contract was changed so positive wheel RPM means vehicle-forward motion.

This change addresses only the P0 balance speed-damping regression. It does not change host-command timeouts, balance P/D gains, pitch setpoint, motor direction, fast-mode limits, race assist, or leg control.

## Root Cause

Commit `cc95de4` made controller-visible positive wheel RPM mean vehicle-forward motion. The balance controller still computes:

```c
speed_term_rpm = effective_wheel_speed_ks * wheel_speed_rpm;
```

With the default positive `Ks`, a positive wheel-speed disturbance therefore adds a positive motor-speed command. The term reinforces the disturbance instead of opposing it.

The captured `balance_ks3_bs0_retry.csv` data shows the resulting runaway: measured wheel speed grows from `0 RPM` to `92.5 RPM` while pitch moves from approximately `-0.07 deg` to `-6.26 deg`.

## Sign Contract

The installed-car sign contract remains:

- positive controller-visible wheel RPM means vehicle-forward motion;
- positive `Ks` means damping magnitude;
- at zero pitch and pitch rate, positive wheel speed must produce a negative speed term;
- at zero pitch and pitch rate, negative wheel speed must produce a positive speed term;
- zero wheel speed must produce a zero speed term.

The balance speed term will therefore be:

```c
speed_term_rpm = -effective_wheel_speed_ks * wheel_speed_rpm;
```

Both normal and fast-mode `Ks` values remain positive. Fast-mode interpolation continues to interpolate damping magnitude and requires no separate sign change.

## Code Changes

### `project/code/control_balance.c`

Change only the speed-term calculation to apply the explicit negative-feedback sign. Add a short comment documenting that wheel RPM is positive vehicle-forward and positive `Ks` is a damping magnitude.

### `tools/test_motor_installation_semantics.ps1`

Add a regression assertion that ties the installed motor-direction contract to the balance controller. The test must fail against the current positive multiplication and pass only when the speed term explicitly opposes wheel speed.

No public API, command syntax, telemetry layout, configuration value, or IAR project membership changes.

## Test-Driven Sequence

1. Add the sign-contract regression assertion.
2. Run `tools/test_motor_installation_semantics.ps1` and confirm it fails because the balance speed term is still positive feedback.
3. Change the production speed-term calculation.
4. Re-run the regression test and confirm it passes.
5. Run the existing Balance V1 and V2 static tests and related timing/data-tool regressions.
6. Run `git diff --check` and inspect the final diff.

## Software Acceptance Criteria

- Positive `Ks` and positive wheel speed produce a negative `speed_term_rpm`.
- Positive `Ks` and negative wheel speed produce a positive `speed_term_rpm`.
- The installed left/right channel and positive-forward motor contract remains unchanged.
- Normal and fast `Ks` configuration values remain unchanged.
- Existing balance-drive and motor-installation tests pass.
- No unrelated tracked files change.

## Hardware Gate

Software verification does not establish safe unsupported standing.

After flashing the corrected firmware:

1. Keep the chassis restrained and wheels clear or otherwise fail-safe.
2. Start with `STOP`, verify current IMU and BLDC telemetry health, and enable the chassis only with an explicit `C,0,0`.
3. Validate pitch/rate PD with `Ks=0` before adding wheel-speed damping.
4. Reintroduce a small positive damping magnitude and increase it only from measured response.
5. Do not reuse `Ks=3` as a hardware-validated value under the corrected sign without retuning.
6. Do not run unsupported `B,2`, `B,3`, or 220 RPM testing until the restrained gate passes.

## Rollback

If software regression tests fail, revert only the speed-term and matching regression-test changes. Do not alter the installed motor-direction signs to compensate, because that would break the vehicle-forward motor and chassis contract.

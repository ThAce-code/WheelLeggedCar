# Leg Direct-Step Bench Test Design

## Goal

Provide a temporary, explicit bench-test command that sends the measured
empirical leg pose directly to all four servos. It is for comparing the
current smooth trajectory against the servos' own internal position control.

## Scope

- New command: `LJ,<height_mm>` for the existing 45--65 mm empirical range.
- The command bypasses both the `LH` height trajectory and the actuator PWM
  slew limiter for that command only.
- `LH`, `LIK`, startup lock, and normal actuator behaviour remain unchanged.
- The direct-step test is guarded by `APP_LEG_DIRECT_STEP_TEST_ENABLE` and is
  disabled by default in production firmware.

## Safety Behaviour

- `LJ` is accepted only when the compile-time test switch is enabled and the
  requested height is finite and inside 45--65 mm.
- Direct-step mode uses the existing empirical servo angle validation before
  issuing PWM.
- Direct-step diagnostics force `drive_allowed = APP_FALSE`; the chassis must
  not drive while the mode is active.
- `STOP` exits the mode and returns to the existing LOCK/safe-pose path.
- Hardware procedure: vehicle supported, wheels clear or motor output off,
  hands clear of linkage, and servo supply ready to disconnect before sending
  the command.

## Interfaces

```text
LJ,45 / LJ,55 / LJ,65
     |
control_leg_set_direct_step_height(height)
     |
empirical height mapping
     |
actuator_servo_apply_immediate()
```

`actuator_servo_apply_immediate()` copies the current enabled servo command to
the actuator output state and writes PWM in the same update. It is called only
by `LEG_MODE_DIRECT_STEP`; the normal `actuator_servo_update()` slew path is
unchanged for every other mode.

## Verification

Before flashing, static tests verify the compile guard, parser, drive interlock,
height range, empirical mapping, and immediate PWM path. Numeric tests verify
the 45, 55, and 65 mm mappings. Hardware testing starts with `LJ,55`, followed
by one supported `LJ,45` or `LJ,65` move; no mobile/balance test is allowed in
this mode.

# LH S-Curve Trajectory Design

## Goal

Make `LH,<height_mm>` height changes smooth while keeping them responsive. The
firmware must keep the four leg servos geometrically synchronized, avoid abrupt
velocity and acceleration changes, and stop at the requested valid height.

This is Phase 1 open-loop control. The platform has PWM outputs only; the
reported servo angle is the limited PWM command, not measured servo position or
wheel-center height.

## Scope

Included:

- `LEG_MODE_HEIGHT` / `LH` only.
- The empirical height mapping already used for the validated 45--65 mm range.
- A jerk-limited height motion generator in `control_leg.c`.
- Static/numeric tests for trajectory constraints and re-targeting.

Excluded:

- `LIK` calibration commands and their direct PWM-debugging behaviour.
- Pin mappings, scheduler periods, balance/chassis control laws, and motor limits.
- Closed-loop height control; it requires a height or joint-position sensor.

## Chosen Architecture

The motion generator remains in `control_leg.c`, before the empirical
height-to-servo mapping. It owns a shared height state:

```text
LH target -> jerk-limited height reference -> empirical 4-servo mapping -> PWM slew limiter
```

All four servo targets are computed from the same `height_ref_mm` at every
control update. This preserves the measured differential pose relation:

```text
FL = RR = 90 + 0.5 * (height_ref - 55) / 0.595
FR = RL = 90 - 0.5 * (height_ref - 55) / 0.595
```

The existing actuator PWM limiter remains a secondary protection layer. At the
configured 5 mm/s height speed, each empirical servo command changes by about
4.2 deg/s, safely below the 90 deg/s PWM limiter. Therefore normal `LH`
motion is shaped by the height generator, not by independent per-servo
clipping.

## Motion Behaviour

The generator tracks position, velocity, and acceleration. Each 10 ms update:

1. Calculates a stopping-aware desired acceleration toward the current target.
2. Ramps acceleration toward that demand by no more than `max_height_jerk_mm_s3 * dt`.
3. Integrates acceleration and clamps velocity to `max_height_speed_mm_s`.
4. Integrates velocity into `height_ref_mm`; only an actual target crossing may
   snap position to the target and clear velocity/acceleration.

When a new valid `LH` target arrives mid-motion, the planner retains the
current reference position, velocity, and acceleration and replans toward the
new target. It must not return to 55 mm or reset velocity. Direction reversal
therefore brakes through zero velocity before moving the other way.

Initial Phase 1 limits remain conservative:

- maximum height speed: 5 mm/s
- maximum height acceleration: 10 mm/s2
- maximum height jerk: 80 mm/s3

The jerk value gives a 125 ms acceleration ramp. It removes the instantaneous
acceleration steps in the current trapezoidal profile without making the normal
45--65 mm test needlessly slow. It is a configuration constant so hardware
validation can lower it if the mechanism still resonates.

## Safety and Fault Handling

- Valid height bounds remain 45--65 mm.
- Existing servo-angle validity checks, safe pose fallback, and drive gating
  remain unchanged.
- Invalid `LH` commands remain rejected.
- A non-finite or invalid trajectory state enters the existing soft fault path
  instead of emitting PWM commands.
- `LEG_MOTION_STABLE` still requires the configured position error tolerance,
  zero commanded velocity, and the existing settling dwell time.

## Verification

Before hardware flashing, numeric/static tests must demonstrate:

1. Height command velocity, acceleration, and jerk remain within configured
   limits for a 45 -> 65 -> 45 mm move.
2. The reference reaches and settles exactly at each target.
3. A mid-motion re-target retains state and reverses through zero velocity;
   it has no instantaneous velocity or acceleration jump larger than the jerk
   bound permits.
4. Four servo targets satisfy the empirical synchronized differential mapping
   at every simulated sample.
5. Existing height-control and calibration-fit tests still pass.

Hardware acceptance is a supported, low-speed 45 -> 65 -> 45 mm test. Compare
IMU pitch/rate and servo target-versus-PWM-command traces with the pre-change
CSV. If the vehicle shakes while stationary at 55 mm, treat that as a separate
hardware/servo-hold investigation rather than a trajectory failure.

# Leg IK Zero Calibration Design

## Goal

Make the five-bar inverse-kinematics (IK) joint angles independent from the
four physical BDS300 installation angles.  The first reference pose is a
level, left/right-symmetric standing pose with the wheel center at
approximately `X = 0 mm`, `Y = 55 mm`.

This work must not change the validated empirical height behavior of `LH` or
`LHF` until the new IK path has passed hardware verification.

## Calibration model

For each servo channel `i`, store:

- `mid_deg`: actual PWM angle command that holds the reference pose.
- `direction`: `+1` or `-1`, expressing the installed servo direction.
- `offset_deg`: a small additive correction; initial value is `0`.

Let `theta_ik_i` be the IK active-joint angle and `theta_ref_i` the IK angle
at the reference point.  The output PWM angle is:

```
servo_cmd_i = mid_deg_i + direction_i * (theta_ik_i - theta_ref_i) + offset_deg_i
```

The final command is checked against the configured per-servo limits before
it is sent to the PWM actuator.

## Interfaces

- `LIKREF`: command all four channels to the stored `mid_deg` values.  It is
  the safe physical reference-pose check and contains no IK calculation.
- `LXY,x_mm,y_mm`: an IK validation command.  Initial limits are
  `x_mm = [-10, 10]` and `y_mm = [50, 60]`.
- Existing `LIK`, `LH`, and `LHF` keep their current behavior.  In particular,
  `LH` and `LHF` continue to use the calibrated empirical height mapping in
  the first phase.

## IK safety rules

Before outputting an `LXY` command, firmware shall:

1. Verify that both circle-intersection discriminants are non-negative within
   a small numerical tolerance.
2. Generate both possible branches for each active joint.
3. Select a valid left/right branch pair nearest to the previous accepted IK
   angles, subject to servo limits.
4. Reject the command and preserve the previous safe servo command if no
   valid pair exists, a value is non-finite, or a servo limit would be crossed.

The implementation uses `atan2` plus `acos` rather than the half-angle
division used in the example code, avoiding division instability near a zero
denominator.

## Calibration and verification procedure

1. Securely support the vehicle so an unexpected command cannot make it fall.
2. Use existing `LIK` commands to obtain a level, symmetric pose at about
   `Y = 55 mm`.
3. Record the four final command angles as `mid_deg[0..3]`.
4. Build and flash firmware containing only the new calibration layer and
   restricted IK validation path.
5. Issue `LIKREF`; the pose must reproduce the recorded reference pose.
6. Issue `LXY,0,55`; it must closely match `LIKREF`.
7. Test `LXY,5,55`, `LXY,-5,55`, `LXY,0,52`, and `LXY,0,58`, one command at a
   time.  Stop immediately on unexpected motion.

## Acceptance criteria

- `LIKREF` reproduces the measured reference pose.
- `LXY,0,55` is visually equivalent to `LIKREF`.
- Positive and negative X commands move in the intended opposite directions.
- All five validation points complete without an IK rejection fault, servo
  limit violation, branch jump, sustained oscillation, or IMU safety stop.
- `LH` and `LHF` behavior remains unchanged during the entire validation.

## Out of scope

- Roll compensation and independent left/right height control.
- Switching production height control from empirical mapping to IK.
- Automatic physical wheel-center sensing or closed-loop X control.

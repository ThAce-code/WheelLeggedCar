# Leg IK Zero Calibration Hardware Test

This procedure validates the calibrated conversion between five-bar geometry,
four BDS300 command angles, and physical wheel-centre millimetres relative to
the fixed chassis marker. The link dimensions are the measured
`60/90/90/60/37 mm` values. Offline fit quality is necessary but does not
replace unloaded hardware verification.

`LH` and `LHF` continue to use the existing empirical height map. Do not use
this procedure to tune normal height control, PWM frequency, servo slew, roll
compensation, or balance gains.

## Safety prerequisites

1. Put the vehicle on a rigid support so neither wheels nor chassis can fall.
2. Disconnect or mechanically unload wheel drive where practical. Keep the
   wheels clear of the bench and remove terrain obstacles.
3. Start with `STOP`; this locks the leg path and stops balance and wheel
   output.
4. Build `cyt4bb7_cm_7_0` in `project/iar/cyt4bb7.eww`, flash it, and wait
   for the IMU to become healthy.
5. Use the PWM frequency selected by the current servo-control build. Ignore
   the superseded 100 Hz shaking record because it came from an incorrect
   control algorithm. For the new 300 Hz build, verify the frame period and
   pulse width on an oscilloscope before moving the linkage. Do not enable the
   direct-step bench switch for this procedure.
6. On any failure (chatter, heating, supply sag, unexpected motion, nonzero
   fault), stop PWM immediately and return to the 50 Hz build. Do not continue
   with the 300 Hz build until the root cause is resolved.

Stop immediately if a linkage approaches an end stop, a servo chatters or
heats, the supply sags, a wheel moves, telemetry reports a nonzero leg fault,
or `ik_valid` becomes zero.

## 1. Reproduce the measured physical reference

The nominal reference command remains `LIK,90,90,90,90`. Camera collection
measured its wheel centre three times and established the mean physical
position `(-20.7667,47.3567) mm`. Do not shift that position to `(0,55)` and
do not hand-edit the fitted IK directions.

1. Begin from the known-safe direct command:

   ```text
   STOP
   LIK,90,90,90,90
   ```

2. Verify that all four channels hold their established nominal 90-degree
   physical pose. The servo0 pulse midpoint correction belongs in the
   angle-to-duty calibration; the logical angle remains 90 degrees.
3. Record the camera measurement and four commanded angles:

   ```text
   servo0 = __
   servo1 = __
   servo2 = __
   servo3 = __
   ```

4. If the pose no longer matches the captured reference within the camera's
   repeatability, stop and diagnose marker, pulse mapping, linkage, or mounting
   drift. Do not compensate by changing the physical coordinate origin.

The visible-side direction pair is fitted from all 20 samples. The opposite
side uses the nominal mechanical mirror and is not independently calibrated.
A wrong observed direction is a stop condition.

## 2. Verify the compiled reference pose

`LIKREF` computes IK at the measured physical reference
`(-20.7667,47.3567) mm`, applies the four stored midpoint conversions, stops
balance/drive first, and holds the result. The fixed chassis cross-circle is
the physical origin `(0,0)`; +X is vehicle forward and +Y is downward. The
reference is the mean of `ref_start`, `ref_mid`, and `ref_end`, not a new
origin. This is the first required check after changing `neutral_deg`.

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\collect_balance_data.ps1 `
  -Port COM6 -Duration 5 `
  -Commands "0:STOP;1:LIKREF;4:STOP" `
  -Out data\ik_zero_likref.csv `
  -Note ik_zero_likref
```

Pass only if the physical pose matches the measured reference pose, the four
servo targets remain stable, `leg_fault_reason=0`, and no wheel or linkage
motion is unexpected. The telemetry fields `ik_valid`, `ik_margin`, and
`servo_target_deg[0..3]` are the evidence to retain.

## 3. Calibrated physical-coordinate IK validation

`LXY,x,y` now means the absolute wheel-center position in millimetres relative
to the fixed chassis marker. The current measurements cover approximately
`X=[-40.62,-15.04] mm` and `Y=[32.17,88.49] mm`, but the extrema do not define a
safe rectangle. Firmware accepts only the measured convex hull contracted
inward by 2 mm, followed by the existing model-workspace, branch, IK-margin,
and servo-angle checks.

`LXY,0,55` is therefore not the reference pose. It is outside the current
calibrated X coverage and must be rejected without moving the servos. Send
`LIKREF` once after boot or after `STOP`, then test one interior point at a time:

```text
STOP
LIKREF
LXY,-20.7667,47.3567
LXY,-18.0,47.3567
LXY,-23.5,47.3567
LXY,-20.7667,44.0
LXY,-20.7667,51.0
LXY,0,55
STOP
```

Do not chain these as an unattended script during first validation. Stop and
visually inspect after every accepted command. The last command must be
rejected and must leave the preceding safe target unchanged.

Suggested individual traces are:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\collect_balance_data.ps1 `
  -Port COM6 -Duration 5 `
  -Commands "0:STOP;1:LIKREF;2:LXY,-20.7667,47.3567;4:STOP" `
  -Out data\ik_physical_reference.csv `
  -Note ik_physical_reference
```

Repeat this command with exactly one interior test point at second 2 and use a
distinct output filename.

Required observations:

- `LXY,-20.7667,47.3567` is visually equivalent to `LIKREF`.
- `LXY,-18.0,47.3567` and `LXY,-23.5,47.3567` produce opposite wheel-centre X directions.
- `LXY,-20.7667,44.0` and `LXY,-20.7667,51.0` produce the expected small opposite height
  changes without a branch jump.
- All five accepted points keep `leg_fault_reason=0`, `ik_valid=1`, enabled servo
  output, and no sustained chatter.
- `LXY,0,55` is rejected and does not alter the held target.

Do not increase the range, drive the chassis, or enable balance from this IK
path if any point fails. Keep the failed CSV and note the exact servo pose;
the next action is geometry/direction diagnosis, not a larger command.

## Completion record

Record in the test log:

- firmware commit SHA and IAR build result;
- the four nominal reference command values and pulse calibration revision;
- wheel-centre measurement method and measured `(-20.7667,47.3567) mm` result;
- five CSV filenames and visual observations;
- minimum observed IK margin and any fault/IMU event.

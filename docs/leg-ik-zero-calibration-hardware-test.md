# Leg IK Zero Calibration Hardware Test

This procedure calibrates the conversion between the five-bar IK joint-angle
zero and the four physical BDS300 PWM angle commands. It validates only the
reference-pose repeatability and the sign of small open-loop X/Y moves. It
does **not** validate measured millimetre X accuracy: the configured link
dimensions are still temporary.

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

## 1. Set the physical reference pose and record PWM midpoints

The reference is a level, left/right-symmetric pose with the wheel centre
approximately at `X = 0 mm`, `Y = 55 mm`.

1. Begin from the known-safe command:

   ```text
   STOP
   LIK,90,90,90,90
   ```

2. With the chassis supported, make small `LIK,a0,a1,a2,a3` adjustments until
   the chassis is level, both sides are symmetric, and the wheel centre is
   about 55 mm below the chosen chassis datum. Change no channel by more than
   1-2 degrees between observations.
3. Record the final four numbers in physical channel order:

   ```text
   servo0 = __
   servo1 = __
   servo2 = __
   servo3 = __
   ```

4. Replace only the four `neutral_deg` values in
   `project/code/leg_config.c`. They are the third value in each servo
   initializer, for example `{0, safe_deg, neutral_deg, ...}`. Keep all four
   `ik_offset_deg` values at `0.0f` for this first bring-up.
5. Rebuild and flash CM7_0 again. The four `neutral_deg` values are compiled
   configuration, not UART-persistent settings.

The existing `direction` values are the initial mechanical-direction model.
Do not change them during the midpoint step. A wrong direction is detected
in the restricted X checks below and is a stop condition.

## 2. Verify the compiled reference pose

`LIKREF` computes IK at the configured reference point `(0,55)`, applies the
four stored midpoint conversions, stops balance/drive first, and holds the
result. It is the first required check after changing `neutral_deg`.

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

## 3. Restricted open-loop IK validation

Each `LXY,x_mm,y_mm` command accepts only `x=[-10,10] mm` and `y=[50,60] mm`.
It turns balance and wheel drive off before the leg command. Send `LIKREF`
once after boot or after `STOP` before issuing any `LXY` command.

Run one point at a time, with visual confirmation before proceeding:

```text
STOP
LIKREF
LXY,0,55
LXY,5,55
LXY,-5,55
LXY,0,52
LXY,0,58
STOP
```

Suggested individual traces are:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\collect_balance_data.ps1 `
  -Port COM6 -Duration 5 `
  -Commands "0:STOP;1:LIKREF;2:LXY,0,55;4:STOP" `
  -Out data\ik_zero_xy_0_55.csv `
  -Note ik_zero_xy_0_55
```

Repeat this command with exactly one of `LXY,5,55`, `LXY,-5,55`, `LXY,0,52`,
or `LXY,0,58` at second 2 and use a distinct output filename.

Required observations:

- `LXY,0,55` is visually equivalent to `LIKREF`.
- `LXY,5,55` and `LXY,-5,55` produce opposite wheel-centre X directions.
- `LXY,0,52` and `LXY,0,58` produce the expected small opposite height
  changes without a branch jump.
- All five points keep `leg_fault_reason=0`, `ik_valid=1`, enabled servo
  output, and no sustained chatter.

Do not increase the range, drive the chassis, or enable balance from this IK
path if any point fails. Keep the failed CSV and note the exact servo pose;
the next action is geometry/direction diagnosis, not a larger command.

## Completion record

Record in the test log:

- firmware commit SHA and IAR build result;
- the four final `neutral_deg` values;
- wheel-centre measurement method and approximate 55 mm result;
- five CSV filenames and visual observations;
- minimum observed IK margin and any fault/IMU event.

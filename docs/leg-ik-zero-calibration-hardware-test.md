# Leg IK Zero Calibration Hardware Test

This procedure validates the physical wheel-centre command contract and the
conversion between the five-bar IK joint-angle zero and the four BDS300 PWM
angle commands. PWM is open loop: telemetry reports a calibrated command
estimate derived from the actuator output commands, not a measured servo or
wheel-centre position.

`LH` and `LHF` continue to use the existing legacy stance map. Do not use
this procedure to tune legacy stance control, PWM frequency, servo slew, roll
compensation, or balance gains.

## BODY_WHEEL coordinate contract

- Origin: chassis-fixed cross-circle marker center.
- Unit: millimetres.
- +X: vehicle forward.
- +Y: downward.
- Reference command: `LIKREF = (-20.766667, 47.356667) mm`.
- `LXY,x,y` consumes absolute `BODY_WHEEL` millimetres, not relative motion.
- Runtime X/Y is a calibrated command estimate from actuator output commands;
  no position feedback is fitted.
- Left source: measured calibration.
- Right source: mirror assumption until the independent checks below pass.

`LXY` accepts a point only when it is model-reachable: the physical point maps
to the five-bar lower-half-plane with real roots, the IK margin is at least
`0.02`, and both left and right candidates map to valid commands within the
configured servo limits. This is a command/model estimate gate, not measured
feedback or an empirical coordinate boundary. `LXY,0,55` is accepted only
when the same model-reachable root, margin, and two-sided servo-limit checks
pass; its coordinates are not pre-rejected.

### Model-reachable low command example

The model/command example `LIK,40,140,140,40` places the left-leg logical pair
at `40,140`. The current model estimates this pose as
`BODY_WHEEL=(-18.831,25.076) mm`. This is not a measured wheel-centre
calibration sample or hardware acceptance for the full workspace.

With the chassis supported and wheel power disconnected, run one command at a
time and wait for the output to settle:

```text
STOP
LIKREF
LXY,-18.83,25.08
```

Require telemetry `servo_settled=1`, `fault_reason=0`, and planner targets near
`[40,140,140,40]`. Stop immediately for mechanical interference. Passing this
static gate does not enable dynamic wheel shift or balance fast mode.

## Safety prerequisites

1. Put the vehicle on a rigid support so neither wheels nor chassis can fall.
2. Keep wheel power disconnected. Keep the wheels clear of the bench and
   remove terrain obstacles.
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

Stop immediately for interference, branch surprise, invalid IK, or a leg
fault. Also stop if a linkage approaches an end stop, a servo chatters or
heats, the supply sags, or a wheel moves.

## 1. Set the physical reference pose and record PWM midpoints

The reference is the calibrated `BODY_WHEEL` point
`P0 = (-20.766667, 47.356667) mm`. It must be checked with the chassis-fixed
cross-circle marker as origin, rather than with an arbitrary local ruler zero.

1. Begin from the known-safe command:

   ```text
   STOP
   LIK,90,90,90,90
   ```

2. With the chassis supported, make small `LIK,a0,a1,a2,a3` adjustments until
   the chassis is level and both sides are symmetric at P0. Change no channel by more than
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

`LIKREF` computes IK at P0 `(-20.766667,47.356667)`, applies the
four stored midpoint conversions, stops balance/drive first, and holds the
result. It is the first required check after changing `neutral_deg`.

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\collect_balance_data.ps1 `
  -Port COM6 -Duration 5 `
  -Commands "0:STOP;1:LIKREF;4:STOP" `
  -Out data\ik_zero_likref.csv `
  -Note ik_zero_likref
```

Pass only if the physical pose matches the P0 measurement, the four servo
targets remain stable, `leg_fault_reason=0`, and no wheel or linkage motion is
unexpected. Retain `leg_pose_status_flags`, left/right command estimates and
sources, `ik_valid`, `ik_margin`, and `servo_target_deg[0..3]`.

## 3. Physical open-loop IK validation

The IK margin check, per-servo limits, and fault-safe return remain active, but
they do not prove that a pose is mechanically safe. The command stops balance
and wheel drive first. Send `LIKREF` once after boot or `STOP` before each
individual `LXY` command. Do not chain first validation commands unattended.

Choose two model-reachable points (`P1` and `P2`) with lower-half-plane real
roots, margin at least `0.02`, and valid mapped commands within servo limits
on both sides. Record their absolute BODY_WHEEL coordinates. Change one axis
at a time by no more than `2 mm`, wait for settled output, and inspect before
each command:

```text
STOP
LIKREF
LXY,<P1_X>,<P1_Y>
STOP
LIKREF
LXY,<P2_X>,<P2_Y>
STOP
```

Then verify `LXY,1000,1000`, a truly unreachable command, must be rejected
without servo motion:

```text
STOP
LIKREF
LXY,1000,1000
STOP
```

Do not increase the range, drive the chassis, or enable balance if any point
fails. Stop immediately for interference, branch surprise, invalid IK, or leg
fault. Keep the CSV and the exact servo pose; the next action is
geometry/direction diagnosis, not a larger command.

Suggested individual trace (replace the point with P0, P1, or P2) is:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\collect_balance_data.ps1 `
  -Port COM6 -Duration 5 `
  -Commands "0:STOP;1:LIKREF;2:LXY,<X>,<Y>;4:STOP" `
  -Out data\ik_physical_<point>.csv `
  -Note ik_physical_<point>
```

## 4. Motor-disabled physical acceptance record

Keep wheel-motor output disabled. At P0 (`LIKREF`) and at P1/P2, measure each
wheel centre from the BODY_WHEEL origin. Record target-minus-measured component
errors, the packed pose-status flags, provenance, and the four servo outputs.

| Side | Point / command | Target X (mm) | Target Y (mm) | Measured X (mm) | Measured Y (mm) | Error X (mm) | Error Y (mm) | Pose-status flags | Servo outputs (deg) | Result / notes |
|---|---|---:|---:|---:|---:|---:|---:|---|---|---|
| Left | P0 / `LIKREF` | -20.766667 | 47.356667 | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Left | P1 / `LXY,<P1_X>,<P1_Y>` | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Left | P2 / `LXY,<P2_X>,<P2_Y>` | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Right | P0 / `LIKREF` | -20.766667 | 47.356667 | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Right | P1 / `LXY,<P1_X>,<P1_Y>` | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Right | P2 / `LXY,<P2_X>,<P2_Y>` | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

The left leg source is measured calibration. The right leg has mirror
assumption provenance until it is measured independently at P0, P1, and P2.
Do not infer right measurements from left values. Dynamic wheel shift remains
prohibited until those independent right-leg measurements are recorded and
reviewed. Verify `LXY,1000,1000` is rejected and produces no servo output
change; also verify invalid FK clears the relevant validity bit without
replacing X/Y with a legacy stance value.

## Completion record

Record in the test log:

- firmware commit SHA and evidenced IAR build result (or unavailable);
- the four final `neutral_deg` values;
- BODY_WHEEL measurement method plus the completed motor-disabled table;
- P0/P1/P2 CSV filenames, rejection trace, and visual observations;
- minimum observed IK margin and any fault/IMU event.

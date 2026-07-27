# Low-race automatic leg assist for 400 RPM

## 1. Status and intent

This design was approved in conversation on 2026-07-19. It defines a staged
straight-line race mode that keeps the validated low-race pose as the normal
stance, uses coordinated wheel-centre motion above 250 RPM to assist
acceleration and normal braking, and ultimately targets 400 RPM.

The implementation must remain fail-closed and staged. Reaching a model-valid
IK target or emitting servo PWM is not, by itself, hardware acceptance.

## 2. Existing constraints

- The normal low-race pose is the absolute `BODY_WHEEL` target
  `(-18.83, 25.08) mm`, corresponding approximately to the tested left-leg
  `40 deg, 140 deg` pose.
- `BODY_WHEEL +X` points forward and `+Y` points down. A more-negative X moves
  the wheel centre rearward relative to the chassis. Increasing Y raises the
  chassis because the wheel centre moves farther below the body origin.
- At fixed `Y = 25.08 mm`, the current five-bar model accepts only about
  `X = -18.83 .. -18.16 mm`. The low-race zero point is already at the rear
  edge. The narrow interval is caused by the two linkage solutions approaching
  their reachability boundaries, not by the configured servo-angle limits.
- Therefore a useful longitudinal assist cannot keep Y fixed. It must follow a
  coupled X/Y path that temporarily raises the chassis by a few millimetres.
- Manual `LXY` is a stopped, bench-oriented command. Its existing stop and
  validation behavior must not be weakened to implement moving assist.
- The servo system is open loop. `servo_settled` means the commanded planner
  has settled; it is not measured joint-position feedback.
- The current fast forward target limit is about 220 RPM, the balance output is
  clamped at 300 RPM, and the motor layer retains a separate 1000 RPM hard
  target limit. Both upper control-layer limits must be changed before a
  measured 400 RPM can be requested.

## 3. Goals and non-goals

### Goals

1. Make the low-race pose the normal stance for straight-line race operation.
2. Below 230 RPM, leave longitudinal motion to the motor and balance loops.
3. Above 250 RPM, automatically move the wheel centres rearward to assist
   acceleration.
4. When the requested speed decreases, automatically move the wheel centres
   forward to assist normal braking.
5. Preserve a small rearward hold at high cruise speed instead of snapping the
   legs back to the zero pose.
6. Validate in four speed levels: 250, 300, 350, and 400 RPM.
7. Preserve a runtime enable so assisted and unassisted runs can be compared on
   the same course and the feature can be disabled immediately.

### Non-goals

- Repeated rowing or cyclic leg pumping is not part of this design.
- Emergency stop, sensor timeout, or a balance fault must not wait for the leg
  braking trajectory.
- This design does not add servo encoders or claim measured leg position.
- The first implementation does not add speed-loop integral gain or high-speed
  speed feedforward. Those remain separate tuning decisions based on captured
  data.
- High-speed turning performance is not a goal; steering authority is reduced
  above 300 RPM to preserve straight-line balance authority.

## 4. Coupled assist trajectory

Define a normalized assist value `u`:

- `u = 0`: normal low-race pose.
- `u > 0`: acceleration assist, wheel centre moves rearward.
- `u < 0`: braking assist, wheel centre moves forward.

For each validation level, map the filtered assist value to a physical target:

```text
x_cmd = -18.83 mm - dx_level * u
y_cmd =  25.08 mm + dy_level * abs(u)
```

This V-shaped path opens vertical clearance whenever the wheel centre moves
away from the zero pose. The exact runtime path must be preflighted using both
legs, the persisted IK branch identity, servo limits, and the configured
`0.02` minimum singularity margin before it is enabled on hardware.

Initial level:

| Purpose | u | Target (mm) |
|---|---:|---:|
| Low-race zero | 0 | `(-18.83, 25.08)` |
| Rearward assist | +1 | `(-20.83, 27.08)` |
| Forward brake | -1 | `(-16.83, 27.08)` |

The initial level therefore uses `dx_level = 2 mm` and `dy_level = 2 mm`.
After hardware acceptance, the design may expand to at most approximately
`dx_level = 5 mm` and `dy_level = 5 mm`, producing candidate endpoints near
`(-23.83, 30.08)` and `(-13.83, 30.08)`. Those expanded endpoints are design
candidates, not yet hardware-approved targets.

All four servos share one jerk-limited trajectory progress value. The first
hardware level retains the existing 90 deg/s command-rate limit. A faster
servo command rate requires separate measured acceptance.

## 5. Control allocation

The motor remains the source of sustained wheel power. Leg motion provides a
finite transient pitch/geometry bias and a small high-speed cruise bias; it
does not replace the wheel-speed controller.

The requested assist is conceptually:

```text
u_request = acceleration_feedforward
          + speed_error_correction
          + high_speed_hold_bias
```

The three terms are blended in only after the feature is armed. Gains, dead
bands, and the hold bias are tuning parameters and must not be guessed from the
model alone.

- Positive requested acceleration and positive speed error command rearward
  assist.
- A falling speed target or negative requested acceleration commands forward
  braking assist.
- Near a steady high-speed target, a smaller positive hold bias prevents the
  return trajectory from creating an unintended braking impulse.
- Below about 200 RPM, the hold bias is removed and the planner recentres at
  the normal low-race pose.

The existing virtual forward-tilt path remains active, but the initial assisted
tests cap its contribution at approximately the observed 7 deg safe region.
The design does not obtain 400 RPM by increasing the virtual angle beyond the
region where the vehicle has already shown falling tendency.

## 6. State machine and ownership

`control_chassis` owns speed intent, requested acceleration, arming decisions,
the normalized assist request, and the staged runtime RPM cap. `control_leg`
owns the continuous physical trajectory, dual-leg IK validation, branch
continuity, S-curve command generation, and leg faults. `control_balance`
continues to own pitch stabilization and final wheel command generation; it
consumes the staged output cap but does not directly command leg geometry.

The new internal state sequence is:

1. `DISABLED`: runtime assist is off; existing behavior is unchanged.
2. `LOW_RACE`: the low-race zero pose is stable and speed is below the arming
   band.
3. `ARMED`: race mode is selected, all inputs are healthy, and measured speed
   is in the 230--250 RPM preparation band.
4. `BOOST`: the target is above 250 RPM and the filtered assist moves positive.
5. `CRUISE_HOLD`: speed is near target and a smaller rearward bias is retained.
6. `BRAKE`: the target ramp is falling or measured speed exceeds the falling
   target; the filtered assist moves negative.
7. `RECENTER`: measured speed is below about 200 RPM and the legs return to the
   low-race zero pose.
8. `FAULT_HOLD`: continued leg motion is inhibited while the higher-priority
   chassis safety path reduces or removes wheel drive.

Unlike ordinary `LXY`, the internal race-assist state is drive-allowed and must
not be mapped to the existing generic leg-transition 30 RPM limit. It receives
its own level-dependent limit. Ordinary `LXY` retains all current stopped-test
semantics.

## 7. Staged speed and output limits

The 1000 RPM motor-layer hard target limit remains unchanged. A lower runtime
limit is applied at the chassis and balance layers:

| Validation level | Forward target limit | Balance output limit | Leg travel |
|---|---:|---:|---:|
| 1 | 250 RPM | 300 RPM | about +/-2 mm X |
| 2 | 300 RPM | about 350 RPM | accepted level-1 travel or a small increase |
| 3 | 350 RPM | about 410 RPM | hardware-accepted intermediate travel |
| 4 | 400 RPM | about 460 RPM | no more than about +/-5 mm X without a new review |

The balance limit stays above the forward target so pitch stabilization retains
command reserve. The runtime level defaults to the lowest level after a fresh
build or reset until the previous level has passed hardware acceptance.

Above 300 RPM, commanded turn differential is progressively reduced. Full turn
authority must not consume the balance reserve needed to keep a 400 RPM
straight-line run upright.

## 8. Arming, braking, and fault behavior

Assist may arm only when all of the following are true:

- the existing fast race mode is selected;
- the low-race zero pose is stable;
- the runtime assist switch is enabled;
- wheel speed, IMU, and leg-control data are fresh and valid;
- both legs pass the complete preflighted path contract;
- pitch and pitch rate are inside configurable safe windows; and
- the requested validation level has been explicitly enabled.

Normal automatic braking is triggered by a falling target ramp or a sufficiently
negative speed error. Its forward leg motion is filtered and limited; it is not
an emergency-stop mechanism.

Fault priority is:

1. Emergency stop, command timeout, IMU fault, or balance fault uses the existing
   immediate chassis safety behavior. It never waits for a servo trajectory.
2. IK rejection, branch mismatch, non-finite target, or leg fault freezes further
   assist progression, removes the high-speed request, and enters a controlled
   speed reduction when chassis feedback is still healthy.
3. Pitch or pitch-rate excursion prevents additional rearward travel and ramps
   the speed request down.
4. A normal requested slowdown follows the automatic forward-assist path and
   recentres only after measured speed is below the recenter threshold.

No fault path commands an unvalidated recovery pose at speed.

## 9. Telemetry and command contract

New telemetry is appended after existing fields so current VOFA channel numbers
do not move. At minimum it publishes:

- assist enable and validation level;
- race-assist state;
- `u_request` and filtered `u_actual`;
- commanded physical X/Y;
- requested acceleration;
- target, ramped, and measured wheel speed plus speed error;
- effective pitch target, measured pitch, and pitch rate;
- final balance output and active runtime output cap;
- left/right IK margin, branch identity, planner-settled state, and fault reason.

The runtime interface provides at least an assist enable/disable control and a
validation-level selector. Disabling assist returns to existing motor/balance
behavior and recentres only when the current speed and balance state make that
motion safe. Wire syntax is chosen during implementation planning after checking
the live host-command namespace for conflicts.

## 10. Verification and hardware acceptance

### Static and supported-chassis gates

1. Sweep every sample of the acceleration and braking paths through production
   IK for both legs.
2. Require fixed branch identity, finite results, servo commands within limits,
   and singularity margin at least `0.02` at every sample.
3. With wheel power disconnected and the chassis supported, command zero,
   rearward, forward, and complete return trajectories one at a time.
4. Confirm no mechanical interference, no unexpected direction, no command
   discontinuity, and no leg fault. Planner-settled output is recorded as
   command evidence only.

### Ground gates

1. At low speed, verify that a falling target produces forward wheel-centre
   motion and the expected braking pitch response.
2. At level 1, compare otherwise identical 250 RPM runs with assist disabled and
   enabled. Record acceleration time, peak pitch, pitch rate, output saturation,
   commanded X/Y, and the complete fault state.
3. Expand leg travel only when the smaller travel has the correct direction and
   materially improves acceleration or braking without increasing instability.
4. Advance through 300, 350, and 400 RPM in order. Failure at any level blocks
   the next level.

The first 400 RPM acceptance requires measured wheel speed in approximately the
`390--410 RPM` band for at least three seconds, no IK or leg fault, no persistent
balance-output saturation, fresh feedback, and controllable straight-line
behavior. Longer sustained operation, thermal acceptance, and high-speed turn
tuning are subsequent gates.

## 11. Implementation boundaries

Implementation planning must cover, at minimum:

- configuration and staged limits;
- the race-assist state machine and normalized request generator;
- continuous model-valid leg trajectory execution that does not use manual
  `LXY` stop semantics;
- balance runtime caps and high-speed turn derating;
- appended telemetry and VOFA field documentation;
- static numeric tests for the complete X/Y paths and fault transitions; and
- a hardware procedure that records level-by-level acceptance evidence.

No implementation task may mark 400 RPM complete based only on compilation,
static IK, commanded servo PWM, or an unloaded wheel-speed reading.

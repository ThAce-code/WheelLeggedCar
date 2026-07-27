# Low-race LXY extension design

## Objective

Allow the existing `LXY` command to reach the low-race pose that has already
been exercised on hardware through `LIK`: left-leg logical servo commands
`40 deg, 140 deg`.  Do not add a new wire command and do not change `LIK`.

With the current left-leg calibration, the command estimate is approximately:

```text
BODY_WHEEL = (-18.831, 25.076) mm
model      = ( 18.810, 23.864) mm
```

This coordinate is an experimental model estimate derived from a hardware-
reachable servo pose.  It is not a new measured wheel-centre calibration
sample.

## Scope and safety boundary

The calibrated eight-vertex physical hull and its 2 mm inset remain unchanged
for all existing `LXY` targets.  A separate, narrowly bounded experimental
low-race target region is added around the estimated pose.  Targets are valid
when either:

1. they pass the existing calibrated inset-hull check; or
2. the experimental feature flag is enabled and they fall inside the configured
   low-race tolerance region.

The experimental region must not turn the physical workspace into a rectangle
or globally bypass target validation.  `LXY,0,55` must remain rejected.

The private model lower Y bound is extended only enough to solve and forward-
project the verified `40 deg, 140 deg` pose.  Existing X bounds, upper Y bound,
IK branch selection, singularity checks, finite-value checks, and per-servo
`10 deg .. 175 deg` limits remain active.

## Command and control flow

The wire syntax remains `LXY,x_mm,y_mm` over the existing UART0/VOFA downlink.
Before accepting the target, the command path continues to stop chassis motion,
turn balance output off, and stop the wheel motors.  Accepted targets continue
through the existing shared S7 leg trajectory and the 300 Hz double-buffered
servo executor.  Direct actuator bypass is not enabled.

No automatic speed-to-wheel-shift mapping and no `B,3` integration are part of
this change.

## Configuration

The low-race extension is explicit configuration, not a magic value embedded in
the parser.  It contains:

- an enable flag;
- the estimated `BODY_WHEEL` X and Y target;
- a small X/Y acceptance tolerance;
- a low-race-only minimum IK margin;
- explicit low-race alpha/beta branch selection;
- the minimally extended private-model Y lower bound.

The initial acceptance tolerance is `0.5 mm` per axis.  This permits rounded
commands such as `LXY,-18.83,25.08` while rejecting unrelated uncalibrated
coordinates.  The hardware-tested pose has a minimum geometric IK margin of
approximately `0.0244`, below the normal `0.20` gate.  The experimental window
therefore uses a nonzero `0.02` minimum while all normal targets retain `0.20`.
The tested `40,140` pose uses the `PLUS` beta root rather than the normal
`MINUS` beta branch.  Inside the narrow experimental window, both alpha and
beta branches are explicitly `PLUS`, and nearest-root continuity must not
override them.  The shared S7 command trajectory remains responsible for a
smooth physical transition.

## Diagnostics

Existing telemetry remains the acceptance surface:

- channels 18..21: servo command outputs;
- channels 22..25: servo planner targets;
- channel 31: servo settled;
- channels 33..36: left/right command-estimate poses;
- channel 37: IK margin;
- channels 38..39: leg motion state and fault reason.

The pose continues to be labelled a command/model estimate, not measured
feedback.  The right-leg coordinate remains a mirror assumption until measured
independently.

## Tests

Tests are written before production changes and must demonstrate:

1. the low-race target fails under the current implementation;
2. the configured low-race target and its rounded command are accepted after
   the extension;
3. left `40,140` and right mirrored `140,40` forward commands resolve near the
   configured low-race target;
4. points outside the narrow tolerance remain rejected;
5. `LXY,0,55`, raw hull vertices, non-finite inputs, and servo-limit violations
   remain rejected;
6. existing physical-IK, coordinate-contract, zero-calibration, transition,
   servo-integration, and IAR build gates still pass.

## Hardware gate

The first flashed test remains supported and stationary with wheel-motor power
disconnected.  Send `LIKREF`, then `LXY,-18.83,25.08`, and require a settled
trajectory with no leg fault or mechanical interference.  This static result
does not authorize dynamic wheel shift, balance fast mode, or higher speed.

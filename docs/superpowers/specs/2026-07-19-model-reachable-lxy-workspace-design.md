# Model-reachable LXY workspace design

## Objective

Replace the calibrated physical hull and the coordinate-specific low-race
exception with one five-bar-model reachability rule for every `LXY` target.
The public command remains an absolute wheel-centre target in the
`BODY_WHEEL` frame.  The fitted similarity transform remains the only mapping
between public physical coordinates and private model coordinates.

For this feature, "model reachable" means all of the following are true:

1. the physical target and transformed model target are finite;
2. the five-bar equations produce real inverse-kinematics candidates;
3. the smaller normalized singularity margin is at least `0.02`;
4. at least one candidate pair maps to valid left-leg servo commands;
5. at least one candidate pair maps to valid right-leg servo commands.

No measured hull, rectangular combination of measured extrema, or
coordinate-specific race window participates in this decision.

## Reachability architecture

Reachability and solution selection use a shared internal candidate engine.
Given one model-space target, the engine calculates the plus and minus roots
for both driven joints, producing up to four alpha/beta combinations.  It then
maps every combination through the existing reference, direction, offset, and
left/right mirror configuration and rejects combinations that violate any
servo limit.

`leg_kinematics_target_valid()` asks this engine whether at least one command-
valid solution exists for both sides.  `leg_kinematics_solve()` asks the same
engine to return the best solution for one side.  Target validation must not
duplicate a weaker approximation of the solver, and the public validation API
must not call itself indirectly.

The existing configured model-space X/Y box is removed as a workspace shape.
The geometric discriminants, the lower-half-plane requirement for the wheel
centre, the `0.02` singularity margin, and servo limits define the usable
boundary.  Forward kinematics may use small numerical tolerances when matching
circle intersections back to an IK solution, but those tolerances must not
become a new empirical workspace polygon or rectangle.

## Branch and continuity policy

Multiple inverse solutions can describe the same wheel-centre coordinate.
Candidate selection follows this order:

1. discard non-finite, below-margin, and servo-limit-invalid candidates;
2. when a valid previous IK result exists, require its stored alpha/beta branch
   identities to be valid and select only that same identity combination;
3. if the stored identity is invalid or its candidate is no longer command
   valid, fail closed instead of changing branches automatically;
4. without a previous result, prefer the configured normal branch combination;
5. if the configured combination is unavailable, select the valid candidate
   nearest the calibrated reference pose;
6. break exact score ties deterministically by candidate enumeration order.

The branch lock prevents a marginal nearest-angle advantage from switching to
a root that is about to leave a servo limit.  For example, on the
right leg at `Y=28 mm`, the reverse `X=-13 -> -14 mm` path must retain the
alpha PLUS identity instead of selecting MINUS for a temporary `0.45 deg`
score advantage and then jumping back when MINUS exceeds `175 deg`.  If a
persisted branch later leaves its servo limit, the inverse solve is rejected;
an automatic branch change requires a separate, explicitly controlled path.
Forward kinematics root matching is a private geometric operation and
continues to match the supplied raw joint angles rather than inventing previous
branch identity.

The low-race target near `BODY_WHEEL=(-18.83,25.08) mm` has two close beta
solutions, mapping to approximately `140 deg` and `137.2 deg`.  In the unified
workspace, the wheel-centre coordinate and branch continuity are the contract;
an exact `140 deg` output is no longer forced by a coordinate exception.  Both
solutions remain subject to the same servo limits and `0.02` margin.

## Configuration changes

Remove configuration that only describes empirical acceptance shapes or the
temporary race exception:

- `physical_workspace` and `physical_workspace_inset_mm`;
- all `experimental_race_*` fields;
- model-space `x_min_mm`, `x_max_mm`, `y_min_mm`, and `y_max_mm` when they are
  used as reachability gates.

Retain link lengths, reference coordinates, similarity-transform parameters,
servo calibration, normal branch preferences, and per-servo minimum/maximum
angles.  Replace the split `0.20`/`0.02` policy with one explicit model
workspace minimum margin of `0.02`.

## Command and safety flow

The wire command remains `LXY,x_mm,y_mm` over the existing UART0/VOFA path.
Before an `LXY` target is applied, the command layer continues to stop chassis
motion, disable balance output, and stop the wheel motors.  Accepted targets
continue through the shared S7 leg trajectory and the 300 Hz servo executor.

Before changing the stored active target, `control_leg_set_xy()` also preflights
the target for both sides against the current persisted branch identities.
Failure to preserve either side's branch rejects the command and leaves the old
target and previous results unchanged.

Rejected targets do not alter the active leg target and continue to publish the
existing IK-valid, IK-margin, motion-state, and fault-reason diagnostics.  NaN,
infinity, unreachable geometry, singularity margin below `0.02`, and servo
limit violations are fail-closed errors.

This change does not enable wheel drive, balance fast mode, or automatic
speed-to-wheel displacement.

## Test strategy

Tests are added before production changes and must first fail because the
current hull and race-window behavior is still present.

The numeric tests will:

1. scan a deterministic physical-coordinate grid covering and extending past
   the former hull;
2. independently classify each target from real IK roots, the `0.02` margin,
   and left/right servo limits;
3. require `leg_kinematics_target_valid()` to match that classification;
4. require every accepted target to solve for both sides and map to finite,
   in-range servo commands;
5. require every rejected target to fail for geometry, margin, lower-half-plane,
   or servo-limit reasons rather than its relation to the old hull;
6. check adjacent accepted targets for bounded mapped-servo movement in each
   side's `+X`, `-X`, `+Y`, and `-Y` directions when a previous result is
   supplied, clearing previous state across independently rejected gaps;
7. preserve the all-90-degree reference pose and forward/inverse round trips;
8. keep the low-race coordinate accepted without requiring an exact beta
   command of `140 deg`;
9. reject non-finite inputs and points with singularity margin below `0.02`.

Static contract tests will reject residual production references to the old
hull and `experimental_race_*` configuration.  Existing coordinate-contract,
zero-calibration, physical-IK, transition, 300 Hz servo integration, and legacy
height-control tests must remain green.  All three IAR projects must build.

The grid reachability oracle is independent of the production validation and
solve APIs: the harness itself transforms the point, calculates both roots for
each driven joint, evaluates `Y > 0` and the `0.02` margin, maps all four root
combinations through left/right servo calibration and limits, and then compares
that per-point classification with `leg_kinematics_target_valid()`.  The fixed
`5637 / 6561` accepted count remains a second-layer baseline, not the oracle.

## Hardware validation boundary

Software verification establishes model consistency only.  The four servos
remain open-loop, so a command angle is not measured joint-position feedback.
After flashing, initial validation remains supported and stationary with wheel
motor power disconnected:

```text
STOP
LIKREF
LXY,-18.83,25.08
```

Further workspace points are introduced from the reference outward in small
steps while recording target coordinates, command-estimate coordinates, servo
outputs, IK margin, settled state, and fault reason.  Any mechanical
interference, unexpected branch motion, or loss of support stops expansion even
when the mathematical model reports the point as reachable.

## Out of scope

- new UART commands;
- independent right-leg geometric recalibration;
- servo position feedback or closed-loop joint control;
- collision detection not represented by link geometry and servo limits;
- automatic race-height transitions or wheel-position speed control;
- balance or LQR retuning.

# Physical-Coordinate Leg IK Calibration Design

Date: 2026-07-17
Status: Approved in conversation for implementation

## Goal

Make `LXY,X,Y` command the wheel center in physical millimetres relative to the
fixed chassis cross-circle marker. Replace the current uncalibrated model
coordinate interpretation without changing the five-bar link dimensions,
servo PWM path, 300 Hz scheduler, S7 trajectory, or actuator filtering.

## Coordinate Contract

The fixed chassis marker is the only physical origin:

```text
O = (0, 0)
+X = vehicle forward
+Y = downward
```

The cross-circle measurement already computes the wheel marker minus the
origin marker after mapping both points into the calibrated side plane. Its
CSV `measured_x_mm` and `measured_y_mm` fields are therefore absolute physical
wheel-center coordinates in this frame. They must not be re-zeroed around the
neutral pose.

The three captures made with `LIK,90,90,90,90` are:

```text
ref_start = (-21.45, 48.09) mm
ref_mid   = (-20.41, 46.59) mm
ref_end   = (-20.44, 47.39) mm
```

Their equal-weight least-squares reference is:

```text
physical_reference = (-20.7667, 47.3567) mm
```

The individual samples remain in the fit and drift checks. Taking their mean
only estimates the repeated 90-degree reference position; it does not hide
the approximately 1.23 mm start-to-end drift.

The required command equivalence is:

```text
LIK,90,90,90,90
LIKREF
LXY,-20.7667,47.3567
```

`LXY,0,55` means the physical point `(0,55)` relative to the chassis marker.
It is not the neutral pose and is outside the X coverage of the current data,
so the initial calibrated workspace must reject it.

## Calibration Model

The fixed link geometry remains:

```text
l1 = 60 mm
l2 = 90 mm
l3 = 90 mm
l4 = 60 mm
l5 = 37 mm
```

Only the visible five-bar side, driven by servo0 and servo2, is identified by
the current 20-point data set. Let `q0` and `q2` be their LIK command angles.
Command angles map to geometric angles by:

```text
alpha = alpha_ref + d0 * (q0 - 90 deg)
beta  = beta_ref  + d2 * (q2 - 90 deg)
```

`alpha_ref` and `beta_ref` are fitted geometric reference angles. `d0` and
`d2` are discrete direction signs selected from `{-1,+1}` by testing all four
combinations. A direction result is accepted only if it is stable across
cross-validation folds.

Let `F(alpha,beta)` be the existing five-bar forward model in its local model
frame. The local-to-physical mapping is an anchored constrained similarity:

```text
Pphysical(q) = Pref + s * Q * (F(alpha,beta) - F(alpha_ref,beta_ref))
```

where:

- `Pref = (-20.7667,47.3567) mm` is the measured physical reference;
- `s` is one uniform scale close to one;
- `Q` is an orthogonal 2-by-2 rotation or reflection-plus-rotation matrix;
- shear, independent X/Y scales, and an unconstrained translation are not
  allowed.

Anchoring the transform at the measured reference preserves the physical
origin while guaranteeing the 90-degree command equivalence. It does not
force the neutral wheel center to `(0,0)` or `(0,55)`.

The inverse used for `LXY` is:

```text
Pmodel_target = F(alpha_ref,beta_ref)
              + transpose(Q) * (Pphysical_target - Pref) / s
```

The existing five-bar inverse solver then converts `Pmodel_target` to
geometric angles, and the inverse command mapping converts those angles to
servo commands.

## Offline Fitter

`tools/fit_leg_ik_calibration.py` will replace its offset-only interpretation
for this data path with the anchored model above. It will:

1. Parse and validate all command and physical-coordinate samples.
2. Require exactly one `ref_start`, `ref_mid`, and `ref_end`, all at 90 degrees.
3. Compute `Pref` from those three samples and report their drift separately.
4. Enumerate all four command-direction combinations.
5. Fit `alpha_ref`, `beta_ref`, similarity orientation, and uniform scale.
6. Run deterministic five-fold cross-validation.
7. Print per-sample physical residuals and a C configuration candidate.
8. Refuse to emit a candidate when any acceptance gate fails.

Initial acceptance gates are:

- all 20 expected samples are present and finite;
- reference start-to-end drift is at most 2 mm;
- measured X and Y coverage is at least 10 mm each;
- the same direction pair wins every fold;
- scale is within `[0.8,1.2]`;
- mean cross-validation radial RMSE is at most 2 mm;
- full-fit maximum radial error is at most 3 mm.

The fitter prints a candidate for review and does not directly edit firmware.

## Firmware Integration

The kinematics configuration will store the physical reference, geometric
reference angles, command directions, scale, and orthogonal matrix. The
coordinate conversion will be isolated from the existing five-bar circle
intersection solver.

Firmware data flow becomes:

```text
LXY physical target
  -> physical-to-model inverse similarity
  -> existing five-bar inverse kinematics
  -> geometric-to-command reference mapping
  -> existing S7 motion and 300 Hz servo output
```

Forward diagnostics use the reverse path:

```text
servo command
  -> command-to-geometric mapping
  -> existing five-bar forward kinematics
  -> model-to-physical similarity
  -> physical X/Y telemetry
```

`LIK` remains a direct four-servo command for calibration and diagnosis.
`LIKREF` commands the nominal 90-degree reference and reports the measured
physical reference coordinates.

## Workspace Safety

The current data covers approximately:

```text
X = -40.62 to -15.04 mm
Y =  32.17 to  88.49 mm
```

A rectangle spanning those extrema would contain unsupported combinations.
Initial `LXY` acceptance therefore uses the convex hull of the calibrated
physical samples, contracted inward by 2 mm. A target must
also pass the transformed model workspace, IK branch, servo angle, and IK
margin checks. Every failure is fail-closed and leaves the previous safe
target unchanged.

The workspace is not expanded toward `X=0` until additional valid calibration
samples cover that region.

## Two-Side Scope

The present CSV identifies only the visible side driven by servo0 and servo2.
It cannot identify independent servo1/servo3 reference or mounting errors.
The opposite side may use the nominal mechanical mirror only for unloaded,
restricted validation; it is not considered calibrated until separately
measured or verified against equivalent physical points.

Full synchronized leg motion and balance testing must not treat the single-side
fit as proof that both sides have equal physical coordinates.

## Verification

Offline verification will cover:

- exact 90-degree reference anchoring at `(-20.7667,47.3567)`;
- direction enumeration and fold stability;
- fit residual and drift rejection gates;
- forward/inverse similarity round trips;
- physical-to-model-to-physical round trips;
- rejection outside the shrunken calibration hull;
- preservation of existing IK branch, limit, and fault behavior;
- regression proving the old offset-only `x_offset=43.013`,
  `y_offset=90.067` candidate is not accepted.

Unloaded hardware acceptance starts at `LIKREF`, then uses small physical
targets inside the calibrated hull around `(-20.7667,47.3567)`. Direction,
measured displacement, settling, and holding force must pass before the range
is increased or balance and wheel drive are enabled.

## Non-Goals

- Do not change the measured link lengths.
- Do not change servo PWM frequency, pulse calibration, filtering, S7 timing,
  or scheduler periods.
- Do not infer independent right-side calibration from left-side data.
- Do not reinterpret the chassis marker origin as the neutral wheel position.
- Do not enable targets outside measured calibration coverage.

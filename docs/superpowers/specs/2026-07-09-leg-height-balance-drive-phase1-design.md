# Phase 1 Leg-Height, Balance, and Low-Speed Drive Coordination

## Goal

Make the robot remain balanced while its two wheel-leg mechanisms move between
the calibrated 35 mm and 120 mm height commands, and allow conservative
low-speed driving during that transition.

Phase 1 is open-loop at the leg mechanism: the four leg servos accept PWM
commands only and provide no position feedback.  The wheel-speed and IMU
feedback loops remain the only live vehicle-state feedback used for control.

## Scope

Included:

- smooth, bounded leg-height trajectories;
- five-bar IK validation and continuous branch selection;
- coordination of height state with balance gain scheduling and chassis speed
  limits;
- safe transition abort behavior;
- offline calibration and numerical trajectory validation;
- staged IAR and hardware verification.

Excluded:

- active leg pitch or roll compensation;
- wheel-center X-axis control;
- closed-loop leg-height control;
- servo or linkage sensor hardware;
- LQR, MPC, or iterative real-time IK.

## Existing Baseline

`control_leg` updates the height trajectory and calls the five-bar analytic
IK every 10 ms.  `control_balance` already interpolates balance gains and
pitch reference by leg height, while `control_chassis` limits forward speed
during a height change.  The current reported `actual_height_mm` is a
commanded/ramped trajectory value, not a measured physical height.

The configured 90 degree command for every leg servo has been verified on the
vehicle as a stable support pose.  It is therefore the Phase 1 soft-fault
return pose.

## Architecture

```text
LH,height command
    |
    v
leg transition supervisor (state, rate, acceleration, validity)
    |                         |
    |                         +--> motion allowance --> chassis / balance
    v
five-bar analytic IK --> calibrated PWM targets --> servo actuator
```

The transition supervisor owns the only height trajectory.  Balance and
chassis controllers consume its diagnostic/state output; they must not infer a
transition independently from a target-versus-current comparison.

## State and Data Contract

Add a leg motion state to `leg_diag_struct`:

- `LOCKED`: safe support pose is commanded;
- `STABLE`: a valid target height is reached and motion has settled;
- `TRANSITION`: a valid bounded height trajectory is active;
- `FAULT`: invalid IK or an unsafe transition has been latched.

Expose the commanded trajectory height, trajectory rate, target height, IK
validity, branch/singularity margin, and a `drive_allowed`/effective speed
limit.  Rename or document the present `actual_height_mm` as trajectory or
estimated height so telemetry does not imply that it is measured feedback.

The state is authoritative for all dependent control modules.  Only
`STABLE` permits the normal height-dependent chassis limit.  `TRANSITION`
permits the configured conservative transition limit and disables fast drive.

## Leg Control Behavior

1. Validate a requested height against calibrated workspace and configuration.
2. Generate a speed- and acceleration-limited S-curve trajectory.  Initial
   limits come from configuration and are hardware-tuned; no new safety number
   is hard-coded into control logic.
3. Solve both legs from the trajectory point, with a candidate set for each
   side.  Choose a valid candidate that is within calibrated servo limits and
   nearest the preceding valid joint command.  Reject candidates with an
   insufficient configured singularity margin.
4. Calibrate the two IK angles into each servo's PWM-command coordinate,
   validate them, and send them to the existing servo rate limiter.
5. Enter `STABLE` only after the trajectory reaches the requested height and
   the configured settle condition is satisfied.

The analytic IK remains the real-time solver.  It is deterministic, bounded,
and inexpensive on this MCU.  Numerical optimization is reserved for offline
calibration of link lengths, neutral angles, and scale/direction parameters.

## Balance and Drive Coordination

During `TRANSITION`:

- force fast-drive off;
- limit forward command to the transition speed limit;
- smoothly schedule pitch gain, pitch-rate damping, wheel-speed damping, and
  pitch reference from the trajectory height;
- rate-limit every scheduled value or blend the controller output so changing
  height cannot cause a wheel-command step;
- retain the wheel/IMU feedback health checks already required by the chassis
  and balance loops.

During `STABLE`, use the configured height-specific limits and scheduled
gains.  A height command never directly changes the wheel command; it changes
only the leg trajectory and the bounded limits/parameters seen by the wheel
controllers.

## Fault Behavior

### Soft transition fault

For invalid IK, a servo-limit violation, branch discontinuity, or insufficient
IK margin:

1. stop wheel output immediately;
2. cancel the active height transition;
3. command the verified 90 degree support pose through the existing servo
   speed limiter;
4. latch `FAULT`, expose a reason in diagnostics, and require a fresh operator
   height command before another transition.

### Vehicle safety fault

Existing IMU roll/pitch or IMU-health faults retain precedence: they stop the
motors and disable servo PWM through `app_safety_force_fault()`.

Because the servos have no feedback, mechanical blockage, loss of torque, and
unplugged servo power are not directly detectable.  These risks are mitigated
by conservative trajectory limits, the IMU safety monitor, mechanical stops,
and the staged hardware test protocol; they cannot be eliminated in software.

## Files Expected to Change

| File | Change |
|---|---|
| `project/code/app_types.h` | Leg transition state, diagnostics, and fault reason. |
| `project/code/leg_config.h/.c` | Trajectory, settle, IK-margin, and transition-drive configuration. |
| `project/code/leg_kinematics.h/.c` | Stable candidate selection, singularity margin, and forward kinematics. |
| `project/code/control_leg.h/.c` | Transition supervisor, fault latch, command gating, and authoritative motion status. |
| `project/code/control_balance.c` | Rate-limited, state-aware height gain scheduling. |
| `project/code/control_chassis.c` | State-aware drive allowance and fast-drive interlock. |
| `project/code/telemetry.c` | State, fault reason, trajectory rate, and effective motion limit. |
| `tools/` | Numeric IK/trajectory checks and calibration-fit tooling. |

## Validation

### Offline

- Sweep the full configured height range and workspace edges.
- Check every candidate for finite values, calibrated servo limits, continuity,
  and required singularity margin.
- Check commanded joint-rate and acceleration against configured limits.
- Fit calibration data offline and record residual position error.

### Firmware

- Build the affected `cyt4bb7_cm_7_0` IAR project.
- Run the project static checks, extended with state-machine and safety-path
  assertions.

### Hardware progression

1. Supported/bench leg motion with wheel output disabled.
2. Stationary supported vehicle at low, default, and high height.
3. Balance-in-place height transitions with no drive command.
4. Low-speed straight driving during height transitions.
5. Low-speed turn and stop behavior during height transitions.

Advance only when the previous step shows no IK fault, servo discontinuity,
sustained balance saturation, or safety trip.  Thresholds for acceptable pitch
excursion, output saturation, and low-speed limit are recorded from the
baseline captures before enabling the next hardware step.

## Acceptance Criteria

- Any command in the calibrated 35--120 mm height range produces a continuous
  valid PWM target trajectory or fails into the defined safe state.
- No branch jump or servo-limit violation occurs in the numeric sweep.
- A transition commands only conservative low-speed drive and never fast drive.
- Height-dependent balance and chassis parameters change without a wheel-output
  step attributable solely to scheduling.
- Any defined leg-control fault stops the wheels and reaches the verified
  90 degree support command through the servo speed limiter.
- IAR build and staged hardware tests are recorded before Phase 1 is accepted.

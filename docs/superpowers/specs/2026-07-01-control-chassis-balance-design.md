# Control Chassis And Balance Design

## Goal

Build the first software layer for wheel-leg chassis and balance control on top of the existing motor actuator interface.

The first version should not promise fully autonomous standing or driving. Its deliverable is a safe, observable, and controllable low-power balance tuning path:

- `control_chassis` accepts forward and turn commands and produces left/right base motor RPM.
- `control_balance` owns balance mode, safety gates, and low-power pitch feedback.
- The final output goes through `actuator_motor_set_mode_motor_rpm()` or `actuator_motor_set_mode_stop()`.
- Debug commands and telemetry expose enough state to verify sensor direction, wheel direction, mode transitions, and safety behavior.

## Current Context

The motor actuator layer already provides the intended lower-level boundary:

- `actuator_motor` owns motor mode, RPM loop, open duty mode, host motion timeout, feedback online checks, limits, and PID state.
- `bldc_foc_uart` remains the CYT2BL3 protocol adapter.
- `host_command` remains a VOFA/UART0 debug parser.
- `control_leg` currently mixes body height/pitch/roll commands into four servo angles.

The new balance/chassis work must build above this boundary. It must not write BLDC UART frames directly, parse VOFA strings inside control logic, or modify actuator PID internals.

## Selected Approach

Use a layered skeleton first, but include a conservative low-power balance test mode.

This combines two needs:

- Get the long-term module boundaries correct before tuning the robot on hardware.
- Provide enough real control output to begin pitch direction, wheel direction, and gain sign validation immediately.

The first version avoids LQR, complex filtering, leg kinematics changes, and servo/body coordination. Those belong after the base balance path is proven.

## Architecture

The intended runtime flow is:

```text
host_command / future remote input
    -> control_chassis command

scheduler
    -> sensor_imu_update
    -> app_safety_update
    -> control_chassis_update
    -> control_balance_update
    -> actuator_motor_update
    -> telemetry_update
```

`control_chassis` owns chassis motion intent.

It stores an enable flag plus `forward_rpm` and `turn_rpm`, clamps them, and mixes them into left/right base RPM. It does not read IMU state and does not call the motor actuator directly.

`control_balance` owns the balance state machine and closed-loop output.

It reads IMU state, motor feedback, app state, and the chassis output. In balance test mode it computes a limited balance RPM correction and adds it to the chassis base RPM before commanding the motor actuator.

`control_leg` remains independent in the first version.

The existing attitude-to-servo command path is left unchanged. Future wheel-leg coordination may let `control_balance` command body pitch or height through `control_leg_set_body_cmd()`, but that is outside this first version.

## Modes

Add a balance mode enum:

```c
typedef enum
{
    BALANCE_MODE_OFF = 0,
    BALANCE_MODE_STANDBY,
    BALANCE_MODE_BALANCE_TEST
}balance_mode_enum;
```

Mode behavior:

- `BALANCE_MODE_OFF`: always commands motor STOP.
- `BALANCE_MODE_STANDBY`: keeps control state alive for telemetry, but commands motor STOP.
- `BALANCE_MODE_BALANCE_TEST`: if all safety gates pass, computes low-power balance output and sends left/right motor RPM.

The application should initialize balance in standby or off, never directly into nonzero motor output.

## Chassis Command

Add a chassis command structure:

```c
typedef struct
{
    float forward_rpm;
    float turn_rpm;
    uint8 enable;
    uint32 last_cmd_ms;
}chassis_cmd_struct;
```

Add a chassis output structure:

```c
typedef struct
{
    float left_base_rpm;
    float right_base_rpm;
    uint8 enable;
}chassis_output_struct;
```

Mixing rule:

```text
left_base_rpm  = forward_rpm - turn_rpm
right_base_rpm = forward_rpm + turn_rpm
```

Both command and mixed outputs are clamped by `APP_CHASSIS_RPM_LIMIT`.

`control_chassis` does not decide whether the robot may balance. It only normalizes motion intent.

## Balance Control

Add a balance diagnostic structure:

```c
typedef struct
{
    balance_mode_enum mode;
    float pitch_deg;
    float pitch_rate_dps;
    float chassis_left_rpm;
    float chassis_right_rpm;
    float balance_rpm;
    float output_left_rpm;
    float output_right_rpm;
    uint8 output_enable;
    uint8 safety_blocked;
}balance_diag_struct;
```

First-version control law:

```text
balance_rpm = (APP_BALANCE_PITCH_KP * pitch_deg)
            + (APP_BALANCE_PITCH_RATE_KD * pitch_rate_dps)
```

Then:

```text
output_left_rpm  = chassis_left_base_rpm  + balance_rpm
output_right_rpm = chassis_right_base_rpm + balance_rpm
```

The balance correction is clamped by `APP_BALANCE_RPM_LIMIT`. The final left/right output is clamped by the motor actuator's own RPM target limit as a second line of defense.

Pitch rate source:

- Prefer an IMU-provided pitch rate if the existing driver exposes one later.
- For the first implementation, derive pitch rate inside `control_balance` from pitch delta over update period.
- Reset the derivative state when mode changes, output is blocked, or the time step is invalid.

## Safety Gates

`control_balance` must command STOP when any of these are true:

- mode is `BALANCE_MODE_OFF` or `BALANCE_MODE_STANDBY`;
- app state is `APP_STATE_FAULT`;
- IMU state is unhealthy;
- motor feedback is offline while balance output would be active;
- absolute pitch exceeds `APP_BALANCE_TEST_PITCH_LIMIT_DEG`;
- chassis/balance command is disabled;
- calculated time step is invalid for derivative update.

The safety behavior must be conservative: blocked balance output means motor STOP, not zero RPM closed-loop output.

The first version should use a low `APP_BALANCE_RPM_LIMIT`, such as 100 to 200 RPM, until hardware direction and gain signs are confirmed.

## Configuration

Add configuration constants in `app_config.h`:

```c
#define APP_CHASSIS_PERIOD_MS              (5U)
#define APP_BALANCE_PERIOD_MS              (5U)
#define APP_CHASSIS_RPM_LIMIT              (200.0f)
#define APP_BALANCE_RPM_LIMIT              (150.0f)
#define APP_BALANCE_TEST_PITCH_LIMIT_DEG   (20.0f)
#define APP_BALANCE_PITCH_KP               (0.0f)
#define APP_BALANCE_PITCH_RATE_KD          (0.0f)
```

The default gains are zero so that enabling the mode cannot unexpectedly move the wheels before the operator intentionally sets tuning values in code or a later runtime tuning path.

## Debug Commands

Extend `host_command` with the minimum balance/chassis commands:

```text
B,0
B,1
B,2
C,forward,turn
```

Meanings:

- `B,0`: set balance mode off.
- `B,1`: set balance mode standby.
- `B,2`: set balance mode balance test.
- `C,forward,turn`: set chassis forward/turn RPM command and enable chassis command.

Existing motor actuator debug commands remain available:

```text
STOP
M,rpm
D,duty
P,kp,ki,kd
PL,kp,ki,kd
PR,kp,ki,kd
```

`STOP` must remain a hard stop. It should stop the motor actuator and put balance output into a non-output state so that the next balance output requires an explicit `B` command.

`M` and `D` are direct actuator tests. They should not be used at the same time as balance testing.

## Telemetry

For the first balance tuning phase, telemetry should prioritize control observability over PID parameter display.

A balance-focused 8-channel VOFA layout is:

```text
I0 = time_ms
I1 = balance_mode
I2 = pitch_deg
I3 = pitch_rate_dps
I4 = chassis_left_rpm
I5 = chassis_right_rpm
I6 = balance_rpm
I7 = motor_feedback_online
```

If motor actuator PID tuning is still being performed, keep the current motor telemetry layout until that work is complete, then switch to the balance layout. Do not expand the first version into a large telemetry frame unless hardware debugging proves the 8-channel layout is insufficient.

## File Responsibilities

Create:

- `project/code/control_chassis.h`: chassis command/output API.
- `project/code/control_chassis.c`: command storage, clamping, and left/right RPM mixing.
- `project/code/control_balance.h`: balance mode and diagnostic API.
- `project/code/control_balance.c`: safety gates, pitch derivative, low-power PD output, actuator command handoff.

Modify:

- `project/code/app_types.h`: shared chassis and balance types.
- `project/code/app_config.h`: periods, limits, and first-version balance gains.
- `project/code/app.c`: initialize chassis and balance modules.
- `project/code/app_scheduler.c`: schedule chassis and balance updates after IMU/safety and before motor actuator update.
- `project/code/host_command.c`: parse `B` and `C` debug commands.
- `project/code/telemetry.c`: expose balance tuning telemetry when switching from motor-loop tuning to balance tuning.

Do not modify:

- `project/code/bldc_foc_uart.c` for balance behavior.
- `libraries/*` vendor, SDK, or driver code.

## Verification

Static checks:

- `control_balance` does not call BLDC UART APIs.
- `control_chassis` does not read IMU state and does not call the motor actuator.
- `host_command` parses debug commands but does not own balance state.
- `actuator_motor` remains the only module that sends motor output.

Build checks:

- Build all affected IAR projects: `cyt4bb7_cm_0_plus`, `cyt4bb7_cm_7_0`, and `cyt4bb7_cm_7_1` when possible.

Hardware smoke checks:

- Boot state produces no motor output.
- `B,1` enters standby with no motor output.
- `B,2` with zero gains produces no motor output beyond STOP/zero behavior.
- Tilting the suspended chassis updates pitch and derived pitch rate telemetry.
- With intentionally small nonzero gains, tilting the suspended chassis changes `balance_rpm` in the expected direction.
- `C,50,0` changes chassis left/right base RPM equally.
- `C,0,50` changes chassis left/right base RPM in opposite directions.
- `STOP` immediately stops motor output.
- IMU unhealthy, feedback offline, system fault, or pitch over limit forces STOP.

## Out Of Scope

This first version does not include:

- guaranteed free-standing balance;
- driving while balanced;
- LQR or state-space controller;
- leg height or pitch coordination;
- servo kinematics changes;
- runtime balance gain tuning commands;
- persistent parameter storage;
- current or torque control;
- changes to CYT2BL3 driver firmware.

## Expected First-Version Outcome

The expected outcome is a firmware build that can safely enter a low-power balance test path and expose the key variables needed for tuning.

Success means the operator can verify:

- the software layers are separated cleanly;
- balance mode transitions are explicit;
- wheel and IMU signs are visible;
- low-power balance output is limited and stoppable;
- safety exits command STOP reliably.

It does not mean the robot must already stand or drive without support.

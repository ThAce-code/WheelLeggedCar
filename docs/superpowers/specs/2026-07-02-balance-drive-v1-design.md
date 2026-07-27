# Balance Drive v1 Design

## Goal

Build the first balanced driving layer on top of the now-stable standing controller.

This version should let the robot move slowly while staying balanced, using only the existing serial debug command path:

```text
C,forward,turn
```

The target outcome is safe low-speed motion, not accurate velocity control or heading hold.

Expected operator checks:

- `C,20,0`: low-speed forward motion without falling.
- `C,-20,0`: low-speed reverse motion without falling.
- `C,0,15`: low-speed in-place left turn without falling.
- `C,0,-15`: low-speed in-place right turn without falling.
- `C,0,0`: smooth return to a stable standing state.
- `STOP`: immediate motor stop.

## Current Context

`balance-stand-v1` is already complete and recorded at commit `f56185c`.

The stable standing parameters are:

```c
APP_BALANCE_PITCH_KP            (18.0f)
APP_BALANCE_PITCH_RATE_KD       (8.0f)
APP_BALANCE_WHEEL_SPEED_KS      (3.0f)
APP_BALANCE_WHEEL_POS_KP        (0.0f)
APP_BALANCE_PITCH_SETPOINT_DEG  (5.0f)
APP_BALANCE_RPM_LIMIT           (300.0f)
```

The existing balance output law is:

```text
balance_rpm =
    K_angle * (pitch_deg - pitch_setpoint_deg)
  + K_rate  * pitch_rate_dps
  + K_speed * avg_wheel_speed_rpm
  + K_pos   * wheel_pos_rev
  + ident_rpm

output_left_rpm  = chassis_left_base_rpm  + balance_rpm
output_right_rpm = chassis_right_base_rpm + balance_rpm
```

`control_chassis` currently stores `forward_rpm` and `turn_rpm`, clamps them, and mixes them directly:

```text
left_base_rpm  = forward_rpm - turn_rpm
right_base_rpm = forward_rpm + turn_rpm
```

That direct step input is acceptable for static tests, but it is too abrupt for balanced driving.

## Selected Approach

Use `control_chassis` as the command-shaping layer.

`C,forward,turn` should set target forward and turn RPM values. `control_chassis_update()` should move internal actual forward and turn values toward those targets using independent ramp limits. The mixed left/right base RPM should be computed from the ramped values, not the raw command target.

This keeps the existing balance controller unchanged:

- `control_chassis` owns motion intent, command timeout, motion limits, and ramping.
- `control_balance` owns balance safety gates and the final motor RPM handoff.
- `actuator_motor` remains the only module that sends motor output.

This version deliberately does not add outer-loop speed PI, yaw-rate control, heading hold, remote-control input, or persistent parameter storage.

## Architecture

Runtime flow remains:

```text
host_command
    -> control_chassis_set_cmd(target_forward, target_turn, enable, now_ms)

scheduler
    -> control_chassis_update(now_ms)
    -> control_balance_update(now_ms)
    -> actuator_motor_update(now_ms)
```

`control_chassis_update()` becomes:

```text
if disabled:
    target_forward = 0
    target_turn = 0
    actual_forward ramps toward 0
    actual_turn ramps toward 0

if command timeout:
    target_forward = 0
    target_turn = 0

actual_forward = ramp(actual_forward, target_forward, forward_ramp_rpm_s, dt_s)
actual_turn    = ramp(actual_turn, target_turn,    turn_ramp_rpm_s,    dt_s)

left_base_rpm  = actual_forward - actual_turn
right_base_rpm = actual_forward + actual_turn
```

`STOP` and `B,0` remain immediate hard stops and must call `control_chassis_stop()`, which clears target and actual chassis output.

## Commands

The existing command shape remains:

```text
C,forward,turn
```

The meaning changes from direct output to target command:

- `forward`: target forward base RPM.
- `turn`: target differential turn RPM.
- `C,0,0`: smooth ramp-to-zero stop.

Current turn sign is preserved:

```text
C,0,positive  -> left turn / counterclockwise in-place turn
C,0,negative  -> right turn / clockwise in-place turn
```

With the current mixer:

```text
C,0,15
left_base_rpm  = -15 RPM
right_base_rpm = +15 RPM
```

after ramping.

## Configuration

Add conservative drive-specific configuration in `project/code/app_config.h`:

```c
#define APP_CHASSIS_DRIVE_RPM_LIMIT      (30.0f)
#define APP_CHASSIS_TURN_RPM_LIMIT       (20.0f)
#define APP_CHASSIS_FORWARD_RAMP_RPM_S   (30.0f)
#define APP_CHASSIS_TURN_RAMP_RPM_S      (40.0f)
#define APP_CHASSIS_CMD_TIMEOUT_MS       (1000U)
```

`APP_CHASSIS_RPM_LIMIT` remains the absolute mixed-output clamp. The new drive and turn limits are operator-command limits for this first balanced driving stage.

The initial limits are intentionally low. Raising them belongs after hardware evidence shows stable acceleration, deceleration, and turning.

## Data Model

Extend `chassis_cmd_struct` to distinguish raw command target from ramped output state:

```c
typedef struct
{
    float target_forward_rpm;
    float target_turn_rpm;
    float actual_forward_rpm;
    float actual_turn_rpm;
    uint8 enable;
    uint32 last_cmd_ms;
    uint32 last_update_ms;
}chassis_cmd_struct;
```

Keep `chassis_output_struct` as the consumer-facing base RPM output:

```c
typedef struct
{
    float left_base_rpm;
    float right_base_rpm;
    uint8 enable;
}chassis_output_struct;
```

Telemetry already exposes `chassis_left_rpm` and `chassis_right_rpm`. In this version those fields should represent the ramped mixed base RPM values.

## Safety Behavior

The following behaviors must not regress:

- `STOP` immediately clears chassis target and actual output, clears balance identification excitation, and stops motors.
- `B,0` clears chassis target and actual output, clears balance identification excitation, and disables balance output.
- If balance safety blocks output, motor output is stopped by `control_balance`.
- Encoder offline, IMU unhealthy, stale IMU, app fault, or pitch over limit still force STOP behavior through the existing balance gates.
- `C` commands do not directly command motors. They only update chassis targets.

Add command timeout behavior:

- If no `C` command has been received for `APP_CHASSIS_CMD_TIMEOUT_MS`, target forward and target turn are forced to zero.
- Timeout should ramp actual values back to zero instead of hard-stopping motors.
- `STOP` remains the hard-stop path.

## Verification

Static checks:

- `control_chassis` still has no dependency on IMU, BLDC UART, or motor actuator APIs.
- `control_balance` still has no dependency on BLDC UART or host-command parsing.
- `actuator_motor` remains the only module that sends motor output.
- `git diff --check` has no whitespace errors.

Build checks:

- Build `cyt4bb7_cm_7_0` in IAR.
- If the workspace configuration requires it, also build `cyt4bb7_cm_0_plus` and `cyt4bb7_cm_7_1`.

Hardware checks:

1. Boot and run `IMU_ZERO`.
2. Enter standing mode with the frozen v1 parameters.
3. Confirm standing still works before motion tests.
4. Run low-speed forward:

   ```text
   C,20,0
   C,0,0
   ```

5. Run low-speed reverse:

   ```text
   C,-20,0
   C,0,0
   ```

6. Run in-place left turn:

   ```text
   C,0,15
   C,0,0
   ```

7. Run in-place right turn:

   ```text
   C,0,-15
   C,0,0
   ```

8. Confirm `STOP` immediately stops the robot.
9. Confirm `C` command timeout ramps output back toward zero.
10. Confirm pitch-limit and single-encoder fault safety behavior still stops both wheels.

Pass criteria:

- The robot does not fall during low-speed forward, reverse, left turn, right turn, or return-to-zero.
- `C,0,0` returns to stable standing without large overshoot.
- `STOP` remains immediate.
- No safety regression from `balance-stand-v1`.

## Out Of Scope

This version does not include:

- speed PI control;
- yaw-rate control;
- heading hold;
- line/path following;
- joystick, PWM, or wireless remote input;
- position hold;
- automatic startup from lying-down or arbitrary angle;
- leg motion coordination;
- persistent saving of runtime parameters.

## Expected Outcome

The expected outcome is `Balance Drive v1`: a robot that can stand and move slowly under serial commands without falling.

This is the bridge between `balance-stand-v1` and later closed-loop mobile-base work. If this version is stable, the next stage can add speed and yaw control on top of a proven low-speed movement path.

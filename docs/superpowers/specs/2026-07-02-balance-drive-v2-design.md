# Balance Drive v2 Design

## Goal

Upgrade balanced driving from the V1 low-speed command shaper to a usable closed-loop mobile base.

`C,forward,turn` should become:

```text
forward = target average wheel speed in RPM
turn    = target yaw rate in deg/s
```

Expected operator behavior:

- `C,30,0`: drive forward with a 30 RPM average wheel-speed target.
- `C,-30,0`: drive backward.
- `C,0,10`: turn in place with a small target yaw rate.
- `C,30,10`: drive while turning.
- `C,0,0`: return to stable standing without hard-stopping.
- `STOP`: immediately clear drive state and stop motor output.

This version should preserve `balance-stand-v1` stability while adding the first closed-loop driving layer.

## Current Evidence

`balance-stand-v1` is stable with:

```c
APP_BALANCE_PITCH_KP            (18.0f)
APP_BALANCE_PITCH_RATE_KD       (8.0f)
APP_BALANCE_WHEEL_SPEED_KS      (3.0f)
APP_BALANCE_WHEEL_POS_KP        (0.0f)
APP_BALANCE_PITCH_SETPOINT_DEG  (5.0f)
APP_BALANCE_RPM_LIMIT           (300.0f)
```

V1 added chassis command ramping, but hardware tests showed direct forward base RPM is cancelled by the standing controller:

```text
C,40,0 -> chassis base is clamped to +30 RPM
balance_rpm averages about -30 RPM
final motor target is near 0 RPM
robot barely moves
```

Manual pitch-setpoint tests confirmed the correct forward/backward direction:

```text
BS increase -> forward
BS decrease -> backward
```

But open-loop pitch setpoint is too slow and not speed-controlled:

```text
BS,8.0 -> about 10 RPM average wheel speed
BS,7.0 -> about 12 RPM average wheel speed in one capture
BS,3.5 -> about -5.5 RPM average wheel speed
```

The conclusion is that forward motion must be driven through a speed-to-pitch loop, not by adding equal base RPM to both wheels.

## Reference Pattern

The P5 note in `D:\KnowledgeBase\03 Knowledge\技术\嵌入式\轮足机器人\轮足机器人_P5_转弯控制与闭环走直线.md` uses two outer loops:

```text
targetSpeed - speedAvg
    -> velocity controller
    -> targetAngle
    -> balance controller drives the wheels

turnTarget - GyroZ
    -> turn controller
    -> left/right differential output
```

That pattern matches this project, but the inner balance law remains the existing four-state feedback:

```text
balance_rpm =
    K_angle * (pitch - pitch_setpoint)
  + K_rate  * pitch_rate
  + K_speed * wheel_speed
  + K_pos   * wheel_pos
```

V2 should add targets around the existing state feedback. It should not replace the balance controller with a separate PID-only architecture.

## Selected Approach

Use `control_chassis` for operator motion targets and outer-loop drive control.

Use `control_balance` for the inner balance output and final motor handoff.

The data flow becomes:

```text
host_command
    -> C,forward,turn
    -> control_chassis target forward RPM and target yaw-rate dps

control_chassis_update()
    -> ramp target forward and target yaw-rate
    -> compute speed-loop pitch offset
    -> compute turn-loop differential RPM
    -> publish drive request

control_balance_update()
    -> read drive request
    -> pitch_setpoint = base_pitch_setpoint + speed_pitch_offset
    -> balance_rpm using four-state feedback
    -> output_left  = balance_rpm - turn_rpm
    -> output_right = balance_rpm + turn_rpm
    -> actuator_motor_set_mode_motor_rpm()
```

This keeps the inner balance loop stable and lets driving commands bias its target state instead of fighting it.

## Forward Speed Loop

The forward loop should compare the ramped forward target with actual average wheel speed:

```text
speed_error_rpm = actual_forward_target_rpm - avg_wheel_speed_rpm
speed_integral += speed_error_rpm * dt_s
speed_pitch_offset_deg =
    speed_kp * speed_error_rpm
  + speed_ki * speed_integral
```

Then clamp:

```text
speed_pitch_offset_deg = clamp(speed_pitch_offset_deg,
                               -APP_CHASSIS_SPEED_PITCH_LIMIT_DEG,
                               +APP_CHASSIS_SPEED_PITCH_LIMIT_DEG)
```

The effective pitch setpoint becomes:

```text
pitch_setpoint_deg =
    control_balance_pitch_setpoint_deg
  + speed_pitch_offset_deg
```

In current hardware direction:

```text
positive speed_pitch_offset -> forward
negative speed_pitch_offset -> backward
```

The first implementation should start with P-only speed control:

```c
APP_CHASSIS_SPEED_KP              (0.03f)
APP_CHASSIS_SPEED_KI              (0.0f)
APP_CHASSIS_SPEED_PITCH_LIMIT_DEG (3.0f)
```

This maps roughly:

```text
30 RPM error -> 0.9 deg pitch offset
60 RPM error -> 1.8 deg pitch offset
100 RPM error -> clamped to 3.0 deg
```

Integrator support should exist but default to zero. Integral tuning belongs after the P-only direction and stability are confirmed.

## Turn Yaw-Rate Loop

The turn loop should compare the ramped turn target with IMU yaw-rate:

```text
turn_error_dps = actual_turn_target_dps - gyro_z_dps
turn_rpm = turn_kp * turn_error_dps
```

Then clamp:

```text
turn_rpm = clamp(turn_rpm,
                 -APP_CHASSIS_TURN_RPM_LIMIT,
                 +APP_CHASSIS_TURN_RPM_LIMIT)
```

Final output:

```text
output_left_rpm  = balance_rpm - turn_rpm
output_right_rpm = balance_rpm + turn_rpm
```

The default turn gain should be conservative:

```c
APP_CHASSIS_TURN_KP (0.0f)
```

This means the code path exists, but the turn loop does not affect output until runtime tuning sets a nonzero gain. This avoids a first-boot failure if `gyro_z_dps` sign or yaw-rate scale is not yet validated.

## Command Semantics

Keep the existing command:

```text
C,forward,turn
```

Change its semantics:

- `forward`: target average wheel speed in RPM.
- `turn`: target yaw-rate in deg/s.
- `C,0,0`: ramp target speed and target yaw-rate back to zero.

Keep existing balance commands:

```text
B,0 / B,1 / B,2
BL,angle,rate,speed,pos
BP,kp,kd
BS,pitch_setpoint
BZ
BI,amp,period_ms
STOP
```

Add a drive tuning command:

```text
BD,speed_kp,speed_ki,turn_kp
```

Behavior:

- Validate all three values are finite.
- Clamp or reject values beyond safe limits.
- Reset speed-loop integral when accepted.
- Record command success/failure through the existing command-error path.

The drive tuning command is intentionally volatile. It does not persist values across reset.

## Configuration

Add or update drive-specific constants in `project/code/app_config.h`:

```c
#define APP_CHASSIS_FORWARD_RPM_LIMIT         (60.0f)
#define APP_CHASSIS_TURN_RATE_LIMIT_DPS       (60.0f)
#define APP_CHASSIS_FORWARD_RAMP_RPM_S        (60.0f)
#define APP_CHASSIS_TURN_RATE_RAMP_DPS_S      (120.0f)

#define APP_CHASSIS_SPEED_KP                  (0.03f)
#define APP_CHASSIS_SPEED_KI                  (0.0f)
#define APP_CHASSIS_SPEED_INTEGRAL_LIMIT      (100.0f)
#define APP_CHASSIS_SPEED_PITCH_LIMIT_DEG     (3.0f)

#define APP_CHASSIS_TURN_KP                   (0.0f)
#define APP_CHASSIS_TURN_RPM_LIMIT            (60.0f)
#define APP_CHASSIS_DRIVE_GAIN_ABS_LIMIT      (100.0f)
```

`APP_CHASSIS_RPM_LIMIT` remains as an absolute base/differential protection limit if still needed, but forward motion should no longer use equal left/right base RPM.

## Data Model

Extend `chassis_cmd_struct` so it clearly separates targets, ramped targets, and control outputs:

```c
typedef struct
{
    float target_forward_rpm;
    float target_turn_dps;
    float actual_forward_rpm;
    float actual_turn_dps;
    float speed_pitch_offset_deg;
    float turn_rpm;
    float speed_kp;
    float speed_ki;
    float turn_kp;
    float speed_integral;
    uint8 enable;
    uint32 last_cmd_ms;
    uint32 last_update_ms;
}chassis_cmd_struct;
```

Replace or reinterpret `chassis_output_struct`:

```c
typedef struct
{
    float pitch_offset_deg;
    float turn_rpm;
    float forward_target_rpm;
    float forward_actual_rpm;
    float turn_target_dps;
    float gyro_z_dps;
    uint8 enable;
}chassis_output_struct;
```

The names should make the new architecture explicit:

- `pitch_offset_deg`: forward speed-loop output.
- `turn_rpm`: yaw-rate loop output.
- `forward_target_rpm`: ramped speed target.
- `forward_actual_rpm`: average wheel speed.
- `turn_target_dps`: ramped yaw-rate target.
- `gyro_z_dps`: measured yaw-rate.

## Balance Integration

`control_balance_update()` should use:

```text
pitch_setpoint =
    control_balance_pitch_setpoint_deg
  + chassis->pitch_offset_deg
```

Then:

```text
balance_rpm =
    K_angle * (pitch - pitch_setpoint)
  + K_rate  * pitch_rate
  + K_speed * wheel_speed
  + K_pos   * wheel_pos
  + ident_rpm
```

Final motor command:

```text
output_left_rpm  = balance_rpm - chassis->turn_rpm
output_right_rpm = balance_rpm + chassis->turn_rpm
```

This removes V1's forward base RPM cancellation problem.

## Telemetry

The current 16-channel balance telemetry is not enough for V2 diagnosis.

Expand balance telemetry to include at least:

```text
time_ms
mode
roll_deg
pitch_deg
yaw_deg
pitch_rate_dps
balance_rpm
feedback_online
left_motor_rpm
right_motor_rpm
left_duty
right_duty
balance_kp
balance_kd
forward_target_rpm
forward_actual_rpm
speed_pitch_offset_deg
pitch_setpoint_deg
turn_target_dps
gyro_z_dps
turn_rpm
```

The collection script must be updated to parse the new frame length and headers.

The MATLAB/Powershell analysis can be updated later, but the collection script must preserve the new columns immediately so hardware tests are inspectable.

## Safety Behavior

Required safety behavior:

- `STOP` clears chassis targets, speed integral, pitch offset, turn output, balance identification excitation, and motor output.
- `B,0` clears chassis targets and balance output.
- `C,0,0` ramps motion targets to zero but does not hard-stop the robot.
- Encoder offline, IMU unhealthy, stale IMU, app fault, pitch over limit, or non-finite output still stop both wheels.
- Non-finite `C` or `BD` input is rejected or forces safe zero state.
- Speed-loop pitch offset is clamped.
- Turn output is clamped.

Do not reintroduce `C` command timeout. The user explicitly rejected this behavior because it breaks the existing "send `C,0,0` once and stand" workflow.

## Verification

Static checks:

- `control_chassis` still has no dependency on BLDC UART or host-command parsing.
- `control_chassis` may read IMU and wheel feedback if it owns outer loops, but it must not call motor actuator output APIs.
- `control_balance` remains the only balance motor-output owner.
- `actuator_motor` remains the only module that sends BLDC output.
- `C` timeout is absent.
- `BD` parser exists and validates finite gains.
- Telemetry and collection script agree on frame length and column order.

Build checks:

- Build `cyt4bb7_cm_7_0` in IAR.
- If practical, also build the other core projects.
- Do not claim build success without IAR output.

Hardware checks:

1. Standing regression:

   ```text
   STOP -> B,1 -> B,2 -> C,0,0
   ```

2. Forward P-only:

   ```text
   BD,0.03,0,0
   C,20,0
   C,0,0
   ```

3. Reverse P-only:

   ```text
   BD,0.03,0,0
   C,-20,0
   C,0,0
   ```

4. Direction test:

   - Positive `C,forward,0` should move in the same direction as `BS` increase.
   - Negative `C,forward,0` should move in the same direction as `BS` decrease.

5. Turn loop disabled:

   ```text
   BD,0.03,0,0
   C,20,0
   ```

   The robot should not introduce unexpected yaw correction from the new turn loop.

6. Turn loop sign test:

   ```text
   BD,0.03,0,small_turn_kp
   C,0,10
   C,0,-10
   ```

   If direction is wrong or yaw correction grows unstable, stop and flip the sign in one place only after data review.

7. Straight-line correction:

   ```text
   BD,0.03,0,small_turn_kp
   C,20,0
   ```

   Check whether `gyro_z_dps` is reduced compared with turn loop disabled.

## Out Of Scope

This V2 does not include:

- position hold;
- path following;
- yaw angle hold;
- joystick/PWM/wireless input;
- persistent parameter save;
- automatic get-up;
- leg coordination;
- MATLAB redesign of the balance gains;
- current/torque control of the external BLDC driver.

## Expected Outcome

Balance Drive v2 should make the robot respond to forward speed targets through pitch-setpoint control and support a conservative yaw-rate turn loop.

Success does not mean high-speed driving. Success means the robot can:

- stand as before;
- move forward and backward more meaningfully than V1's cancelled base-RPM command;
- return to standing with `C,0,0`;
- expose enough telemetry to tune speed and turn loops;
- begin turn-rate tuning without destabilizing the standing controller.

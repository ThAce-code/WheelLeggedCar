# Balance Fast Blend Design

## Goal

Add a high-speed balanced-drive path that can reach and validate 100+ RPM without breaking the current low-speed standing and slow-drive behavior.

The operator-facing command remains:

```text
C,forward_rpm,turn_dps
```

The mode behavior becomes:

```text
B,2  -> current low-speed balance drive; high-speed blend forced to 0
B,3  -> fast balance drive armed; high-speed blend rises smoothly with ramped forward target
STOP -> hard stop; clear fast blend, drive integrals, and motor output
```

`B,3` is not a hard control-law jump. It only permits a continuous internal blend. The actual transition is driven by the already-ramped forward target, so a step command such as `C,100,0` does not immediately switch all gains.

## Problem Statement

The current balance law is:

```c
balance_rpm = (pitch_kp * (imu->pitch - pitch_setpoint_deg)) +
              (pitch_rate_kd * pitch_rate_dps) +
              (wheel_speed_ks * wheel_speed_rpm) +
              (wheel_pos_kp * wheel_pos_rev);
```

With the current defaults:

```c
APP_BALANCE_PITCH_KP            (18.0f)
APP_BALANCE_WHEEL_SPEED_KS      (3.0f)
APP_CHASSIS_SPEED_PITCH_LIMIT_DEG (8.0f)
APP_CHASSIS_FORWARD_RPM_LIMIT   (60.0f)
```

The theoretical speed ceiling from pitch offset and wheel-speed damping is approximately:

```text
18 / 3.0 * 8 deg = 48 RPM
```

This is consistent with the observed low forward speed. The controller is not simply under-tuned; its low-speed damping architecture deliberately resists continuous high wheel speed.

## Design Principles

- Preserve the tuned low-speed mode. `B,2` must behave like today.
- Do not remove `wheel_speed_ks` globally. It is useful low-speed damping.
- Do not hard-switch gains. Use a bounded blend with a ramp limit.
- Base blend on ramped forward target, not raw operator command and not noisy measured wheel speed.
- Keep speed feedforward infrastructure signed and tunable, but default it to `0.0f` until hardware data confirms the correct sign and magnitude.
- Keep `wheel_pos_kp` at `0.0f` for racing mode. Position hold fights continuous driving.
- Expose term-level telemetry before serious high-speed tuning.

## Fast Blend

Add these helper functions in `project/code/control_chassis.c`:

```c
static float control_chassis_clamp01(float value)
{
    if(0.0f > value)
    {
        return 0.0f;
    }
    if(1.0f < value)
    {
        return 1.0f;
    }
    return value;
}

static float control_chassis_lerp(float low, float high, float blend)
{
    blend = control_chassis_clamp01(blend);
    return low + ((high - low) * blend);
}

static float control_chassis_smoothstep(float edge0, float edge1, float value)
{
    float t;

    if(edge1 <= edge0)
    {
        return 0.0f;
    }

    t = control_chassis_clamp01((value - edge0) / (edge1 - edge0));
    return t * t * (3.0f - (2.0f * t));
}
```

Add a ramped `fast_blend` state in `chassis_cmd_struct`:

```c
raw_blend = control_chassis_smoothstep(APP_CHASSIS_FAST_BLEND_START_RPM,
                                       APP_CHASSIS_FAST_BLEND_FULL_RPM,
                                       control_chassis_absf(control_chassis_cmd.actual_forward_rpm));

if(APP_FALSE == control_chassis_cmd.fast_enable)
{
    raw_blend = 0.0f;
}

control_chassis_cmd.fast_blend =
    control_chassis_ramp_toward(control_chassis_cmd.fast_blend,
                                raw_blend,
                                APP_CHASSIS_FAST_BLEND_RAMP_S * dt_s);
```

Recommended initial constants:

```c
#define APP_CHASSIS_FAST_FORWARD_RPM_LIMIT       (180.0f)
#define APP_CHASSIS_FAST_BLEND_START_RPM         (40.0f)
#define APP_CHASSIS_FAST_BLEND_FULL_RPM          (90.0f)
#define APP_CHASSIS_FAST_BLEND_RAMP_S            (1.0f)
#define APP_CHASSIS_FAST_SPEED_PITCH_LIMIT_DEG   (12.0f)
#define APP_BALANCE_FAST_WHEEL_SPEED_KS          (0.8f)
#define APP_BALANCE_FAST_SPEED_FF_GAIN           (0.0f)
```

The `0.0f` feedforward default is intentional. The current motor sign conventions must be validated with hardware before enabling nonzero forward feedforward.

## Chassis Behavior

When fast mode is disabled:

- forward target limit remains `APP_CHASSIS_FORWARD_RPM_LIMIT`;
- pitch offset limit remains `APP_CHASSIS_SPEED_PITCH_LIMIT_DEG`;
- `fast_blend` ramps back to zero;
- `speed_ff_rpm` is zero.

When fast mode is enabled:

- forward target limit becomes `APP_CHASSIS_FAST_FORWARD_RPM_LIMIT`;
- pitch offset limit is interpolated:

```c
speed_pitch_limit_deg =
    control_chassis_lerp(APP_CHASSIS_SPEED_PITCH_LIMIT_DEG,
                         APP_CHASSIS_FAST_SPEED_PITCH_LIMIT_DEG,
                         control_chassis_cmd.fast_blend);
```

- speed pitch offset uses the interpolated limit;
- speed feedforward request is computed from ramped target and blend:

```c
speed_ff_rpm = APP_BALANCE_FAST_SPEED_FF_GAIN *
               control_chassis_cmd.actual_forward_rpm *
               control_chassis_cmd.fast_blend;
```

`BD,speed_kp,speed_ki,turn_kp` remains the runtime way to tune the speed PI loop. Do not silently force a nonzero `speed_ki` just because fast mode is active.

## Balance Behavior

Extend `control_balance.c` to compute terms explicitly:

```c
effective_wheel_speed_ks =
    control_balance_lerp(control_balance_wheel_speed_ks,
                         APP_BALANCE_FAST_WHEEL_SPEED_KS,
                         chassis->fast_blend);

pitch_term_rpm = control_balance_pitch_kp * (imu->pitch - pitch_setpoint_deg);
rate_term_rpm = control_balance_pitch_rate_kd * pitch_rate_dps;
speed_term_rpm = effective_wheel_speed_ks * wheel_speed_rpm;
pos_term_rpm = control_balance_wheel_pos_kp * control_balance_wheel_pos_rev;
ff_term_rpm = chassis->speed_ff_rpm;

balance_rpm = pitch_term_rpm +
              rate_term_rpm +
              speed_term_rpm +
              pos_term_rpm +
              ff_term_rpm;
```

The effective `wheel_speed_ks` must be diagnostic-only state. Do not overwrite the operator-tuned low-speed `control_balance_wheel_speed_ks`, because `B,2` must return to exactly the low-speed value.

## Runtime Commands

Add `B,3`:

```text
B,3 -> control_chassis_set_fast_enable(APP_TRUE);
       control_balance_set_mode(BALANCE_MODE_BALANCE_FAST);
```

Update existing balance mode commands:

```text
B,0 / STOP -> control_chassis_stop(...); fast disabled
B,1        -> standby; fast disabled
B,2        -> balance test/drive; fast disabled
B,3        -> balance fast drive; fast enabled
```

Optional tuning command for a later commit, only if static checks and hardware tuning need it:

```text
BF,fast_forward_limit,fast_pitch_limit,fast_ks,fast_ff_gain
```

If `BF` is implemented, all four values must be finite, bounded, and reflected in telemetry. Do not implement persistent storage.

## Telemetry

Expand the balance telemetry frame by appending fields after the existing 28 floats so old columns remain stable:

```text
28 fast_blend
29 speed_integral
30 speed_pitch_limit_deg
31 speed_ff_rpm
32 wheel_speed_ks
33 pitch_term_rpm
34 rate_term_rpm
35 speed_term_rpm
36 pos_term_rpm
37 ff_term_rpm
```

The resulting balance frame has 38 floats.

Update `tools/collect_balance_data.ps1`, `tools/test_collect_balance_data.ps1`, and `tools/tune_drive_loops.m` so captures and analysis can see the blended controller state.

## Safety Rules

- Stale IMU or stale wheel feedback clears speed integral, turn integral, fast blend, and feedforward request.
- `STOP` and `B,0` clear fast blend immediately.
- `C,0,0` ramps the target to zero and then clears speed integral under the existing zero-target rule.
- Do not raise `APP_BALANCE_RPM_LIMIT` in this change.
- Do not enable nonzero feedforward by default.
- Do not change BLDC UART protocol, motor PID code, or vendor libraries.

## Hardware Validation Sequence

Do not test 100 RPM first. Use staged captures without `IMU_ZERO`:

```powershell
powershell -ExecutionPolicy Bypass -File tools/collect_balance_data.ps1 -Port COM6 -Duration 12 -Commands "0:STOP;0.3:BD,0.3,0,1.5;0.6:B,3;1.0:C,0,0;2.0:C,40,0;7.0:C,0,0;11:STOP" -Note "fast_b3_40_ff0"
```

Then, if pitch, balance output, feedback age, and motor feedback remain clean:

```powershell
powershell -ExecutionPolicy Bypass -File tools/collect_balance_data.ps1 -Port COM6 -Duration 12 -Commands "0:STOP;0.3:BD,0.3,0,1.5;0.6:B,3;1.0:C,0,0;2.0:C,70,0;7.0:C,0,0;11:STOP" -Note "fast_b3_70_ff0"
```

Then:

```powershell
powershell -ExecutionPolicy Bypass -File tools/collect_balance_data.ps1 -Port COM6 -Duration 12 -Commands "0:STOP;0.3:BD,0.3,0,1.5;0.6:B,3;1.0:C,0,0;2.0:C,100,0;7.0:C,0,0;11:STOP" -Note "fast_b3_100_ff0"
```

Only after the sign and magnitude are understood should nonzero speed feedforward be enabled.

## Acceptance Criteria

- Static checks pass.
- `B,2` telemetry shows `fast_blend = 0`, original pitch limit, original low-speed `wheel_speed_ks`.
- `B,3` with `C,0,0` remains still and does not inject feedforward.
- `B,3` with ramped targets shows smooth `fast_blend` from 0 to 1 across the configured range.
- No stale sensor path leaves stale `fast_blend` or nonzero feedforward active.
- 40/70/100 RPM captures include all new term-level telemetry.
- IAR builds for affected cores are run before merge, or the reason they were not run is explicitly reported.

## Out of Scope

- 300 RPM racing limit.
- Leg X gravity assist.
- Position hold.
- Automatic gain identification.
- Persistent flash-stored controller profiles.
- Rewriting the motor speed inner loop.

# IK Height Control Design

## Goal

Implement direct inverse-kinematics height control for the wheel-legged car so the robot can stand and drive stably at any commanded body height inside a calibrated safe workspace.

The first implementation uses the existing command meanings:

```text
C,forward_rpm,turn_dps
B,2 = low-speed balance drive
B,3 = fast balance drive armed
```

Height control adds a leg command path. It does not reinterpret `C`.

## Current Project Context

The active worktree is:

```text
D:\smartcar\WheelLeggedCar_cyt4bb7_v1\.worktree\leg-control-speed-assist
```

The active branch is:

```text
codex/leg-control-speed-assist
```

The current control split is:

```text
host_command
    -> control_chassis: forward target, yaw-rate target, speed pitch offset, turn RPM, fast blend
    -> control_balance: inner pitch/wheel feedback balance and final motor RPM command
    -> control_leg: current height/pitch/roll-to-servo angle mixer
    -> actuator_motor / actuator_servo
```

Fast balance mode is already present:

```text
B,3
BALANCE_MODE_BALANCE_FAST
fast_blend
38-float balance telemetry
```

The existing leg controller is not yet true height control. It mixes `height + pitch * mount_x + roll * mount_y` directly into servo angles. This must be replaced by a foot-coordinate interface:

```text
target body height -> foot coordinate (x, y) -> inverse kinematics -> servo angles
```

## Notes Used

The design follows these local notes:

- `D:\KnowledgeBase\03 Knowledge\技术\嵌入式\轮足机器人\轮足机器人_P3_变腿高与俯仰自稳定.md`
- `D:\KnowledgeBase\03 Knowledge\技术\嵌入式\轮足机器人\轮足机器人_P6_运动学逆解.md`
- `D:\KnowledgeBase\03 Knowledge\技术\嵌入式\轮足机器人\轮足机器人_P7_滚转姿态控制.md`
- `D:\KnowledgeBase\03 Knowledge\技术\嵌入式\轮足机器人\轮足机器人_P8_复杂地形自适应.md`
- `D:\KnowledgeBase\03 Knowledge\技术\嵌入式\轮足机器人\轮足机器人_P9_仿生腿实现高机动性.md`

Key takeaways:

- P3: body height changes the center-of-mass to wheel-center lever arm, so balance gains must vary with height.
- P6: height control should be foot-coordinate based and smooth, not fixed servo-angle presets.
- P7/P8: roll/terrain adaptation comes from left/right height difference, but that is a later layer.
- P9: fore-aft `x` assist is useful for acceleration and braking, but it should wait until pure height control is stable.

## Selected Approach

Use direct IK in the first implementation, but keep the first controlled motion simple:

```text
x_left_mm  = 0
x_right_mm = 0
y_left_mm  = actual_height_mm
y_right_mm = actual_height_mm
```

This gives continuous height control and height-aware balance tuning without adding terrain roll or speed-assist `x` movement in the same pass.

The first version intentionally does not implement:

- roll terrain adaptation;
- left/right height difference;
- P9 fore-aft `x` speed assist;
- jump preload/launch/landing;
- dynamic 450 RPM racing envelope;
- automatic fitting of all link lengths at runtime.

Those features depend on stable IK, reliable height state, and height-scheduled balance behavior.

## Architecture

Add a focused kinematics module:

```text
project/code/leg_kinematics.h
project/code/leg_kinematics.c
```

Responsibilities:

- solve one side of the five-bar leg from target foot coordinate `(x_mm, y_mm)` to two servo angles;
- reject unreachable points;
- reject non-finite math;
- reject servo angles outside configured limits;
- expose a forward-check function for calibration and offline validation.

Keep responsibilities separate:

```text
leg_config
    static mechanical dimensions, servo calibration, workspace limits, height gain schedule

leg_kinematics
    five-bar IK/FK math only

control_leg
    height command, height ramp, IK invocation, lock/manual/height modes, diagnostics

control_balance
    reads current height and interpolates balance gains/setpoint

control_chassis
    reads current height and applies motion envelope limits

host_command
    parses height and calibration commands only

actuator_servo
    remains the only PWM servo output owner
```

## Kinematics Model

Use the P6 five-bar model.

For one leg:

```text
O = left base joint origin
D = right base joint origin = (L5, 0)
B = target wheel-center / foot point = (x, y)
L1 = O-to-A driven link
L2 = A-to-B passive link
L4 = D-to-C driven link
L3 = C-to-B passive link
L5 = O-to-D base spacing
```

The left driven angle solution uses:

```text
a = 2 * x * L1
b = 2 * y * L1
c = x*x + y*y + L1*L1 - L2*L2
```

The right driven angle solution uses:

```text
d = 2 * (x - L5) * L4
e = 2 * y * L4
f = (x - L5)^2 + y*y + L4*L4 - L3*L3
```

Each side computes the two mathematical candidates and chooses the configured branch:

```text
LEG_IK_BRANCH_PLUS
LEG_IK_BRANCH_MINUS
```

The branch is not guessed dynamically during normal control. It is configured after calibration and validated by sweep tests. Dynamic branch switching can create sudden servo jumps.

## Calibration Strategy

Do not trust tape-measure values alone, but do not fit all link lengths freely in the first pass.

Calibration uses three layers:

1. Coarse mechanical measurement:
   - measure `L1` through `L5` in mm;
   - enter them as initial constants;
   - allow only small later corrections.

2. Servo and coordinate calibration:
   - determine `servo_zero_deg[4]`;
   - determine `servo_direction[4]`;
   - determine `servo_angle_scale[4]`;
   - determine `x_offset_mm` and `y_offset_mm`;
   - determine IK branch for left and right leg.

3. Residual check:
   - if measured foot coordinates still miss by more than the acceptance threshold, allow `L1` through `L5` to vary only inside a bounded correction window such as plus/minus 5 mm.

This prevents the fitter from hiding servo-zero, origin, or measurement errors inside unrealistic link lengths.

## Calibration Commands

Add a direct servo calibration command:

```text
LIK,a0,a1,a2,a3
```

Behavior:

- accepts four servo angles in degrees;
- validates each angle against configured servo limits;
- sets `LEG_MODE_IK_CALIB`;
- commands the four servos directly;
- does not change balance or chassis state;
- records command success/failure through the existing command error path.

`LIK` is for supported calibration only. The robot must be held or on a stand.

Add a height command:

```text
LH,height_mm
```

Behavior:

- accepts a target body height in mm;
- clamps or rejects outside calibrated safe height range;
- sets `LEG_MODE_HEIGHT`;
- height is ramped inside `control_leg_update()`;
- `x=0` for both legs in the first implementation.

Do not add a new meaning to `C`.

## Data Model

Add shared leg diagnostics in `app_types.h`:

```c
typedef struct
{
    float target_height_mm;
    float actual_height_mm;
    float height_norm;
    float left_x_mm;
    float left_y_mm;
    float right_x_mm;
    float right_y_mm;
    float servo_target_deg[4];
    float servo_actual_deg[4];
    uint8 mode;
    uint8 ik_valid;
    uint8 output_enable;
    uint32 ik_error_count;
}leg_diag_struct;
```

`height_norm` maps the calibrated workspace to `0.0f..1.0f`:

```text
height_norm = (actual_height_mm - height_low_mm) /
              (height_high_mm - height_low_mm)
```

Clamp `height_norm` to `0.0f..1.0f`.

## Configuration

Extend `leg_config` with mechanical and control calibration:

```c
typedef enum
{
    LEG_IK_BRANCH_PLUS = 0,
    LEG_IK_BRANCH_MINUS = 1
}leg_ik_branch_enum;

typedef struct
{
    float l1_mm;
    float l2_mm;
    float l3_mm;
    float l4_mm;
    float l5_mm;
    float x_min_mm;
    float x_max_mm;
    float y_min_mm;
    float y_max_mm;
    float x_offset_mm;
    float y_offset_mm;
    leg_ik_branch_enum left_alpha_branch;
    leg_ik_branch_enum left_beta_branch;
    leg_ik_branch_enum right_alpha_branch;
    leg_ik_branch_enum right_beta_branch;
}leg_kinematics_config_struct;

typedef struct
{
    float low_height_mm;
    float high_height_mm;
    float default_height_mm;
    float max_height_speed_mm_s;
    float balance_pitch_kp_low;
    float balance_pitch_kp_high;
    float balance_pitch_rate_kd_low;
    float balance_pitch_rate_kd_high;
    float balance_wheel_speed_ks_low;
    float balance_wheel_speed_ks_high;
    float balance_pitch_setpoint_low_deg;
    float balance_pitch_setpoint_high_deg;
    float chassis_forward_limit_low_rpm;
    float chassis_forward_limit_high_rpm;
    float chassis_fast_forward_limit_low_rpm;
    float chassis_fast_forward_limit_high_rpm;
}leg_height_profile_struct;
```

Keep initial values conservative. The first implementation should compile and run with placeholder-safe measured constants, but the hardware validation must not claim success until the values are measured.

## Balance Integration

`control_balance` reads:

```c
const leg_diag_struct *leg = control_leg_get_diag();
```

Then computes height-scheduled values:

```text
pitch_kp_eff       = lerp(low, high, height_norm)
pitch_rate_kd_eff  = lerp(low, high, height_norm)
wheel_speed_ks_eff = lerp(low, high, height_norm)
base_setpoint_eff  = lerp(low, high, height_norm)
```

The operator-tuned low-speed values must remain stored separately. Effective values are diagnostics, not destructive overwrites of runtime `BL` settings.

In the first implementation:

- use scheduled values only when `LEG_MODE_HEIGHT` has valid IK;
- if IK is invalid or leg output disabled, fall back to current compiled balance values;
- publish effective values in telemetry.

## Chassis Integration

`control_chassis` reads current height and applies envelope limits:

```text
forward_limit_rpm      = lerp(low_limit, high_limit, height_norm)
fast_forward_limit_rpm = lerp(low_fast_limit, high_fast_limit, height_norm)
```

For safety, the profile should usually allow higher speed at low height and lower speed at high height:

```text
low height  -> larger allowed forward RPM
high height -> smaller allowed forward RPM
height moving -> conservative temporary limit
```

If `actual_height_mm` is still moving toward target, apply a transition limit:

```text
forward_limit_rpm = min(forward_limit_rpm, APP_LEG_HEIGHT_TRANSITION_FORWARD_LIMIT_RPM)
```

This prevents height movement and fast driving from stacking during early tests.

## Scheduler Order

The current scheduler runs leg after balance and telemetry. Height-aware balance needs the current leg state before chassis and balance run.

Change order to:

```text
host_command
imu
safety
leg
chassis
balance
motor
telemetry
servo
```

`control_leg_update()` computes target servo angles and diagnostics. `actuator_servo_update()` still runs later and physically ramps PWM output.

## Telemetry

The existing balance frame has 38 floats. Append leg and height scheduling fields after existing fields so older columns remain stable.

Append:

```text
38 leg_mode
39 leg_target_height_mm
40 leg_actual_height_mm
41 leg_height_norm
42 leg_left_x_mm
43 leg_left_y_mm
44 leg_right_x_mm
45 leg_right_y_mm
46 leg_ik_valid
47 leg_output_enable
48 leg_servo0_target_deg
49 leg_servo1_target_deg
50 leg_servo2_target_deg
51 leg_servo3_target_deg
52 balance_pitch_kp_eff
53 balance_pitch_rate_kd_eff
54 balance_wheel_speed_ks_eff
55 balance_pitch_setpoint_base_eff_deg
56 chassis_forward_limit_eff_rpm
57 chassis_fast_forward_limit_eff_rpm
```

The new balance telemetry frame has 58 floats.

Update:

- `project/code/telemetry.c`
- `tools/collect_balance_data.ps1`
- `tools/test_collect_balance_data.ps1`

## Safety Rules

IK output must fail closed.

Hard safety rules:

- If target `(x, y)` is outside configured workspace, reject it.
- If discriminant is negative, reject it.
- If candidate angle is non-finite, reject it.
- If servo angle is outside per-servo limit, reject it.
- If IK is invalid during `LEG_MODE_HEIGHT`, command safe lock angles or hold previous valid targets according to configuration.
- `STOP` must set leg mode to lock and disable height assist.
- `B,0` must clear balance and may leave legs in current safe height only if IK is valid.
- `LH` commands are rejected while `APP_STATE_FAULT` is active.
- Height transition rate is limited.
- Chassis forward limit is reduced while height is moving.

Do not drive servos near mechanical hard stops. Keep at least 5 to 10 degrees margin in calibrated limits.

## Static Verification

Add a new script:

```text
tools/test_ik_height_control_static.ps1
```

It should assert:

- `leg_kinematics.c/.h` exist;
- IK solve APIs exist;
- `leg_config` contains link lengths, workspace limits, branch config, height profile;
- `control_leg` no longer uses the old direct `height + pitch * mount_x + roll * mount_y` path for height mode;
- `control_leg_get_diag()` exists;
- `host_command` parses `LH` and `LIK`;
- `control_balance` reads `control_leg_get_diag`;
- `control_chassis` reads `control_leg_get_diag`;
- telemetry emits 58 floats;
- collector parses 58 floats;
- scheduler runs `control_leg_update()` before `control_chassis_update()` and `control_balance_update()`.

## Hardware Validation

Validation order:

1. Servo-only `LIK` calibration on a stand:
   - command safe servo angles;
   - record measured wheel-center `(x, y)`;
   - confirm direction and zero offsets.

2. IK offline sweep:
   - command low, mid, high `LH` targets while supported;
   - confirm smooth servo motion;
   - confirm no branch jump;
   - confirm telemetry `ik_valid=1`.

3. Height standing:
   - `LH,low`, `B,2`, `C,0,0`;
   - `LH,mid`, `B,2`, `C,0,0`;
   - `LH,high`, `B,2`, `C,0,0`.

4. Height transition standing:
   - stand at low height;
   - ramp to mid;
   - ramp to high;
   - ramp back to low;
   - robot remains balanced or returns to safe lock.

5. Height driving:
   - repeat low/mid/high with `C,20,0`, then `C,40,0`;
   - do not enter `B,3` fast mode until standing and low-speed tests pass.

6. Fast-mode regression:
   - verify `B,3 C,40/70/100` still works at the validated height profile.

Do not claim arbitrary-height success until low, mid, high, and transition tests all pass.

## Acceptance Criteria

The first IK height-control implementation is acceptable when:

- static checks pass;
- collector tests pass;
- IAR build result is known;
- `LIK` calibration command works on a supported robot;
- `LH` commands produce smooth `actual_height_mm` transitions;
- IK rejects unreachable coordinates and unsafe servo angles;
- low/mid/high `B,2 C,0,0` standing tests pass;
- low/mid/high low-speed driving tests pass;
- telemetry contains height, IK validity, servo targets, and height-scheduled balance/chassis values;
- `C,forward_rpm,turn_dps` semantics remain unchanged.

## Follow-On Work

After this spec is implemented and validated:

- add P7 left/right height difference for commanded roll;
- add P8 terrain `stab_roll` compensation;
- add P9 `x` speed assist;
- raise speed envelope from 220 RPM toward 250, 350, and 450 RPM with height-aware limits;
- design jump preload/launch/landing/recover as a separate state machine.

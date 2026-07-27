# Low-race leg-assist formula reference

## 1. Purpose and status labels

This document collects the calculations involved in the approved automatic
low-race leg-assist design. Every formula is labelled as one of:

- **Existing:** already implemented in the current firmware.
- **Designed:** approved behavior that still requires implementation.
- **Approximation:** useful for explaining or estimating the mechanism, but not
  sufficient for final controller tuning.

The coordinate contract is `BODY_WHEEL` in millimetres: `+X` points forward and
`+Y` points down. A more-negative X moves the wheel centre rearward relative to
the chassis.

## 2. Physical acceleration mechanism

### 2.1 Equivalent lean caused by wheel-centre displacement

**Approximation**

If the wheel centre moves rearward by `delta_x` relative to the chassis, the
chassis centre of mass is effectively farther forward of the wheel axis. The
equivalent lean is approximately:

```text
theta_leg = atan(delta_x / h_eff)
```

For a small angle:

```text
theta_leg ~= delta_x / h_eff
```

where:

- `delta_x` is rearward wheel-centre travel;
- `h_eff` is the effective vertical distance from wheel axis to centre of mass;
- `theta_leg` is the equivalent forward-lean bias.

The approximate gravity-driven acceleration associated with that lean is:

```text
a ~= g * tan(theta_leg)
```

and, for a small angle:

```text
a ~= g * delta_x / h_eff
```

These expressions explain direction only. Servo dynamics, moving linkage
inertia, tyre grip, chassis mass distribution, pitch feedback, and motor torque
prevent them from predicting the final acceleration accurately without data.

## 3. Coupled BODY_WHEEL trajectory

### 3.1 Normalized assist value

**Designed**

```text
-1 <= u <= 1
```

- `u = 0`: normal low-race pose;
- `u > 0`: acceleration assist, wheel centre moves rearward;
- `u < 0`: braking assist, wheel centre moves forward.

### 3.2 Physical target

**Designed**

```text
x_cmd = x0 - dx_level * u
y_cmd = y0 + dy_level * abs(u)
```

The approved zero pose is:

```text
x0 = -18.83 mm
y0 =  25.08 mm
```

Initial level:

```text
dx_level = 2 mm
dy_level = 2 mm
```

Therefore:

| Condition | u | BODY_WHEEL target |
|---|---:|---:|
| Low-race zero | 0 | `(-18.83, 25.08) mm` |
| Rearward acceleration assist | +1 | `(-20.83, 27.08) mm` |
| Forward braking assist | -1 | `(-16.83, 27.08) mm` |

Increasing Y for either direction moves the trajectory away from the narrow
lowest-pose reachability tip. Later hardware-accepted levels may expand to no
more than approximately:

```text
dx_level = 5 mm
dy_level = 5 mm
```

giving design candidates near `(-23.83, 30.08) mm` and
`(-13.83, 30.08) mm`. These expanded endpoints are not yet hardware-approved.

## 4. Speed target and requested acceleration

### 4.1 Target ramp

**Existing**

For each control step:

```text
delta_v_max = forward_ramp_rpm_s * dt
v_ramp(k) = move_toward(v_ramp(k-1), v_target, delta_v_max)
```

The current forward ramp is:

```text
forward_ramp_rpm_s = 60 RPM/s
```

### 4.2 Requested acceleration

**Designed**

```text
a_cmd(k) = (v_ramp(k) - v_ramp(k-1)) / dt
```

- `a_cmd > 0`: request rearward assist;
- `a_cmd < 0`: request forward braking assist;
- `a_cmd = 0`: acceleration feedforward contributes nothing.

## 5. Automatic assist request

### 5.1 Wheel-speed feedback

**Existing**

```text
v_measured = (rpm_left + rpm_right) / 2
e_v = v_ramp - v_measured
```

### 5.2 Raw assist request

**Designed**

```text
u_raw = K_a * a_cmd
      + K_e * e_v
      + u_hold * B_250
```

```text
u_request = clamp(u_raw, -1, 1)
```

where:

- `K_a * a_cmd` reacts immediately to a changing speed target;
- `K_e * e_v` increases assist when measured speed falls behind the ramped
  target and produces braking assist when measured speed exceeds a falling
  target;
- `u_hold * B_250` retains a smaller rearward bias at high cruise speed so
  returning the leg does not create an unintended braking impulse;
- `K_a`, `K_e`, dead bands, and `u_hold` require 250 RPM hardware tuning and
  have no approved numeric values yet.

Normal automatic braking follows from the same signed terms: a falling target
makes `a_cmd` negative, and measured speed above target makes `e_v` negative.

### 5.3 Smooth high-speed enable

**Designed using an existing smoothstep primitive**

```text
t = clamp((abs(v_measured) - 230) / (250 - 230), 0, 1)
B_250 = t^2 * (3 - 2*t)
```

- below 230 RPM: `B_250 = 0`;
- between 230 and 250 RPM: smooth entry;
- at or above 250 RPM: `B_250 = 1`.

The final state machine also requires race mode, fresh feedback, a stable
low-race pose, valid IK, and safe pitch state. Speed alone cannot arm assist.

## 6. Existing S7 servo trajectory

### 6.1 Shared blend

**Existing**

```text
p = clamp(elapsed_time / duration, 0, 1)
S(p) = 35*p^4 - 84*p^5 + 70*p^6 - 20*p^7
```

Each servo command is:

```text
q_i(t) = q_i_start + (q_i_target - q_i_start) * S(p)
```

All four servos share `p`, so they start and finish together. The seventh-order
blend has zero velocity, acceleration, and jerk at both endpoints.

### 6.2 Duration from the servo-rate limit

**Existing**

```text
duration = max(0.10,
               2.1875 * max_i(abs(q_i_target - q_i_start)) /
               servo_speed_limit_dps)
```

The factor `2.1875` converts the S7 endpoint distance into a duration whose peak
rate respects the configured servo command-rate limit. Initial assisted testing
retains `servo_speed_limit_dps = 90 deg/s`.

## 7. Existing virtual forward-tilt speed loop

### 7.1 Integral and pitch offset

**Existing**

```text
I_v(k) = clamp(I_v(k-1) + e_v * dt, -I_max, I_max)
delta_pitch_v = clamp(Kp_speed * e_v + Ki_speed * I_v,
                      -pitch_limit,
                      pitch_limit)
```

Current gains:

```text
Kp_speed = 0.30
Ki_speed = 0.0
```

For example, while unsaturated, a 20 RPM error produces:

```text
delta_pitch_v = 0.30 * 20 = 6 deg
```

The current normal pitch-offset limit is 8 deg and the current fast-mode blend
can raise it to 12 deg. Initial leg-assisted testing instead keeps the virtual
forward-tilt contribution near the observed approximately 7 deg safe region;
that value remains a hardware gate rather than a proven universal limit.

### 7.2 Effective pitch target

**Existing**

```text
pitch_ref = pitch_base + delta_pitch_v
```

The current low-stance base target is:

```text
pitch_base = -1.35 deg
```

Leg motion is not added numerically to `pitch_ref`. It changes the real wheel
axis/chassis geometry, and the pitch loop reacts to the resulting physical
bias.

## 8. Existing balance output

### 8.1 State-feedback sum

**Existing**

```text
pitch_term = K_pitch * (pitch - pitch_ref)
rate_term  = K_rate  * pitch_rate
speed_term = K_speed * v_measured
pos_term   = K_pos   * wheel_position
ff_term    = speed_ff

balance_raw = pitch_term
            + rate_term
            + speed_term
            + pos_term
            + ff_term
```

The nominal low-stance values are:

```text
K_pitch = 18
K_rate  = 8
K_speed = 3
K_pos   = 0
```

Fast blending currently moves the wheel-speed coefficient toward `0.5`.
High-speed speed feedforward is:

```text
speed_ff = K_ff * v_ramp * fast_blend
```

but the current `K_ff` is `0`, so the feedforward term is currently zero.

### 8.2 Wheel-position state

**Existing**

```text
delta_position_rev = v_measured * dt / 60
wheel_position = clamp((wheel_position + delta_position_rev) * 0.999,
                       -2 rev,
                       2 rev)
```

Its current output gain is zero, but the state remains available for diagnosis
or later separately approved tuning.

### 8.3 Balance clamp and wheel mixing

**Existing, with a designed runtime extension**

```text
balance_rpm = clamp(balance_raw, -U_level, U_level)
left_target_rpm  = balance_rpm - turn_rpm
right_target_rpm = balance_rpm + turn_rpm
```

The current compile-time balance clamp is 300 RPM. The approved design adds a
lower runtime validation-level cap and ultimately leaves approximately 60 RPM
of balance authority above the 400 RPM straight-line target.

## 9. Staged runtime limits

**Designed**

```text
v_limited = clamp(v_requested, -V_level, V_level)
balance_rpm = clamp(balance_raw, -U_level, U_level)
```

| Level | `V_level` | `U_level` | Longitudinal travel |
|---|---:|---:|---:|
| 1 | 250 RPM | 300 RPM | about +/-2 mm X |
| 2 | 300 RPM | about 350 RPM | accepted level-1 travel or a small increase |
| 3 | 350 RPM | about 410 RPM | hardware-accepted intermediate travel |
| 4 | 400 RPM | about 460 RPM | no more than about +/-5 mm X without review |

The motor layer's independent 1000 RPM hard target limit remains unchanged.
`U_level` is intentionally greater than `V_level` so balance correction retains
headroom.

A later implementation may express turn derating above 300 RPM as:

```text
turn_scale = clamp((400 - abs(v_measured)) / (400 - 300),
                   turn_scale_min,
                   1)
turn_limited = turn_requested * turn_scale
```

The exact minimum turn scale is not approved yet and must be chosen from
straight-line hardware data before high-speed turn testing.

## 10. BODY_WHEEL to model-coordinate transform

**Existing**

Let the physical point be `p`, physical reference be `p0`, model reference be
`m0`, calibrated scale be `s`, and the orthogonal fitted rotation/reflection be
`Q`. Then:

```text
delta = (p - p0) / s
m = m0 + transpose(Q) * delta
```

Current values:

```text
p0 = (-20.766667, 47.356667) mm
m0 = ( 22.830129, 46.929213) mm
s  = 0.955219899

Q = [ -0.996313812   0.085783378 ]
    [  0.085783378   0.996313812 ]
```

The matrix is orthogonal, so its inverse is its transpose.

## 11. Five-bar inverse kinematics

### 11.1 Link equations

**Existing**

Current model lengths:

```text
l1 = 60 mm
l2 = 90 mm
l3 = 90 mm
l4 = 60 mm
l5 = 37 mm
```

For model point `(x, y)`, the first driven joint satisfies:

```text
a*cos(alpha) + b*sin(alpha) = c

a = 2*x*l1
b = 2*y*l1
c = x^2 + y^2 + l1^2 - l2^2
```

The second driven joint satisfies:

```text
d*cos(beta) + e*sin(beta) = f

d = 2*(x - l5)*l4
e = 2*y*l4
f = (x - l5)^2 + y^2 + l4^2 - l3^2
```

### 11.2 Reachability discriminants

**Existing**

```text
D_alpha = a^2 + b^2 - c^2
D_beta  = d^2 + e^2 - f^2
```

Both must be non-negative. A negative discriminant means the corresponding
two linkage circles do not intersect, so no real joint angle exists.

At `Y = 25.08 mm`, rearward physical X movement makes the beta-side
discriminant fail quickly, while forward physical X movement makes the
alpha-side discriminant fail quickly. This is why the model-valid fixed-Y
interval is only approximately `X = -18.83 .. -18.16 mm`.

### 11.3 Candidate angles

**Existing**

For either equation `a*cos(theta) + b*sin(theta) = c`:

```text
phase  = atan2(b, a)
offset = atan2(sqrt(D), c)
theta_plus  = wrap_positive(phase + offset)
theta_minus = wrap_positive(phase - offset)
```

The two solutions are the two IK branches. A moving trajectory must preserve
its persisted branch identity instead of switching to whichever solution is
numerically closest at each step.

### 11.4 Singularity margin

**Existing**

For either side:

```text
margin = sqrt(D) / sqrt(a^2 + b^2)
```

The target's published margin is:

```text
ik_margin = min(alpha_margin, beta_margin)
```

Current acceptance requires:

```text
ik_margin >= 0.02
```

As the linkage circles approach tangency, `D` and the margin approach zero.
Near that point, a small Cartesian displacement can require a large joint-angle
change.

### 11.5 Servo command mapping

**Existing**

For each joint/servo:

```text
servo_command = neutral
              + direction * wrapped_delta(joint_angle, reference_angle)
              + ik_offset
```

The current configured command range is 10--175 deg. A physical point is valid
only when at least one branch candidate satisfies the geometric margin and the
mapped command limits on both legs.

## 12. Wheel RPM to linear speed

**Existing physical conversion**

For loaded effective wheel diameter `D_wheel` in metres:

```text
v_linear = RPM / 60 * pi * D_wheel
```

At 400 RPM:

```text
v_400 = 400 / 60 * pi * D_wheel
```

Use the loaded effective diameter measured on the vehicle. An unloaded outside
diameter can overestimate real ground speed.

## 13. Source pointers

- Approved behavior: `docs/superpowers/specs/2026-07-19-low-race-leg-assist-400rpm-design.md`
- Speed target, smoothstep, and virtual tilt: `project/code/control_chassis.c`
- Balance state feedback and wheel mixing: `project/code/control_balance.c`
- S7 servo planning: `project/code/control_leg.c`
- Coordinate transform and five-bar IK: `project/code/leg_kinematics.c`
- Current constants: `project/code/app_config.h` and `project/code/leg_config.c`

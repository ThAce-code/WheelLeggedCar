# Balance Stand v1 — Test Report

**Tag:** `balance-stand-v1`  
**Commit:** `f56185c`  
**Date:** 2026-07-02

---

## Parameters

```c
APP_BALANCE_PITCH_KP            18.0f    // angle feedback
APP_BALANCE_PITCH_RATE_KD        8.0f    // rate damping (gyro-derived)
APP_BALANCE_WHEEL_SPEED_KS       3.0f    // speed → deceleration tilt
APP_BALANCE_WHEEL_POS_KP         0.0f    // position integral (disabled)
APP_BALANCE_PITCH_SETPOINT_DEG   5.0f    // COM forward offset
APP_BALANCE_RPM_LIMIT          300.0f    // output saturation
```

VOFA command: `BL,18,8,3.0,0 BS,5.0`

## Standing (30 s captures)

| Capture | pitch mean | pitch P95 | speed mean | speed P95 | bal P95 | bal max |
|---|:---:|:---:|:---:|:---:|:---:|:---:|
| 235304 | 4.88° | 6.42° | -0.57 RPM | 36.5 RPM | 59 RPM | 179 RPM |
| 235350 | 4.94° | 7.53° | -0.19 RPM | 54.5 RPM | 77 RPM | 216 RPM |

- **Uns supported standing:** >30 s (capture limited, not stability limited)
- **Pitch steady-state error vs setpoint:** <0.12°
- **Speed drift:** <1 RPM mean (effectively stationary)

## Disturbance Recovery

- **bal max observed:** 179–216 RPM (72% of 300 limit — headroom confirmed)
- **No RPM saturation events** in disturbance captures
- **Recovery:** robot returns to upright after pushes in both forward/backward directions
- **Previous issue resolved:** 300 RPM saturation that degraded controller to constant torque no longer occurs

## Safety

| check | limit | observed max | pass |
|---|---|---|---|
| pitch angle | ±45° | 8.5° | ✅ |
| bal output | ±300 RPM | 216 RPM | ✅ |
| IMU age | 15 ms | <9 ms | ✅ |
| motor online | — | always | ✅ |

## Prerequisites

- **IMU_ZERO** required after each power-on (K_angle=18 amplifies zero drift)
- Robot must be held near equilibrium angle (~5° forward) at B,2 activation

## Changelog vs Previous

| param | old | new | reason |
|---|---|---|---|
| K_angle | 12 | 18 | LQR ID, clipped from raw ~28 |
| K_speed | 0.2 | 3.0 | τ decay 36s→2.4s via A(3,1) physics |
| BS | 4.5° | 5.0° | COM offset, verified by steady-state pitch |
| RPM limit | 150 | 300 | headroom for disturbance recovery |

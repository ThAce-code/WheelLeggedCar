# WheelLeggedCar CYT4BB7 Project Memory

Last updated: 2026-07-02

This file records hardware assumptions, firmware progress, command semantics, test status, and next steps for `D:\smartcar\WheelLeggedCar_cyt4bb7_v1`.

## Repository State

Branch layout (2026-07-02 reorganized):

```text
main ────────────────── CYT4BB7 (active development)
hardware/cyt2bl3 ────── CYT2BL3 (archived)
```

Stable tag:

```text
balance-stand-v1 → f56185c  (2026-07-02)
  BL,18,8,3.0,0 BS,5.0 RPM=300
  Confirmed: 30s+ unsupported standing, disturbance recovery within 216 RPM
  Test report: docs/balance-stand-v1-test-report.md
```

Key recent commits (balance control hardening):

```text
a470ea0 docs: add balance-stand-v1 test report
f56185c balance: tune gains for disturbance rejection (BL,18,8,3.0,0 BS,5.0)
d15c36a Raise RPM limit 150->300, setpoint 4.0->4.5
8eeed2d Stable standing: BL,12,8,0.2,0 BS,4
fd9dc25 Save standing parameters: BL,16.1,6.7,0,0 BS,2
6783d60 Add pitch setpoint offset command BS
3a967c4 Open LQR clip limits: angle 6->18, rate 0.4->8, speed 0.2->0.5, pos 1->3
27df412 Swap IMU roll/pitch axes for PCB orientation
a9d7893 Add YPR telemetry and IMU_ZERO command
414232d Expand telemetry to 16ch and make gain setters return status
6e838cd Use gyro rate for balance damping
f4a9fe0 Add optional IMU INT1 scheduling
99f8063 Publish IMU pitch rate
19b7351 Expose calibrated IMU gyro rate
```

Untracked folders (do not commit casually):

```text
data/
docs/assets/
docs/balance_lqr_ppt.html
```

---

## Hardware And UART Mapping

Debug / VOFA:

```text
COM6 = UART0
Purpose: VOFA JustFloat telemetry (16-float balance frames) and ASCII downlink commands.
```

BLDC driver link:

```text
UART_1 (CYT2BL3 binary A5 protocol)
Baudrate: 460800
TX pin: UART1_TX_P04_1
RX pin: UART1_RX_P04_0
```

IMU:

```text
LSM6DSV16X over SPI_2
SFLP (game rotation vector) at 120 Hz
INT1 pin: P19_3 (APP_IMU_USE_INT1 = 1U)
```

**IMU orientation note:** The LSM6DSV16X chip is rotated 90° on the PCB. `sensor_imu_copy_angle()` swaps roll ↔ pitch to correct this. The IMU must be level during boot for proper SFLP initialization.

---

## Balance Controller — Current State

### Standing parameters (saved to app_config.h)

```c
#define APP_BALANCE_PITCH_KP            (18.0f)
#define APP_BALANCE_PITCH_RATE_KD       (8.0f)
#define APP_BALANCE_WHEEL_SPEED_KS      (3.0f)
#define APP_BALANCE_WHEEL_POS_KP        (0.0f)
#define APP_BALANCE_PITCH_SETPOINT_DEG  (5.0f)
#define APP_BALANCE_RPM_LIMIT           (300.0f)
```

**The robot can stand unsupported and recover from pushes** with these gains.
VOFA command: `BL,18,8,3.0,0 BS,5.0`

### Deceleration tilt coefficient (K_speed/K_angle)

$$K_{tilt} = \frac{K_{speed}}{K_{angle}} = \frac{3.0}{18.0} = 0.167 \text{ °/RPM}$$

Physical meaning: speed error → equivalent tilt angle → gravity horizontal component → wheel acceleration.

From identified A(3,1) = -0.0125 (deg→RPM/step at 5ms), 1° tilt produces ~2.5 RPM/s acceleration.
Velocity decay time constant τ relates to gains as:

$$K_{speed} = \frac{K_{angle}}{2.5 \cdot \tau}$$

With K_angle=18, τ≈2s → K_speed ≈ 3.6. Chose 3.0 (slightly conservative).
Old K_speed=0.2 gave τ≈36s — effectively no speed regulation.

### Control law

```text
balance_rpm = K_angle * (pitch_deg - setpoint)
            + K_rate  * pitch_rate_dps       (gyro-derived, continuous)
            + K_speed * wheel_speed_rpm
            + K_pos   * wheel_pos_rev        (currently 0)
```

Then clamped to ±APP_BALANCE_RPM_LIMIT and added to chassis base RPM.

### pitch_rate source

Uses `imu->pitch_rate_dps` = gyro_y (post axis swap: gyro_x from chip). **Never** finite-difference from pitch_deg. The SFLP runs at 120 Hz and the balance loop at 200 Hz — finite difference on stair-stepped SFLP output produces `0, spike, 0, spike` pattern that kills damping.

### IMU timing

- INT1 interrupt → scheduler consumes ready flag → `sensor_imu_update()` reads SPI FIFO
- `imu_age_ms` consistently < 9ms during normal operation
- Safety gate blocks balance output if IMU age exceeds `APP_BALANCE_IMU_MAX_AGE_MS` (15ms)

---

## VOFA Downlink Commands

### Motor commands

```text
STOP        Stop motors, clear PID, clear excitation, clear chassis.
D,duty      Open-loop duty. Both wheels.
M,rpm       RPM closed-loop target. Both wheels.
P,kp,ki,kd  Set both wheel PID gains.
PL,kp,ki,kd Set left wheel PID only.
PR,kp,ki,kd Set right wheel PID only.
```

### Balance commands

```text
B,0         Stop balance, stop chassis, stop motors. Clears BI excitation.
B,1         Balance standby (telemetry only, no motor output).
B,2         Balance test mode (active control + motor output).
BP,kp,kd    Set pitch PD gains (compatibility; sets wheel terms to 0).
BL,angle,rate,speed,pos   Set all four balance gains. Signed.
BS,offset   Set pitch setpoint offset in degrees. Default = 4.0.
BZ          Reset wheel position integrator and motion state.
BI,amp,ms   Balance identification excitation. Square wave ±amp RPM at half-period ms. BI,0,0 disables.
```

### Chassis commands

```text
C,forward,turn   Set chassis forward/turn RPM. C,0,0 enables chassis for balance.
```

### IMU commands

```text
IMU_ZERO    Recalibrate gyro offsets. Board MUST be stationary for ~1 second.
```

### Safety

- `STOP` and `B,0` clear BI excitation and chassis.
- `B,1` does NOT clear BI excitation (intentional — for scheduled collection).
- `BP` and `BL` parameters are validated and return error count if rejected (out of range or non-finite).
- Balance output blocked when: FAULT state, IMU unhealthy, BLDC offline, chassis disabled, IMU age > 15ms, non-finite sensor values, |pitch| > 45°.

---

## VOFA Telemetry Channels (16-float balance frame)

```text
I0  = time_ms
I1  = balance_mode          0=OFF, 1=STANDBY, 2=BALANCE_TEST
I2  = roll_deg              (post axis-swap: chip pitch → car roll)
I3  = pitch_deg             (post axis-swap: chip roll → car pitch)
I4  = yaw_deg
I5  = pitch_rate_dps        (gyro-derived, continuous)
I6  = balance_rpm           (total balance command before clamping)
I7  = feedback_online       (AND of wheel->online, left_online, right_online)
I8  = left_motor_rpm
I9  = right_motor_rpm
I10 = left_duty
I11 = right_duty
I12 = balance_kp            (current K_angle)
I13 = balance_kd            (current K_rate)
I14 = chassis_left_rpm
I15 = chassis_right_rpm
```

Frame: 16 floats + 4-byte tail (0x00 0x00 0x80 0x7F) = 68 bytes at 200 Hz.

---

## Data Collection Tools

```text
tools/collect_balance_data.ps1   — 16-float balance telemetry capture
tools/test_collect_balance_data.ps1
tools/analyze_balance_pd.m       — PD analysis from balance captures
tools/test_analyze_balance_pd.ps1
tools/identify_balance_model.m   — LQR model identification from BI excitation data
tools/test_identify_balance_model.ps1
tools/collect_motor_steps.ps1    — 8-float motor telemetry capture
tools/analyze_motor_steps.m
```

Example balance capture:

```powershell
powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 `
  -Duration 30 `
  -Commands "0:STOP;0.5:BL,12,8,0.2,0;1:BS,4;1.5:B,1;2:B,2;2.5:C,0,0" `
  -Note "stable_standing"
```

---

## Model Identification Workflow

1. Flash firmware with BI support and gyro-derived pitch_rate.
2. Run `IMU_ZERO` with robot stationary and level.
3. Collect BI excitation data (robot held in ±15° range, BI square wave active):
   ```powershell
   powershell -ExecutionPolicy Bypass -File tools\collect_balance_data.ps1 `
     -Duration 30 `
     -Commands "0:STOP;0.5:BL,3,0.15,0,0;0.8:BI,8,500;1:B,1;2:B,2;2.2:C,0,0;28:BI,0,0" `
     -Note "id_bi8_p500"
   ```
   Collect 2-3 captures with different BI periods (500/900ms) for better rank.
4. MATLAB: `tools/identify_balance_model`
   - Reads `data/balance_capture_*.csv`, groups by note, keeps newest per note.
   - Filters: mode=2, feedback_online≥0.5, isfinite, |pitch|≤18°, |bal|≤140.
   - Identifies A(4×4), B(4×1) via least squares `pinv(phi)`.
   - Sweeps LQR Q(2,2) to find positive K_rate, prints candidate table.
   - Outputs: `data/balance_model_summary.csv`, `data/balance_model_fit.png`.
5. Test the "Best candidate: BL,..." command on hardware.

**Prerequisites for good identification:**
- Gyro-derived pitch_rate (NOT finite difference from pitch_deg)
- Correct IMU axis mapping (roll ↔ pitch swap for rotated chip)
- At least 2 captures with different BI periods for rank=5

---

## Key Lessons From Balance Tuning

1. **Signal quality > parameter tuning.** SFLP at 120Hz + balance loop at 200Hz = stair-step pitch. Numerical derivative produced `0, spike, 0, spike`. Fix the sensor pipeline before touching gains.

2. **Verify IMU axes before anything.** The chip was rotated 90° — pitch and roll were swapped. The user tilted forward but pitch_deg barely moved. A 5-second level-check capture saves hours.

3. **LQR needs good data.** First identification with noisy pitch_rate: RMSE=27 dps, K_rate=-0.22 (negative damping!). Second with gyro: RMSE=5.7 dps, K_rate always positive, rank=5, cond=1.98e4.

4. **Clip limits must be physics-based.** Original limits (K_angle≤6, K_rate≤0.4) cut LQR output by 50-70%. The robot couldn't fight gravity. After opening limits to 18/8, LQR gains could take effect.

5. **Geometric zero ≠ physical balance point.** IMU pitch=0° is the chip's level, not the robot's balance. The COM is behind the axle, requiring +4° forward lean. `BS` command solves this without changing gains.

6. **Simplify when debugging.** K_speed and K_pos were zeroed first. Once PD+BS worked, K_speed was added back gradually. Don't enable all terms at once.

7. **CSV column parsing can lie.** Quoted commands with commas (`"BL,16.1,6.7,0,0"`) shift awk/simple-split columns. Led to false "left motor dead" diagnosis. Verify with VOFA raw hex.

---

## Current Motor Control Features

Implemented and tested:

```text
BLDC UART communication: OK
Open-loop D command: OK
Motor RPM closed-loop M command: OK
Runtime PID tuning P/PL/PR: OK
Balance standing: OK (BL,12,8,0.2,0 BS,4)
Chassis C command: OK
BI identification excitation: OK
STOP/B,0 safety cleanup: OK
```

## Next Steps

1. Improve standing robustness — test with disturbances, different surfaces.
2. Consider re-enabling K_pos (small value, e.g. 0.5) to counter steady-state drift.
3. Tune K_speed for better translational damping.
4. Leg/servo coordination for chassis height control during balance.
5. Remote control integration.

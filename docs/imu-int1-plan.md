# IMU INT1 Implementation Plan

## 1. Objective

Enable the LSM6DSV16X `INT1` pin so IMU updates are event-driven instead of fixed-period polling. This is needed before motor control becomes timing-sensitive.

The target design is:

```text
LSM6DSV16X FIFO event
  -> INT1 rising edge on P11_0
  -> MCU EXTI ISR sets data-ready flag
  -> scheduler consumes flag
  -> sensor_imu_update() reads FIFO over SPI
```

## 2. Work Items

### P1. Hardware Wiring

| Step | Action | Result |
| --- | --- | --- |
| 1 | Connect LSM6DSV16X `INT1` to CYT2BL3 `P11_0` | Interrupt signal physically available |
| 2 | Confirm IMU and MCU share GND | Stable interrupt logic level |
| 3 | Probe `P11_0` with oscilloscope or logic analyzer | Verify interrupt pulse after software configuration |

### P2. Configuration

Add project-level configuration in `app_config.h`:

```c
#define APP_IMU_USE_INT1        (1U)
#define APP_IMU_INT1_PIN        P11_0
#define APP_IMU_STALE_TIMEOUT_MS (50U)
```

Keep a compile-time switch so polling can be restored quickly during bring-up:

```text
APP_IMU_USE_INT1 = 0 -> existing fixed-period polling
APP_IMU_USE_INT1 = 1 -> INT1 flag-driven update
```

### P3. IMU Interrupt Flag API

Add a small API around the data-ready flag:

```c
void sensor_imu_int1_isr(void);
uint8 sensor_imu_take_data_ready(void);
uint32 sensor_imu_get_last_update_ms(void);
```

Behavior:

- `sensor_imu_int1_isr()` sets a volatile flag.
- `sensor_imu_take_data_ready()` atomically reads and clears the flag.
- The IMU update path refreshes the last valid sample timestamp.

### P4. MCU EXTI Setup

After IMU SPI and LSM6DSV16X initialization succeed:

```c
#if APP_IMU_USE_INT1
exti_init(APP_IMU_INT1_PIN, EXTI_TRIGGER_RISING);
#endif
```

Expected include:

```c
#include "zf_driver_exti.h"
```

### P5. EXTI ISR Routing

In the GPIO external interrupt ISR for port 11:

```c
if(exti_flag_get(APP_IMU_INT1_PIN))
{
    sensor_imu_int1_isr();
}
```

Rules:

- Do not call `sensor_imu_update()` in ISR.
- Do not access SPI in ISR.
- Do not print in ISR.

### P6. LSM6DSV16X Interrupt Configuration

Configure LSM6DSV16X so FIFO events are routed to `INT1`.

Preferred behavior:

- `INT1` active high.
- Push-pull output.
- FIFO watermark or FIFO threshold interrupt enabled.
- Watermark tuned so scheduler gets fresh SFLP data without excessive interrupt load.

Implementation should stay inside the existing IMU driver layer, not in scheduler or application control logic.

### P7. Scheduler Update

Current IMU-only mode reads IMU periodically. Change the IMU task to:

```text
if APP_IMU_USE_INT1:
    if sensor_imu_take_data_ready():
        sensor_imu_update(now_ms)
    else:
        skip IMU SPI read
else:
    keep current 5 ms polling behavior
```

Add stale detection:

```text
if now_ms - last_imu_update_ms > APP_IMU_STALE_TIMEOUT_MS:
    mark IMU stale/fault
```

### P8. Telemetry

During bring-up, keep VOFA+ output simple:

| Channel | Meaning |
| --- | --- |
| I0 | roll |
| I1 | pitch |
| I2 | yaw |
| I3 | imu data age in ms |
| I4 | imu interrupt counter or missed timeout counter |

After verification, optional debug channels can be removed to reduce serial load.

## 3. Verification Plan

### Step 1. Static Check

- Confirm `APP_IMU_INT1_PIN` is `P11_0`.
- Confirm `P11_0` is not used by another module.
- Confirm UART0 still uses `P00_1/P00_0`.
- Confirm servo 0 still uses `P00_3`.

### Step 2. Interrupt Signal Check

- Flash firmware with `APP_IMU_USE_INT1=1`.
- Use oscilloscope or logic analyzer on `P11_0`.
- Expect rising pulses after IMU FIFO interrupt routing is enabled.

### Step 3. Firmware Check

Expected behavior:

- IMU interrupt counter increases.
- IMU data age remains low and bounded.
- VOFA+ roll/pitch/yaw update without the previous delayed response.

### Step 4. Motor Coexistence Check

Enable motor PWM/control after IMU INT1 is stable:

- Motor update task should not be delayed by IMU SPI reads in ISR.
- IMU update should remain stable under motor PWM noise.
- If IMU data becomes stale, safety should be able to detect it.

### Step 5. Fallback Check

Set `APP_IMU_USE_INT1=0` and verify the old polling path still works. This gives a quick recovery path if hardware wiring or interrupt routing needs more debugging.

## 4. Risks And Mitigations

| Risk | Impact | Mitigation |
| --- | --- | --- |
| ISR reads SPI by mistake | Blocks motor timing | Keep ISR flag-only |
| Wrong INT polarity | No interrupt or constant interrupt | Use active-high rising edge first; verify with oscilloscope |
| FIFO watermark too high | IMU data latency | Start with low watermark, tune after VOFA+ observation |
| FIFO watermark too low | Excessive interrupt rate | Increase watermark if interrupt rate is too high |
| `P11_0` reused later | Resource conflict | Keep pin table updated and reserve `P11_0` for IMU INT1 |
| Motor noise affects IMU line | False interrupts | Short wiring, common ground, optional pull/RC only if needed |

## 5. Done Definition

This work is complete when:

- `P11_0` is documented as IMU INT1.
- IMU INT1 EXTI can be enabled or disabled by `APP_IMU_USE_INT1`.
- ISR only sets an IMU data-ready flag.
- Scheduler reads IMU FIFO only when the flag is present.
- VOFA+ shows timely roll/pitch/yaw updates.
- Motor PWM can run without IMU interrupt logic blocking real-time tasks.

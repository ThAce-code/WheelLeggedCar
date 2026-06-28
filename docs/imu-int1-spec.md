# IMU INT1 Specification

## 1. Purpose

This document defines the hardware and software specification for using the LSM6DSV16X `INT1` pin as the IMU data-ready interrupt source.

The goal is to reduce IMU update latency while keeping motor control timing deterministic. `INT1` is used only as a "new IMU data available" signal. SPI FIFO reading remains in scheduler/main context, not in the interrupt service routine.

## 2. Pin Assignment

| Signal | Source | Target | Status | Notes |
| --- | --- | --- | --- | --- |
| IMU INT1 | LSM6DSV16X `INT1` | CYT2BL3 `P11_0` | Planned | External interrupt input |
| IMU SPI CLK | CYT2BL3 `SPI3_CLK_P13_2` | LSM6DSV16X SCK | Existing | Already verified |
| IMU SPI MOSI | CYT2BL3 `SPI3_MOSI_P13_1` | LSM6DSV16X SDI | Existing | Already verified |
| IMU SPI MISO | CYT2BL3 `SPI3_MISO_P13_0` | LSM6DSV16X SDO | Existing | Already verified |
| IMU CS | CYT2BL3 `P13_3` | LSM6DSV16X CS | Existing | Manual GPIO chip select |

`P11_0` is selected because it is currently free and does not conflict with the known occupied resources:

- `P13_0/P13_1/P13_2/P13_3`: IMU SPI bus.
- `P00_0/P00_1`: UART0 debug / VOFA+.
- `P00_3`: servo 0 PWM.
- `P23_7`: heartbeat LED.
- `P02_0/P02_1`: should be kept available for possible encoder or motor feedback use.

## 3. Electrical Requirements

| Item | Requirement |
| --- | --- |
| IMU interrupt output | Active high |
| IMU output type | Push-pull preferred |
| MCU input mode | High impedance GPIO external interrupt input |
| Trigger edge | Rising edge |
| Logic level | Match IMU VDDIO and MCU input level, normally 3.3 V |
| Ground | IMU GND and MCU GND must be common |

Connection:

```text
LSM6DSV16X INT1  ->  CYT2BL3 P11_0
LSM6DSV16X GND   ->  MCU GND
LSM6DSV16X VDDIO ->  3.3 V IO rail
```

## 4. Interrupt Source

Preferred interrupt source:

1. FIFO watermark / FIFO threshold interrupt routed to `INT1`.
2. SFLP-related FIFO data is then consumed from the existing SPI FIFO read path.

Raw accelerometer or gyroscope data-ready interrupt is not the first choice for the wheel-legged car stage, because it can generate unnecessary interrupt pressure after motor control is enabled.

## 5. MCU Behavior

The MCU side uses `zf_driver_exti`:

```c
exti_init(APP_IMU_INT1_PIN, EXTI_TRIGGER_RISING);
```

Expected project-level configuration:

```c
#define APP_IMU_USE_INT1        (1U)
#define APP_IMU_INT1_PIN        P11_0
```

ISR rule:

- Clear/check the external interrupt flag.
- Set an IMU data-ready flag.
- Do not read SPI.
- Do not parse FIFO.
- Do not print.
- Do not run attitude estimation inside ISR.

The ISR should be short and deterministic:

```text
P11_0 rising edge
  -> GPIO EXTI ISR
  -> clear EXTI flag
  -> imu_data_ready = 1
```

## 6. Scheduler Behavior

The scheduler consumes the interrupt flag:

```text
scheduler task
  -> if imu_data_ready == 1
       clear imu_data_ready
       sensor_imu_update()
       update imu timestamp
  -> else
       skip SPI read
```

The IMU task should also keep a stale-data timeout. If no valid IMU sample arrives within 20-50 ms, the IMU state should be marked stale and safety logic should be able to react.

## 7. Acceptance Criteria

| Check | Expected Result |
| --- | --- |
| Oscilloscope on `P11_0` | Rising pulses appear when IMU FIFO watermark is reached |
| VOFA+ attitude output | Roll/pitch/yaw update smoothly without visible delayed response |
| Motor PWM enabled | IMU update remains stable and does not block motor timing |
| ISR duration | ISR only sets a flag and stays short |
| Fault test | Disconnecting or stopping IMU interrupt causes stale/fault state after timeout |

## 8. Constraints

- `INT1` is a readiness signal, not a data transport path.
- SPI reads must remain outside ISR.
- `P11_0` should not be reused by keys, encoders, motor feedback, or other peripherals while IMU INT1 is enabled.
- If future hardware layout changes, update this file and `app_config.h` together.

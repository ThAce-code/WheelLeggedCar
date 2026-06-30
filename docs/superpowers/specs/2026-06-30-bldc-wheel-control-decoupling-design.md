# BLDC Wheel Protocol and Control Decoupling Design

## Goal

Build the CYT2BL3 dual FOC BLDC UART protocol into this CYT4BB7 wheel-legged robot project while keeping the future whole-body wheel-leg controller decoupled from the low-level UART protocol.

The first implementation target is the diagnostic-enhanced control baseline:

- send left and right wheel duty commands to the BLDC driver;
- receive left and right wheel speed feedback;
- receive left and right reduced wheel angle feedback;
- keep diagnostic counters for UART/protocol health;
- expose a clean actuator-level interface that future balance or chassis control can use without including `bldc_foc_uart.h`.

## Current Project Context

The current project already has these relevant modules:

- `bldc_foc_uart.c/.h`: CYT2BL3 BLDC UART protocol adapter.
- `actuator_motor.c/.h`: motor actuator command limiter and periodic sender.
- `app_scheduler.c`: cooperative fixed-period scheduler.
- `control_leg.c/.h`: leg and servo target mixer.
- `actuator_servo.c/.h`: servo PWM output.
- `app_safety.c`: safety fault handling that stops motor and servo actuators.

The current scheduler has actuator slots, but it does not yet have a whole-body controller slot that coordinates wheel and leg commands together. That future layer should be added above the actuator modules, not inside the UART protocol module.

## Architecture

Use a lightweight three-layer split:

```text
future control_balance / control_chassis
    reads IMU, wheel feedback, leg state, command input, safety state
    writes generic wheel and leg commands
        |
        v
actuator_motor
    owns generic motor command and wheel feedback API
    limits duty, handles stop behavior, tracks feedback age/online status
        |
        v
bldc_foc_uart
    owns CYT2BL3 binary UART protocol, frame parsing, checksum, diagnostics
        |
        v
UART1 ISR / zf_driver_uart
```

The key rule is that future control modules must not depend on `bldc_foc_uart.h`. They should use actuator-level types from `app_types.h` or `actuator_motor.h`.

## Protocol Scope

Use the binary 7-byte protocol from `CYT2BL3双路FOC无刷驱动通讯协议.xlsx`.

Frame format:

```text
byte0: 0xA5
byte1: function
byte2: left high byte
byte3: left low byte
byte4: right high byte
byte5: right low byte
byte6: 8-bit sum of byte0..byte5
```

Required functions:

- `0x01 SET_DUTY`: main controller sends left and right duty commands.
- `0x02 UPLOAD_SPEED`: driver feedback for left and right speed.
- `0x05 UPLOAD_RDT_ANGLE`: driver feedback for left and right reduced wheel angle.

Supported but not control-critical:

- `0x04 UPLOAD_ANGLE`: raw motor angle, useful for diagnostics.
- `0x03 ZERO_CALIBRATE`: zero calibration command.
- `0x06 SET_ANGLE_ZERO`: set current motor angle as zero.

String commands remain out of the first control path. Keep `STOP-SEND` available for stopping driver-side streaming, but do not add `GET-VOLTAGE`, `GET-ENCODER`, or `GET-PHASE-DUTY` unless a later diagnostic task needs them.

## Data Interfaces

Add a generic wheel feedback type in `app_types.h`:

```c
typedef struct
{
    int16 left_speed;
    int16 right_speed;
    int16 left_reduced_angle;
    int16 right_reduced_angle;
    uint32 last_rx_ms;
    uint32 age_ms;
    uint8 online;
}wheel_feedback_struct;
```

Add a generic motor diagnostic type that does not expose protocol ownership to future control code:

```c
typedef struct
{
    int16 left_raw_angle;
    int16 right_raw_angle;
    uint32 checksum_error_count;
    uint32 unknown_frame_count;
    char last_unknown_ascii[64];
}motor_diag_struct;
```

`bldc_foc_feedback_struct` can remain in `bldc_foc_uart.h`, but it should be consumed by `actuator_motor.c`. Future control layers should read:

```c
const motor_cmd_struct *actuator_motor_get_cmd(void);
const wheel_feedback_struct *actuator_motor_get_feedback(void);
const motor_diag_struct *actuator_motor_get_diag(void);
```

## Scheduler Boundary

Keep protocol parsing in the UART RX interrupt path:

```text
uart1_isr() -> bldc_foc_uart_rx_isr() -> parse bytes into BLDC feedback snapshot
```

Keep command output in the cooperative scheduler:

```text
future control_balance_update()
actuator_motor_update()
actuator_servo_update()
telemetry_update()
```

The immediate BLDC work should not implement `control_balance_update()`. It should only prepare `actuator_motor` so that adding the whole-body layer later is straightforward.

## Safety and Fault Behavior

- On fault, `app_safety_force_fault()` stops motor and servo actuators.
- `actuator_motor_stop()` sends zero duty once and clears the motor command.
- Fault state should not cause repeated 1 ms stop-frame spam.
- BLDC feedback timeout should not directly fault the system in the first implementation. It should mark `wheel_feedback.online = APP_FALSE` and expose the state to telemetry and future control logic.

Add a configuration constant:

```c
#define APP_BLDC_FEEDBACK_TIMEOUT_MS    (100U)
```

## Diagnostics

Expose enough diagnostics for VOFA and IAR debug:

- online state;
- feedback age in ms;
- left and right speed;
- left and right reduced angle;
- raw angle if available;
- checksum error count;
- unknown frame count;
- last unknown ASCII line.

Telemetry may display a compact subset first. The core requirement is that the data exists behind clean actuator APIs.

## Hardware and Configuration Assumptions

The current target configuration uses:

```c
#define APP_BLDC_UART_INDEX             (UART_1)
#define APP_BLDC_UART_BAUDRATE          (460800)
#define APP_BLDC_UART_TX_PIN            (UART1_TX_P04_1)
#define APP_BLDC_UART_RX_PIN            (UART1_RX_P04_0)
```

These pins must be verified against the actual wiring before hardware testing. The protocol design does not assume UART5/UART6 from the E07 example; this project uses one UART link to a dual-channel driver.

## Validation Plan

Build validation:

- Build the affected CM7_0 IAR project.
- Confirm `bldc_foc_uart.c`, `actuator_motor.c`, and any new headers are included in the CM7_0 project file.

Hardware smoke test:

- Power-on default does not spin wheels.
- Zero duty frame is sent during motor init/stop.
- Small positive and negative duty commands move the expected wheel directions.
- Speed feedback updates after feedback streaming is enabled.
- Reduced angle feedback changes when wheels rotate.
- Disconnecting or silencing the driver causes `online` to become false after `APP_BLDC_FEEDBACK_TIMEOUT_MS`.
- UART noise or invalid frames increment diagnostics without blocking the scheduler.

## Out of Scope

- Full wheel-leg balance controller implementation.
- Remote control or host command parser.
- Closed-loop velocity or torque control in the main controller.
- String protocol expansion for voltage, encoder, or phase duty.
- BLDC driver-board firmware changes.

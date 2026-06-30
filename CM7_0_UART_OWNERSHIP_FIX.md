# CM7_0 UART Ownership And BLDC Bring-Up Notes

## Current State

The application now runs on `CM7_0` and owns the peripherals used by the wheel-leg control stack:

- `UART0`: VOFA/debug telemetry on `COM6`.
- `UART1`: CYT2BL3 BLDC driver command/feedback link on `COM11`.
- IMU, safety, leg control, servo, and BLDC actuator logic.

`CM0+` only initializes the system and starts the application cores. It does not initialize or use `UART0`.

## Confirmed Hardware Debug Result

UART1 output was verified with VOFA RawData:

- `COM6` is `UART0` telemetry.
- `COM11` is `UART1` BLDC communication.
- UART1 at `460800 8N1` can output readable ASCII during debug mode.
- The final command mode is binary CYT2BL3 protocol, matching the E07 example.

## Key Code Decisions

### UART Ownership

- `project/user/main_cm0plus.c`: removed `debug_init()` from `CM0+`.
- `project/user/main_cm7_0.c`: uses `debug_init()` so `CM7_0` owns `UART0`.

### BLDC Protocol

BLDC commands are sent through `project/code/bldc_foc_uart.c`.

Final duty command format:

```text
A5 01 left_H left_L right_H right_L checksum
```

Checksum is the low 8 bits of the sum of the first 6 bytes.

Current config:

```c
APP_BLDC_UART_INDEX             UART_1
APP_BLDC_UART_BAUDRATE          460800
APP_BLDC_UART_TX_PIN            UART1_TX_P04_1
APP_BLDC_UART_RX_PIN            UART1_RX_P04_0
APP_BLDC_USE_ASCII_COMMANDS     0U
APP_BLDC_START_FEEDBACK         0U
```

ASCII command support remains in code as a diagnostic path, but is disabled by default.

### ISR Compatibility

The restored Seekfree `zf_driver` expects additional PIT and UART ISR symbols. `project/user/cm7_0_isr.c` defines:

- `pit0_ch10_isr` through `pit0_ch21_isr`
- `uart5_isr`
- `uart6_isr`

The UART receive handlers use `uart_isr_mask()` before dispatching to upper-level callbacks. This is required by the restored UART driver because received bytes are buffered inside the driver before `uart_query_byte()` reads them.

## Useful VOFA Channels

The current telemetry includes BLDC and safety diagnostics:

```text
I20 motor command enable
I21 left target duty
I22 right target duty
I23 BLDC TX frame count
I24 last TX function
I25 last TX left duty
I26 last TX right duty
I27 app state
I28 safety fault
I29 IMU healthy
I30 roll
I31 pitch
```

State values:

```text
0 BOOT
1 CALIBRATE
2 STANDBY
3 RUN
4 FAULT
```

If `I27 = 4`, BLDC test output is forced to zero duty even when `I28 = 0`.

## Hardware Smoke Test Order

1. Build and flash `cyt4bb7_cm_7_0`.
2. Confirm VOFA telemetry is present on `COM6`.
3. Confirm `I27 != 4`, `I28 = 0`, and `I29 = 1`.
4. Confirm UART1 command frames on `COM11` in Hex RawData.
5. Connect UART1 TX/RX/GND to the CYT2BL3 driver.
6. Keep wheels suspended and test low duty first.


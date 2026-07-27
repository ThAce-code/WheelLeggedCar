# Task 6 CM7_1 UART0 ownership fix

## Root cause

`project/user/cm7_1_isr.c` defined a `uart0_isr` that serviced `UART_0`, while CM7_0 is the host/debug UART0 owner. The CM7_1 IAR project already includes `cm7_1_isr.c` and has no explicit UART0 vector entry; vector binding is symbol-based through the startup table.

## Fix

- Removed only the CM7_1 `uart0_isr` definition.
- Kept the other CM7_1 ISR handlers (`uart1_isr` through `uart4_isr`) unchanged.
- Added `tools/test_cm7_uart_ownership_static.ps1`, which parses the CM7_1 EWP XML and checks CM7_1/CM7_0 source ownership and required ISR preservation.

## Verification

- RED baseline: the new ownership check failed before the fix with `CM7_1 ISR source still defines uart0_isr.`
- GREEN: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/test_cm7_uart_ownership_static.ps1` passed.
- Source/EWP scan: CM7_1 has no `uart0_isr` or `UART_0`; CM7_0 still defines `uart0_isr` and references `UART_0`; CM7_1 EWP XML parses and includes `cm7_1_isr.c` with no explicit UART0 vector reference.
- `git diff --check` passed for the changed source and static check.

## Concerns

IAR Embedded Workbench is not available in this shell, so a fresh CM7_1/CM7_0 hardware build was not run. No files under `libraries` were changed.

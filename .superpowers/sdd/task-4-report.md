# Task 4 report — CM7_0 motion-command router

## Status

Implemented the CM7_0-only motion-command router and chassis adapter. The router stores one request per source, validates finite values/enable/timeout/sequence, gates remote sources on remote arm and maintenance/emergency state, arbitrates UART local over wireless manual over autonomous, and stops on stale, maintenance, emergency, or safety conditions. No `libraries` files were changed and no CM7_1 control path was added.

## TDD evidence

- RED: the required host compile failed because `motion_command_router.h` and `motion_command_router.c` were absent.
- GREEN: `gcc -std=c11 -Wall -Wextra -Werror -DINTERCORE_HOST_TEST ...` completed successfully; executable output was `intercore_control_foundation_test: PASS`.

## Additional verification

- Static `rg` scan confirmed the only chassis calls are in `motion_command_router_port.c`; no router symbols occur under `project/user` (CM7_1 does not acquire the control path).
- `git diff --check` completed without errors.
- IAR CM7_0/CM7_1/CM0+ builds were not run in this environment; hardware smoke testing remains required.

## Concerns

The new router is intentionally standalone per the task file; wiring it into the application scheduler and selecting the CM7_0 call site should be handled by the integration task.

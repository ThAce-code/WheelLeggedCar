# Task 3 report: nonblocking IPC doorbell

- Baseline: `01bfac20`
- Implementation: added `intercore_notify` state machine and Cypress IPC pipe port; host test mocks the port and covers accepted send, in-flight coalescing, release, rejected/busy send, pending receive, and pending clear.

## RED

Before adding notification code, the required GCC command failed as expected:

```text
fatal error: intercore_notify.h: No such file or directory
cc1.exe: fatal error: project/code/intercore_notify.c: No such file or directory
```

## GREEN and verification

With `C:\msys64\ucrt64\bin` first in `PATH`:

```text
gcc -std=c11 -Wall -Wextra -Werror -DINTERCORE_HOST_TEST -Ilibraries/zf_common -Iproject/code project/tests/intercore_control_foundation_test.c project/code/intercore_protocol.c project/code/intercore_transport.c project/code/intercore_notify.c -o project/tests/build/intercore_control_foundation_test.exe
intercore_control_foundation_test: PASS
```

The notification port also compiles as a host-test translation unit with `-Wall -Wextra -Werror`. The blocking-call scan found no `while`, `system_delay`, or `ipc_send_data` matches in the notification implementation. `git diff --check` passed (Git emitted only its normal LF/CRLF conversion warning).

An IAR target build and hardware smoke test were not available in this environment; target integration still needs validation against the CYT4BB project include configuration and IPC endpoint IRQs.

## Concerns

- `intercore_notify_take_pending()` uses a bounded read-and-clear handoff; verify interrupt atomicity on the target if receive IRQ and deferred consumer can preempt each other.
- The port maps every non-CM7_0 role value to CM7_1; callers should pass only the declared role enum values.
- The checked-in SDK sets `CY_IPC_PIPE_MAX_CLIENTS` to 8 while the brief fixes client ID `0x31`; a target build must confirm an override or resolve this endpoint/client-ID mismatch before hardware use.

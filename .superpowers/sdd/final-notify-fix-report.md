# Final inter-core notification pending fix

## Root cause

`intercore_notify_try()` returned immediately when a doorbell was in flight,
discarding notification bits.  `intercore_notify_release_callback()` only
cleared `in_flight`, so a busy pipe could lose STOP/FAULT-style coalesced bits.

## Fix

- Added `pending_out_bits` to the notification diagnostic/state structure.
- Protected outbound state transitions with the existing target critical
  section and host memory barrier abstraction.
- Coalesced bits while a send is in flight, and merged retained bits into the
  next send attempt after a transient port-busy failure.
- Made release perform one bounded pending handoff; no wait, delay, or retry
  loop is introduced.  A busy handoff remains queued for a later send attempt.
- Added host assertions for coalescing, final delivery, busy handoff retention,
  and bit preservation.

## Verification

The host GCC C11 suite was rebuilt and executed with `-Wall -Wextra -Werror`:

```text
compile_exit=0
intercore_control_foundation_test: PASS
run_exit=0
```

`git diff --check` passed.  The notification blocking scan
(`ipc_send_data|system_delay|while`) returned no matches in the notification
implementation or port adapter.  No files under `libraries` were changed.

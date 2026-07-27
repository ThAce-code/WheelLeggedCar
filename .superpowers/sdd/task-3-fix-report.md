# Task 3 target integration fix

## SDK/client ID

`libraries/sdk/common/src/drivers/ipc/cy_ipc_config.h` defines
`CY_IPC_PIPE_MAX_CLIENTS` as 8. `cy_ipc_pipe.c` uses `clientId` directly as
the callback-array index and rejects values `>= CY_IPC_PIPE_MAX_CLIENTS`.
The brief's `0x31` therefore could never register or dispatch. This fix uses
client ID `3`, which is in the SDK's legal range and is not used elsewhere in
this checkout; both the message header and callback registration use the same
macro.

## Shared doorbell storage and ordering

The doorbell is eight bytes at `INTERCORE_NOTIFY_MESSAGE_ADDRESS`, defined as
`0x28081FF8` (the final eight bytes of SRAM1's documented
`0x28080000-0x28081FFF` shared window). Target builds use that fixed shared
address instead of a private `.bss` object; host tests retain a local object.
`__DMB()` is issued after writing `clientId` and `data` and before submitting
the IPC message.

## Pending-bit atomicity and role validation

`intercore_notify_take_pending()` and the receive ISR now exchange/merge
`pending_bits` inside a preserved-PRIMASK critical section (host tests use an
atomic signal fence). Invalid role values are rejected by both the state layer
and the Cypress port before endpoint selection.

## IAR integration

The `intercore_notify` source group (C/H plus port C/H) is present in both
`cyt4bb7_cm_7_0.ewp` and `cyt4bb7_cm_7_1.ewp`.

## Verification

- Added host assertions for legal client ID, shared address/range, and invalid
  role before implementation changes.
- `git diff --check`: clean (only normal LF/CRLF conversion warnings).
- IAR Embedded Workbench is not installed in this environment, so CM7 IAR
  builds were not run; XML source-group edits were inspected directly.
- The bundled GCC invocation was attempted; this Windows session's GCC exits
  non-zero without diagnostics even for a trivial compile, so the pre-existing
  host executable was not treated as fresh build evidence.

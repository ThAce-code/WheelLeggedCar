# Camera Handoff Task 3 Report

## Superseding outcome

Task 3 passes on the corrected transport path. This report supersedes the earlier `236.192 s` / `+3296` observation and its camera-disabled comparison; those figures came from a debugger-read workflow that was not a trustworthy uninterrupted measurement and must not be used as acceptance evidence.

CM7_1 now acquires the newest CM7_0-owned slot, exposes the read-only vision boundary, sends the same held slot through the existing Seekfree Assistant framing, and releases it after the synchronous send attempt returns. No vision algorithm or motion output was added. Wheel motor power remained off; no wheel, servo, motion, or `LXY` command was sent.

## Correct send semantics

The vendor Assistant camera function returns `void`, but that does not make every return a successful send. `camera_seekfree_transport` installs the supported custom Assistant transfer callback and records the WiFi-SPI `remaining` result for both expected segments: the Assistant header and the 22,560-byte grayscale payload. A frame increments `sent_count` only when exactly two segments were requested with the expected lengths and both returned zero remaining bytes. A missing/partial header, missing/partial payload, or unexpected segment increments `send_failure_count`; release still runs so a failed network send cannot strand a slot in `READING`.

The automatic evidence snapshot is copied with an odd/even generation guard. Gate start is delayed five seconds after connection to exclude startup behavior, then a 60,000 ms interval is captured without debugger Break/F5 intervention. Startup and steady-state send-duration histograms are separate.

## Software and build verification

- The Task 3 contract was observed RED before implementation and GREEN afterward.
- Camera handoff/foundation host tests compile with GCC C11 `-Wall -Wextra -Werror`; the handoff ABI asserts `producer_period_drop_count` at offset 184, reserve at offset 188, and reserve size 68.
- Capture-service, camera API/ownership, and CM7 UART static checks pass, along with IMU numeric, servo 300 Hz, leg-zero, and IK-height regressions.
- Fresh IAR 9.40.1 builds passed: CM0+ 0 errors/0 warnings, CM7_0 0 errors/3 unchanged `Pe550` warnings, and CM7_1 0 errors/0 warnings. The final capture-enabled CM7_0 rebuild also passed 0 errors/3 warnings before the final full-core download.

## Formal automatic 60-second steady-state gate

The valid automatic evidence block had even `generation=4` and `complete_count=1`. It came from the timestamp-correction/adapter build; the later recovery correction changed only failure-state assignment so that retry reinitializes WiFi, not the steady send path measured here.

| Diagnostic | Start | End | Delta |
| --- | ---: | ---: | ---: |
| Consumer time | 15,920 ms | 75,920 ms | 60,000 ms |
| Captured | 5,290 | 8,292 | 3,002 (50.033 FPS) |
| Published | 58 | 651 | 593 (9.883 FPS) |
| Notify | 57 | 650 | 593 |
| No-free drops | 4,995 | 4,995 | 0 |
| Period-gate drops | 237 | 2,646 | 2,409 |
| Acquired | 56 | 649 | 593 |
| Valid sends | 53 | 646 | 593 |
| Released | 56 | 649 | 593 |
| Reconnect | 2 | 2 | 0 |
| Invalid / timeout / stale | 0 / 2 / 2 | 0 / 2 / 2 | 0 / 0 / 0 |
| Send failures | 1 | 1 | 0 |

Accounting closes exactly: `593 published + 0 no-free + 2409 period-drop = 3002 captured`. Acquired, valid-send, and released deltas are each 593. Slots were `FREE/FREE` at both endpoints; sequences changed from `[5289,4739]` to `[8290,4739]`. The startup histogram stayed `[53,0,0,0,0,0,1]`; the steady histogram changed from all zero to `[593,0,0,0,0,0,0]`, so all 593 steady sends completed in the `<=25 ms` bucket.

## Disconnect recovery: failed hypothesis and accepted fix

The first recovery hypothesis called the vendor `wifi_spi_socket_close()` API before reconnect. It is present, but all five observed close attempts failed at the first `wait_idle` because the module INT signal remained low; `close_failure_count=5`. Stopping/restarting Assistant therefore did not recover on this path. This attempt is `FAIL` evidence and is not accepted.

The accepted path marks both socket and WiFi failed when connect or transfer validation fails. After the 5 s retry gate, the existing `wifi_spi_init()` performs the module hard reset/reassociation before `wifi_spi_socket_connect()` is retried. In the formal test, stopping Assistant produced the first observed failure with acquired/sent/released `2338/2334/2338`, reconnect 3, failure 2, and slots `[READY,READY]` in the transient snapshot. After Assistant restarted, the corrected hard-reset build recovered to an established TCP session; at the stable recovery start it showed consumer time 486,387 ms, acquired/sent/released `2915/2907/2916`, reconnect 10, failure 2, timeout/stale `6/6`, and `FREE/FREE`. Across two non-atomic Live Watch reads 97,792 ms apart, valid sends increased by 967 and releases by 966, with no new send failure or reconnect. This is recovery-duration evidence only, not a closed accounting window; the automatic 60 s gate above is the only acceptance ledger.

## True capture-disabled control baseline

The compile-switch path is covered by the static contract, whose delivered-state assertion intentionally requires `APP_CAMERA_CAPTURE_ENABLE=1`. For the control run it was temporarily set to 0, CM7_0 rebuilt with 0 errors/7 expected disabled-path warnings, and the complete image set was downloaded. After 255.643 s of continuous running:

- scheduler missed ticks remained 0 and maximum gap was 1 ms;
- servo count was 76,707, or approximately 300.05 Hz;
- producer publish, period-drop, no-free, invalid, and timeout diagnostics all remained 0;
- IMU timestamp was 255,642 ms at a scheduler time of 255,643 ms (1 ms age);
- safety fault and both BLDC duties remained 0; leg fault was `LEG_FAULT_NONE`.

Capture was then restored to 1; the static contract passed, and the final capture-enabled CM7_0 build/download completed successfully.

## Final capture-enabled confirmation

The final hard-reset firmware produced valid frames with producer init state 3 and `frame_valid=1`. Early live evidence showed frame count 6,579, publish count 1,250, last copy 76 us, max copy 94 us, safety fault 0, BLDC duties `0/0`, and no leg fault. Later shared counters advanced to published 6,904 and consumed 6,903; no-free was 252, stale 1, invalid 0, timeout 1, with both slot states `FREE/FREE` at the final halt.

Windows showed `192.168.137.1:8086 -> 192.168.137.42:6666` as `Established` and the Assistant listener on `0.0.0.0:8086`. Two foreground Assistant captures 1.8 s apart showed 188 x 120, FPS `11 -> 9`, throughput `226,692 -> 228,140 B/s`, and differed in 47,703 encoded screenshot characters. This is independent display/traffic evidence; it is not substituted for transport-return validation.

## Cleanup and distribution boundary

IAR was broken and exited from debugging to `Ready`. All three generated `.sim` files under the worktree were deleted and a recursive recount returned zero. The hash-locked local `.ewx` was not pushed, packaged, copied, or externally distributed. This task makes only the approved local commit; no push or pull request is performed.

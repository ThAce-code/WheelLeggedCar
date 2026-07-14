# Camera handoff Task 2 report

## Result

- Status: `DONE_WITH_CONCERNS`
- Baseline: `c7c56a7937a4b6571ff3e1bedc227a603bde56f1`
- Scope: Task 2 only, plus the explicitly authorized minimal Task 1 release-accounting correction and its host regression.
- Hardware disposition: Gate 1 `FAIL`; Gate 2 `NOT RUN` by gate rule.

## Implementation

- Moved MT9V03X initialization, finish-flag handling, exact-size frame copy, 100 ms latest-frame publish gate, and diagnostics to CM7_0 in `camera_capture_producer`.
- Added a polling, notification-hint-tolerant CM7_1 `camera_frame_consumer` with no camera init/driver ownership, no local frame copy, and no WiFi/Assistant calls.
- CM7_0 services capture immediately after each `app_run_once()`; CM7_1 PIT_CH2 only clears the flag and advances the consumer tick.
- Source masking uses only `Cy_SysInt_DisableIRQ(tcpwm_0_interrupts_59_IRQn)` and the matching enable. No global disable, NVIC CPUIntIdx3 mask, or pending clear was added.
- Removed `camera_debug_app` and moved `zf_device_mt9v03x.c` plus the three retained fixed camera symbols from CM7_1 to CM7_0.

## TDD evidence

The Task 2 static test was observed RED before production implementation with exactly 21 expected new failures. They covered missing producer/consumer modules and main/ISR wiring, wrong project ownership, retained CM7_1 driver/keep symbols, absent TCPWM59-only masking and exact copy/publish behavior, and the old debug app still present. Existing provenance, linker, MPU, and safety checks did not fail.

The authorized Task 1 regression was also observed RED: after a successful matching release, `last_consume_ms` remained `0` instead of the test heartbeat `222`. The minimal fix updates shared `consumed_count` and `last_consume_ms` only after the matching `READING` state and sequence are verified and the slot is released. The mismatch path remains unchanged and the added host test verifies that it does not record consumption.

After implementation, camera static, camera host, and foundation host tests passed.

## Fresh IAR build and map evidence

All three projects were cleaned and rebuilt with IAR 9.40.1:

| Core | Result | Errors | Warnings |
| --- | --- | ---: | ---: |
| CM0+ | Build succeeded | 0 | 0 |
| CM7_0 | Build succeeded | 0 | 3 pre-existing `Pe550` warnings |
| CM7_1 | Build succeeded | 0 | 0 |

Fresh map timestamps were local 02:30:38, 02:32:11, and 02:33:23. CM7_0 contains `camera_capture_producer.o`, `zf_device_mt9v03x.o`, `camera_finish_callback`, `mt9v03x_init`, `producer_diag`, and the three fixed camera objects. CM7_1 contains `camera_frame_consumer.o` and no linked camera driver/callback or WiFi/Assistant call.

Memory contracts remain exact: camera data is `0x28060000` size `0x10000`; shared control is `0x28080000` size `0x2000`; CM7_0 `HEAP_STACK` is `0x2807E000-0x2807FFFF`; CM7_1 ordinary placement begins at `0x28082000`. No overlap was found.

## Hardware evidence

IAR GUI downloads completed for all three fresh images and stopped at `main()`: CM0+ local 02:37:49, CM7_0 02:42:09, and CM7_1 02:43:47. Each displayed Errors 0 / Warnings 0.

Gate 1 ran on CM7_0 from UTC `2026-07-14T09:54:03.479Z` to `2026-07-14T09:56:02.079Z` (118.600 seconds):

| Diagnostic | Start | End |
| --- | ---: | ---: |
| `producer_diag.init_state` | `0x03` (`CAMERA_CAPTURE_INIT_OK`) | `0x03` |
| `producer_diag.frame_count` | 0 | 0 |
| `producer_diag.last_frame_ms` | 0 | 0 |
| `producer_diag.frame_valid` | 0 | 0 |
| `mt9v03x_finish_flag` | 0 | 0 |
| `mt9v03x_image[0][0]` | 0 | 0 |
| `mt9v03x_image[60][94]` | 0 | 0 |

Gate 1 therefore failed: initialization returned success, but no camera-finish event or completed frame was observed. This is the limit of the evidence; no unsupported DMA/GPIO/TCPWM root cause is asserted.

A controlled high-contrast scene-change result is `NOT RUN` because there was no completed frame to evaluate. Gate 2 is also `NOT RUN`, exactly as required after a zero-frame Gate 1. Consequently no consumer sequence, slot endpoint, frame-age, accounting-closure, or copy-duration runtime claim is made.

Wheel motor power was operator-confirmed OFF. Camera wiring and the servo all-90-degree reference were operator-confirmed. No motion, wheel, servo, or `LXY` command was issued. Safety/control fault variables were not independently watched, so the report does not claim a runtime no-fault observation.

## Disposition

Static ownership, host behavior, IAR builds, and map ownership are complete and passing. Hardware capture acceptance is not met, and cross-core runtime acceptance remains unproven because Gate 2 was correctly stopped. Final status is `DONE_WITH_CONCERNS`.

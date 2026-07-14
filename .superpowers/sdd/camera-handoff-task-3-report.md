# Camera Handoff Task 3 Report

## Outcome

Task 3 passes. CM7_1 now acquires the newest CM7_0 camera slot, exposes the approved read-only vision boundary, sends that same held slot through the existing Seekfree Assistant WiFi-SPI API, and releases it only after the synchronous send call returns. Seekfree Assistant displayed changing 188 x 120 grayscale imagery for more than 60 seconds, and the camera-disabled versus streaming control comparison met every required safety and timing threshold.

No vision algorithm, navigation command, wheel command, servo command, motion test, or `LXY` command was added or run. Wheel motor power remained off and the all-90-degree pose remained the safe reference.

## Implementation

- `APP_CAMERA_WIFI_ENABLE` is enabled for the CM7_1 consumer path.
- CM7_1 alone initializes WiFi, opens the TCP socket, selects `SEEKFREE_ASSISTANT_WIFI_SPI`, and configures one camera object for each fixed shared-memory slot.
- Reconnect attempts are gated by `APP_CAMERA_WIFI_RETRY_MS` (5000 ms), and no frame is acquired during a blocking reconnect attempt.
- The acquired inter-core view is converted to `camera_vision_frame_view_struct`. Processing duration is deliberately 0 us because Task 3 inserts no fake vision work and publishes no motion output.
- A fresh frame remains `READING` through `seekfree_assistant_camera_send()` and is released afterward. Stale frames are not sent.
- Diagnostics separate attach, WiFi, socket, stale/timeout, send-call, and release state.

## Test-first and software verification

The Task 3 static contract was observed RED with exactly ten new missing requirements before implementation and GREEN afterward. Final verification passed:

- camera API/ownership static contract;
- capture-service provenance/static contract;
- CM7 UART ownership static contract;
- camera handoff and inter-core foundation host tests compiled with GCC C11 `-Wall -Wextra -Werror`;
- IMU gyro numeric regression;
- servo 300 Hz integration regression;
- leg IK zero and IK-height regressions;
- `git diff --check`.

Fresh IAR 9.40.1 builds passed for all cores: CM0+ 0 errors/0 warnings, CM7_0 0 errors/3 unchanged pre-existing `Pe550` warnings, and CM7_1 0 errors/0 warnings. Map review kept the camera data/control reservations disjoint, retained MT9V03X ownership on CM7_0, and placed WiFi/Assistant ownership on CM7_1 without linking an MT9V03X implementation there.

## Hardware Gate 3

Formal window: local `2026-07-14 07:04:41.520 -07:00` to `07:08:37.712 -07:00` (236.192 s).

| Evidence | Start | End / delta |
| --- | ---: | ---: |
| Captured sequence | 16161 | 32842 |
| Latest sequence | 16160 | 32841 |
| Published count | 3155 | 6451 / +3296 |
| Consumed count | 3154 | 6450 / +3296 |
| Notify count | 3155 | 6451 / +3296 |
| No-free drops | 195 | 195 / +0 |
| Stale-ready drops | 1 | 1 / +0 |
| Invalid / timeout | 0 / 1 | 0 / 1 |
| Copy last/max | 82/94 us | 78/95 us |
| Send last/max | 19/210 ms | 19/210 ms |

Producer/consumer lag remained one frame, consumer age remained 0 ms, valid remained 1, sampled pixels changed, process last/max remained 0/0 us by design, and both slots ended `FREE`. TCP remained `Established`; hotspot RX increased by 46,539,623 bytes and TX by 389,544 bytes. Assistant displayed changing 188 x 120 imagery at approximately 9-10 FPS and about 225 kB/s. After final single-F5 recovery from the endpoint read, two consecutive viewer captures changed and the UI reported 11 FPS and 229,048 B/s while TCP was still established.

## Honest send semantics and blocking observation

The Seekfree send API returns `void`. Consequently, `sent_count` means only: a non-stale frame was acquired, the synchronous call returned, and release followed. It does not claim TCP acknowledgement or successful remote rendering. The established socket, interface byte growth, matching consumed progress, and visibly changing Assistant frames are independent end-to-end evidence.

The fresh streaming-control endpoint recorded `last_send_duration_ms=18` and `max_send_duration_ms=1468`. The maximum is reported as a startup or exceptional blocking-send observation, not hidden and not treated as an acknowledgement. The formal Gate 3 window separately held at 19/210 ms. With the two-slot latest-frame/no-queue design, a long synchronous send holds one slot `READING`; intermediate camera frames may be dropped and the other slot can contain only the newest publishable frame, but no stale-frame queue can grow.

## Control-regression comparison

| Metric | Camera disabled | Streaming | Threshold |
| --- | ---: | ---: | --- |
| Window duration | 247.870 s | 240.993 s | >= 60 s |
| Missed ticks | 0 | 0 | delta 0 |
| Max scheduler gap | 1 ms | 1 ms | <= baseline + 1 ms |
| Servo rate | 300.054 Hz | 300.055 Hz | within 1% of 300 Hz |
| IMU ages | 3/5/5 ms | 3/7/5 ms | < 30 ms |
| Copy last/max | 79/94 us | 78/93 us | max < 1 ms |
| Safety fault | 0 | 0 | no new fault |
| BLDC duties | 0 / 0 | 0 / 0 | remain zero |

CM7_0 heartbeat advanced throughout both windows. Streaming introduced no missed tick, scheduler-gap increase, servo-rate drift, IMU-age violation, source-mask overrun, safety fault, or motor duty.

## Repository and distribution boundary

The generated `project/iar/project_config/mt9v03x_cm0plus_capture_service.sim` sidecar was removed. The local hash-locked `.ewx` was not added to Task 3, pushed, packaged, copied, or externally distributed. This task creates only the approved local commit and performs no push or pull request.

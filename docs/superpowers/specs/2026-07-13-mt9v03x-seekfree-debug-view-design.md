# MT9V03X and Seekfree WiFi-SPI Debug View Design

**Date:** 2026-07-13
**Branch:** `codex/camera-gps-research`
**Status:** Approved design
**Related design:** `2026-07-12-camera-gps-wireless-architecture-design.md`

## 1. Decision and Purpose

The wireless image path exists only to display the MT9V03X image and later
visualize slalom/obstacle-avoidance results during development. It is not a
product video service, a control channel, or a general telemetry transport.

Use the existing Seekfree Assistant camera protocol over the source-compatible
Seekfree WiFi-SPI module. Do not implement the custom UDP frame fragmentation,
host reconstruction, retransmission, or video-session protocol described in
the earlier architecture design.

This document supersedes only the wireless-video portions of sections 7, 8,
14, 16, 17, 18, and 19 of the related design. It does not change:

- CM7_0 ownership of balance, motor, servo, safety, and motion arbitration;
- CM7_1 ownership of camera, vision, GNSS, and wireless peripherals;
- the shared-memory and inter-core command architecture;
- the planned GNSS data model or future navigation safety gates.

## 2. Scope

### Included

- Source-matched MT9V03X initialization and frame capture on CM7_1.
- Native 188 x 120, 8-bit grayscale frames.
- A stable frame for the future slalom vision algorithm.
- A separate debug snapshot sent with the existing Seekfree Assistant protocol.
- Optional left boundary, right boundary, and centerline overlays.
- A compile-time switch for removing the debug stream from normal operation.
- Camera, vision, WiFi, frame-drop, and timeout diagnostics.
- Motor-disabled and concurrent-load hardware acceptance tests.

### Excluded

- A custom UDP video protocol or a custom PC image receiver.
- Video compression, storage, retransmission, or guaranteed delivery.
- Using the WiFi link to close the balance or actuator-control loop.
- Simultaneous general-purpose remote control, GNSS upload, and debug video on
  the same WiFi socket in this phase.
- Implementing the slalom algorithm itself. This phase provides its frame and
  overlay interfaces.

## 3. Reused Source Components

The repository already contains:

- `libraries/zf_device/zf_device_wifi_spi.c/.h`;
- `libraries/zf_components/seekfree_assistant.c/.h`;
- `libraries/zf_components/seekfree_assistant_interface.c/.h`.

The source-matched CYT4BB library supplies:

- `zf_device_mt9v03x.c/.h`;
- an MT9V03X plus WiFi-SPI Seekfree Assistant demonstration;
- a cross-core MT9V03X demonstration.

The implementation restores or enables only files from that matching library
version and adapts core ownership and memory placement. It does not invent a
new camera register protocol.

The selected settings are:

| Item | Initial setting |
|---|---|
| Camera | MT9V03X |
| Pixel format | 8-bit grayscale |
| Resolution | 188 x 120 |
| Configured capture rate | 50 FPS |
| Wireless module | Seekfree WiFi-SPI |
| SPI | SPI0, 10 MHz |
| Display protocol | Seekfree Assistant camera protocol |
| Socket mode | TCP, matching the official demonstration |
| Initial debug display rate | 10 FPS |

The existing WiFi-SPI pin assignment remains SPI0 on P02.0 through P02.4 with
reset on P23.0. The MT9V03X pin assignment must be copied from the matching
CYT4BB source header and checked against the IAR project and every current pin
owner before hardware initialization is enabled.

## 4. Core Ownership and Runtime Boundary

All MT9V03X, WiFi-SPI, and Seekfree Assistant calls execute on CM7_1. CM7_0
never initializes or services these peripherals.

```text
MT9V03X
  -> capture/DMA buffer
  -> stable vision frame on CM7_1
       -> slalom vision algorithm
       -> debug snapshot + optional overlays
            -> Seekfree Assistant protocol
            -> WiFi-SPI TCP connection
            -> PC Seekfree Assistant display
```

The debug stream is a side effect of a completed frame. It cannot publish a
motion command, modify a control target, or write an actuator peripheral.

`APP_CAMERA_DEBUG_STREAM_ENABLE` controls compilation or initialization of the
WiFi/Assistant path. Disabling it leaves camera capture and vision processing
operational.

While this switch is enabled, the WiFi-SPI socket is dedicated to Seekfree
Assistant. GNSS and control data are not multiplexed into the camera stream.
Any later need for a combined wireless protocol requires a separate design.

## 5. Frame Ownership and Memory Budget

Use three logically separate 188 x 120 buffers:

| Buffer | Writer | Reader | Rule |
|---|---|---|---|
| `capture_buffer` | Camera capture path | Frame publication service | Never read while capture is active |
| `vision_buffer` | Frame publication service | Vision pipeline | Stable for one vision invocation |
| `debug_buffer` | Debug snapshot service | Seekfree Assistant/WiFi | Never overwrite during a send |

Each buffer is 22,560 bytes. The three buffers require 67,680 bytes before
alignment, leaving the remaining CM7_1 private SRAM for code data, vision
scratch space, stack, and diagnostics. The implementation must confirm the
actual IAR map-file budget before enabling all three.

Image buffers remain in CM7_1-private SRAM and never cross into the 8 KiB
inter-core shared region. Inter-core snapshots may carry only small frame
sequence, age, confidence, and health values.

## 6. Source Driver Portability Gate

The matching MT9V03X source currently fixes its temporary image and two state
variables at these addresses:

```text
mt9v03x_image_temp  0x28026024
mt9v03x_h_num       0x28006bf0
mt9v03x_w_num       0x28006bf2
```

Those addresses are in the CM7_0 memory area in this firmware architecture.
The driver must not be copied into CM7_1 unchanged.

Before normal integration, perform a bounded portability probe:

1. Reproduce the official MT9V03X capture behavior with the matching source and
   record initialization, interrupt, frame-counter, and image results.
2. Determine whether the fixed addresses are required by the underlying
   capture mechanism or are only IAR placement choices in the example.
3. If they are placement choices, replace them with aligned CM7_1-private
   linker allocation and verify identical capture behavior with data cache on.
4. If they are a hard hardware/library contract, stop the CM7_1 integration
   and revise the architecture explicitly. Do not silently write CM7_0 SRAM or
   reserve an undocumented cross-core aperture.

Passing this gate is mandatory before the camera can be considered integrated.

## 7. Capture, Vision, and Display Data Flow

The capture completion ISR performs only bounded work:

- clear or acknowledge the camera completion interrupt;
- perform the cache operation required by the verified capture buffer;
- record the completed frame sequence;
- set a frame-ready flag.

It does not run the vision algorithm, copy the debug frame, connect WiFi, or
send an image.

The cooperative CM7_1 service loop performs work in this order:

1. Publish the newest complete capture as a stable `vision_buffer` frame.
2. Run the vision pipeline once for that frame and record processing time.
3. When the debug period expires and the sender is idle, copy the stable frame
   to `debug_buffer`.
4. Copy the matching boundary/centerline result for the same frame sequence.
5. Call `seekfree_assistant_camera_send()` through the WiFi-SPI interface.

The first display period is 100 ms, or approximately 10 FPS. The implementation
may expose compile-time 5, 10, and 20 FPS choices, but 10 FPS is the initial
acceptance configuration.

The debug path is latest-frame-first:

- it has no frame queue;
- it never waits for an older frame;
- it drops a debug opportunity when the previous send is still active or when
  the available service budget is exceeded;
- it never delays publication of a new vision frame.

The current WiFi-SPI send API is synchronous. Therefore its calls remain in
the lowest-priority CM7_1 service and their duration is measured. A slow call
may lower display FPS, but cannot delay CM7_0 control.

## 8. Vision Overlay Contract

Phase 1 sends only the raw grayscale image.

Phase 2 configures up to three Seekfree Assistant boundaries:

- left track or obstacle boundary;
- right track or obstacle boundary;
- planned or detected centerline.

The overlay arrays and image snapshot must come from the same source frame
sequence. If the vision result is invalid, stale, or belongs to another frame,
send the image without overlays rather than displaying mismatched results.

The overlay contract contains validity, source frame sequence, point count,
and arrays compatible with `seekfree_assistant_camera_boundary_config()`.
The vision algorithm owns generation of these arrays; the debug service only
copies a stable result and configures the Assistant interface.

## 9. Startup and Reconnect State

CM7_1 starts the debug-view path without blocking CM7_0:

1. Initialize the CM7_1 scheduler and diagnostics.
2. Initialize MT9V03X and confirm complete frames.
3. Start vision-frame publication.
4. If debug streaming is enabled, initialize WiFi-SPI.
5. Connect to the PC-side TCP server configured for Seekfree Assistant.
6. Configure Assistant camera information and enter `STREAMING`.

No initialization step uses an infinite retry loop. A failed WiFi operation
changes the debug path to `RECONNECT_WAIT`; camera and vision continue. Retry
occurs at a bounded low rate, initially once per second.

## 10. Fault and Degradation Policy

| Condition | Required response |
|---|---|
| MT9V03X initialization fails | Keep vision invalid; report the error; retry at a bounded interval |
| No complete camera frame for 200 ms | Invalidate the current frame and vision result; count timeout; restart capture |
| Vision misses a frame | Process the newest complete frame; never build a backlog |
| WiFi initialization or TCP connection fails | Disable only display sending; keep camera and vision running |
| WiFi send fails or exceeds its budget | Drop that debug frame; record the error; enter reconnect when required |
| Display cannot keep up | Reduce effective display FPS by dropping debug frames |
| Overlay frame does not match image frame | Send raw image without overlays |
| Camera driver would access CM7_0 fixed RAM | Fail the portability gate; do not enable capture |

The vision pipeline must not continue producing steering output from an image
older than 200 ms. The later navigation design remains responsible for turning
that invalid result into a safe motion response.

## 11. Diagnostics

At minimum CM7_1 records:

- `camera_init_ok`;
- `camera_frame_seq`;
- `camera_fps`;
- `camera_frame_age_ms`;
- `camera_timeout_count`;
- `camera_last_error`;
- `vision_frame_seq`;
- `vision_process_ms`;
- `wifi_connected`;
- `wifi_reconnect_count`;
- `debug_frame_sent`;
- `debug_frame_dropped`;
- `debug_send_last_ms` and `debug_send_max_ms`.

These diagnostics may be exposed through the existing compact CM7_1 health
snapshot or inspected in the debugger. Adding a new general-purpose wireless
telemetry protocol is not required for this phase.

## 12. Verification and Acceptance

### Static and build checks

- Verify MT9V03X and WiFi-SPI pins have no conflict with current owners.
- Verify all camera and WiFi calls are linked only into CM7_1.
- Verify no camera buffer overlaps CM7_0 SRAM or inter-core shared SRAM.
- Inspect the IAR map for all three buffers, stack margin, and vision scratch
  margin.
- Build `cyt4bb7_cm_0_plus`, `cyt4bb7_cm_7_0`, and `cyt4bb7_cm_7_1` without new
  errors or warnings attributable to this integration.

### Hardware gates

1. **Driver portability probe:** With motors disabled, prove the source driver
   works and then prove the CM7_1-private allocation works with cache enabled.
2. **Raw capture:** Run MT9V03X alone at 188 x 120 and configured 50 FPS for at
   least 60 seconds. Confirm changing scene content, monotonically increasing
   frame sequence, and no unexplained timeout or buffer overwrite.
3. **Raw display:** Connect WiFi-SPI to the PC and show a continuous grayscale
   image in Seekfree Assistant at approximately 10 FPS.
4. **Overlay display:** Show left boundary, right boundary, and centerline from
   the same frame as the image; invalid overlays must disappear cleanly.
5. **Reconnect:** Disconnect the PC server or WiFi, confirm camera/vision
   continue, then confirm display returns after reconnection.
6. **Concurrent load:** Run camera, a bounded test vision workload, and display while CM7_0
   maintains its existing IMU, scheduler, servo, and motor services. Confirm no
   CM7_0 timing or actuator regression.
7. **Debug-disabled build:** Disable `APP_CAMERA_DEBUG_STREAM_ENABLE` and confirm
   camera/vision operation no longer depends on WiFi or the PC tool.

Record capture FPS, display FPS, camera timeouts, debug drops, maximum WiFi send
duration, CM7_0 scheduler maximum gap, and observed actuator behavior for the
hardware test report.

## 13. Implementation Sequence

1. Restore the matching MT9V03X driver and complete the memory-portability
   probe on CM7_1.
2. Add the CM7_1 camera wrapper, bounded ISR, stable frame publication, and
   diagnostics.
3. Add the debug snapshot buffer and compile-time stream switch.
4. Initialize WiFi-SPI and Seekfree Assistant on CM7_1 using the official TCP
   display flow.
5. Validate raw display and reconnect behavior.
6. Add the overlay contract and validate frame-sequence matching.
7. Run the concurrent-load and debug-disabled acceptance gates.

The first implementation milestone ends when a stable raw MT9V03X image is
visible in Seekfree Assistant without affecting CM7_0. Overlay and slalom
algorithm work starts only after that milestone passes.

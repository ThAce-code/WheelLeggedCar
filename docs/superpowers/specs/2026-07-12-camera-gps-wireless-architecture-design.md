# Camera, GPS, and Wireless Video Architecture

**Date:** 2026-07-12
**Branch:** `codex/camera-gps-research`
**Status:** Approved design

## 1. Purpose

Add source-compatible camera, GNSS, and wireless modules to the CYT4BB7
wheel-legged robot without disturbing the existing balance and actuator timing.
The camera must support both onboard perception and wireless video. The first
GNSS phase parses and uploads positioning data while reserving an interface for
later waypoint navigation.

The architecture keeps hard real-time control on CM7_0 and places perception,
navigation, and networking on CM7_1. CM0+ remains the boot core.

## 2. Existing Architecture

The current firmware has these relevant characteristics:

- CM0+ initializes the system and starts CM7_0 and CM7_1.
- CM7_0 runs the application, a 1 ms cooperative scheduler, 5 ms chassis and
  balance loops, a 1 ms motor service, and the servo timing path.
- CM7_0 owns UART0 for VOFA/debug and commands, UART1 for the dual BLDC link,
  SPI2 for the LSM6DSV16X IMU, and the servo PWM channels.
- `control_chassis` owns motion targets and limits; `control_balance` remains
  the normal owner of motor RPM output.
- `app_safety` already disables actuators for IMU or attitude faults.
- CM7_1 currently runs an empty main loop and therefore has capacity for the
  new non-control workload.
- The linker assigns about 384 KiB SRAM to CM7_0 and about 256 KiB to CM7_1.
- Both CM7 data caches are enabled.
- The source library provides GNSS support on UART2 and Wi-Fi support on SPI0.
  Camera type hooks and sensor configuration support are present; integration
  will enable the source-matched driver for the installed camera rather than
  creating a new sensor protocol.
- The current `ipc_send_data()` carries one `uint32` and may wait up to about
  5 ms. It is unsuitable as a real-time data transport.

## 3. Scope

### Included

- Camera capture on CM7_1 using the matched source driver and DMA path.
- Onboard vision processing on CM7_1.
- Wireless transmission of camera frames and diagnostic data.
- GNSS reception, parsing, health reporting, and wireless upload on CM7_1.
- A future-facing waypoint-navigation interface.
- A safe CM7_1-to-CM7_0 navigation command path.
- A CM7_0 command router that arbitrates local, remote, and autonomous inputs.
- Shared-memory layout, IPC notification, fault handling, and diagnostics.
- Static, IAR build, bench, and hardware acceptance gates.

### Excluded from the first implementation phase

- Closed-loop waypoint driving.
- SLAM, visual odometry, object classification, or neural-network inference.
- Video compression codecs that are not already supplied by the camera/module.
- Continuing autonomous motion after loss of the wireless supervisor.
- Changes to balance gains, leg geometry, motor limits, or existing pin owners.
- Rewriting vendor drivers. The source-matched camera files may be enabled or
  restored from the same library version without changing their wire protocol.

## 4. Architectural Decision

Use a dual-domain design:

```text
CM0+
  system initialization and dual-core boot

CM7_0 - hard real-time control domain
  IMU -> safety -> leg/chassis/balance -> motor and servo actuators
  local UART command path
  inter-core command receiver and motion command router

CM7_1 - perception and communications domain
  camera DMA -> vision pipeline -> navigation manager
             -> video streamer -> SPI Wi-Fi
  UART2 GNSS -> GNSS snapshot -> navigation manager and telemetry
  wireless commands -> command validator -> navigation manager
```

CM7_1 never writes motor, servo, balance, or low-level chassis state directly.
It publishes a normalized navigation request. CM7_0 validates and arbitrates
that request before calling the existing chassis interface.

Raw image buffers never cross cores. Only small, versioned snapshots cross the
shared-memory boundary.

## 5. Core and Peripheral Ownership

| Resource | Owner | Purpose |
|---|---|---|
| System initialization | CM0+ | Clock/power startup and launching both CM7 cores |
| UART0, P00.0/P00.1 | CM7_0 | VOFA telemetry, host commands, maintenance |
| UART1, P04.0/P04.1 | CM7_0 | Dual BLDC command and feedback |
| SPI2 and IMU INT1 P19.3 | CM7_0 | LSM6DSV16X |
| Servo PWM channels | CM7_0 | Four leg servos |
| Camera GPIO, VSYNC, pixel clock, DMA | CM7_1 | Frame acquisition |
| UART2, P10.0/P10.1 | CM7_1 | Source-supported GNSS module |
| SPI0, P02.0-P02.4, P23.0 | CM7_1 | Source-supported Wi-Fi module |
| PIT_CH0 and PIT_CH1 | CM7_0 | 1 ms application tick and servo timing |
| PIT_CH2 | CM7_1 | 1 ms perception/communications tick |

CM7_1 must not call `debug_info_init()` or any other initializer that claims
UART0. Its diagnostics go through the wireless link and inter-core health
snapshot. CM7_0 retains the existing UART0 VOFA path.

## 6. Module Boundaries

### Shared application modules

- `intercore_protocol.h`
  - Fixed-width message types, version, magic, source enumeration, and fault
    codes.
  - Contains no pointers or core-private addresses.
- `intercore_transport.c/.h`
  - Shared-memory initialization, snapshot publication, snapshot validation,
    event-ring access, barriers, and best-effort IPC notification.
  - Compiled into both CM7 projects.

### CM7_0 additions

- `motion_command_router.c/.h`
  - Owns source priority, source timeout, maintenance-mode exclusion, finite
    checks, and final delivery to `control_chassis_set_cmd()`.
- `intercore_control.c/.h`
  - Reads the latest CM7_1 command, validates sequence/version/CRC, and submits
    it to the motion command router.
  - Publishes a compact control and safety status snapshot back to CM7_1.

### CM7_1 additions

- `perception_app.c/.h`
  - CM7_1 initialization, state machine, and nonblocking main-loop service.
- `perception_scheduler.c/.h`
  - Owns the 1 ms CM7_1 tick and periodic task release.
- `sensor_camera.c/.h`
  - Wraps the source-compatible camera driver, frame buffers, DMA completion,
    timestamps, ownership, and camera diagnostics.
- `vision_pipeline.c/.h`
  - Consumes the newest complete frame and publishes a compact perception
    result. The first algorithm may be minimal, but the interface is stable.
- `sensor_gnss.c/.h`
  - Wraps `gnss_init()`, `gnss_uart_callback()`, and `gnss_data_parse()` into a
    timestamped immutable snapshot.
- `navigation_manager.c/.h`
  - Combines validated perception, GNSS, remote mode, and future waypoint data.
  - Publishes only normalized forward-RPM and yaw-rate targets.
- `wireless_link.c/.h`
  - Owns Wi-Fi initialization, socket state, receive parsing, transmit
    priorities, reconnect state, and link diagnostics.
- `video_stream.c/.h`
  - Fragments frames, applies rate limits, and drops stale/incomplete work.

Core-specific main and ISR changes stay in `project/user`. Application modules
stay in `project/code` and are added only to the relevant IAR core project.

## 7. CM7_1 Scheduling Model

CM7_1 uses the existing bare-metal cooperative style rather than introducing an
RTOS. PIT_CH2 releases a 1 ms tick. Work runs in this priority order:

1. Finalize completed camera DMA and arm the next buffer.
2. Parse pending GNSS data.
3. Run vision for the newest complete frame.
4. Update navigation and publish a command if enabled.
5. Read and validate wireless commands.
6. Send command acknowledgements and health data.
7. Send GNSS, vision, and control telemetry.
8. Send at most one video fragment per service round.

ISRs only move bytes, switch DMA state, set flags, record sequence numbers, and
capture timestamps. They do not parse GNSS sentences, run vision, assemble
packets, or transmit video.

The current Wi-Fi SPI API is synchronous. All of its calls remain on CM7_1. A
video service call submits one bounded fragment, records the elapsed time, and
returns to higher-priority work. A long or failed Wi-Fi transfer can reduce or
stop video but cannot delay CM7_0.

## 8. Camera Buffering and Vision

Use three frame buffers when this memory condition holds:

```text
3 * camera_image_size + vision_scratch + stack_margin <= CM7_1 usable SRAM
```

Otherwise use two frame buffers and restrict the vision scratch area. The
implementation must calculate the budget from the selected driver's
`*_IMAGE_SIZE`; it must not assume a particular camera resolution.

Each buffer has a capture state (`FREE`, `CAPTURING`, or `READY`) plus separate
`VISION` and `STREAMING` consumer-reference bits. Vision and streaming may
reference the same completed frame, but capture may reuse it only after both
references are released. If no buffer is available, the system drops the oldest
`READY` frame with no consumer reference and increments a drop counter. If all
completed frames are still referenced, it drops the incoming frame instead of
overwriting data in use.

Processing is latest-frame-first:

- Vision never builds an unbounded queue.
- Video never delays vision.
- The navigation manager accepts a perception result only when its age is less
  than 200 ms and its `valid` flag is set.
- The first performance target is at least 20 valid vision frames per second
  with perception age below 100 ms.

## 9. GNSS Data Model

`sensor_gnss` converts the library's mutable global state into a snapshot with:

- local receive timestamp and parsed UTC time;
- fix-valid flag and data age;
- latitude, longitude, height, ground speed, and course;
- satellite count;
- dual-antenna direction state and heading when supplied by the module;
- parse, checksum, timeout, and invalid-fix counters.

The UART2 ISR only feeds the source driver. The CM7_1 scheduler calls
`gnss_data_parse()` outside the ISR.

In the first phase GNSS cannot enable or change motion. It is uploaded for
inspection and exposed to `navigation_manager` through a read-only interface.
Waypoint mode remains disarmed when the fix is invalid or older than 500 ms.

## 10. Navigation Command Contract

The CM7_1-to-CM7_0 payload contains:

```c
typedef struct
{
    float forward_rpm;
    float turn_rate_dps;
    float confidence;
    uint32 source_sequence;
    uint16 valid_for_ms;
    uint8 enable;
    uint8 source;
    uint8 mode;
    uint8 stop_reason;
    uint8 reserved[2];
} navigation_command_struct;
```

Initial limits are enforced independently on both cores. CM7_1 applies
navigation limits; CM7_0 still applies the existing chassis, leg-height, and
safety limits. CM7_1 cannot select fast mode or alter balance gains.

Commands are published at 20 Hz when a navigation or wireless-control mode is
active. CM7_0 checks the shared sequence from its 1 ms scheduler and consumes a
valid command before the next 5 ms chassis update. CM7_0 measures freshness
from its own local receive time, not from the CM7_1 clock.

No valid update for 200 ms causes a chassis stop. This zeros forward and yaw
targets while leaving the balance loop active so the robot can remain upright.

## 11. Motion Command Arbitration

`motion_command_router` applies this priority:

1. Local safety fault.
2. Local or wireless emergency stop.
3. UART0 maintenance command.
4. Wireless manual command.
5. Autonomous vision/GNSS command.

Only the router calls `control_chassis_set_cmd()` for normal motion. The UART0
`C` command and STOP path are moved through the router.

Existing direct motor, leg, servo, and calibration commands remain available
only in an explicit maintenance mode. Entering maintenance mode clears the
active CM7_1 source and blocks new autonomous commands. Leaving maintenance
mode requires a fresh arm command; stale navigation commands cannot resume
motion.

## 12. Shared SRAM and Cache Coherency

Reserve 8 KiB at `0x28080000`, the current start of the CM7_1 SRAM allocation.
Advance the CM7_1 private SRAM base to `0x28082000`, leaving about 248 KiB for
its code data, image buffers, scratch space, heap, and stack.

| Shared offset | Size | Purpose |
|---|---:|---|
| `0x000` | 256 B | Layout magic, version, boot epochs, active indices |
| `0x100` | 512 B | Two CM7_1-to-CM7_0 navigation snapshot slots |
| `0x300` | 1024 B | Two CM7_0-to-CM7_1 control-status snapshot slots |
| `0x700` | 1024 B | Sixteen 64-byte low-frequency event entries |
| `0xB00` | 256 B | Core heartbeat and transport diagnostics |
| `0xC00` | 5120 B | Reserved for compatible protocol growth |

The linker reserves this range for all three core images so no normal section
can overlap it. Both CM7 cores configure the aligned 8 KiB MPU region as
shareable and non-cacheable before accessing it. The transport still uses
`__DMB()` around publication and consumption. `volatile` alone is not treated
as a coherency mechanism.

CM7_0 owns initial shared-layout clearing and increments a boot epoch. CM7_1
waits for the expected magic and version, records the epoch, and then starts
publishing. A changed epoch invalidates every previously received command.

## 13. Snapshot and IPC Protocol

Every record starts with:

```c
typedef struct
{
    uint32 magic;
    uint16 version;
    uint16 type;
    uint32 size;
    uint32 sequence;
    uint32 source_ms;
    uint32 crc32;
} intercore_header_struct;
```

Snapshot publication uses inactive-slot write followed by CRC, memory barrier,
active-index update, and sequence update. The consumer copies a slot and
accepts it only when the active index and sequence remain unchanged and the
version, size, type, and CRC are valid.

IPC is a best-effort doorbell only. A new nonblocking `intercore_notify_try()`
attempts `Cy_IPC_Pipe_SendMessage()` once and never waits for release. If the
pipe is busy, the notification is dropped; the consumer's periodic sequence
poll still finds the snapshot. The real-time path does not call the existing
blocking `ipc_send_data()`.

## 14. Wireless Protocol

Use the source SPI0 Wi-Fi driver in UDP mode. One socket carries typed packets:

- `VIDEO_CHUNK`
- `TELEMETRY`
- `GNSS`
- `VISION_RESULT`
- `REMOTE_COMMAND`
- `COMMAND_ACK`
- `HEARTBEAT`
- `FAULT`

Each video payload is at most 1200 bytes and contains protocol version, frame
sequence, chunk index, chunk count, width, height, pixel format, capture time,
payload length, and CRC. Video is not retransmitted. The host drops an
incomplete frame and waits for the next frame sequence.

Low-bandwidth commands contain a session identifier, monotonic sequence,
operation, arguments, and CRC. They receive an acknowledgement and use finite
host-side retries. Duplicate command sequences return the previous result
without executing twice. Motion remains disarmed until an explicit arm command
for the current boot session is accepted.

This first phase is restricted to a trusted laboratory WLAN. CRC detects
corruption but does not provide cryptographic authentication.

Transmit priority is:

```text
emergency stop and command acknowledgement
  > fault and heartbeat
  > GNSS, perception, and control telemetry
  > video chunks
```

Congestion first reduces video frame rate and then drops video frames. It never
queues stale navigation commands or suppresses a stop/fault message.

The first video acceptance target on the laboratory WLAN is at least 10 FPS at
the configured native source resolution with median end-to-end latency below
250 ms.

## 15. Startup and Runtime States

### CM7_0

1. Initialize clock, UART0, existing application state, safety, control, and
   actuators.
2. Initialize the shared layout and local inter-core consumer.
3. Start PIT_CH0/PIT_CH1 and the existing scheduler.
4. Publish `CONTROL_READY` without waiting for CM7_1.
5. Remain stopped or in the explicitly selected local balance mode.

### CM7_1

1. Initialize clock without claiming UART0.
2. Configure shared-region MPU attributes and wait for the supported layout.
3. Start PIT_CH2 and the CM7_1 scheduler.
4. Initialize camera and confirm frame acquisition.
5. Initialize GNSS and begin parsing.
6. Initialize Wi-Fi, connect the UDP socket, and start heartbeats.
7. Enter `STREAM_ONLY` after camera/Wi-Fi readiness.
8. Enter `NAV_ARMED` only after an explicit arm request and all required
   sources are healthy.

Wi-Fi initialization and reconnect may block CM7_1, but never CM7_0. Every
power cycle and every CM7_1 reboot clears navigation arm state.

## 16. Fault and Degradation Policy

| Condition | Response |
|---|---|
| CM7_1 heartbeat stale for 200 ms | CM7_0 zeros chassis targets and keeps balance active |
| Command version, size, CRC, finite, range, or sequence error | Reject, count, and stop after the last valid command expires |
| Camera has no new frame for 200 ms | Invalidate perception; stop vision-dependent navigation; keep GNSS upload |
| GNSS fix invalid or older than 500 ms | Mark invalid; keep first-phase streaming; block waypoint mode |
| Wi-Fi disconnects | Stop streaming; stop wireless/manual navigation; run reconnect state |
| Video transmit overload | Reduce FPS, then drop video frames |
| CM7_0 enters `APP_STATE_FAULT` | Existing actuator shutdown dominates; CM7_1 stops commands and sends `FAULT` |
| CM7_1 restarts | Boot epoch invalidates old commands; handshake and arm are required again |
| Shared protocol version mismatch | Disable inter-core navigation; preserve independent CM7_0 control |

Offline autonomous continuation is disabled. Adding it later requires a new
explicit mode and a separate safety review.

## 17. Diagnostics

CM7_1 records and uploads:

- camera init result, frame sequence, capture FPS, stale age, capture drops,
  vision drops, stream drops, and buffer-starvation count;
- vision valid/confidence/source age and processing duration;
- GNSS fix, satellites, data age, parse errors, and last valid position;
- Wi-Fi state, reconnect count, TX bytes, RX bytes, packet errors, and maximum
  synchronous call duration;
- IPC publish/receive counts, busy-doorbell drops, CRC errors, version errors,
  duplicate sequences, and heartbeat ages;
- active command source, arm state, stop reason, and last command age.

CM7_0 keeps its current VOFA telemetry. A compact reverse snapshot gives CM7_1
the application state, safety fault, pitch, pitch rate, wheel RPM, active motion
source, command age, scheduler missed-tick count, and scheduler maximum gap.

## 18. Verification and Acceptance Gates

### Static verification

- CM7_1 modules do not include actuator or balance private headers.
- CM7_0 does not initialize camera, GNSS, or Wi-Fi peripherals.
- Only the motion router submits normal chassis motion commands.
- The real-time path does not call `ipc_send_data()` or a Wi-Fi API.
- Shared structures contain no pointers and fit their reserved slots.
- All shared records and offsets are 32-byte aligned.
- IAR project membership matches core ownership.

### Build verification

Build all affected IAR projects:

- `cyt4bb7_cm_0_plus`
- `cyt4bb7_cm_7_0`
- `cyt4bb7_cm_7_1`

Record code size, SRAM size, warnings, and the selected camera image-memory
budget.

### Hardware gates

1. **CM7_0 baseline:** Run with CM7_1 features disabled. Confirm IMU, heartbeat,
   UART0, BLDC feedback, servo behavior, safety, and scheduler diagnostics are
   unchanged.
2. **Individual CM7_1 drivers:** With motors disabled, validate GNSS, camera,
   and Wi-Fi separately.
3. **Perception:** Run camera DMA and vision continuously. Achieve at least 20
   valid perception frames per second and perception age below 100 ms.
4. **Video:** Achieve at least 10 FPS and median latency below 250 ms on the lab
   WLAN. Packet loss may drop a frame but must not create an accumulating queue.
5. **Inter-core negative tests:** Inject valid, duplicate, stale, out-of-order,
   wrong-version, non-finite, out-of-range, and bad-CRC commands.
6. **Concurrent load:** Run camera, vision, GNSS, and maximum configured video
   load. CM7_0 scheduler maximum gap must remain no more than 2 ms, with no
   balance, safety, or motor-feedback regression.
7. **Bench fault tests:** On a stand with motor output controlled, disconnect
   CM7_1, camera, GNSS, and Wi-Fi in turn. Verify the specified stop/degrade
   response and diagnostics.
8. **Low-speed rollout:** Test wireless manual motion first, vision assistance
   second, and GPS waypoint behavior only in a later implementation phase.

Every gate records camera FPS and drops, GNSS age, Wi-Fi maximum blocking time,
IPC errors, CM7_0 scheduler gap, command latency, and stop response time.

## 19. Implementation Decomposition

This architecture is implemented as ordered subprojects so each safety boundary
is independently testable:

1. Shared SRAM, nonblocking notification, and CM7_0 motion command router.
2. CM7_1 application skeleton, PIT_CH2 scheduler, health heartbeat, and reverse
   CM7_0 diagnostics.
3. GNSS driver wrapper and wireless GNSS/health upload with motion disabled.
4. Camera driver integration, DMA buffering, and camera diagnostics.
5. Vision-pipeline interface and first bounded vision algorithm.
6. UDP protocol, command validation, telemetry, and wireless manual control.
7. Rate-limited video streaming and host frame reconstruction.
8. Navigation-manager integration and closed-loop safety gates.
9. Waypoint navigation as a separate, later design and implementation plan.

No subproject proceeds to motor-enabled hardware testing until the preceding
build, static, and motor-disabled hardware gates pass.

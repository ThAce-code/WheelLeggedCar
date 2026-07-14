# MT9V03X Cross-Core Handoff Design

**Date:** 2026-07-14
**Status:** Approved architecture, pending implementation plan
**Branch:** `codex/camera-gps-research`

## Background

The unchanged Seekfree MT9V03X driver initializes on CM7_1 but does not deliver frames there. The Task 4 hardware gate ran CM7_1 for approximately 448.479 seconds with `init_state == CAMERA_DEBUG_INIT_OK`, while `frame_count`, `snapshot_count`, `last_frame_ms`, `mt9v03x_finish_flag`, and sampled pixels remained zero. The same source-matched driver produced changing frames on CM7_0 in the reference gate. This triggered STOP B in the original camera plan.

The fallback therefore keeps camera capture on CM7_0 and moves frame consumption, future vision processing, and WiFi-SPI Assistant output to CM7_1.

## Goals

- Capture MT9V03X frames on CM7_0 with the unchanged source-matched driver.
- Hand complete 188 x 120 grayscale frames to CM7_1 without tearing.
- Let CM7_0 continue running IMU, scheduler, servo, safety, and control work.
- Let CM7_1 own future slalom vision processing and the existing WiFi-SPI Assistant display.
- Use a 64 KiB non-cacheable two-slot data plane with a latest-ready, no-FIFO policy.
- Keep the existing 8 KiB inter-core control plane and the CM7_1 ordinary SRAM base at `0x28082000`.
- Preserve the camera-debug wheel-duty hard lock and P06_5 bus-current ADC exclusion.
- Produce measurable frame, drop, age, copy-time, processing-time, send-time, scheduler, IMU, and servo evidence.

## Non-Goals

- Do not rewrite the MT9V03X DMA, GPIO, TCPWM, or pixel-capture implementation.
- Do not move capture ownership back to CM7_1.
- Do not implement the slalom, boundary, centerline, or pile-recognition algorithm in this milestone.
- Do not add a FIFO frame queue, compression, retransmission, or a custom network protocol.
- Do not re-enable wheel motion during camera integration.
- Do not expand the existing 8 KiB inter-core control plane or move the CM7_1 SRAM base.
- Do not support an independent CM7_0 reset while CM7_1 is actively reading a frame; a producer reset invalidates the camera data plane and requires consumer reattachment.

## Selected Architecture

```text
MT9V03X
   |
   v
CM7_0 camera callback
   |  unchanged driver copy into mt9v03x_image
   v
CM7_0 producer service after app_run_once()
   |  at most one publication opportunity per 100 ms
   v
64 KiB non-cacheable two-slot camera data plane
   |
   v
CM7_1 consumer service
   |-- future vision processing on the acquired frame
   |-- WiFi-SPI Assistant send using the same acquired frame
   v
Existing 8 KiB control plane
   |  compact vision/navigation result only
   v
CM7_0 motion router and safety logic
```

CM7_0 never initializes WiFi or calls Assistant send functions. CM7_1 never initializes the camera or links the MT9V03X implementation file. The MT9V03X header may remain visible to CM7_1 for image dimension constants.

## Alternatives Considered

### 32 KiB single slot

This most closely follows the E8_09 reference and is the simplest handshake. It was rejected for the selected architecture because CM7_1 will eventually perform both vision processing and synchronous WiFi sending. A single slot prevents CM7_0 from publishing another complete frame for the entire processing-and-send interval.

### Expand the existing 8 KiB shared region

A 22,560-byte image cannot fit in the current 8 KiB layout. Expanding that region would move CM7_1 ordinary SRAM from `0x28082000`, invalidate existing maps and tests, and couple navigation/control records to large image storage. This option was rejected.

### Direct CM7_1 capture

This was tested and failed under STOP B. Making it work would require a driver-level interrupt/DMA/TCPWM port, which is outside the accepted risk boundary.

## Memory Layout

The existing control plane remains unchanged in address and total size:

| Region | Range | Purpose |
|---|---:|---|
| Inter-core control | `0x28080000-0x28081FFF` | Navigation, control status, events, health, camera metadata, doorbell |
| CM7_1 ordinary SRAM | starts at `0x28082000` | CM7_1 code/data/stack |

The new camera data plane is carved out of CM7_0 SRAM and explicitly removed from ordinary linker allocation:

| Region | Range | Size |
|---|---:|---:|
| Camera data plane | `0x28060000-0x2806FFFF` | 64 KiB |
| Slot 0 pixels | `0x28060000-0x2806581F` | `0x5820` bytes |
| Slot 1 pixels | `0x28065820-0x2806B03F` | `0x5820` bytes |
| Reserved camera space | `0x2806B040-0x2806FFFF` | `0x4FC0` bytes |

Both slots are 32-byte aligned, and `0x5820` is a multiple of 32 bytes. The linker must export camera base and size symbols and split CM7_0 ordinary SRAM around the 64 KiB hole. CM7_0 heap and stack stay in the continuous range above the hole, so the existing stack-at-SRAM-end behavior is preserved.

Both CM7 cores configure the 8 KiB control plane and the 64 KiB camera data plane in one `Cy_MPU_Setup()` call. Both regions use `CY_MPU_ATTR_NORM_SHR_MEM_NC`, full privileged access, execute-never, and correctly aligned MPU sizes. Calling `Cy_MPU_Setup()` twice is forbidden because the SDK setup function clears previously configured regions.

## Control-Plane Layout

The protocol version advances from 1 to 2. Existing offsets through health remain unchanged. A 256-byte camera control block is inserted at offset `0xC00`; the following reserved byte array shrinks from 5,120 to 4,864 bytes so the total shared layout remains exactly 8,192 bytes. The final doorbell address remains unchanged.

The camera control block is exactly 256 bytes and contains:

```c
typedef struct
{
    uint32 state;
    uint32 sequence;
    uint32 capture_ms;
    uint32 publish_ms;
    uint32 frame_bytes;
    uint32 reserved[3];
} intercore_camera_slot_struct; /* 32 bytes */

typedef struct
{
    uint32 magic;
    uint16 version;
    uint16 format;
    uint16 width;
    uint16 height;
    uint16 stride;
    uint16 slot_count;
    uint32 frame_bytes;
    uint32 producer_boot_epoch;
    uint32 capture_sequence;
    uint32 latest_published_sequence;
    intercore_camera_slot_struct slot[2];
    uint32 captured_count;
    uint32 published_count;
    uint32 no_free_drop_count;
    uint32 stale_ready_drop_count;
    uint32 consumed_count;
    uint32 invalid_layout_count;
    uint32 timeout_count;
    uint32 last_capture_ms;
    uint32 last_publish_ms;
    uint32 last_consume_ms;
    uint32 producer_heartbeat_ms;
    uint32 consumer_heartbeat_ms;
    uint32 last_copy_duration_us;
    uint32 max_copy_duration_us;
    uint32 last_send_duration_ms;
    uint32 max_send_duration_ms;
    uint32 notify_count;
    uint32 last_process_duration_us;
    uint32 max_process_duration_us;
    uint8 reserved[84];
} intercore_camera_control_struct; /* 256 bytes */
```

The camera magic is `0x43414D52UL` (`CAMR`), camera-control version is 1, format 1 means unsigned 8-bit grayscale, width is 188, height is 120, stride is 188, slot count is 2, and frame bytes is `0x5820`.

Compile-time layout assertions cover both structure sizes, the camera offset `0xC00`, the following reserved offset `0xD00`, total size 8,192, and the unchanged navigation/control/event/health offsets.

## Slot Ownership and State Machine

Each slot has one of four aligned 32-bit states:

```text
FREE -> WRITING -> READY -> READING -> FREE
```

The transitions deliberately have a single writer:

- CM7_0 performs only `FREE -> WRITING -> READY`.
- CM7_1 performs only `READY -> READING -> FREE`.
- CM7_0 never overwrites `READY` or `READING`.
- CM7_1 never changes `FREE` or `WRITING`.

This removes the need for a cross-core compare-and-swap in the first implementation.

CM7_0 selects a `FREE` slot only when a complete source frame is available and at least 100 ms has elapsed since the previous publication start. If neither slot is free, it increments `no_free_drop_count` and returns immediately. It never waits.

CM7_1 examines all `READY` slots and chooses the highest sequence. If both slots are ready, it releases the older slot without processing and increments `stale_ready_drop_count`. It marks the selected slot `READING`, executes the consumer pipeline, and releases it only after every pointer user has returned.

This is a latest-ready policy, not a FIFO. At most one slot is being read and one complete newer slot is waiting. Capture events that arrive while both slots are occupied are dropped and counted.

All pixel writes and metadata writes complete before CM7_0 publishes `READY` with a data memory barrier. CM7_1 uses a data memory barrier after observing `READY` and before reading pixels. CM7_1 finishes all reads before publishing `FREE` with a data memory barrier.

## CM7_0 Capture Producer

CM7_0 initializes the unchanged MT9V03X driver after the existing application and safety initialization has succeeded. Camera failure does not bypass IMU or safety initialization. The producer service runs in the main loop after `app_run_once()` and never from PIT, EXTI, IPC, or camera interrupt context.

The source driver callback copies the completed capture buffer into `mt9v03x_image`. A second camera completion interrupt could otherwise replace `mt9v03x_image` halfway through the producer's handoff copy. `CPUIntIdx3_IRQn` cannot be masked at the NVIC level because both the MT9V03X TCPWM59 system source and UART system sources map to that aggregate CPU interrupt index. Therefore the producer masks only the camera's system interrupt source:

1. Calls `Cy_SysInt_DisableIRQ(tcpwm_0_interrupts_59_IRQn)` to invalidate only the TCPWM59-to-CPU route.
2. Copies exactly `MT9V03X_IMAGE_SIZE` bytes from `mt9v03x_image[0]` into the selected shared slot.
3. Publishes the slot metadata and `READY` state.
4. Calls `Cy_SysInt_EnableIRQ(tcpwm_0_interrupts_59_IRQn)` without clearing the TCPWM59 peripheral flag or the aggregate CPU pending state.

The implementation must not call `NVIC_DisableIRQ(CPUIntIdx3_IRQn)`, because that would also mask UART sources mapped to the same CPU index. Global interrupts are never disabled. PIT, servo, IMU, UART, and other external interrupts remain serviceable during the handoff copy. A camera completion that arrives while its system route is invalid remains pending at its source and is handled after the route is restored. Copy duration and maximum source-mask duration are measured.

CM7_0 may send a one-way `INTERCORE_NOTIFY_CAMERA_READY` hint after publishing. The shared slot state and sequence remain the source of truth, so lost or coalesced notifications do not lose frames. CM7_1 does not send a camera-release doorbell; CM7_0 observes `FREE` by polling. This avoids adding another bidirectional use of the existing single shared doorbell object.

## CM7_1 Consumer, Vision Boundary, and WiFi

CM7_1 attaches to the control and camera data planes, validates magic, versions, dimensions, stride, slot count, frame size, and producer boot epoch, then polls or consumes the camera-ready hint.

The consumer exposes an acquired-frame view containing the slot index, sequence, capture timestamp, width, height, stride, byte count, and a pointer to the non-cacheable pixels. The future slalom vision module will execute on this acquired view before WiFi sending and will publish only a compact `navigation_command_struct` back through the existing 8 KiB transport. The current milestone verifies the acquired-frame boundary and WiFi display; it does not implement or fake a vision result.

When WiFi is connected, CM7_1 keeps the selected slot in `READING` for the complete synchronous `seekfree_assistant_camera_send()` call because that API retains the supplied pixel pointer until it returns. It then records send duration and releases the slot. `sent_count` means that the void send call returned; it does not claim per-frame network acknowledgement.

WiFi initialization and reconnect attempts occur only on CM7_1, no faster than once every 5 seconds. A disconnected consumer does not hold a slot during a blocking reconnect attempt. It releases or skips acquired frames, records the transport state, and retries networking separately.

## Initialization and Recovery

- CM7_0 owns initialization of the camera control block and slots. It writes dimensions, format, counters, and `FREE` states, executes a memory barrier, and publishes the camera magic last.
- CM7_1 does not consume until all layout fields and the producer boot epoch match.
- On CM7_1 startup, stale `READING` states left by the previous CM7_1 instance may be returned to `FREE`; `WRITING` is never reclaimed by CM7_1 and `READY` remains consumable.
- CM7_0 never reclaims a `READING` slot solely from a timeout because CM7_1 may still be inside a synchronous send. A hung consumer therefore causes counted frame drops rather than producer overwrite.
- A CM7_0 boot reinitializes the camera plane and changes the producer epoch. CM7_1 stops consumption and reattaches when the epoch changes.
- Independent CM7_0 reset during an active CM7_1 read is outside this milestone's supported behavior because CM7_0 startup can clear its SRAM range. Hardware tests use coordinated three-core startup and do not reset CM7_0 independently while streaming.
- Invalid magic, version, dimensions, slot state, or frame size prevents acquisition and increments diagnostic counters. It never falls back to an unchecked pointer.
- A frame older than 200 ms is not used as a future navigation input and is counted stale. The display path does not repeatedly resend stale data.

## Safety and Real-Time Boundaries

- `APP_CAMERA_DEBUG_ONLY` remains 1 for this milestone.
- Both BLDC duty values remain forced to zero at `actuator_motor_send_duty()` before the FOC UART call.
- P06_5 bus-current ADC initialization and conversion remain compiled out, with derived bus-current values forced to benign zero.
- Servo PWM remains active only for the previously validated all-90-degree reference pose.
- CM7_0 performs no WiFi or Assistant work.
- CM7_0 performs the additional 22,560-byte handoff copy only in its main loop and no faster than 10 times per second.
- CM7_1 processing or WiFi stalls cannot block CM7_0 control; they can only occupy slots and increase camera drop counters.
- No wheel motion or `LXY` command is executed during these gates.

## Verification Strategy

### Static and host checks

- Exact control-structure sizes and offsets remain compile-time checked.
- The 64 KiB image region and both slots are exactly aligned and sized.
- CM7_0 ordinary SRAM excludes `0x28060000-0x2806FFFF`; heap and stack stay above it.
- CM7_1 ordinary SRAM still begins at `0x28082000`.
- Both MPU regions are configured in one setup call and are non-cacheable, shared, and execute-never.
- CM7_0 owns the MT9V03X implementation and fixed symbols; CM7_1 does not link the implementation.
- CM7_1 owns WiFi-SPI and Assistant send calls; CM7_0 contains no reachable send path.
- State-machine tests cover normal handoff, consumer choosing the newest of two READY slots, stale READY release, no-free drops, no overwrite of READING, invalid layout, epoch change, and CM7_1 restart cleanup.
- Existing UART ownership, IMU calibration, servo 300 Hz, leg IK, inter-core transport, and notification tests remain passing.

### Build and map gate

Fresh-build CM0+, CM7_0, and CM7_1 with IAR 9.40.1. The maps must prove:

- The existing MT9V03X absolute objects retain their approved addresses.
- The camera data plane is exactly `0x28060000-0x2806FFFF` and absent from ordinary allocations.
- Shared control remains `0x28080000-0x28081FFF`.
- CM7_1 ordinary SRAM starts at `0x28082000`.
- CM7_0 heap and stack do not overlap the camera plane.
- CM7_1 contains no MT9V03X implementation or camera interrupt path.

### Hardware Gate 1: CM7_0 capture

Before enabling the consumer, run CM7_0 capture for at least 60 seconds. Require successful camera initialization, continuously increasing completed-frame count, controlled scene response in sampled pixels, wheel duty fixed at zero, and no change to the safe servo reference. If the unchanged driver does not produce frames in the robot three-core firmware on CM7_0, stop before cross-core or WiFi work.

### Hardware Gate 2: cross-core handoff without WiFi

Run both cores for at least 60 seconds. Require increasing publication and consumption sequences, correct latest-ready behavior, changing pixels on CM7_1, no invalid states, no overwrite of `READING`, and closed accounting between captured, published, consumed, stale-ready, and no-free counts.

### Hardware Gate 3: WiFi display

Run the existing Assistant TCP framing over WiFi-SPI for at least 60 seconds. Require changing 188 x 120 imagery, approximately 10 FPS with every publication start at least 100 ms after the previous start, frame age at most 200 ms, recorded send duration, no growing queue, and stable TCP connection.

### Control-regression gate

Compare a 60-second pre-camera baseline with a 60-second streaming window. Require:

- no new safety fault;
- IMU age remains below the existing 30 ms stale limit;
- 1 ms scheduler missed-tick count does not increase;
- scheduler maximum gap does not increase by more than 1 ms over baseline;
- servo tick count remains within 1 percent of 300 Hz;
- the CM7_0 heartbeat continues without a visible stall;
- maximum handoff-copy TCPWM59 source masking remains below 1 ms;
- both commanded BLDC duties remain zero.

If a timing threshold fails, leave camera display evidence intact but do not claim control-safe integration.

## Delivery Boundary

This design delivers a verified CM7_0-to-CM7_1 frame transport and raw WiFi display foundation. The next separate milestone implements the slalom vision pipeline on the acquired CM7_1 frame and publishes compact, time-bounded vision navigation commands through the existing inter-core protocol.

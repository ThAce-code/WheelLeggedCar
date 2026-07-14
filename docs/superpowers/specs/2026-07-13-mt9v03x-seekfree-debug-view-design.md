# MT9V03X and Seekfree WiFi-SPI Debug View Design

**Date:** 2026-07-13
**Branch:** `codex/camera-gps-research`
**Status:** Revised design approved in conversation
**Related design:** `2026-07-12-camera-gps-wireless-architecture-design.md`

## 1. Decision

Reuse the source-matched Seekfree camera and Assistant APIs as the supported
hardware abstraction. Do not reverse engineer or rewrite the per-pixel capture
path before bring-up.

The first milestone is deliberately narrow: show a stable 188 x 120 MT9V03X
grayscale image in Seekfree Assistant through the Seekfree WiFi-SPI module. The
wireless image is a development view for later slalom-algorithm verification;
it is not a product video service or a control link.

The authoritative references are:

- `E9_01_seekfree_assistant_mt9v03x_demo` for camera configuration, frame copy,
  raw image sending, and optional boundary overlays;
- `E8_09_mt9v03x_uart_seekfree_assistant_cross_ram_m7_1_demo` for the proven
  CM7_0-to-CM7_1 frame handoff pattern;
- `seekfree_assistant_interface.c` for selecting WiFi-SPI as the transport;
- `zf_device_wifi_spi.c/.h` for module initialization, TCP connection, and
  internal SPI transfer chunking.

The existing implementation plan
`2026-07-13-mt9v03x-seekfree-raw-display.md` predates this decision and must not
be executed. Its replacement is
`docs/superpowers/plans/2026-07-13-mt9v03x-seekfree-api-display.md`.

## 2. Scope

### Included

- Reproduce the unmodified E9_01 camera-to-Assistant flow on the learning board.
- Use the documented camera pins and the source-matched MT9V03X API.
- Change only the Assistant transport from debug UART to WiFi-SPI after the
  reference flow works.
- Send raw grayscale images at an initial display limit of 10 FPS.
- Preserve the E9_01 boundary-overlay capability for later slalom debugging.
- Validate whether the same camera API can run on CM7_1 before assigning final
  core ownership in the robot firmware.
- Reserve and verify every fixed RAM address required by the source driver.
- Measure camera, WiFi, and CM7_0 timing behavior before enabling motors.

### Excluded

- Reimplementing GPIO, TCPWM, Trigger Mux, or DMA pixel sampling.
- Relocating the driver's fixed capture addresses during the first milestone.
- A custom UDP video protocol, custom fragmentation, or a custom PC receiver.
- Image compression, storage, retransmission, or a frame queue.
- The slalom algorithm itself.
- GPS integration; E9_04 may inform a later GPS trajectory-display plan.

## 3. Reference API Flow

The camera API is treated as an opaque, source-matched board-support component:

```c
if(mt9v03x_init())
{
    /* Camera initialization failed. */
}

seekfree_assistant_camera_config(
    &camera_information,
    SEEKFREE_ASSISTANT_CAMERA_TYPE_MT9V03X,
    MT9V03X_W,
    MT9V03X_H,
    image_copy[0]);

if(mt9v03x_finish_flag)
{
    mt9v03x_finish_flag = 0U;
    memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    seekfree_assistant_camera_send(&camera_information);
}
```

The first reference run keeps E9_01's debug-UART transport unchanged. The
second run initializes WiFi-SPI, establishes the documented TCP connection,
and selects:

```c
seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
```

`seekfree_assistant_camera_send()` remains unchanged. The Assistant interface
routes its bytes to `wifi_spi_send_buffer()`, and the WiFi-SPI driver divides a
large image packet into its supported transfer chunks. Application code does
not add another packetization layer.

## 4. Hardware Contract

The confirmed learning-board camera wiring is:

| Signal | CYT4BB pin |
|---|---|
| SCCB SCL | P17_1 |
| SCCB SDA | P17_2 |
| PCLK | P06_5 |
| VSYNC | P06_6 |
| D0-D7 | P18_0-P18_7 |

The source-matched WiFi-SPI wiring is:

| Signal | CYT4BB pin |
|---|---|
| MISO | P02_0 |
| MOSI | P02_1 |
| SCK | P02_2 |
| CS | P02_3 |
| INT | P02_4 |
| RST | P23_0 |

The robot source currently names P06_5 as `BUS_PHASE_PORT`, but
`adc_collection_init()` has no caller in the current checkout. This is a latent
ownership conflict, not proof that the pin is currently configured as ADC.
Camera-enabled firmware must nevertheless prevent P06_5 from being initialized
or used as the bus-current ADC channel. Other current-sense channels are outside
this decision.

## 5. Fixed-Memory Compatibility Contract

The source-matched MT9V03X driver places these objects at fixed addresses:

```text
mt9v03x_image_temp  0x28026024, size 22560 bytes
mt9v03x_h_num       0x28006bf0, size 2 bytes
mt9v03x_w_num       0x28006bf2, size 2 bytes
```

For the first milestone these addresses are an API compatibility contract. Do
not relocate them and do not infer a replacement pixel-capture implementation.

The current CM7_0 map places ordinary read-write data below approximately
`0x28021680`; therefore the temporary image beginning at `0x28026024` does not
currently overlap linked CM7_0 data. This is only a current-map observation.
The linker configuration must explicitly reserve the full fixed range before
camera objects are added, and every build must reject an overlap.

If the exact E9_01 API works on CM7_0 but fails when moved unchanged to CM7_1,
the failure is a core-portability result. It does not authorize rewriting DMA.
At that point choose the documented cross-core fallback or investigate the
specific core dependency separately.

## 6. Core-Ownership Decision Gate

Core ownership is decided by hardware evidence in this order:

1. **Reference gate:** Run E9_01 unchanged on CM7_0 with motors disabled.
2. **Transport gate:** Run the same camera path with WiFi-SPI and Assistant on
   CM7_0 in the isolated demonstration.
3. **Preferred-core gate:** Move the complete camera and WiFi application to
   CM7_1 without changing the MT9V03X driver internals.
4. **Integration decision:**
   - if CM7_1 passes, CM7_1 owns camera, frame snapshot, Assistant, and WiFi;
   - if CM7_1 fails, retain MT9V03X capture on CM7_0 and use the E8_09-style
     completed-frame handoff to CM7_1 for WiFi, subject to a separate timing and
     shared-memory acceptance gate.

CM7_0 always retains balance, motor, servo, IMU, safety, and motion arbitration.
WiFi transmission must not run on CM7_0 in the integrated fallback because the
WiFi-SPI send path is synchronous.

## 7. Frame and Display Policy

The API already supplies `mt9v03x_image`. Add one stable `image_copy` snapshot,
matching E9_01. A completed frame is copied only when the 100 ms display period
has expired and the previous synchronous send has returned.

Rules:

- configured camera resolution is 188 x 120, 8-bit grayscale;
- initial sensor configuration remains the source default;
- initial display limit is 10 FPS;
- there is no frame queue;
- a missed display period is dropped rather than delayed;
- `image_copy` is not overwritten during `seekfree_assistant_camera_send()`;
- Phase 1 sets `INCLUDE_BOUNDARY_TYPE` to raw-image-only behavior;
- Phase 2 adds left boundary, right boundary, and centerline overlays derived
  from the same copied frame.

The future vision algorithm may consume a stable frame interface added after
raw display passes. The first milestone does not add three speculative image
buffers or a new camera abstraction layer.

## 8. Startup and Fault Behavior

The isolated WiFi reference performs:

1. clock and debug initialization;
2. MT9V03X initialization with a bounded failure indication;
3. WiFi-SPI initialization with the configured SSID and password;
4. TCP connection to the configured Assistant endpoint;
5. Assistant interface selection with `SEEKFREE_ASSISTANT_WIFI_SPI`;
6. camera-information configuration;
7. a cooperative latest-frame send loop.

The integrated firmware must not use an infinite initialization retry. Camera
or WiFi failure disables only image display, records the failure, and retries at
a bounded low rate. It must not modify a motion target or actuator output.

If a frame is older than 200 ms, mark camera output invalid. If WiFi cannot keep
up, reduce effective display FPS by dropping display opportunities.

## 9. Verification Gates

### Gate A: Source reference

- Build the supplied E9_01 CM7_0 and CM7_1 projects with IAR 9.40.1.
- Run E9_01 with its original debug-UART Assistant transport.
- Display changing 188 x 120 grayscale content for at least 60 seconds.
- Confirm `mt9v03x_finish_flag` repeats and `mt9v03x_image` changes with the
  scene.

Failure here is investigated inside the reference environment before any robot
integration.

### Gate B: WiFi-SPI reuse

- Initialize the source-matched WiFi-SPI module on P02.0-P02.4 and P23.0.
- Establish the documented TCP connection.
- Select `SEEKFREE_ASSISTANT_WIFI_SPI` without changing camera packet format.
- Display raw images at approximately 10 FPS for at least 60 seconds.
- Record initialization result, connection result, sent frames, failed sends,
  and maximum synchronous send duration.

### Gate C: Core portability

- Run the same API flow on CM7_1 in an isolated example.
- Verify fixed-address reservations and cache behavior from the generated map.
- Accept CM7_1 ownership only after the same 60-second image test passes.

### Gate D: Robot integration

- Build CM0+, CM7_0, and CM7_1.
- Check that camera and WiFi pin owners are unique.
- Check fixed camera ranges and the shared-memory range for overlap.
- Start with motors disabled and servos in the safe middle position.
- Verify IMU initialization, control heartbeat, scheduler maximum gap, servo
  behavior, camera frame age, display FPS, and WiFi send duration.
- Enable motion testing only after camera-disabled and camera-enabled behavior
  show no control regression.

## 10. Follow-up Use of E9 Examples

After raw display passes:

- use E9_01 boundary APIs to overlay slalom perception results;
- use E9_02 oscilloscope APIs for confidence, lateral error, and steering target;
- use E9_03 parameter-debug APIs only after defining safe writable parameters;
- evaluate E9_04's trajectory text format in the separate GPS-display plan;
- use E9_05's attitude text format only as an optional IMU visualization aid.

These follow-ups do not expand the first milestone. Completion of this design's
first milestone means a stable raw camera image is visible through WiFi-SPI and
the chosen core arrangement has passed its timing and memory gates.

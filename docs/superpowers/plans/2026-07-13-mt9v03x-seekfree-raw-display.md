# MT9V03X Seekfree Raw Display Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Display a stable 188 x 120 MT9V03X grayscale image in Seekfree Assistant through the Seekfree WiFi-SPI module without changing CM7_0 control behavior.

**Architecture:** CM7_1 owns camera capture, a stable vision frame, a debug snapshot, WiFi-SPI, and the Seekfree Assistant camera protocol. The implementation starts with a mandatory fixed-address portability gate, then builds host-tested frame and stream services around narrow hardware ports. CM7_0 remains independent and the debug path always drops work instead of queuing frames.

**Tech Stack:** Embedded C, CYT4BB/Traveo II dual Cortex-M7, IAR Embedded Workbench 9.40.1, Seekfree MT9V03X driver, Seekfree WiFi-SPI driver, Seekfree Assistant protocol, host GCC C11 tests, PowerShell.

## Global Constraints

- Keep IMU, safety, chassis, balance, motors, and servos on CM7_0.
- Initialize MT9V03X, SPI0 WiFi, and Seekfree Assistant only on CM7_1.
- Use MT9V03X 188 x 120, 8-bit grayscale, configured for 50 FPS.
- Start debug display at 10 FPS; never build a frame queue.
- Use three logical buffers: capture, stable vision, and debug snapshot; each is 22,560 bytes.
- Place image storage only in CM7_1-private SRAM. Do not use the 8 KiB inter-core region or CM7_0-private SRAM.
- Do not retain the source driver's absolute `0x28026024`, `0x28006bf0`, or `0x28006bf2` placements in the robot firmware.
- The camera completion ISR may clear the interrupt, synchronize cache, update a sequence, and set a flag. It must not send WiFi data or run vision.
- `APP_CAMERA_DEBUG_STREAM_ENABLE` must remove the WiFi/Assistant runtime dependency when set to `0U`.
- A camera frame older than 200 ms is invalid.
- A failed or slow WiFi operation drops only debug output and cannot alter a CM7_0 motion or actuator state.
- Use the existing Seekfree Assistant camera protocol over TCP. Do not add custom UDP fragmentation or a custom PC receiver.
- Keep motors disabled until the raw camera and display gates pass.
- Build all three IAR projects after linker or project-membership changes.

---

## Scope Split

This plan implements the first milestone from the approved design: stable raw
MT9V03X capture and display. Boundary/centerline overlays, the actual slalom
algorithm, and GNSS transport are separate follow-up plans after this milestone
passes. The frame interface in this plan is the input to those later plans.

## File and Responsibility Map

### Create

| File | Responsibility |
|---|---|
| `libraries/zf_device/zf_device_mt9v03x.c` | Source-matched MT9V03X initialization and bounded capture-completion handoff |
| `libraries/zf_device/zf_device_mt9v03x.h` | MT9V03X pins, 188 x 120 constants, and capture API |
| `project/code/camera_frame_service.c/.h` | Stable vision frame, debug snapshot copy, timeout, sequence, and diagnostics |
| `project/code/camera_capture_port.c/.h` | Narrow CM7_1 adapter from `camera_frame_service` to the MT9V03X driver |
| `project/code/camera_debug_stream.c/.h` | 10 FPS latest-frame state machine, drop counters, and reconnect timing |
| `project/code/camera_debug_port.c/.h` | WiFi-SPI TCP and Seekfree Assistant adapter; records short sends |
| `project/code/camera_debug_app.c/.h` | CM7_1 initialization, 1 ms tick, task ordering, and top-level service |
| `project/code/camera_debug_config.h` | Compile-time enable, WiFi credentials, TCP endpoint, periods, and timeouts |
| `project/tests/camera_debug_view_test.c` | Host tests for frame ownership, timeout, rate limiting, drops, and reconnect state |
| `tools/test_camera_debug_view_static.ps1` | Pin/core ownership, forbidden address, project membership, and protocol static checks |
| `docs/camera-debug-view-hardware-test.md` | Recorded IAR builds, map budget, camera results, WiFi results, and CM7_0 regression checks |

### Modify

| File | Change |
|---|---|
| `libraries/zf_common/zf_common_headfile.h` | Enable the restored MT9V03X header |
| `project/iar/icf/linker_directives_tviibh.icf` | Reserve an aligned CM7_1 capture section without changing shared SRAM |
| `project/user/main_cm7_1.c` | Initialize and service `camera_debug_app`; never initialize UART0 |
| `project/user/cm7_1_isr.c` | Forward PIT_CH2 1 ms ticks only |
| `project/iar/project_config/cyt4bb7_cm_7_1.ewp` | Add the restored driver and CM7_1 camera/debug modules |

## Locked Interfaces

`camera_frame_service.h` exposes:

```c
#define CAMERA_FRAME_WIDTH       (188U)
#define CAMERA_FRAME_HEIGHT      (120U)
#define CAMERA_FRAME_SIZE        (CAMERA_FRAME_WIDTH * CAMERA_FRAME_HEIGHT)
#define CAMERA_FRAME_TIMEOUT_MS  (200U)

typedef struct
{
    const uint8 *pixels;
    uint32 sequence;
    uint32 capture_ms;
    uint8 valid;
}camera_frame_view_struct;

typedef struct
{
    uint32 frame_sequence;
    uint32 last_frame_ms;
    uint32 frame_age_ms;
    uint32 timeout_count;
    uint32 debug_copy_count;
    uint8 init_ok;
    uint8 frame_valid;
}camera_frame_diag_struct;

uint8 camera_frame_service_init(uint8 *vision_buffer, uint32 vision_buffer_size);
void camera_frame_service_set_init_result(uint8 init_ok);
uint8 camera_frame_service_publish(const uint8 *pixels, uint32 now_ms);
uint8 camera_frame_service_get_vision(camera_frame_view_struct *view);
uint8 camera_frame_service_copy_debug(uint8 *destination,
                                      uint32 destination_size,
                                      camera_frame_view_struct *view);
void camera_frame_service_update_age(uint32 now_ms);
const camera_frame_diag_struct *camera_frame_service_get_diag(void);
```

`camera_capture_port.h` exposes:

```c
uint8 camera_capture_port_init(void);
uint8 camera_capture_port_take_completed(const uint8 **pixels);
uint8 camera_capture_port_restart(void);
```

The restored `zf_device_mt9v03x.h` adds this narrow handoff without changing the
sensor wire protocol:

```c
uint8 mt9v03x_take_completed(const uint8 **pixels);
uint8 mt9v03x_restart(void);
```

`camera_debug_stream.h` exposes:

```c
typedef enum
{
    CAMERA_DEBUG_DISABLED = 0,
    CAMERA_DEBUG_RECONNECT_WAIT,
    CAMERA_DEBUG_CONNECTED
}camera_debug_state_enum;

typedef struct
{
    uint32 sent_count;
    uint32 dropped_count;
    uint32 reconnect_count;
    uint32 last_send_ms;
    uint32 last_send_duration_ms;
    uint32 max_send_duration_ms;
    uint8 state;
}camera_debug_diag_struct;

void camera_debug_stream_init(uint8 enabled, uint32 now_ms);
void camera_debug_stream_service(uint32 now_ms);
const camera_debug_diag_struct *camera_debug_stream_get_diag(void);
```

`camera_debug_port.h` exposes:

```c
uint8 camera_debug_port_connect(void);
void camera_debug_port_disconnect(void);
uint8 camera_debug_port_send(const uint8 *pixels, uint16 width, uint16 height);
uint32 camera_debug_port_now_ms(void);
```

`camera_debug_app.h` exposes:

```c
uint8 camera_debug_app_init(void);
void camera_debug_app_tick_1ms(void);
void camera_debug_app_service(void);
uint32 camera_debug_app_now_ms(void);
```

### Task 1: Restore the MT9V03X Driver and Pass the Memory-Portability Gate

**Files:**
- Create: `libraries/zf_device/zf_device_mt9v03x.c`
- Create: `libraries/zf_device/zf_device_mt9v03x.h`
- Create: `tools/test_camera_debug_view_static.ps1`
- Modify: `libraries/zf_common/zf_common_headfile.h`
- Modify: `project/iar/icf/linker_directives_tviibh.icf`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_1.ewp`

**Interfaces:**
- Consumes: matching CYT4BB source driver at `D:\smartcar\CYT4BB7_Library\Seekfree_CYT4BB_Opensource_Library\libraries\zf_device\zf_device_mt9v03x.c/.h`.
- Produces: the `mt9v03x_init()`, completion flag, and stable completed-frame source consumed by `camera_capture_port`.

- [ ] **Step 1: Reproduce the official driver before changing placement**

Build and run the official MT9V03X demo outside the robot repository with motors
disconnected. Confirm `mt9v03x_init() == 0`, a changing image, and an increasing
completion count for 60 seconds. Record the demo path, board, result, and observed
FPS in the task execution notes. If the official demo fails, stop here and fix
wiring, power, camera revision, or the reference environment first.

- [ ] **Step 2: Copy the exact matched driver files**

Copy the two files into `libraries/zf_device`. Verify their source hashes before
editing:

```powershell
Get-FileHash -Algorithm SHA256 `
  D:\smartcar\CYT4BB7_Library\Seekfree_CYT4BB_Opensource_Library\libraries\zf_device\zf_device_mt9v03x.c, `
  D:\smartcar\CYT4BB7_Library\Seekfree_CYT4BB_Opensource_Library\libraries\zf_device\zf_device_mt9v03x.h
```

Expected hashes:

```text
33C9B8C1D5641CB48933B4E6796788BB247C813909B4A3F16D963A6B40D7E7E0
0C489FABA1851C861E33B44E222D7D72B331EEC851B09F93DC694A20FA17DED2
```

- [ ] **Step 3: Replace absolute placement with a named CM7_1 section**

In `zf_device_mt9v03x.c`, replace the three absolute pragmas with a 32-byte
aligned named capture section and normal state variables. Retain
`mt9v03x_image` as the stable vision buffer, but remove the full-frame copy from
`camera_finish_callback()`; the callback performs cache invalidation and sets
the completion flag only:

```c
#pragma data_alignment = 32
#pragma location = ".mt9v03x_capture"
__no_init uint8 mt9v03x_image_temp[MT9V03X_H][MT9V03X_W];

static uint16 mt9v03x_h_num;
static uint16 mt9v03x_w_num;

void camera_finish_callback(void)
{
    Cy_Tcpwm_Counter_ClearTC_Intr(TCPWM0_GRP0_CNT59);
    SCB_InvalidateDCache_by_Addr(mt9v03x_image_temp[0], MT9V03X_IMAGE_SIZE);
    mt9v03x_finish_flag = 1U;
}
```

Do not change SCCB commands, resolution, FPS, pins, or exposure settings in this
step. Add `mt9v03x_take_completed()` so it returns `mt9v03x_image_temp[0]` and
clears one completion event atomically. Add an idempotent `mt9v03x_restart()`
that stops and clears the capture counter/interrupt before reinitialization.
Keep the source copyright header.

- [ ] **Step 4: Reserve the capture section in CM7_1 SRAM**

Add a `CAMERA_CAPTURE` linker region starting at the current CM7_1 private base,
with 32-byte-rounded size `22560`, and move the normal CM7_1 `SRAM` start after
that region. Place only `.mt9v03x_capture` in `CAMERA_CAPTURE`. Keep
`_base_SRAM_CM7_SHARED`, `intercore_shared_sram_size`, and exported inter-core
symbols unchanged.

The map must show the capture buffer wholly inside CM7_1 private SRAM and the
normal read/write sections beginning after it.

- [ ] **Step 5: Add the initial static gate**

Create `tools/test_camera_debug_view_static.ps1` so it fails unless:

```powershell
$ErrorActionPreference = 'Stop'
$driver = Get-Content libraries/zf_device/zf_device_mt9v03x.c -Raw
$cm71 = Get-Content project/iar/project_config/cyt4bb7_cm_7_1.ewp -Raw
$cm70 = Get-Content project/iar/project_config/cyt4bb7_cm_7_0.ewp -Raw
$icf = Get-Content project/iar/icf/linker_directives_tviibh.icf -Raw

foreach ($forbidden in @('0x28026024', '0x28006bf0', '0x28006bf2')) {
    if ($driver.Contains($forbidden)) { throw "Forbidden camera address: $forbidden" }
}
if (-not $driver.Contains('.mt9v03x_capture')) { throw 'Missing capture section' }
if (-not $icf.Contains('CAMERA_CAPTURE')) { throw 'Missing CAMERA_CAPTURE region' }
if (-not $cm71.Contains('zf_device_mt9v03x.c')) { throw 'MT9V03X missing from CM7_1' }
if ($cm70.Contains('zf_device_mt9v03x.c')) { throw 'MT9V03X added to CM7_0' }
Write-Output 'camera_debug_view_static: PASS'
```

- [ ] **Step 6: Build and run the CM7_1 placement probe**

Build all three IAR projects, inspect the CM7_1 map, then run with motors
disabled. Confirm initialization, completion interrupts, changing pixels, 50 FPS
configuration, and 60 seconds without a frame timeout.

If moving `.mt9v03x_capture` changes or stops capture, the fixed addresses are a
hardware/library contract. Stop the entire plan, revert the uncommitted probe,
and return to architecture review. Never fall back to CM7_0-private addresses.

- [ ] **Step 7: Commit the passed portability gate**

```powershell
git add libraries/zf_device/zf_device_mt9v03x.c `
        libraries/zf_device/zf_device_mt9v03x.h `
        libraries/zf_common/zf_common_headfile.h `
        project/iar/icf/linker_directives_tviibh.icf `
        project/iar/project_config/cyt4bb7_cm_7_1.ewp `
        tools/test_camera_debug_view_static.ps1
git commit -m "Add CM7_1 MT9V03X capture driver"
```

### Task 2: Build the Host-Tested Stable Frame Service

**Files:**
- Create: `project/code/camera_frame_service.c`
- Create: `project/code/camera_frame_service.h`
- Create: `project/tests/camera_debug_view_test.c`

**Interfaces:**
- Consumes: a stable completed 22,560-byte source frame from `camera_capture_port_take_completed()`.
- Produces: the locked `camera_frame_service` API and immutable latest-frame views.

- [ ] **Step 1: Write failing frame-service tests**

Add tests that assert:

```c
uint8 vision_storage[CAMERA_FRAME_SIZE];

TEST_EQ(1U, camera_frame_service_init(vision_storage,
                                      sizeof(vision_storage)));
camera_frame_service_set_init_result(1U);
TEST_EQ(1U, camera_frame_service_publish(source_a, 10U));
TEST_EQ(1U, camera_frame_service_get_vision(&view));
TEST_EQ(1U, view.sequence);
TEST_EQ(10U, view.capture_ms);
TEST_EQ(source_a[137], view.pixels[137]);

TEST_EQ(1U, camera_frame_service_copy_debug(debug_copy,
                                            CAMERA_FRAME_SIZE,
                                            &debug_view));
TEST_EQ(view.sequence, debug_view.sequence);
TEST_EQ(view.pixels[4096], debug_copy[4096]);

camera_frame_service_update_age(211U);
TEST_EQ(0U, camera_frame_service_get_vision(&view));
TEST_EQ(1U, camera_frame_service_get_diag()->timeout_count);
camera_frame_service_update_age(500U);
TEST_EQ(1U, camera_frame_service_get_diag()->timeout_count);
```

Also assert null pointers, a short vision buffer, a short debug destination, and
publish-before-init are rejected without changing sequence or destination data.

- [ ] **Step 2: Compile and verify the tests fail**

```powershell
New-Item -ItemType Directory -Force project/tests/build | Out-Null
gcc -std=c11 -Wall -Wextra -Werror `
  -Ilibraries/zf_common -Iproject/code `
  project/tests/camera_debug_view_test.c `
  project/code/camera_frame_service.c `
  -o project/tests/build/camera_debug_view_test.exe
```

Expected: compilation fails because the frame-service files or symbols are not
implemented.

- [ ] **Step 3: Implement the locked frame API**

Store the injected vision-buffer pointer only when its size is exactly
`CAMERA_FRAME_SIZE`. Copy into that buffer only in
`camera_frame_service_publish()`, increment sequence after the copy, and return a
read-only view. `camera_frame_service_update_age()` increments `timeout_count`
only on the valid-to-invalid transition. Use unsigned subtraction for tick
wraparound.

- [ ] **Step 4: Run the host tests**

Run the command from Step 2, then:

```powershell
project/tests/build/camera_debug_view_test.exe
```

Expected: `camera_debug_view_test: PASS` and exit code 0.

- [ ] **Step 5: Commit the stable frame service**

```powershell
git add project/code/camera_frame_service.c `
        project/code/camera_frame_service.h `
        project/tests/camera_debug_view_test.c
git commit -m "Add stable camera frame service"
```

### Task 3: Integrate Camera Capture into the CM7_1 Service Loop

**Files:**
- Create: `project/code/camera_capture_port.c/.h`
- Create: `project/code/camera_debug_app.c/.h`
- Create: `project/code/camera_debug_config.h`
- Modify: `project/user/main_cm7_1.c`
- Modify: `project/user/cm7_1_isr.c`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_1.ewp`
- Modify: `tools/test_camera_debug_view_static.ps1`

**Interfaces:**
- Consumes: MT9V03X completion state and `camera_frame_service_publish()`.
- Produces: a 1 ms CM7_1 clock and continuously updated stable vision frame.

- [ ] **Step 1: Add compile-time configuration**

Create `camera_debug_config.h` with these initial values:

```c
#define APP_CAMERA_ENABLE                 (1U)
#define APP_CAMERA_DEBUG_STREAM_ENABLE    (1U)
#define APP_CAMERA_DEBUG_PERIOD_MS        (100U)
#define APP_CAMERA_RECONNECT_PERIOD_MS    (1000U)
#define APP_CAMERA_WIFI_SSID              "SEEKFREE"
#define APP_CAMERA_WIFI_PASSWORD          "12345678"
#define APP_CAMERA_TCP_TARGET_IP          "192.168.137.1"
#define APP_CAMERA_TCP_TARGET_PORT        "8080"
#define APP_CAMERA_WIFI_LOCAL_PORT        "6666"
```

Only this file may contain bench network settings.

- [ ] **Step 2: Implement the capture port**

`camera_capture_port_init()` returns `1U` only when `mt9v03x_init()` returns zero.
`camera_capture_port_take_completed()` delegates to
`mt9v03x_take_completed()`. `camera_capture_port_restart()` delegates to
`mt9v03x_restart()` and returns the same success convention.

The implementation must preserve the bounded-ISR decision proven in Task 1. If
the only working implementation requires a full-frame ISR copy, stop and revise
the approved design before proceeding.

- [ ] **Step 3: Implement the CM7_1 app order**

`camera_debug_app_init()` initializes the frame service with
`mt9v03x_image[0]`, initializes capture, and records the init result.
`camera_debug_app_service()` must execute exactly:

```c
if(camera_capture_port_take_completed(&pixels))
{
    camera_frame_service_publish(pixels, camera_debug_app_now_ms());
}
camera_frame_service_update_age(camera_debug_app_now_ms());
```

WiFi is not initialized in this task.

- [ ] **Step 4: Wire PIT_CH2 and main**

After clock and inter-core memory setup, initialize `camera_debug_app`, configure
PIT_CH2 for 1 ms, and call `camera_debug_app_service()` in the CM7_1 loop.
`pit0_ch2_isr()` clears the PIT flag and calls only
`camera_debug_app_tick_1ms()`. Do not add UART0 initialization or printing.

- [ ] **Step 5: Extend and run the static gate**

Assert that `main_cm7_1.c` calls `camera_debug_app_init()` and service, the CM7_1
ISR calls the tick, UART0/debug initialization is absent, and the three new
modules appear only in `cyt4bb7_cm_7_1.ewp`.

- [ ] **Step 6: Build and perform the raw-capture hardware gate**

Build all cores. With motors disabled, inspect `camera_frame_service_get_diag()`
for 60 seconds. Expected: `init_ok == 1`, monotonically increasing sequence,
changing pixels, measured FPS near the configured 50 FPS, and zero unexpected
200 ms timeouts.

- [ ] **Step 7: Commit CM7_1 raw capture**

```powershell
git add project/code/camera_capture_port.c project/code/camera_capture_port.h `
        project/code/camera_debug_app.c project/code/camera_debug_app.h `
        project/code/camera_debug_config.h project/user/main_cm7_1.c `
        project/user/cm7_1_isr.c `
        project/iar/project_config/cyt4bb7_cm_7_1.ewp `
        tools/test_camera_debug_view_static.ps1
git commit -m "Run MT9V03X capture on CM7_1"
```

### Task 4: Add the Host-Tested 10 FPS Debug Stream State Machine

**Files:**
- Create: `project/code/camera_debug_stream.c/.h`
- Modify: `project/tests/camera_debug_view_test.c`

**Interfaces:**
- Consumes: immutable `camera_frame_view_struct` and mocked `camera_debug_port` functions.
- Produces: rate-limited connection/send decisions and the locked debug diagnostics.

- [ ] **Step 1: Add failing stream tests with port mocks**

Test these exact behaviors:

```c
camera_debug_stream_init(1U, 0U);
camera_frame_service_publish(source_a, 0U);
camera_debug_stream_service(0U);
TEST_EQ(1U, mock_connect_calls);
TEST_EQ(0U, mock_send_calls);

camera_debug_stream_service(99U);
TEST_EQ(0U, mock_send_calls);
camera_debug_stream_service(100U);
TEST_EQ(1U, mock_send_calls);
camera_frame_service_publish(source_b, 150U);
camera_debug_stream_service(150U);
TEST_EQ(1U, mock_send_calls);
TEST_EQ(0U, camera_debug_stream_get_diag()->dropped_count);

mock_send_result = 0U;
camera_frame_service_publish(source_c, 200U);
camera_debug_stream_service(200U);
TEST_EQ(1U, camera_debug_stream_get_diag()->dropped_count);
TEST_EQ(CAMERA_DEBUG_RECONNECT_WAIT,
        camera_debug_stream_get_diag()->state);
camera_debug_stream_service(1199U);
TEST_EQ(1U, mock_connect_calls);
camera_debug_stream_service(1200U);
TEST_EQ(2U, mock_connect_calls);
```

Also verify disabled mode makes zero port calls and invalid frames are never sent.

- [ ] **Step 2: Compile and verify failure**

Add `project/code/camera_debug_stream.c` to the Task 2 GCC command. Expected:
failure because the stream API is not implemented.

- [ ] **Step 3: Implement the minimal state machine**

Use unsigned time differences, one connection attempt per 1000 ms, one send
attempt per 100 ms, no queue, and only the newest frame copied through
`camera_frame_service_copy_debug()`. Count skipped frame sequences when a newer
frame replaces one that was never sent. A failed send calls
`camera_debug_port_disconnect()` and enters `RECONNECT_WAIT`. Record send
duration using `camera_debug_port_now_ms()` before and after the synchronous send.

- [ ] **Step 4: Run host tests and commit**

Expected: `camera_debug_view_test: PASS`.

```powershell
git add project/code/camera_debug_stream.c `
        project/code/camera_debug_stream.h `
        project/tests/camera_debug_view_test.c
git commit -m "Add rate-limited camera debug stream"
```

### Task 5: Connect WiFi-SPI and Seekfree Assistant on CM7_1

**Files:**
- Create: `project/code/camera_debug_port.c/.h`
- Modify: `project/code/camera_debug_app.c`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_1.ewp`
- Modify: `tools/test_camera_debug_view_static.ps1`

**Interfaces:**
- Consumes: existing `wifi_spi_*`, `seekfree_assistant_*`, and debug-stream APIs.
- Produces: TCP connection and raw MT9V03X Assistant frames.

- [ ] **Step 1: Implement a measurable custom Assistant transfer**

Initialize Assistant with `SEEKFREE_ASSISTANT_CUSTOM`. Provide a strong
`seekfree_assistant_transfer()` that calls `wifi_spi_send_buffer()` and records
failure when its returned unsent length is nonzero. `camera_debug_port_send()`
must reset that per-frame error, configure:

```c
seekfree_assistant_camera_information_config(
    SEEKFREE_ASSISTANT_MT9V03X,
    (void *)pixels,
    CAMERA_FRAME_WIDTH,
    CAMERA_FRAME_HEIGHT);
seekfree_assistant_camera_boundary_config(
    NO_BOUNDARY, 0U, NULL, NULL, NULL, NULL, NULL, NULL);
seekfree_assistant_camera_send();
```

Return `1U` only when every Assistant transfer returned zero unsent bytes.

- [ ] **Step 2: Implement one TCP connection attempt**

`camera_debug_port_connect()` calls `wifi_spi_init()` with the config credentials,
then `wifi_spi_socket_connect("TCP", target_ip, target_port, local_port)`, and
initializes the custom Assistant interface. It returns instead of retrying in a
loop. `camera_debug_stream` owns retry timing.

- [ ] **Step 3: Copy a debug snapshot before every send**

In `camera_debug_stream.c`, allocate a private aligned
`debug_buffer[CAMERA_FRAME_SIZE]`. Only after the 100 ms send period expires,
copy the latest vision frame through `camera_frame_service_copy_debug()` and pass
that immutable snapshot to `camera_debug_port_send()`. Never pass the live
capture or vision buffer to WiFi. `camera_debug_port_now_ms()` returns
`camera_debug_app_now_ms()` for send-duration measurement.

Initialize the stream at the end of `camera_debug_app_init()` with
`APP_CAMERA_DEBUG_STREAM_ENABLE`. Call
`camera_debug_stream_service(camera_debug_app_now_ms())` after capture publication
and age update in every `camera_debug_app_service()` pass.

- [ ] **Step 4: Extend static verification**

Assert WiFi/Assistant calls occur only in `camera_debug_port.c`, the source contains
`SEEKFREE_ASSISTANT_MT9V03X` and `NO_BOUNDARY`, no custom UDP/video-chunk symbols
exist, and CM7_0 project membership is unchanged.

- [ ] **Step 5: Build and run the raw-display gate**

Configure the PC as the TCP server at `192.168.137.1:8080`, select the network
camera function in Seekfree Assistant, and power the WiFi module from a suitable
5 V supply with common ground. Build all cores and run with motors disabled.

Expected: a changing 188 x 120 grayscale image at approximately 10 FPS with no
tearing. Record sent/dropped counts and maximum synchronous send duration.

- [ ] **Step 6: Characterize reconnect blocking**

Disconnect and restore the PC server. If a single `wifi_spi_init()` or socket
attempt prevents camera-frame publication for 200 ms or longer, leave automatic
runtime reconnect disabled before slalom work and create a separate nonblocking
WiFi-driver plan. Do not claim the reconnect acceptance gate passed.

- [ ] **Step 7: Commit raw display integration**

```powershell
git add project/code/camera_debug_port.c project/code/camera_debug_port.h `
        project/code/camera_debug_app.c `
        project/iar/project_config/cyt4bb7_cm_7_1.ewp `
        tools/test_camera_debug_view_static.ps1
git commit -m "Stream MT9V03X frames to Seekfree Assistant"
```

### Task 6: Run Final Static, Build, and Hardware Acceptance

**Files:**
- Create: `docs/camera-debug-view-hardware-test.md`
- Modify: only files needed to fix failures found by the gates

**Interfaces:**
- Consumes: the complete raw-display milestone.
- Produces: reproducible evidence that the milestone is ready for the overlay/slalom plan.

- [ ] **Step 1: Run host and static tests from a clean build directory**

```powershell
Remove-Item -LiteralPath project/tests/build -Recurse -Force -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Force project/tests/build | Out-Null
gcc -std=c11 -Wall -Wextra -Werror `
  -Ilibraries/zf_common -Iproject/code `
  project/tests/camera_debug_view_test.c `
  project/code/camera_frame_service.c `
  project/code/camera_debug_stream.c `
  -o project/tests/build/camera_debug_view_test.exe
project/tests/build/camera_debug_view_test.exe
powershell -ExecutionPolicy Bypass -File tools/test_camera_debug_view_static.ps1
git diff --check
```

Expected: both tests print `PASS`, all commands exit zero, and `git diff --check`
prints nothing.

- [ ] **Step 2: Build every IAR core and inspect the map**

Build `cyt4bb7_cm_0_plus`, `cyt4bb7_cm_7_0`, and `cyt4bb7_cm_7_1`. Record warnings,
code size, SRAM size, capture-section address, three image-buffer addresses, stack
margin, and heap margin. Reject any overlap with `0x28080000..0x28081FFF` or the
CM7_0 allocation.

- [ ] **Step 3: Run hardware acceptance**

Record these observed checks in `docs/camera-debug-view-hardware-test.md`:

- camera initialization and 60-second raw capture;
- capture FPS, timeouts, and changing scene content;
- Seekfree Assistant resolution, display FPS, tearing, sent frames, and drops;
- WiFi disconnect/reconnect behavior and maximum blocking duration;
- CM7_0 IMU initialization, heartbeat, servo middle position, motor feedback,
  scheduler maximum gap, and fault behavior;
- behavior with `APP_CAMERA_DEBUG_STREAM_ENABLE` set to `0U`.

- [ ] **Step 4: Commit the verified hardware report**

Commit only after entering actual observed results; do not pre-fill pass results.

```powershell
git add docs/camera-debug-view-hardware-test.md
git commit -m "Document camera debug view hardware validation"
```

The milestone is complete only when raw display works, the map is safe, host and
static tests pass, and CM7_0 behavior has no regression. Overlay and slalom work
must use a new plan based on the locked stable-frame interface.

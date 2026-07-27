# MT9V03X Seekfree API Display Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Stop at every hardware gate and record the result before continuing.

**Goal:** Show a stable 188 x 120 MT9V03X grayscale image in Seekfree Assistant through the source-matched Seekfree WiFi-SPI module, while keeping wheel motors forcibly disabled during camera bring-up.

**Architecture:** First prove the unmodified E9_01 camera flow on CM7_0, then prove its existing Assistant packet stream over WiFi-SPI. In the robot worktree, import the source-matched MT9V03X driver and E9 object-style Assistant API without rewriting capture internals. Prefer CM7_1 ownership only after an unchanged-driver portability test; if that fails, stop and write the E8_09-style CM7_0-capture/CM7_1-send fallback plan. The first milestone uses one stable image copy, a 10 FPS latest-frame policy, no queue, and no custom UDP or packetization.

**Tech Stack:** Embedded C, CYT4BB/Traveo II CM0+/dual Cortex-M7, IAR Embedded Workbench 9.40.1, Seekfree MT9V03X driver, Seekfree WiFi-SPI driver, Seekfree Assistant protocol, PowerShell static checks.

---

## Locked Decisions and Stop Conditions

- Camera pins are fixed at P17_1/P17_2 SCCB, P06_5 PCLK, P06_6 VSYNC, and P18_0-P18_7 data.
- WiFi-SPI pins are fixed at P02_0 MISO, P02_1 MOSI, P02_2 SCK, P02_3 CS, P02_4 INT, and P23_0 RST.
- Reuse `mt9v03x_init()`, `mt9v03x_finish_flag`, `mt9v03x_image`, `seekfree_assistant_camera_config()`, and `seekfree_assistant_camera_send()`.
- Keep the current `seekfree_assistant_interface.c/.h` and `zf_device_wifi_spi.c/.h`; they match this checkout's CH9141 integration and already route Assistant traffic through `wifi_spi_send_buffer()`.
- Import the E9 V2 `seekfree_assistant.c/.h`; the current V1 global camera API is not compatible with E9_01's object-style calls.
- Do not copy E9's `zf_device_config.a/.h`; retain the CYT4BB-specific versions already in this project.
- Do not relocate the MT9V03X driver's fixed addresses in this milestone.
- Do not add UDP, a custom receiver, compression, retransmission, or a frame queue.
- `P06_5` cannot be camera PCLK and bus-current ADC at the same time. In camera-debug firmware, skip that ADC channel and force every BLDC duty command to zero.
- The previously validated safe leg reference is all four servos at 90 degrees. Do not run `LXY` or wheel motion during the camera gates.
- **STOP A:** If the original E9_01 debug-UART demo cannot display a changing image for 60 seconds, diagnose the reference wiring/source; do not modify robot firmware.
- **STOP B:** If the unchanged MT9V03X driver builds but does not repeat frames on CM7_1, do not rewrite DMA/GPIO/TCPWM. Record the failure and replace Tasks 4-5 with a separate E8_09 cross-core handoff plan.
- **STOP C:** Do not re-enable motors merely because video works. Motor enable requires a later timing/control-regression decision outside this plan.

## Reference Sources and Provenance

Use these exact sources:

```text
D:\smartcar\CYT4BB7_Library\Example\Motherboard_Demo\E9_seekfree_assistant\E9_01_seekfree_assistant_mt9v03x_demo
D:\smartcar\CYT4BB7_Library\Example\Motherboard_Demo\E9_seekfree_assistant\libraries
D:\smartcar\CYT4BB7_Library\Example\Motherboard_Demo\E8_camera\E8_09_mt9v03x_uart_seekfree_assistant_cross_ram_m7_1_demo
```

Expected SHA-256 values for imported files:

```text
33C9B8C1D5641CB48933B4E6796788BB247C813909B4A3F16D963A6B40D7E7E0  zf_device_mt9v03x.c
0C489FABA1851C861E33B44E222D7D72B331EEC851B09F93DC694A20FA17DED2  zf_device_mt9v03x.h
6C6BABD379FAFCCBB64F4DE27A8E837EC3C73EF4E7ECD169656CF80B0C13A28B  seekfree_assistant.c
FA6FC8DD75323AF03A49FB48BBA63029AD2CB36C1C8484F4B9596835EADEBBB5  seekfree_assistant.h
```

Fixed-address compatibility contract:

```text
0x28006BF0-0x28006BF1  mt9v03x_h_num
0x28006BF2-0x28006BF3  mt9v03x_w_num
0x28026024-0x2802B843  mt9v03x_image_temp (22,560 bytes)
```

---

### Task 1: Prove the untouched E9_01 reference path

**Files:**
- Create: `docs/camera-seekfree-api-hardware-test.md`
- Reference only: `D:\smartcar\CYT4BB7_Library\Example\Motherboard_Demo\E9_seekfree_assistant\E9_01_seekfree_assistant_mt9v03x_demo\user\main_cm7_0.c`

- [ ] **Step 1: Create the hardware evidence template before testing**

Create `docs/camera-seekfree-api-hardware-test.md` with sections for board/wiring, reference build, UART image result, WiFi result, CM7_1 portability, three-core robot build, map ranges, timing, and final disposition. Every result row must have `NOT RUN`, `PASS`, or `FAIL`; do not pre-fill a pass.

- [ ] **Step 2: Verify wiring with power removed**

Record continuity for:

```text
MT9V03X: P17_1 SCL, P17_2 SDA, P06_5 PCLK, P06_6 VSYNC, P18_0..P18_7 D0..D7
WiFi-SPI: P02_0 MISO, P02_1 MOSI, P02_2 SCK, P02_3 CS, P02_4 INT, P23_0 RST
```

Keep wheel power disabled. Place the legs in the validated all-90-degree reference pose if the robot chassis is used as the host.

- [ ] **Step 3: Build the original reference without source edits**

Open the E9_01 IAR workspace with IAR 9.40.1 and build both supplied projects: CM7_0 and CM7_1. Record warnings/errors and generated map paths.

Expected: both supplied reference projects build. The E9_01 workspace does not supply a CM0+ project. If either supplied project fails, set Gate A to `FAIL` and stop.

- [ ] **Step 4: Run the debug-UART Assistant display for 60 seconds**

Use the original `SEEKFREE_ASSISTANT_DEBUG_UART` flow. In IAR Live Watch or Watch, observe:

```text
mt9v03x_finish_flag
mt9v03x_image[0][0]
mt9v03x_image[60][94]
```

Move a high-contrast object in front of the camera and confirm the Assistant image and sampled pixels change. Record the 60-second result and any initialization LED/error behavior.

Expected: repeated frames and a changing 188 x 120 grayscale image for at least 60 seconds.

- [ ] **Step 5: Commit only the evidence template and actual Gate A result**

```powershell
git add docs/camera-seekfree-api-hardware-test.md
git commit -m "Record MT9V03X reference bring-up"
```

Do not continue if Gate A is not `PASS`.

---

### Task 2: Prove the existing Assistant packet over WiFi-SPI on the reference demo

**Files:**
- Modify: `docs/camera-seekfree-api-hardware-test.md`
- Temporary test copy only: a copy of E9_01 outside this repository

- [ ] **Step 1: Make an isolated copy of E9_01**

Copy the E9_01 demo to a temporary bring-up directory. Do not edit the authoritative reference directory and do not add the copied demo to this repository.

- [ ] **Step 2: Change transport only**

Keep the camera init, `image_copy`, `memcpy`, camera configuration, and `seekfree_assistant_camera_send()` calls unchanged. In the temporary copy, initialize the existing WiFi-SPI API and select the WiFi interface:

```c
if(wifi_spi_init("SEEKFREE", "SEEKFREE123"))
{
    /* Record WiFi init failure and stop this gate. */
}

if(wifi_spi_socket_connect("TCP", "192.168.137.1", "8086", "6666"))
{
    /* Record socket failure and stop this gate. */
}

seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
```

Configure the Windows hotspot/Assistant endpoint to match, or replace all four strings with the actual test network values and record them. Do not change Assistant framing or call `wifi_spi_send_buffer()` from application code.

- [ ] **Step 3: Rate-limit reference display to 10 FPS**

Send only when a frame is complete and at least 100 ms has elapsed since the previous send. Keep one `image_copy`; do not queue frames.

- [ ] **Step 4: Build and run for 60 seconds**

Confirm:

```text
wifi_spi_init == success
wifi_spi_socket_connect == success
Assistant receives changing raw image
display duration >= 60 seconds
effective display rate approximately 10 FPS
```

Record whether synchronous sending ever visibly stalls capture. This gate proves transport reuse, not robot real-time safety.

- [ ] **Step 5: Record and commit Gate B**

```powershell
git add docs/camera-seekfree-api-hardware-test.md
git commit -m "Record WiFi-SPI Assistant reference test"
```

Do not continue if Gate B is not `PASS`.

---

### Task 3: Import the source-matched driver and E9 Assistant API

**Files:**
- Create: `libraries/zf_device/zf_device_mt9v03x.c`
- Create: `libraries/zf_device/zf_device_mt9v03x.h`
- Replace: `libraries/zf_components/seekfree_assistant.c`
- Replace: `libraries/zf_components/seekfree_assistant.h`
- Modify: `libraries/zf_common/zf_common_headfile.h`
- Modify: `project/iar/icf/linker_directives_tviibh.icf`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_1.ewp`
- Create: `tools/test_camera_seekfree_api_static.ps1`

- [ ] **Step 1: Write the failing provenance and ownership test**

Create `tools/test_camera_seekfree_api_static.ps1`. It must fail unless all of these are true:

```text
the four imported files match the SHA-256 values in this plan
zf_common_headfile.h includes zf_device_mt9v03x.h
CM7_1 .ewp contains zf_device_mt9v03x.c/.h
current seekfree_assistant_interface.c still maps WIFI_SPI to wifi_spi_send_buffer/read_buffer
current zf_device_wifi_spi.h still declares the confirmed six pins
MT9V03X header declares P06_5, P06_6, P18_0..P18_7 and 188 x 120
MT9V03X source retains all three fixed placements
E9 Assistant header exposes object-style camera_config and camera_send
no application source introduces a custom UDP packetizer
```

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_camera_seekfree_api_static.ps1
```

Expected: FAIL because the MT9V03X files and E9 object API are not present yet.

- [ ] **Step 2: Copy only the approved source files**

Copy the two MT9V03X files and the two E9 Assistant files from the reference libraries. Do not replace:

```text
seekfree_assistant_interface.c/.h
zf_device_wifi_spi.c/.h
zf_device_config.a/.h
```

- [ ] **Step 3: Enable the driver header and CM7_1 project membership**

Uncomment the MT9V03X include in `zf_common_headfile.h`. Add the driver `.c/.h` to the device group in `cyt4bb7_cm_7_1.ewp`. The Assistant and WiFi files are already members; do not duplicate them.

- [ ] **Step 4: Reserve the fixed ranges from other cores' ordinary allocations**

In `linker_directives_tviibh.icf`, split the CM0+ SRAM region around `0x28006BF0-0x28006BF3` and the CM7_0 SRAM region around `0x28026024-0x2802B843`. Keep the existing 8 KiB inter-core range at `0x28080000-0x28081FFF` and CM7_1 ordinary base at `0x28082000` unchanged.

Use IAR region unions so ordinary CM0+/CM7_0 read-write sections cannot occupy the camera driver's absolute locations. Do not move the driver objects into the 8 KiB inter-core range.

- [ ] **Step 5: Run static checks**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_camera_seekfree_api_static.ps1
git diff --check
```

Expected: PASS.

- [ ] **Step 6: Build all three cores and inspect maps**

Build `cyt4bb7_cm_0_plus`, `cyt4bb7_cm_7_0`, and `cyt4bb7_cm_7_1`. Confirm the map files show:

```text
mt9v03x_h_num       0x28006BF0
mt9v03x_w_num       0x28006BF2
mt9v03x_image_temp  0x28026024, size 0x5820
CM0+ ordinary data does not occupy 0x28006BF0-0x28006BF3
CM7_0 ordinary data does not occupy 0x28026024-0x2802B843
shared SRAM remains 0x28080000-0x28081FFF
CM7_1 ordinary SRAM still begins at 0x28082000
```

Record map evidence in the hardware-test document. A link or overlap failure is a Task 3 failure; do not weaken the reservation.

- [ ] **Step 7: Commit the source-compatible API layer**

```powershell
git add libraries/zf_device/zf_device_mt9v03x.c libraries/zf_device/zf_device_mt9v03x.h libraries/zf_components/seekfree_assistant.c libraries/zf_components/seekfree_assistant.h libraries/zf_common/zf_common_headfile.h project/iar/icf/linker_directives_tviibh.icf project/iar/project_config/cyt4bb7_cm_7_1.ewp tools/test_camera_seekfree_api_static.ps1 docs/camera-seekfree-api-hardware-test.md
git commit -m "Add source-matched MT9V03X API"
```

---

### Task 4: Add a camera-only CM7_1 portability application with hard motor lock

**Files:**
- Create: `project/code/camera_debug_config.h`
- Create: `project/code/camera_debug_app.h`
- Create: `project/code/camera_debug_app.c`
- Modify: `project/code/app_config.h`
- Modify: `project/code/actuator_motor.c`
- Modify: `project/code/board_adc.c`
- Modify: `project/user/main_cm7_1.c`
- Modify: `project/user/cm7_1_isr.c`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_1.ewp`
- Modify: `tools/test_camera_seekfree_api_static.ps1`
- Modify: `docs/camera-seekfree-api-hardware-test.md`

- [ ] **Step 1: Extend the failing static test for the safety and core contract**

Add assertions that fail until:

```text
APP_CAMERA_DEBUG_ONLY is 1U
APP_CAMERA_WIFI_ENABLE is 0U for the portability gate
actuator_motor_send_duty clamps left and right duty to zero under APP_CAMERA_DEBUG_ONLY
board_adc.c does not initialize or convert BUS_PHASE_PORT under APP_CAMERA_DEBUG_ONLY
main_cm7_1 initializes PIT_CH2 at 1 ms and calls camera_debug_app_init/service
pit0_ch2_isr clears PIT_CH2 and calls camera_debug_app_tick_1ms
no camera or WiFi send code is called from an ISR
CM7_1 .ewp contains camera_debug_app.c/.h and camera_debug_config.h
```

Run the test and expect FAIL.

- [ ] **Step 2: Define the narrow debug configuration**

In `app_config.h` add:

```c
#define APP_CAMERA_DEBUG_ONLY           (1U)
```

In `camera_debug_config.h` add:

```c
#define APP_CAMERA_WIFI_ENABLE          (0U)
#define APP_CAMERA_DISPLAY_PERIOD_MS    (100U)
#define APP_CAMERA_STALE_TIMEOUT_MS     (200U)
#define APP_CAMERA_RETRY_PERIOD_MS      (1000U)
#define APP_CAMERA_WIFI_RETRY_MS        (5000U)
#define APP_CAMERA_WIFI_SSID            "SEEKFREE"
#define APP_CAMERA_WIFI_PASSWORD        "SEEKFREE123"
#define APP_CAMERA_WIFI_TARGET_IP       "192.168.137.1"
#define APP_CAMERA_WIFI_TARGET_PORT     "8086"
#define APP_CAMERA_WIFI_LOCAL_PORT      "6666"
```

Keep network values overridable with `#ifndef` guards. Do not store personal hotspot credentials in Git.

- [ ] **Step 3: Make camera-debug firmware physically unable to drive wheels**

At the single BLDC duty choke point in `actuator_motor_send_duty()`, force both values to zero when `APP_CAMERA_DEBUG_ONLY` is enabled before calling `bldc_foc_uart_set_duty()`. This covers RPM, open-loop, test, and stop paths because all nonzero duty output currently passes through this function.

In `board_adc.c`, include `app_config.h` and guard both `adc_init(BUS_PHASE_PORT, ...)` and `adc_convert(BUS_PHASE_PORT)` with `#if !APP_CAMERA_DEBUG_ONLY`. Set the derived bus-current value/filter to a benign zero only in debug mode. Do not pretend that over-current protection remains available; the hard zero-duty gate is the safety mechanism.

Do not disable servo PWM. The servos remain available only for the already validated 90-degree reference pose.

- [ ] **Step 4: Define observable camera diagnostics**

Expose from `camera_debug_app.h`:

```c
typedef enum
{
    CAMERA_DEBUG_INIT_NOT_RUN = 0,
    CAMERA_DEBUG_INIT_CAMERA_FAILED,
    CAMERA_DEBUG_INIT_WIFI_FAILED,
    CAMERA_DEBUG_INIT_SOCKET_FAILED,
    CAMERA_DEBUG_INIT_OK
} camera_debug_init_state_enum;

typedef struct
{
    uint32 frame_count;
    uint32 snapshot_count;
    uint32 sent_count;
    uint32 dropped_count;
    uint32 timeout_count;
    uint32 last_frame_ms;
    uint32 last_send_ms;
    uint32 last_send_duration_ms;
    uint32 max_send_duration_ms;
    uint8 frame_valid;
    uint8 init_state;
} camera_debug_diag_struct;

uint8 camera_debug_app_init(void);
void camera_debug_app_tick_1ms(void);
void camera_debug_app_service(void);
uint32 camera_debug_app_now_ms(void);
const camera_debug_diag_struct *camera_debug_app_get_diag(void);
```

- [ ] **Step 5: Implement the unchanged-driver camera-only flow**

In `camera_debug_app.c`:

```c
static uint8 image_copy[MT9V03X_H][MT9V03X_W];
static seekfree_assistant_camera_struct camera_information;
```

Behavior:

1. `camera_debug_app_init()` clears diagnostics and attempts `mt9v03x_init()` once.
2. A nonzero camera init return records `CAMERA_DEBUG_INIT_CAMERA_FAILED`; `service()` retries no faster than once per second.
3. PIT_CH2 only increments a volatile millisecond counter.
4. `service()` observes `mt9v03x_finish_flag`, clears it, increments `frame_count`, records `last_frame_ms`, and sets frame validity.
5. At most once per 100 ms, copy exactly `MT9V03X_IMAGE_SIZE` bytes from `mt9v03x_image[0]` to `image_copy[0]` and increment `snapshot_count`.
6. With WiFi disabled, do not call any WiFi or Assistant transport function.
7. If no frame arrives for more than 200 ms after the first valid frame, clear `frame_valid` and increment `timeout_count` once per stale transition.
8. Never block, allocate dynamically, or send from the PIT/camera interrupt context.

- [ ] **Step 6: Wire CM7_1 main and PIT_CH2**

`main_cm7_1.c` must keep `clock_init()` and `intercore_memory_configure()`, initialize `PIT_CH2` for 1 ms, call `camera_debug_app_init()`, and call `camera_debug_app_service()` forever.

`pit0_ch2_isr()` must contain only:

```c
pit_isr_flag_clear(PIT_CH2);
camera_debug_app_tick_1ms();
```

Add new camera files to the CM7_1 IAR project only.

- [ ] **Step 7: Run static checks and build all cores**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_camera_seekfree_api_static.ps1
git diff --check
```

Build all three IAR projects. Confirm no new UART0 initialization appears on CM7_1 and re-check all fixed-address map evidence.

- [ ] **Step 8: Run the 60-second CM7_1 portability gate**

With wheel power disabled, watch:

```text
camera_debug_diag.init_state
camera_debug_diag.frame_count
camera_debug_diag.snapshot_count
camera_debug_diag.last_frame_ms
camera_debug_diag.frame_valid
image_copy[0][0]
image_copy[60][94]
```

Acceptance:

```text
init_state == CAMERA_DEBUG_INIT_OK
frame_count increases continuously for 60 seconds
snapshot_count increases about 10 times/second
sample pixels respond to scene changes
frame_valid remains 1
wheel duty remains zero even after sending a motion command
servos remain in the safe 90-degree reference pose
```

If this fails for a core-dependent reason, mark Gate C `FAIL`, commit the evidence, and stop under STOP B.

- [ ] **Step 9: Commit the camera-only portability result**

```powershell
git add project/code/camera_debug_config.h project/code/camera_debug_app.c project/code/camera_debug_app.h project/code/app_config.h project/code/actuator_motor.c project/code/board_adc.c project/user/main_cm7_1.c project/user/cm7_1_isr.c project/iar/project_config/cyt4bb7_cm_7_1.ewp tools/test_camera_seekfree_api_static.ps1 docs/camera-seekfree-api-hardware-test.md
git commit -m "Prove MT9V03X capture on CM7_1"
```

---

### Task 5: Enable the existing WiFi-SPI Assistant display on CM7_1

**Files:**
- Modify: `project/code/camera_debug_config.h`
- Modify: `project/code/camera_debug_app.c`
- Modify: `tools/test_camera_seekfree_api_static.ps1`
- Modify: `docs/camera-seekfree-api-hardware-test.md`

- [ ] **Step 1: Write the failing transport assertions**

Require:

```text
APP_CAMERA_WIFI_ENABLE is 1U
wifi_spi_init uses config macros
wifi_spi_socket_connect uses TCP and explicit target/local ports
seekfree_assistant_interface_init selects SEEKFREE_ASSISTANT_WIFI_SPI
seekfree_assistant_camera_config uses the E9 object API and image_copy[0]
seekfree_assistant_camera_send receives &camera_information
send is reachable only from camera_debug_app_service
display period remains 100 ms
there is no direct application call to wifi_spi_send_buffer
```

Run and expect FAIL while WiFi remains disabled.

- [ ] **Step 2: Add bounded WiFi initialization and reconnect state**

Set `APP_CAMERA_WIFI_ENABLE` to `1U`. After camera init succeeds:

```c
wifi_spi_init(APP_CAMERA_WIFI_SSID, APP_CAMERA_WIFI_PASSWORD);
wifi_spi_socket_connect("TCP",
                        APP_CAMERA_WIFI_TARGET_IP,
                        APP_CAMERA_WIFI_TARGET_PORT,
                        APP_CAMERA_WIFI_LOCAL_PORT);
seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
seekfree_assistant_camera_config(&camera_information,
                                 SEEKFREE_ASSISTANT_CAMERA_TYPE_MT9V03X,
                                 MT9V03X_W,
                                 MT9V03X_H,
                                 image_copy[0]);
```

On init/socket failure, record the matching state and retry no faster than once every 5 seconds. Do not retry inside an ISR and do not block CM7_0.

- [ ] **Step 3: Send latest snapshots at 10 FPS**

On a completed frame and an expired 100 ms display period:

1. copy the frame to `image_copy`;
2. take the millisecond timestamp;
3. call `seekfree_assistant_camera_send(&camera_information)`;
4. record duration, maximum duration, and `sent_count` after return;
5. if another frame arrived while sending, count the skipped display opportunity as dropped rather than queueing it.

The Assistant send API returns `void`; do not invent a per-frame success result. `sent_count` means the synchronous call returned, while connection/init failures remain separate state.

- [ ] **Step 4: Run static checks and all-core builds**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_camera_seekfree_api_static.ps1
git diff --check
```

Build CM0+, CM7_0, and CM7_1, then repeat the fixed-address and SRAM map checks.

- [ ] **Step 5: Run the 60-second WiFi display gate**

Acceptance:

```text
camera and WiFi/socket initialization succeed
Assistant shows changing 188 x 120 grayscale video for >= 60 seconds
sent_count increases about 10 times/second
frame age remains <= 200 ms
max_send_duration_ms is recorded
no growing frame queue exists
CM7_0 heartbeat/IMU continue normally
all BLDC duty output remains zero
servos stay at the safe reference pose
```

Record actual frame counts, sent counts, drops, timeouts, maximum send duration, and any visible tearing.

- [ ] **Step 6: Commit the raw WiFi display milestone**

```powershell
git add project/code/camera_debug_config.h project/code/camera_debug_app.c tools/test_camera_seekfree_api_static.ps1 docs/camera-seekfree-api-hardware-test.md
git commit -m "Stream MT9V03X frames through Seekfree WiFi"
```

---

### Task 6: Final verification and handoff

**Files:**
- Modify: `docs/camera-seekfree-api-hardware-test.md`
- Modify only if a check is incomplete: `tools/test_camera_seekfree_api_static.ps1`

- [ ] **Step 1: Run repository checks from a clean build**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_camera_seekfree_api_static.ps1
git diff --check
git status --short
```

Clean generated IAR output with `project\iar\删除临时文件IAR.bat`, then rebuild all three cores in IAR 9.40.1. Do not commit generated IAR output.

- [ ] **Step 2: Repeat map and ownership audit**

Verify the three fixed MT9V03X objects, the two linker holes, the 8 KiB inter-core range, CM7_1 ordinary SRAM, and unique ownership of every camera/WiFi pin. Confirm P06_5 bus-current ADC reads are compiled out in camera-debug mode.

- [ ] **Step 3: Repeat final hardware smoke test**

Record:

```text
CM0+ starts both M7 cores
CM7_0 IMU init and heartbeat
servo safe 90-degree reference behavior
wheel duty observed as zero
CM7_1 camera init and repeated frame count
Assistant image stability for 60 seconds
display FPS, drops, timeouts, max send duration
```

Do not perform wheel motion in this plan.

- [ ] **Step 4: Complete the disposition**

Set exactly one outcome in the hardware-test document:

```text
PASS — CM7_1 owns camera snapshot and WiFi Assistant display
FAIL — use E8_09-style CM7_0 capture / CM7_1 send fallback plan
BLOCKED — reference hardware or toolchain gate not completed
```

List the next planned milestone as boundary/centerline overlays for the slalom algorithm. Keep GPS and product wireless architecture as separate follow-up work.

- [ ] **Step 5: Commit final evidence if changed**

```powershell
git add docs/camera-seekfree-api-hardware-test.md tools/test_camera_seekfree_api_static.ps1
git commit -m "Verify MT9V03X WiFi display milestone"
```

Skip this commit if the files are unchanged.

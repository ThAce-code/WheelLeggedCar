# MT9V03X Cross-Core Handoff Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Capture MT9V03X frames on CM7_0, hand complete latest-ready frames through a non-cacheable two-slot data plane, and process/display them from CM7_1 without disturbing the existing control loops.

**Architecture:** Keep the existing 8 KiB inter-core region as a versioned control plane and carve a separate 64 KiB non-cacheable image plane at `0x28060000`. CM7_0 is the sole camera producer and may publish into a `FREE` slot at most every 100 ms; CM7_1 selects the newest `READY` slot, holds it as `READING` through future processing and synchronous WiFi sending, then releases it. Slot states and sequence numbers are authoritative; IPC notification is only a one-way hint.

**Tech Stack:** Embedded C, CYT4BB/Traveo II dual Cortex-M7, IAR Embedded Workbench 9.40.1, Seekfree MT9V03X/WiFi-SPI/Assistant APIs, host GCC C11 tests, PowerShell static tests.

## Global Constraints

- Follow `docs/superpowers/specs/2026-07-14-mt9v03x-cross-core-handoff-design.md` exactly.
- Keep `APP_CAMERA_DEBUG_ONLY` equal to `1U`; both wheel duties must remain hard-clamped to zero.
- Keep P06_5 bus-current ADC initialization and conversion compiled out in camera-debug firmware.
- Keep the existing control plane at `0x28080000-0x28081FFF` and CM7_1 ordinary SRAM at `0x28082000`.
- Reserve the camera data plane at `0x28060000-0x2806FFFF`; do not use the E8_09 raw addresses.
- Slot 0 is `0x28060000-0x2806581F`; slot 1 is `0x28065820-0x2806B03F`.
- Keep the source-matched MT9V03X and Assistant file hashes unchanged.
- CM7_0 owns `zf_device_mt9v03x.c`, camera initialization, capture, and handoff copying.
- CM7_1 owns future vision processing and WiFi-SPI Assistant sending; it must not initialize the camera.
- Use `Cy_SysInt_DisableIRQ(tcpwm_0_interrupts_59_IRQn)` for source stability. Never call `NVIC_DisableIRQ(CPUIntIdx3_IRQn)` because UART sources share that aggregate CPU interrupt index.
- Do not disable global interrupts, rewrite the camera driver, add a frame FIFO, add custom packetization, or send from an ISR.
- Do not run wheel motion or `LXY` during hardware gates. Keep the servos at the validated all-90-degree reference.
- Stop after any failed hardware gate and commit honest evidence before proposing a deeper driver or architecture change.

## Fixed Interfaces

The implementation uses these names consistently across all tasks:

```c
#define INTERCORE_CAMERA_DATA_BASE_ADDRESS   (0x28060000UL)
#define INTERCORE_CAMERA_DATA_SIZE_BYTES     (65536U)
#define INTERCORE_CAMERA_SLOT_COUNT          (2U)
#define INTERCORE_CAMERA_SLOT_SIZE_BYTES     (22560U)
#define INTERCORE_CAMERA_MAGIC               (0x43414D52UL)
#define INTERCORE_CAMERA_VERSION             (1U)
#define INTERCORE_CAMERA_FORMAT_GRAY8        (1U)
#define INTERCORE_CAMERA_WIDTH               (188U)
#define INTERCORE_CAMERA_HEIGHT              (120U)
#define INTERCORE_CAMERA_STRIDE              (188U)

typedef enum
{
    INTERCORE_CAMERA_SLOT_FREE = 0,
    INTERCORE_CAMERA_SLOT_WRITING = 1,
    INTERCORE_CAMERA_SLOT_READY = 2,
    INTERCORE_CAMERA_SLOT_READING = 3
} intercore_camera_slot_state_enum;

typedef enum
{
    INTERCORE_CAMERA_NO_FRAME = 0,
    INTERCORE_CAMERA_OK = 1,
    INTERCORE_CAMERA_INVALID = 2,
    INTERCORE_CAMERA_NO_FREE_SLOT = 3,
    INTERCORE_CAMERA_EPOCH_CHANGED = 4
} intercore_camera_result_enum;

typedef struct
{
    uint8 slot_index;
    uint32 sequence;
    uint32 capture_ms;
    uint32 publish_ms;
    uint16 width;
    uint16 height;
    uint16 stride;
    uint32 frame_bytes;
    volatile uint8 *pixels;
} intercore_camera_frame_view_struct;

typedef struct
{
    volatile intercore_camera_control_struct *control;
    volatile uint8 *data_plane;
    uint32 boot_epoch;
    uint32 last_consumed_sequence;
    uint8 role;
    uint8 attached;
} intercore_camera_transport_struct;
```

The public transport functions are:

```c
uint8 intercore_camera_producer_init(
    intercore_camera_transport_struct *transport,
    volatile intercore_shared_layout_struct *shared,
    volatile uint8 *data_plane,
    uint32 boot_epoch);
uint8 intercore_camera_consumer_attach(
    intercore_camera_transport_struct *transport,
    volatile intercore_shared_layout_struct *shared,
    volatile uint8 *data_plane,
    uint32 boot_epoch);
void intercore_camera_producer_record_capture(
    intercore_camera_transport_struct *transport,
    uint32 capture_ms);
intercore_camera_result_enum intercore_camera_producer_claim(
    intercore_camera_transport_struct *transport,
    uint8 *slot_index,
    volatile uint8 **pixels);
uint8 intercore_camera_producer_publish(
    intercore_camera_transport_struct *transport,
    uint8 slot_index,
    uint32 capture_ms,
    uint32 publish_ms);
intercore_camera_result_enum intercore_camera_consumer_acquire_latest(
    intercore_camera_transport_struct *transport,
    intercore_camera_frame_view_struct *view);
uint8 intercore_camera_consumer_release(
    intercore_camera_transport_struct *transport,
    const intercore_camera_frame_view_struct *view);
```

---

### Task 1: Add the two-slot protocol, linker reservation, and MPU mapping

**Files:**
- Create: `project/code/intercore_camera.h`
- Create: `project/code/intercore_camera.c`
- Create: `project/tests/intercore_camera_handoff_test.c`
- Modify: `project/code/intercore_protocol.h`
- Modify: `project/code/intercore_memory.h`
- Modify: `project/code/intercore_memory.c`
- Modify: `project/code/intercore_notify.h`
- Modify: `project/iar/icf/linker_directives_tviibh.icf`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_0.ewp`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_1.ewp`
- Modify: `tools/test_camera_seekfree_api_static.ps1`
- Modify: `docs/camera-seekfree-api-hardware-test.md`

**Interfaces:**
- Consumes: existing `intercore_shared_layout_struct`, `intercore_transport_struct`, and non-cacheable shared-memory setup.
- Produces: the fixed interfaces above, the 256-byte camera control block at offset `0xC00`, linker symbols `__camera_shared_sram_base` and `__camera_shared_sram_size`, and `INTERCORE_NOTIFY_CAMERA_READY` equal to `0x00000008UL`.

- [ ] **Step 1: Write the failing host state-machine tests**

Create `project/tests/intercore_camera_handoff_test.c` using the existing `TEST_CHECK` style. Define the shared fixture and deterministic publish helper first:

```c
static _Alignas(32) intercore_shared_layout_struct shared;
static _Alignas(32) uint8 camera_data[INTERCORE_CAMERA_DATA_SIZE_BYTES];
static intercore_camera_transport_struct producer;
static intercore_camera_transport_struct consumer;

static void fixture_init(void)
{
    memset(&shared, 0, sizeof(shared));
    memset(camera_data, 0, sizeof(camera_data));
    shared.metadata.magic = INTERCORE_PROTOCOL_MAGIC;
    shared.metadata.version = INTERCORE_PROTOCOL_VERSION;
    shared.metadata.layout_size = (uint16)sizeof(shared);
    shared.metadata.boot_epoch = 1U;
    shared.metadata.cm7_0_ready = 1U;
    TEST_CHECK(1U == intercore_camera_producer_init(
                         &producer, &shared, camera_data, 1U));
    TEST_CHECK(1U == intercore_camera_consumer_attach(
                         &consumer, &shared, camera_data, 1U));
}

static void fixture_publish(uint8 expected_slot, uint32 sequence)
{
    uint8 slot_index = 0xFFU;
    volatile uint8 *pixels = NULL;

    shared.camera.capture_sequence = sequence - 1U;
    intercore_camera_producer_record_capture(&producer, sequence * 10U);
    TEST_CHECK(INTERCORE_CAMERA_OK ==
               intercore_camera_producer_claim(&producer, &slot_index, &pixels));
    TEST_CHECK(expected_slot == slot_index);
    pixels[0] = (uint8)sequence;
    TEST_CHECK(1U == intercore_camera_producer_publish(
                         &producer, slot_index, sequence * 10U, sequence * 10U + 1U));
}
```

Include complete cases for layout, normal handoff, newest-ready selection, occupied-slot protection, and epoch rejection:

```c
static void test_normal_handoff(void)
{
    uint8 slot_index = 0xFFU;
    volatile uint8 *pixels = NULL;
    intercore_camera_frame_view_struct view;

    fixture_init();
    intercore_camera_producer_record_capture(&producer, 100U);
    TEST_CHECK(INTERCORE_CAMERA_OK ==
               intercore_camera_producer_claim(&producer, &slot_index, &pixels));
    TEST_CHECK(INTERCORE_CAMERA_SLOT_WRITING == shared.camera.slot[slot_index].state);
    pixels[0] = 0x12U;
    pixels[INTERCORE_CAMERA_SLOT_SIZE_BYTES - 1U] = 0x34U;
    TEST_CHECK(1U == intercore_camera_producer_publish(&producer, slot_index, 100U, 101U));
    TEST_CHECK(INTERCORE_CAMERA_SLOT_READY == shared.camera.slot[slot_index].state);

    TEST_CHECK(INTERCORE_CAMERA_OK ==
               intercore_camera_consumer_acquire_latest(&consumer, &view));
    TEST_CHECK(slot_index == view.slot_index);
    TEST_CHECK(0x12U == view.pixels[0]);
    TEST_CHECK(0x34U == view.pixels[INTERCORE_CAMERA_SLOT_SIZE_BYTES - 1U]);
    TEST_CHECK(INTERCORE_CAMERA_SLOT_READING == shared.camera.slot[slot_index].state);
    TEST_CHECK(1U == intercore_camera_consumer_release(&consumer, &view));
    TEST_CHECK(INTERCORE_CAMERA_SLOT_FREE == shared.camera.slot[slot_index].state);
}

static void test_newest_ready_wins_without_fifo(void)
{
    intercore_camera_frame_view_struct view;

    fixture_publish(0U, 10U);
    fixture_publish(1U, 11U);
    TEST_CHECK(INTERCORE_CAMERA_OK ==
               intercore_camera_consumer_acquire_latest(&consumer, &view));
    TEST_CHECK(11U == view.sequence);
    TEST_CHECK(INTERCORE_CAMERA_SLOT_FREE == shared.camera.slot[0].state);
    TEST_CHECK(1U == shared.camera.stale_ready_drop_count);
}

static void test_reading_slot_is_never_overwritten(void)
{
    intercore_camera_frame_view_struct view;
    uint8 slot_index;
    volatile uint8 *pixels;

    fixture_publish(0U, 20U);
    TEST_CHECK(INTERCORE_CAMERA_OK ==
               intercore_camera_consumer_acquire_latest(&consumer, &view));
    fixture_publish(1U, 21U);
    TEST_CHECK(INTERCORE_CAMERA_NO_FREE_SLOT ==
               intercore_camera_producer_claim(&producer, &slot_index, &pixels));
    TEST_CHECK(INTERCORE_CAMERA_SLOT_READING == shared.camera.slot[view.slot_index].state);
    TEST_CHECK(1U == shared.camera.no_free_drop_count);
}

static void test_epoch_change_is_rejected(void)
{
    uint8 slot_index = 0xFFU;
    volatile uint8 *pixels = NULL;
    intercore_camera_frame_view_struct view;

    fixture_init();
    shared.metadata.boot_epoch = 2U;
    TEST_CHECK(INTERCORE_CAMERA_EPOCH_CHANGED ==
               intercore_camera_producer_claim(&producer, &slot_index, &pixels));
    TEST_CHECK(INTERCORE_CAMERA_EPOCH_CHANGED ==
               intercore_camera_consumer_acquire_latest(&consumer, &view));
}

static void test_consumer_restart_releases_only_stale_reading(void)
{
    fixture_init();
    shared.camera.slot[0].state = INTERCORE_CAMERA_SLOT_READING;
    shared.camera.slot[1].state = INTERCORE_CAMERA_SLOT_READY;
    TEST_CHECK(1U == intercore_camera_consumer_attach(
                         &consumer, &shared, camera_data, 1U));
    TEST_CHECK(INTERCORE_CAMERA_SLOT_FREE == shared.camera.slot[0].state);
    TEST_CHECK(INTERCORE_CAMERA_SLOT_READY == shared.camera.slot[1].state);
}

static void test_invalid_layout_is_rejected(void)
{
    fixture_init();
    shared.camera.frame_bytes = INTERCORE_CAMERA_SLOT_SIZE_BYTES - 1U;
    TEST_CHECK(0U == intercore_camera_consumer_attach(
                         &consumer, &shared, camera_data, 1U));
    TEST_CHECK(1U == shared.camera.invalid_layout_count);
}
```

Also assert:

```c
TEST_CHECK(32U == sizeof(intercore_camera_slot_struct));
TEST_CHECK(256U == sizeof(intercore_camera_control_struct));
TEST_CHECK(0xC00U == offsetof(intercore_shared_layout_struct, camera));
TEST_CHECK(0xD00U == offsetof(intercore_shared_layout_struct, reserved));
TEST_CHECK(8192U == sizeof(intercore_shared_layout_struct));
```

- [ ] **Step 2: Run the host test to verify RED**

Run:

```powershell
New-Item -ItemType Directory -Force project/tests/build | Out-Null
gcc -std=c11 -Wall -Wextra -Werror -DINTERCORE_HOST_TEST `
  -Ilibraries/zf_common -Iproject/code `
  project/tests/intercore_camera_handoff_test.c `
  project/code/intercore_protocol.c project/code/intercore_camera.c `
  -o project/tests/build/intercore_camera_handoff_test.exe
```

Expected: compilation fails because `intercore_camera.h/.c` and the camera control layout do not exist.

- [ ] **Step 3: Add the exact camera control layout**

In `intercore_protocol.h`:

```c
#define INTERCORE_PROTOCOL_VERSION              (2U)
#define INTERCORE_CAMERA_DATA_BASE_ADDRESS      (0x28060000UL)
#define INTERCORE_CAMERA_DATA_SIZE_BYTES        (65536U)
#define INTERCORE_CAMERA_SLOT_COUNT             (2U)
#define INTERCORE_CAMERA_SLOT_SIZE_BYTES        (22560U)
#define INTERCORE_CAMERA_MAGIC                  (0x43414D52UL)
#define INTERCORE_CAMERA_VERSION                (1U)
#define INTERCORE_CAMERA_FORMAT_GRAY8           (1U)
#define INTERCORE_CAMERA_WIDTH                  (188U)
#define INTERCORE_CAMERA_HEIGHT                 (120U)
#define INTERCORE_CAMERA_STRIDE                 (188U)

typedef struct
{
    uint32 state;
    uint32 sequence;
    uint32 capture_ms;
    uint32 publish_ms;
    uint32 frame_bytes;
    uint32 reserved[3];
}intercore_camera_slot_struct;

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
}intercore_camera_control_struct;
```

Insert `intercore_camera_control_struct camera;` after `health` in `intercore_shared_layout_struct`, reduce `reserved` to 4,864 bytes, and add compile-time checks for both camera structure sizes, camera offset `0xC00`, reserved offset `0xD00`, and total size 8,192. Do not move the existing metadata, navigation, control, events, or health offsets.

- [ ] **Step 4: Implement the host-testable state machine**

Create `intercore_camera.h/.c`. The producer chooses only `FREE`; the consumer chooses the highest-sequence `READY`. Use DMB macros matching `intercore_transport.c`:

```c
#if defined(INTERCORE_HOST_TEST)
#include <stdatomic.h>
#define INTERCORE_CAMERA_DMB() atomic_signal_fence(memory_order_seq_cst)
#else
#include "cy_project.h"
#define INTERCORE_CAMERA_DMB() __DMB()
#endif
```

`intercore_camera_producer_record_capture()` increments `captured_count`, advances `capture_sequence` with the same wrap rule as the existing transport (zero maps to 1), and records `last_capture_ms`. Claim does not change the sequence. Publish copies the current `capture_sequence` into the chosen slot.

Producer initialization clears only `shared->camera`, writes version/format/dimensions/stride/slot count/frame bytes/boot epoch, sets both states to `FREE`, executes a DMB, and writes `INTERCORE_CAMERA_MAGIC` last. Consumer attach validates every field and the shared metadata epoch before setting `attached`; on a CM7_1 restart it changes stale `READING` states to `FREE` but leaves `WRITING` and `READY` unchanged.

The claim/publish boundary must be explicit:

```c
slot = intercore_camera_find_free_slot(control);
if(INTERCORE_CAMERA_SLOT_COUNT == slot)
{
    control->no_free_drop_count++;
    return INTERCORE_CAMERA_NO_FREE_SLOT;
}
control->slot[slot].state = INTERCORE_CAMERA_SLOT_WRITING;
INTERCORE_CAMERA_DMB();
*slot_index = (uint8)slot;
*pixels = transport->data_plane + (slot * INTERCORE_CAMERA_SLOT_SIZE_BYTES);
return INTERCORE_CAMERA_OK;
```

Publish fields before state:

```c
control->slot[slot_index].sequence = control->capture_sequence;
control->slot[slot_index].capture_ms = capture_ms;
control->slot[slot_index].publish_ms = publish_ms;
control->slot[slot_index].frame_bytes = INTERCORE_CAMERA_SLOT_SIZE_BYTES;
control->latest_published_sequence = control->capture_sequence;
control->published_count++;
INTERCORE_CAMERA_DMB();
control->slot[slot_index].state = INTERCORE_CAMERA_SLOT_READY;
INTERCORE_CAMERA_DMB();
```

Acquire the newest ready slot, mark it `READING`, then release any older `READY` slot and increment `stale_ready_drop_count`. Release succeeds only when both state and sequence match the view. Validate magic, versions, dimensions, frame bytes, role, attachment, and boot epoch on every public state transition.

- [ ] **Step 5: Run host tests to GREEN**

Run the compile command from Step 2 and then:

```powershell
project/tests/build/intercore_camera_handoff_test.exe
```

Expected: `intercore_camera_handoff_test: PASS` and exit code 0.

Run the existing foundation test:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -DINTERCORE_HOST_TEST `
  -Ilibraries/zf_common -Iproject/code `
  project/tests/intercore_control_foundation_test.c `
  project/code/intercore_protocol.c project/code/intercore_transport.c `
  project/code/intercore_notify.c project/code/motion_command_router.c `
  project/code/intercore_control.c `
  -o project/tests/build/intercore_control_foundation_test.exe
project/tests/build/intercore_control_foundation_test.exe
```

Expected: `intercore_control_foundation_test: PASS`.

- [ ] **Step 6: Extend the static test to RED for linker and MPU contracts**

Add assertions to `tools/test_camera_seekfree_api_static.ps1` requiring:

```text
camera base/size linker symbols are 0x28060000/64K
CM7_0 ordinary SRAM excludes 0x28060000-0x2806FFFF
CM7_0 SRAM_HEAP_STACK begins at 0x28070000
shared control remains 0x28080000/8K
CM7_1 ordinary base remains 0x28082000
intercore_memory_configure validates both linker regions
one Cy_MPU_Setup call receives a two-element region array
both regions are NORM_SHR_MEM_NC and execute-never
both CM7 projects contain intercore_camera.c/.h
INTERCORE_NOTIFY_CAMERA_READY is 0x00000008UL
```

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_camera_seekfree_api_static.ps1
```

Expected: FAIL only on the new linker/MPU/project assertions.

- [ ] **Step 7: Reserve the image plane and configure both MPU regions**

In the ICF export:

```text
define symbol camera_shared_sram_base = 0x28060000;
define symbol camera_shared_sram_size = 64K;
define exported symbol __camera_shared_sram_base = camera_shared_sram_base;
define exported symbol __camera_shared_sram_size = camera_shared_sram_size;
```

Change CM7_0 ordinary regions to:

```text
0x28020000-0x28026023
0x2802B844-0x2805FFFF
0x28070000-0x2807FFFF
```

Set `SRAM_HEAP_STACK` to `0x28070000-0x2807FFFF`. Do not alter CM0+ holes or the fixed MT9V03X addresses.

In `intercore_memory.c`, validate both exported regions, clean/invalidate both before changing their attributes, and call `Cy_MPU_Setup(regions, 2U, ...)` once:

```c
const cy_stc_mpu_region_cfg_t regions[2] =
{
    {
        .addr = INTERCORE_SHARED_BASE_ADDRESS,
        .size = CY_MPU_SIZE_8KB,
        .permission = CY_MPU_ACCESS_P_FULL_ACCESS,
        .attribute = CY_MPU_ATTR_NORM_SHR_MEM_NC,
        .execute = CY_MPU_INST_ACCESS_DIS,
        .srd = 0U,
        .enable = CY_MPU_ENABLE
    },
    {
        .addr = INTERCORE_CAMERA_DATA_BASE_ADDRESS,
        .size = CY_MPU_SIZE_64KB,
        .permission = CY_MPU_ACCESS_P_FULL_ACCESS,
        .attribute = CY_MPU_ATTR_NORM_SHR_MEM_NC,
        .execute = CY_MPU_INST_ACCESS_DIS,
        .srd = 0U,
        .enable = CY_MPU_ENABLE
    }
};
```

Expose `intercore_memory_get_camera_data()` from `intercore_memory.h/.c`.

- [ ] **Step 8: Run static, host, and diff checks**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_camera_seekfree_api_static.ps1
project/tests/build/intercore_camera_handoff_test.exe
project/tests/build/intercore_control_foundation_test.exe
git diff --check
```

Expected: all PASS and `git diff --check` exit code 0.

- [ ] **Step 9: Fresh-build all cores and inspect maps**

Use IAR 9.40.1 to fresh-build CM0+, CM7_0, and CM7_1:

```powershell
$iarBuild = 'D:\IAR\common\bin\iarbuild.exe'
$projects = @(
    'project\iar\project_config\cyt4bb7_cm_0_plus.ewp',
    'project\iar\project_config\cyt4bb7_cm_7_0.ewp',
    'project\iar\project_config\cyt4bb7_cm_7_1.ewp'
)
foreach($project in $projects)
{
    & $iarBuild $project -clean Debug -log all
    if($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $iarBuild $project -build Debug -log all
    if($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
```

Record errors and warnings. Confirm maps show:

```text
camera data plane       0x28060000 size 0x10000
CM7_0 ordinary ranges   end at 0x2805FFFF and resume at 0x28070000
CM7_0 heap/stack        only in 0x28070000-0x2807FFFF
shared control          0x28080000 size 0x2000
CM7_1 ordinary base     0x28082000
```

Also re-check the three fixed MT9V03X objects. A linker overlap or map mismatch is a Task 1 failure; do not weaken the hole.

- [ ] **Step 10: Record and commit Task 1**

Update the hardware evidence with host/static/build/map results, then:

```powershell
git add project/code/intercore_camera.h project/code/intercore_camera.c `
  project/code/intercore_protocol.h project/code/intercore_memory.h `
  project/code/intercore_memory.c project/code/intercore_notify.h `
  project/tests/intercore_camera_handoff_test.c `
  project/iar/icf/linker_directives_tviibh.icf `
  project/iar/project_config/cyt4bb7_cm_7_0.ewp `
  project/iar/project_config/cyt4bb7_cm_7_1.ewp `
  tools/test_camera_seekfree_api_static.ps1 `
  docs/camera-seekfree-api-hardware-test.md
git commit -m "Add dual-slot camera handoff protocol"
```

---

### Task 2: Move capture to CM7_0 and prove cross-core handoff without WiFi

**Files:**
- Create: `project/code/camera_capture_producer.h`
- Create: `project/code/camera_capture_producer.c`
- Create: `project/code/camera_frame_consumer.h`
- Create: `project/code/camera_frame_consumer.c`
- Delete: `project/code/camera_debug_app.h`
- Delete: `project/code/camera_debug_app.c`
- Modify: `project/code/camera_debug_config.h`
- Modify: `project/user/main_cm7_0.c`
- Modify: `project/user/main_cm7_1.c`
- Modify: `project/user/cm7_1_isr.c`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_0.ewp`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_1.ewp`
- Modify: `tools/test_camera_seekfree_api_static.ps1`
- Modify: `docs/camera-seekfree-api-hardware-test.md`

**Interfaces:**
- Consumes: Task 1 camera transport, `mt9v03x_finish_flag`, `mt9v03x_image`, `app_run_once()`, `app_get_ms()`, and the existing notification API.
- Produces: `camera_capture_producer_init/service/get_diag`, `camera_frame_consumer_init/tick_1ms/service/get_diag`, changing CM7_1 sample pixels, and completed Hardware Gates 1 and 2.

- [ ] **Step 1: Extend static tests to RED for core ownership and source masking**

Replace the old Task 4 assertions that require `camera_debug_app` and CM7_1 camera ownership, then require all of the following:

```text
CM7_0 project contains zf_device_mt9v03x.c and the three IlinkKeepSymbols
CM7_1 project does not contain zf_device_mt9v03x.c or those keep symbols
CM7_0 main initializes and services camera_capture_producer after app_run_once
CM7_1 main initializes and services camera_frame_consumer
CM7_1 PIT_CH2 ISR calls only flag-clear plus camera_frame_consumer_tick_1ms
producer uses Cy_SysInt_DisableIRQ(tcpwm_0_interrupts_59_IRQn)
producer uses Cy_SysInt_EnableIRQ(tcpwm_0_interrupts_59_IRQn)
producer never calls NVIC_DisableIRQ(CPUIntIdx3_IRQn) or a global interrupt disable
producer copies exactly MT9V03X_IMAGE_SIZE bytes into a claimed shared slot
producer publishes no faster than APP_CAMERA_DISPLAY_PERIOD_MS=100U
consumer contains no camera init, camera driver flag, WiFi, or Assistant call
no camera copy, WiFi, or Assistant work appears in any ISR
```

Run the camera static test and expect these new assertions to fail.

- [ ] **Step 2: Define producer and consumer diagnostics**

In `camera_capture_producer.h` define:

```c
typedef enum
{
    CAMERA_CAPTURE_INIT_NOT_RUN = 0,
    CAMERA_CAPTURE_INIT_HANDOFF_FAILED,
    CAMERA_CAPTURE_INIT_CAMERA_FAILED,
    CAMERA_CAPTURE_INIT_OK
} camera_capture_init_state_enum;

typedef struct
{
    uint32 frame_count;
    uint32 publish_count;
    uint32 period_drop_count;
    uint32 no_free_drop_count;
    uint32 invalid_count;
    uint32 timeout_count;
    uint32 last_frame_ms;
    uint32 last_publish_ms;
    uint32 last_copy_duration_us;
    uint32 max_copy_duration_us;
    uint8 init_state;
    uint8 frame_valid;
} camera_capture_producer_diag_struct;

uint8 camera_capture_producer_init(void);
void camera_capture_producer_service(void);
const camera_capture_producer_diag_struct *camera_capture_producer_get_diag(void);
```

In `camera_frame_consumer.h` define:

```c
typedef enum
{
    CAMERA_CONSUMER_INIT_NOT_RUN = 0,
    CAMERA_CONSUMER_INIT_HANDOFF_FAILED,
    CAMERA_CONSUMER_INIT_WIFI_FAILED,
    CAMERA_CONSUMER_INIT_SOCKET_FAILED,
    CAMERA_CONSUMER_INIT_OK
} camera_consumer_init_state_enum;

typedef struct
{
    uint32 acquired_count;
    uint32 released_count;
    uint32 stale_ready_drop_count;
    uint32 invalid_count;
    uint32 timeout_count;
    uint32 last_sequence;
    uint32 last_capture_ms;
    uint32 last_frame_age_ms;
    uint8 sample_0_0;
    uint8 sample_center;
    uint8 frame_valid;
    uint8 init_state;
} camera_frame_consumer_diag_struct;

uint8 camera_frame_consumer_init(void);
void camera_frame_consumer_tick_1ms(void);
void camera_frame_consumer_service(void);
uint32 camera_frame_consumer_now_ms(void);
const camera_frame_consumer_diag_struct *camera_frame_consumer_get_diag(void);
```

With WiFi disabled, `CAMERA_CONSUMER_INIT_OK` means only that control/data-plane attachment succeeded. The WiFi/socket failure values remain unused until Task 3.

- [ ] **Step 3: Implement the CM7_0 producer**

Initialize `TC_TIME2_CH0` in microsecond mode for copy timing, initialize the camera transport as producer, and call `mt9v03x_init()` once with the existing one-second retry behavior.

In `camera_capture_producer_service()`:

```c
uint8 publish_ok;
uint8 slot_index;
uint32 copy_duration_us;
uint32 copy_start_us;
uint32 now_ms;
volatile uint8 *slot_pixels;
intercore_camera_result_enum result;

if(0U != mt9v03x_finish_flag)
{
    mt9v03x_finish_flag = 0U;
    now_ms = app_get_ms();
    intercore_camera_producer_record_capture(&camera_transport, now_ms);
    producer_diag.frame_count++;
    producer_diag.last_frame_ms = now_ms;
    producer_diag.frame_valid = 1U;

    if(APP_CAMERA_DISPLAY_PERIOD_MS <= (now_ms - producer_diag.last_publish_ms))
    {
        result = intercore_camera_producer_claim(&camera_transport, &slot_index, &slot_pixels);
        if(INTERCORE_CAMERA_OK == result)
        {
            copy_start_us = timer_get(TC_TIME2_CH0);
            Cy_SysInt_DisableIRQ(tcpwm_0_interrupts_59_IRQn);
            memcpy((void *)slot_pixels, mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
            publish_ok = intercore_camera_producer_publish(
                             &camera_transport, slot_index, now_ms, app_get_ms());
            copy_duration_us = timer_get(TC_TIME2_CH0) - copy_start_us;
            Cy_SysInt_EnableIRQ(tcpwm_0_interrupts_59_IRQn);
            producer_diag.last_copy_duration_us = copy_duration_us;
            producer_diag.max_copy_duration_us =
                (producer_diag.max_copy_duration_us < copy_duration_us) ?
                copy_duration_us : producer_diag.max_copy_duration_us;
            if(0U != publish_ok)
            {
                producer_diag.last_publish_ms = now_ms;
                producer_diag.publish_count++;
                (void)intercore_notify_try(INTERCORE_NOTIFY_CAMERA_READY);
            }
            else
            {
                producer_diag.invalid_count++;
            }
        }
        else
        {
            producer_diag.no_free_drop_count++;
        }
    }
    else
    {
        producer_diag.period_drop_count++;
    }
}
```

Do not clear TCPWM59 pending status. Do not call any WiFi or Assistant function. In `main_cm7_0.c`, call producer init after `app_init()` succeeds and producer service after every `app_run_once()`.

After the frame block, implement one-shot stale transition handling:

```c
now_ms = app_get_ms();
if((0U != producer_diag.frame_valid) &&
   (APP_CAMERA_STALE_TIMEOUT_MS < (now_ms - producer_diag.last_frame_ms)))
{
    producer_diag.frame_valid = 0U;
    if(0U == producer_stale_latched)
    {
        producer_stale_latched = 1U;
        producer_diag.timeout_count++;
        camera_transport.control->timeout_count++;
    }
}
```

Clear `producer_stale_latched` on every completed frame.

At every producer service pass, mirror `app_get_ms()` into `camera_transport.control->producer_heartbeat_ms`. After a copy, mirror last/max copy duration into the shared camera diagnostics. Increment shared `notify_count` only when `intercore_notify_try(INTERCORE_NOTIFY_CAMERA_READY)` returns success. These diagnostic writes do not change slot ownership.

- [ ] **Step 4: Implement the no-WiFi CM7_1 consumer**

Attach to the existing inter-core transport and camera data plane. Retry attach without blocking if CM7_0 is not ready. On each service call, consume the notification hint if present, but always poll state so notification loss is harmless.

For each acquired view:

```c
consumer_diag.sample_0_0 = view.pixels[0];
consumer_diag.sample_center =
    view.pixels[(60U * view.stride) + 94U];
consumer_diag.last_sequence = view.sequence;
consumer_diag.last_capture_ms = view.capture_ms;
consumer_diag.last_frame_age_ms = camera_frame_consumer_now_ms() - view.capture_ms;
consumer_diag.acquired_count++;
consumer_diag.frame_valid =
    (APP_CAMERA_STALE_TIMEOUT_MS >= consumer_diag.last_frame_age_ms) ? 1U : 0U;
(void)intercore_camera_consumer_release(&camera_transport, &view);
consumer_diag.released_count++;
```

Do not allocate or copy a local image. Delete the failed CM7_1 camera portability implementation and remove `zf_device_mt9v03x.c` plus its keep symbols from the CM7_1 project. Keep only the header if dimension constants are required.

At every consumer service pass, mirror the local millisecond tick into `camera_transport.control->consumer_heartbeat_ms`. A frame age above `APP_CAMERA_STALE_TIMEOUT_MS` increments both local and shared `timeout_count`, releases the slot without treating it as a valid observation, and is never reused. The transport release path updates shared `consumed_count` and `last_consume_ms` only after a matching `READING` state and sequence are verified.

- [ ] **Step 5: Run static and host regressions**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_camera_seekfree_api_static.ps1
project/tests/build/intercore_camera_handoff_test.exe
project/tests/build/intercore_control_foundation_test.exe
powershell -ExecutionPolicy Bypass -File tools/test_cm7_uart_ownership_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_imu_gyro_calibration_numeric.ps1
powershell -ExecutionPolicy Bypass -File tools/test_servo_300hz_integration_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_leg_ik_zero_calibration_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_ik_height_control_static.ps1
git diff --check
```

Expected: all PASS.

- [ ] **Step 6: Fresh-build all cores and inspect ownership maps**

Fresh-build CM0+, CM7_0, and CM7_1:

```powershell
$iarBuild = 'D:\IAR\common\bin\iarbuild.exe'
$projects = @(
    'project\iar\project_config\cyt4bb7_cm_0_plus.ewp',
    'project\iar\project_config\cyt4bb7_cm_7_0.ewp',
    'project\iar\project_config\cyt4bb7_cm_7_1.ewp'
)
foreach($project in $projects)
{
    & $iarBuild $project -clean Debug -log all
    if($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $iarBuild $project -build Debug -log all
    if($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
```

Require:

```text
CM7_0 map contains mt9v03x_h_num, mt9v03x_w_num, mt9v03x_image_temp
CM7_1 map contains no zf_device_mt9v03x.o and no camera interrupt callback
CM7_1 map contains no WiFi/Assistant call from the no-WiFi consumer
camera/control memory ranges remain exact
```

- [ ] **Step 7: Run Hardware Gate 1 — CM7_0 capture**

With wheel power removed and servos in the all-90-degree reference, download all three fresh images. Run for at least 60 seconds and watch:

```text
producer_diag.init_state
producer_diag.frame_count
producer_diag.last_frame_ms
producer_diag.frame_valid
mt9v03x_finish_flag
mt9v03x_image[0][0]
mt9v03x_image[60][94]
```

Move a high-contrast target. Require successful init, continuously increasing frame count, changing pixels, and no safety/control fault. If CM7_0 produces no frames in the robot firmware, set Gate 1 to FAIL, record evidence, commit it, and stop before Gate 2.

- [ ] **Step 8: Run Hardware Gate 2 — two-slot handoff without WiFi**

Run for another 60 seconds and watch producer, shared, and consumer diagnostics. Require:

```text
published and acquired sequences increase
CM7_1 samples respond to the same controlled scene change
acquired_count == released_count at frozen endpoints
no slot remains WRITING or READING after a debugger freeze/release cycle
no invalid layout/state count
frame age <= 200 ms
captured/published/no-free/period/stale/consumed accounting closes
max source-mask/copy duration < 1000 us
```

Record exact start/end values. This gate does not initialize WiFi.

- [ ] **Step 9: Record and commit Task 2**

Update the evidence with both gates, including any drops and timing. Commit:

```powershell
git add project/code/camera_capture_producer.h project/code/camera_capture_producer.c `
  project/code/camera_frame_consumer.h project/code/camera_frame_consumer.c `
  project/code/camera_debug_config.h project/code/camera_debug_app.h `
  project/code/camera_debug_app.c project/user/main_cm7_0.c `
  project/user/main_cm7_1.c project/user/cm7_1_isr.c `
  project/iar/project_config/cyt4bb7_cm_7_0.ewp `
  project/iar/project_config/cyt4bb7_cm_7_1.ewp `
  tools/test_camera_seekfree_api_static.ps1 `
  docs/camera-seekfree-api-hardware-test.md
git commit -m "Move MT9V03X capture to CM7_0"
```

Use `git add -u` only for the two deleted `camera_debug_app` files if their explicit paths cannot be staged after deletion; inspect the index before committing.

---

### Task 3: Enable CM7_1 vision boundary and WiFi-SPI display

**Files:**
- Modify: `project/code/camera_frame_consumer.h`
- Modify: `project/code/camera_frame_consumer.c`
- Modify: `project/code/camera_debug_config.h`
- Modify: `tools/test_camera_seekfree_api_static.ps1`
- Modify: `docs/camera-seekfree-api-hardware-test.md`

**Interfaces:**
- Consumes: Task 2 acquired-frame view and existing E9 Assistant/WiFi-SPI APIs.
- Produces: a CM7_1 consumer that exposes the acquired view boundary, sends the same held slot at approximately 10 FPS, and records honest processing/send diagnostics. It does not implement a fake vision algorithm.

- [ ] **Step 1: Add failing static transport assertions**

Require:

```text
APP_CAMERA_WIFI_ENABLE is 1U
CM7_1 initializes WiFi from config macros and connects TCP explicitly
Assistant selects SEEKFREE_ASSISTANT_WIFI_SPI
two Assistant camera objects are configured, one for each fixed slot pointer
the acquired slot remains READING through seekfree_assistant_camera_send
release occurs only after the send call returns
no direct application call to wifi_spi_send_buffer/read_buffer
reconnect attempts are separated by at least APP_CAMERA_WIFI_RETRY_MS
no WiFi or Assistant call is reachable from an ISR or CM7_0
```

Run the camera static test and expect FAIL while WiFi remains disabled.

- [ ] **Step 2: Extend consumer diagnostics and define the vision boundary**

Add:

```c
typedef struct
{
    uint8 slot_index;
    uint32 sequence;
    uint32 capture_ms;
    uint32 frame_age_ms;
    uint16 width;
    uint16 height;
    uint16 stride;
    uint32 frame_bytes;
    volatile uint8 *pixels;
} camera_vision_frame_view_struct;
```

The current consumer converts the inter-core view to this read-only vision view and records shared/local `last_process_duration_us = 0U` and `max_process_duration_us = 0U`; it does not publish a navigation command. The later slalom milestone will insert the actual vision function at this boundary before send.

Extend diagnostics with WiFi/socket state, sent count, last/max send duration, reconnect count, stale count, and last sent sequence.

- [ ] **Step 3: Add bounded WiFi initialization and two camera objects**

Set `APP_CAMERA_WIFI_ENABLE` to `1U`. Initialize WiFi and TCP only from CM7_1:

```c
wifi_spi_init(APP_CAMERA_WIFI_SSID, APP_CAMERA_WIFI_PASSWORD);
wifi_spi_socket_connect("TCP",
                        APP_CAMERA_WIFI_TARGET_IP,
                        APP_CAMERA_WIFI_TARGET_PORT,
                        APP_CAMERA_WIFI_LOCAL_PORT);
seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
```

Configure two camera objects once, using:

```c
seekfree_assistant_camera_config(&camera_information[0],
                                 SEEKFREE_ASSISTANT_CAMERA_TYPE_MT9V03X,
                                 MT9V03X_W,
                                 MT9V03X_H,
                                 (uint8 *)(uintptr_t)INTERCORE_CAMERA_DATA_BASE_ADDRESS);
seekfree_assistant_camera_config(&camera_information[1],
                                 SEEKFREE_ASSISTANT_CAMERA_TYPE_MT9V03X,
                                 MT9V03X_W,
                                 MT9V03X_H,
                                 (uint8 *)(uintptr_t)(INTERCORE_CAMERA_DATA_BASE_ADDRESS +
                                                     INTERCORE_CAMERA_SLOT_SIZE_BYTES));
```

Do not acquire a frame while a blocking network reconnect attempt is in progress. Retry no faster than every 5 seconds.

- [ ] **Step 4: Send while holding READING ownership**

After acquiring the newest frame and recording the vision boundary:

```c
send_start_ms = camera_frame_consumer_now_ms();
seekfree_assistant_camera_send(&camera_information[view.slot_index]);
send_duration_ms = camera_frame_consumer_now_ms() - send_start_ms;
consumer_diag.last_send_duration_ms = send_duration_ms;
consumer_diag.max_send_duration_ms =
    (consumer_diag.max_send_duration_ms < send_duration_ms) ?
    send_duration_ms : consumer_diag.max_send_duration_ms;
consumer_diag.sent_count++;
consumer_diag.last_sent_sequence = view.sequence;
camera_transport.control->last_send_duration_ms = send_duration_ms;
camera_transport.control->max_send_duration_ms =
    (camera_transport.control->max_send_duration_ms < send_duration_ms) ?
    send_duration_ms : camera_transport.control->max_send_duration_ms;
(void)intercore_camera_consumer_release(&camera_transport, &view);
consumer_diag.released_count++;
```

The send API returns `void`; do not create a frame-success result. Connection/init failures remain separate states.

- [ ] **Step 5: Run static, host, and existing regressions**

Run the full command set from Task 2 Step 5 and require all PASS. Confirm static search finds no CM7_0 WiFi/Assistant call and no direct application `wifi_spi_send_buffer()` call.

- [ ] **Step 6: Fresh-build all cores and recheck maps**

Fresh-build all three cores:

```powershell
$iarBuild = 'D:\IAR\common\bin\iarbuild.exe'
$projects = @(
    'project\iar\project_config\cyt4bb7_cm_0_plus.ewp',
    'project\iar\project_config\cyt4bb7_cm_7_0.ewp',
    'project\iar\project_config\cyt4bb7_cm_7_1.ewp'
)
foreach($project in $projects)
{
    & $iarBuild $project -clean Debug -log all
    if($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $iarBuild $project -build Debug -log all
    if($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
```

Repeat every camera/control memory map check and verify CM7_1 now owns WiFi/Assistant while still containing no MT9V03X implementation.

- [ ] **Step 7: Run Hardware Gate 3 — WiFi display**

Use the proven 2.4 GHz hotspot and Assistant TCP server. With wheel power removed, run at least 60 seconds and record:

```text
camera/socket init state
published/acquired/sent/released counts
no-free and stale-ready drops
frame age and stale count
last/max copy duration
last/max send duration
slot states and sequences
TCP Established state and interface throughput
Assistant 188 x 120 changing image and displayed FPS
```

Acceptance:

```text
changing image for >= 60 s
effective send/display rate approximately 10 FPS
every producer publication start interval >= 100 ms
frame age <= 200 ms
no invalid state/layout/epoch errors
no slot overwrite while READING
no growing queue
TCP remains Established
```

- [ ] **Step 8: Run the control-regression comparison**

Capture a 60-second camera-disabled baseline and a 60-second streaming window from telemetry/Live Watch. Require:

```text
no new safety fault
IMU age remains < 30 ms
scheduler missed-tick delta is 0
streaming max-gap is no more than baseline + 1 ms
servo tick rate remains within 1% of 300 Hz
CM7_0 heartbeat continues without visible stall
max TCPWM59 source-mask duration < 1 ms
both BLDC duties remain 0
servos remain at the safe all-90-degree reference
```

If display passes but a control threshold fails, mark control-safe integration FAIL and do not proceed to a final PASS.

- [ ] **Step 9: Record and commit Task 3**

```powershell
git add project/code/camera_frame_consumer.h project/code/camera_frame_consumer.c `
  project/code/camera_debug_config.h tools/test_camera_seekfree_api_static.ps1 `
  docs/camera-seekfree-api-hardware-test.md
git commit -m "Stream cross-core camera frames on CM7_1"
```

---

### Task 4: Perform final clean verification and handoff

**Files:**
- Modify: `docs/camera-seekfree-api-hardware-test.md`
- Modify only if a missing assertion is discovered: `tools/test_camera_seekfree_api_static.ps1`

**Interfaces:**
- Consumes: all previous task deliverables.
- Produces: a clean, reviewed camera display foundation and an explicit boundary for the later CM7_1 slalom algorithm milestone.

- [ ] **Step 1: Run all repository checks from a clean test build**

Recompile both host tests with `-Wall -Wextra -Werror`, run them, then run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_camera_seekfree_api_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_cm7_uart_ownership_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_imu_gyro_calibration_numeric.ps1
powershell -ExecutionPolicy Bypass -File tools/test_servo_300hz_integration_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_leg_ik_zero_calibration_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_ik_height_control_static.ps1
git diff --check
git status --short
```

Expected: all PASS; only intentional evidence edits may remain.

- [ ] **Step 2: Clean generated IAR output and fresh-build all cores**

From `project/iar`, run `删除临时文件IAR.bat`, then return to the repository root and run:

```powershell
$iarBuild = 'D:\IAR\common\bin\iarbuild.exe'
$projects = @(
    'project\iar\project_config\cyt4bb7_cm_0_plus.ewp',
    'project\iar\project_config\cyt4bb7_cm_7_0.ewp',
    'project\iar\project_config\cyt4bb7_cm_7_1.ewp'
)
foreach($project in $projects)
{
    & $iarBuild $project -clean Debug -log all
    if($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $iarBuild $project -build Debug -log all
    if($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
```

Do not commit generated output. Repeat the complete fixed-address, camera-plane, control-plane, heap/stack, ownership, and CM7_1-base audit.

- [ ] **Step 3: Repeat the final 60-second hardware smoke test**

Record coordinated three-core startup, CM7_0 IMU/heartbeat/servo behavior, wheel-duty zero, CM7_0 camera frames, CM7_1 latest-ready acquisition, WiFi display, FPS, frame age, drop accounting, source-mask duration, send duration, and slot states. Do not reset CM7_0 independently during an active CM7_1 read.

- [ ] **Step 4: Set the final disposition**

Set exactly one result in the evidence document:

```text
PASS — CM7_0 capture / CM7_1 latest-ready vision-display foundation verified
FAIL — cross-core frame, timing, transport, or control regression gate failed
BLOCKED — required hardware, hotspot, Assistant, or IAR gate not completed
```

List the next milestone as: implement boundary/centerline/pile detection on `camera_vision_frame_view_struct`, then publish a time-bounded `NAVIGATION_SOURCE_VISION` command through the existing inter-core navigation transport.

- [ ] **Step 5: Commit final evidence only if changed**

```powershell
git add docs/camera-seekfree-api-hardware-test.md tools/test_camera_seekfree_api_static.ps1
git commit -m "Verify cross-core camera display"
```

Skip the commit if neither file changed.

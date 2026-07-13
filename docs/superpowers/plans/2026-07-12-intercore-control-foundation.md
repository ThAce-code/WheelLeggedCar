# Inter-core Control Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the first independently testable foundation for CM7_1-to-CM7_0 commands: an 8 KiB non-cacheable shared-memory contract, double-buffered snapshots, a nonblocking IPC doorbell, and a CM7_0 motion-command router.

**Architecture:** CM7_0 remains the sole owner of chassis, balance, motor, and servo control. CM7_1 may publish only a versioned `navigation_command_struct` into shared SRAM; CM7_0 polls and validates the snapshot, then submits it to a priority/timeout router. IPC carries notification bits only, and a busy IPC pipe never blocks either core or invalidates data already published in shared memory.

**Tech Stack:** Embedded C, CYT4BB/Traveo II dual Cortex-M7, Cypress PDL IPC/MPU APIs, IAR Embedded Workbench 9.40.1, host-side GCC C11 tests, PowerShell.

## Global Constraints

- Keep hard real-time control, IMU, safety, chassis, balance, motors, and servos on CM7_0.
- CM7_1 must not include actuator or balance private headers and must not initialize UART0.
- Reserve exactly `0x28080000` through `0x28081FFF` as 8 KiB shared SRAM; move CM7_1 private SRAM to `0x28082000`.
- Configure the 8 KiB region on both CM7 cores as shareable, normal, non-cacheable, execute-never, full-access memory before the first access.
- Use `__DMB()` around publication and consumption. `volatile` is not the coherency mechanism.
- Do not call the blocking `ipc_send_data()` from the new path. `intercore_notify_try()` performs one send attempt and returns.
- Raw images and dynamically allocated pointers never enter the shared layout.
- Navigation commands expire no later than 200 ms. CM7_0 measures age from its own receive time.
- CM7_1 cannot enable fast chassis mode or alter balance gains.
- Do not change files under `libraries`; wrap the existing IPC and MPU APIs in `project/code`.
- Build `cyt4bb7_cm_0_plus`, `cyt4bb7_cm_7_0`, and `cyt4bb7_cm_7_1` with IAR 9.40.1 after linker or project-membership changes.
- Keep motors disabled for shared-memory and IPC bring-up. Motor-enabled testing begins only after the static, host-test, build, and stand checks in Task 7 pass.

---

## Scope Split and Dependency Roadmap

The approved architecture contains nine independently reviewable deliverables. This document is the executable plan for item 1 only:

1. **This plan:** shared SRAM, nonblocking notification, CM7_0 inter-core receiver, and motion-command router.
2. CM7_1 application skeleton, PIT_CH2 scheduler, heartbeat, and reverse control status.
3. GNSS wrapper and read-only wireless GNSS/health upload.
4. Camera driver integration, DMA ownership, and frame buffering.
5. Vision-pipeline interface and first bounded algorithm.
6. UDP command protocol, acknowledgements, telemetry, and wireless manual control.
7. Rate-limited video fragmentation and host reconstruction.
8. Navigation-manager integration and vision-assistance safety gates.
9. GPS waypoint navigation under a separate safety review.

Item 2 consumes the interfaces locked by this plan. Items 3 through 9 must not alter the shared offsets, record header, source enumeration, or router semantics without incrementing `INTERCORE_PROTOCOL_VERSION`.

## File and Responsibility Map

### Create

| File | Core membership | Responsibility |
|---|---|---|
| `project/code/intercore_protocol.h` | CM7_0, CM7_1, host test | Fixed types, enums, constants, exact shared layout, compile-time size/offset checks |
| `project/code/intercore_protocol.c` | CM7_0, CM7_1, host test | CRC32, record preparation/validation, structural navigation validation |
| `project/code/intercore_transport.h` | CM7_0, CM7_1, host test | Transport context and publication/consumption API |
| `project/code/intercore_transport.c` | CM7_0, CM7_1, host test | Boot epoch, double-slot snapshot publication, stable-copy consumption, counters |
| `project/code/intercore_notify.h` | CM7_0, CM7_1, host test | Nonblocking doorbell state machine and notification bit API |
| `project/code/intercore_notify.c` | CM7_0, CM7_1, host test | At-most-one-in-flight notification logic |
| `project/code/intercore_notify_port.h` | CM7_0, CM7_1, host test mock | Narrow hardware-port interface for IPC pipe setup and one-shot send |
| `project/code/intercore_notify_port.c` | CM7_0, CM7_1 | Cypress IPC pipe adapter; callback bodies only release/set bits |
| `project/code/intercore_memory.h` | CM7_0, CM7_1 | Linker-symbol access and MPU configuration API |
| `project/code/intercore_memory.c` | CM7_0, CM7_1 | Cache maintenance followed by 8 KiB shareable/non-cacheable MPU setup |
| `project/code/motion_command_router.h` | CM7_0, host test | Source, request, stop-reason, diagnostic types and router API |
| `project/code/motion_command_router.c` | CM7_0, host test | Priority, timeout, arm, emergency-stop, and maintenance arbitration |
| `project/code/motion_command_router_port.h` | CM7_0, host test mock | Two-function adapter boundary to chassis control |
| `project/code/motion_command_router_port.c` | CM7_0 | Only application file allowed to call `control_chassis_set_cmd()`/`stop()` |
| `project/code/intercore_control.h` | CM7_0 | CM7_0 receiver lifecycle and 1 ms service API |
| `project/code/intercore_control.c` | CM7_0 | Snapshot polling, CM7-local range/sequence checks, source mapping, router submission |
| `project/tests/intercore_control_foundation_test.c` | Host only | Protocol, transport, notification, and router behavioral tests with mock ports |

### Modify

| File | Change |
|---|---|
| `.gitignore` | Ignore `/project/tests/build/` |
| `project/iar/icf/linker_directives_tviibh.icf` | Reserve/export the shared range and reduce/move CM7_1 private SRAM |
| `project/code/app_config.h` | Add exact inter-core and router timing/range constants |
| `project/code/app.c` | Initialize router and CM7_0 receiver after application state/safety initialization |
| `project/code/app_scheduler.c` | Poll inter-core state and update the router before `control_chassis_update()` |
| `project/code/host_command.c` | Route `C` and STOP; add arm/maintenance commands; gate direct diagnostics |
| `project/user/main_cm7_0.c` | Configure MPU before `app_init()` and fail safely on configuration error |
| `project/user/main_cm7_1.c` | Remove UART0 debug initialization and configure the shared MPU region only |
| `project/iar/project_config/cyt4bb7_cm_7_0.ewp` | Add all shared modules plus CM7_0 router/receiver modules |
| `project/iar/project_config/cyt4bb7_cm_7_1.ewp` | Add shared protocol/transport/notify/memory modules only |

## Locked Public Interfaces

`intercore_protocol.h` must expose these constants and types. The numeric enum values are wire-format values and cannot be reordered:

```c
#define INTERCORE_SHARED_BASE_ADDRESS      (0x28080000UL)
#define INTERCORE_SHARED_SIZE_BYTES        (8192U)
#define INTERCORE_PROTOCOL_MAGIC           (0x574C4349UL)
#define INTERCORE_PROTOCOL_VERSION         (1U)
#define INTERCORE_NAVIGATION_MAX_VALID_MS  (200U)

typedef enum
{
    INTERCORE_RECORD_NAVIGATION = 1,
    INTERCORE_RECORD_CONTROL_STATUS = 2
}intercore_record_type_enum;

typedef enum
{
    NAVIGATION_SOURCE_NONE = 0,
    NAVIGATION_SOURCE_WIRELESS_MANUAL = 1,
    NAVIGATION_SOURCE_VISION = 2,
    NAVIGATION_SOURCE_WAYPOINT = 3
}navigation_source_enum;

typedef enum
{
    NAVIGATION_MODE_DISARMED = 0,
    NAVIGATION_MODE_MANUAL = 1,
    NAVIGATION_MODE_VISION_ASSIST = 2,
    NAVIGATION_MODE_WAYPOINT = 3
}navigation_mode_enum;

typedef enum
{
    NAVIGATION_STOP_NONE = 0,
    NAVIGATION_STOP_DISABLED = 1,
    NAVIGATION_STOP_STALE = 2,
    NAVIGATION_STOP_EMERGENCY = 3,
    NAVIGATION_STOP_INVALID = 4
}navigation_stop_reason_enum;

typedef struct
{
    uint32 magic;
    uint16 version;
    uint16 type;
    uint32 size;
    uint32 sequence;
    uint32 source_ms;
    uint32 crc32;
}intercore_header_struct;

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
}navigation_command_struct;

uint32 intercore_crc32(const uint8 *data, uint32 size);
void intercore_record_prepare(intercore_header_struct *header,
                              intercore_record_type_enum type,
                              uint32 sequence,
                              uint32 source_ms,
                              const void *payload,
                              uint32 payload_size);
uint8 intercore_record_validate(const intercore_header_struct *header,
                                intercore_record_type_enum expected_type,
                                const void *payload,
                                uint32 expected_payload_size);
uint8 intercore_navigation_is_structurally_valid(
    const navigation_command_struct *command);
```

`intercore_transport.h` must expose:

```c
typedef enum
{
    INTERCORE_ROLE_CM7_0 = 0,
    INTERCORE_ROLE_CM7_1 = 1
}intercore_role_enum;

typedef enum
{
    INTERCORE_TRANSPORT_NO_DATA = 0,
    INTERCORE_TRANSPORT_OK = 1,
    INTERCORE_TRANSPORT_INVALID = 2,
    INTERCORE_TRANSPORT_EPOCH_CHANGED = 3
}intercore_transport_result_enum;

typedef struct
{
    volatile intercore_shared_layout_struct *shared;
    uint32 boot_epoch;
    uint32 last_navigation_sequence;
    uint8 role;
    uint8 attached;
}intercore_transport_struct;

uint8 intercore_transport_cm7_0_init(intercore_transport_struct *transport,
                                     volatile intercore_shared_layout_struct *shared);
uint8 intercore_transport_cm7_1_attach(intercore_transport_struct *transport,
                                       volatile intercore_shared_layout_struct *shared);
uint8 intercore_transport_publish_navigation(intercore_transport_struct *transport,
                                             const navigation_command_struct *command,
                                             uint32 source_ms);
intercore_transport_result_enum intercore_transport_read_navigation(
    intercore_transport_struct *transport,
    navigation_command_struct *command,
    uint32 *record_sequence);
```

`motion_command_router.h` must expose:

```c
typedef enum
{
    MOTION_SOURCE_NONE = 0,
    MOTION_SOURCE_AUTONOMOUS = 1,
    MOTION_SOURCE_WIRELESS_MANUAL = 2,
    MOTION_SOURCE_UART_LOCAL = 3
}motion_command_source_enum;

typedef enum
{
    MOTION_STOP_NONE = 0,
    MOTION_STOP_EXPIRED,
    MOTION_STOP_MAINTENANCE,
    MOTION_STOP_EMERGENCY,
    MOTION_STOP_SAFETY,
    MOTION_STOP_INVALID
}motion_stop_reason_enum;

typedef struct
{
    float forward_rpm;
    float turn_rate_dps;
    uint32 source_sequence;
    uint32 received_ms;
    uint16 valid_for_ms;
    uint8 enable;
}motion_command_request_struct;

typedef struct
{
    uint32 accepted_count;
    uint32 rejected_count;
    uint32 expired_count;
    uint32 emergency_stop_count;
    uint32 last_command_age_ms;
    uint8 active_source;
    uint8 remote_armed;
    uint8 maintenance_mode;
    uint8 emergency_stop_latched;
    uint8 stop_reason;
}motion_command_router_diag_struct;

void motion_command_router_init(void);
uint8 motion_command_router_submit(motion_command_source_enum source,
                                   const motion_command_request_struct *request);
void motion_command_router_cancel(motion_command_source_enum source,
                                  motion_stop_reason_enum reason,
                                  uint32 now_ms);
void motion_command_router_set_maintenance(uint8 enable, uint32 now_ms);
uint8 motion_command_router_arm_remote(uint32 now_ms, uint8 safety_fault);
uint8 motion_command_router_clear_emergency_stop(uint32 now_ms, uint8 safety_fault);
void motion_command_router_emergency_stop(uint32 now_ms);
void motion_command_router_update(uint32 now_ms, uint8 safety_fault);
const motion_command_router_diag_struct *motion_command_router_get_diag(void);
```

---

### Task 1: Lock the Wire Protocol and Host Test Harness

**Files:**
- Create: `project/code/intercore_protocol.h`
- Create: `project/code/intercore_protocol.c`
- Create: `project/tests/intercore_control_foundation_test.c`
- Modify: `.gitignore`

**Interfaces:**
- Consumes: `uint8`, `uint16`, and `uint32` from `libraries/zf_common/zf_common_typedef.h`.
- Produces: the fixed structs above plus `intercore_crc32()`, `intercore_record_prepare()`, `intercore_record_validate()`, and `intercore_navigation_is_structurally_valid()`.

- [ ] **Step 1: Ignore host-test binaries**

Append exactly this line to `.gitignore`:

```gitignore
/project/tests/build/
```

- [ ] **Step 2: Write the first failing protocol tests**

Create `project/tests/intercore_control_foundation_test.c` with an assertion helper and these initial cases:

```c
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "intercore_protocol.h"

static uint32 test_failure_count = 0U;

#define TEST_CHECK(condition)                                                   \
    do                                                                          \
    {                                                                           \
        if(!(condition))                                                        \
        {                                                                       \
            printf("FAIL:%s:%d: %s\n", __FILE__, __LINE__, #condition);         \
            test_failure_count++;                                               \
        }                                                                       \
    }while(0)

static void test_protocol_crc_and_sizes(void)
{
    static const uint8 sample[] = "123456789";

    TEST_CHECK(0xCBF43926UL == intercore_crc32(sample, 9U));
    TEST_CHECK(24U == sizeof(intercore_header_struct));
    TEST_CHECK(24U == sizeof(navigation_command_struct));
    TEST_CHECK(8192U == sizeof(intercore_shared_layout_struct));
}

static void test_navigation_structural_validation(void)
{
    navigation_command_struct command = {0};

    command.forward_rpm = 20.0f;
    command.turn_rate_dps = -10.0f;
    command.confidence = 0.75f;
    command.source_sequence = 1U;
    command.valid_for_ms = 200U;
    command.enable = 1U;
    command.source = NAVIGATION_SOURCE_VISION;
    command.mode = NAVIGATION_MODE_VISION_ASSIST;
    TEST_CHECK(1U == intercore_navigation_is_structurally_valid(&command));

    command.forward_rpm = NAN;
    TEST_CHECK(0U == intercore_navigation_is_structurally_valid(&command));
    command.forward_rpm = 20.0f;
    command.valid_for_ms = 201U;
    TEST_CHECK(0U == intercore_navigation_is_structurally_valid(&command));
    command.valid_for_ms = 200U;
    command.confidence = 1.1f;
    TEST_CHECK(0U == intercore_navigation_is_structurally_valid(&command));
}
```

- [ ] **Step 3: Run the test to prove the protocol is absent**

Run:

```powershell
New-Item -ItemType Directory -Force project/tests/build | Out-Null
gcc -std=c11 -Wall -Wextra -Werror -Ilibraries/zf_common -Iproject/code project/tests/intercore_control_foundation_test.c project/code/intercore_protocol.c -o project/tests/build/intercore_control_foundation_test.exe
```

Expected: compilation fails because `intercore_protocol.h` and `intercore_protocol.c` do not exist.

- [ ] **Step 4: Implement the complete fixed layout**

Create `intercore_protocol.h` with the locked enums/structs and these exact layout members:

```c
typedef struct
{
    uint32 magic;
    uint16 version;
    uint16 layout_size;
    uint32 boot_epoch;
    uint32 cm7_0_ready;
    uint32 cm7_1_ready;
    uint32 navigation_active_index;
    uint32 navigation_sequence;
    uint32 control_active_index;
    uint32 control_sequence;
    uint32 event_write_index;
    uint32 event_sequence;
    uint8 reserved[212];
}intercore_metadata_struct;

typedef struct
{
    uint32 timestamp_ms;
    float pitch_deg;
    float pitch_rate_dps;
    float left_wheel_rpm;
    float right_wheel_rpm;
    uint32 command_age_ms;
    uint32 scheduler_missed_tick_count;
    uint32 scheduler_max_gap_ms;
    uint8 app_state;
    uint8 safety_fault;
    uint8 active_motion_source;
    uint8 reserved0;
    uint8 reserved[28];
}control_status_struct;

typedef struct
{
    intercore_header_struct header;
    navigation_command_struct payload;
    uint8 reserved[208];
}intercore_navigation_slot_struct;

typedef struct
{
    intercore_header_struct header;
    control_status_struct payload;
    uint8 reserved[424];
}intercore_control_slot_struct;

typedef struct
{
    uint32 sequence;
    uint16 type;
    uint16 size;
    uint32 source_ms;
    uint8 data[48];
    uint32 crc32;
}intercore_event_struct;

typedef struct
{
    uint32 cm7_0_heartbeat_ms;
    uint32 cm7_1_heartbeat_ms;
    uint32 cm7_0_publish_count;
    uint32 cm7_1_publish_count;
    uint32 cm7_0_consume_count;
    uint32 cm7_1_consume_count;
    uint32 notify_success_count;
    uint32 notify_busy_count;
    uint32 crc_error_count;
    uint32 version_error_count;
    uint32 duplicate_count;
    uint32 boot_epoch_change_count;
    uint8 reserved[208];
}intercore_health_struct;

typedef struct
{
    intercore_metadata_struct metadata;
    intercore_navigation_slot_struct navigation[2];
    intercore_control_slot_struct control[2];
    intercore_event_struct events[16];
    intercore_health_struct health;
    uint8 reserved[5120];
}intercore_shared_layout_struct;
```

Add negative-size typedef checks for all sizes and offsets. Use `offsetof()` and require metadata `0x000`, navigation `0x100`, control `0x300`, events `0x700`, health `0xB00`, and reserved `0xC00`.

- [ ] **Step 5: Implement CRC and record validation**

Create `intercore_protocol.c`. Use CRC-32/ISO-HDLC polynomial `0xEDB88320`, initial value `0xFFFFFFFF`, and final XOR `0xFFFFFFFF`. `intercore_record_prepare()` must zero `crc32`, populate all header fields, then calculate CRC over the first 20 header bytes followed by exactly `payload_size` bytes. `intercore_record_validate()` must check magic, version, expected type, exact payload size, and CRC in that order.

Use this finite check so IAR and GCC behave identically without a math-library dependency:

```c
static uint8 intercore_float_is_finite(float value)
{
    return ((value == value) &&
            (3.402823466e+38F >= value) &&
            (-3.402823466e+38F <= value)) ? 1U : 0U;
}
```

Structural navigation validation returns false for null pointers, non-finite floats, confidence outside `[0.0f, 1.0f]`, zero `source_sequence`, `valid_for_ms` outside `[1, 200]`, enable outside `{0,1}`, unknown source/mode/stop reason, or nonzero reserved bytes.

- [ ] **Step 6: Add record corruption tests and the test entry point**

Add this test, which prepares a navigation record, validates it, flips one payload byte, and verifies rejection:

```c
static void test_record_rejects_corruption(void)
{
    intercore_header_struct header = {0};
    navigation_command_struct command = {0};

    command.forward_rpm = 15.0f;
    command.turn_rate_dps = 5.0f;
    command.confidence = 0.8f;
    command.source_sequence = 4U;
    command.valid_for_ms = 200U;
    command.enable = 1U;
    command.source = NAVIGATION_SOURCE_VISION;
    command.mode = NAVIGATION_MODE_VISION_ASSIST;

    intercore_record_prepare(&header,
                             INTERCORE_RECORD_NAVIGATION,
                             9U,
                             50U,
                             &command,
                             sizeof(command));
    TEST_CHECK(1U == intercore_record_validate(&header,
                                               INTERCORE_RECORD_NAVIGATION,
                                               &command,
                                               sizeof(command)));
    ((uint8 *)&command)[0] ^= 0x01U;
    TEST_CHECK(0U == intercore_record_validate(&header,
                                               INTERCORE_RECORD_NAVIGATION,
                                               &command,
                                               sizeof(command)));
}
```

End the test file with:

```c
int main(void)
{
    test_protocol_crc_and_sizes();
    test_navigation_structural_validation();
    test_record_rejects_corruption();

    if(0U != test_failure_count)
    {
        printf("intercore_control_foundation_test: %u failure(s)\n",
               (unsigned int)test_failure_count);
        return 1;
    }

    printf("intercore_control_foundation_test: PASS\n");
    return 0;
}
```

- [ ] **Step 7: Run the protocol tests**

Run the GCC command from Step 3, followed by:

```powershell
project/tests/build/intercore_control_foundation_test.exe
```

Expected: `intercore_control_foundation_test: PASS` and exit code 0.

- [ ] **Step 8: Commit the protocol contract**

```powershell
git add .gitignore project/code/intercore_protocol.h project/code/intercore_protocol.c project/tests/intercore_control_foundation_test.c
git commit -m "Add inter-core protocol contract"
```

---

### Task 2: Add Double-buffered Shared-memory Transport

**Files:**
- Create: `project/code/intercore_transport.h`
- Create: `project/code/intercore_transport.c`
- Modify: `project/tests/intercore_control_foundation_test.c`

**Interfaces:**
- Consumes: `intercore_shared_layout_struct`, `navigation_command_struct`, record CRC functions.
- Produces: CM7_0 ownership initialization, CM7_1 attach, navigation publication, stable-copy consumption, and explicit transport results.

- [ ] **Step 1: Add failing transport tests**

Add this 32-byte-aligned test buffer and the initial transport assertions:

```c
static intercore_shared_layout_struct shared __attribute__((aligned(32)));

memset(&shared, 0, sizeof(shared));
TEST_CHECK(1U == intercore_transport_cm7_0_init(&receiver, &shared));
TEST_CHECK(INTERCORE_PROTOCOL_MAGIC == shared.metadata.magic);
TEST_CHECK(INTERCORE_PROTOCOL_VERSION == shared.metadata.version);
TEST_CHECK(1U == shared.metadata.boot_epoch);
TEST_CHECK(1U == intercore_transport_cm7_1_attach(&sender, &shared));
TEST_CHECK(1U == shared.metadata.cm7_1_ready);
TEST_CHECK(1U == intercore_transport_publish_navigation(&sender, &sent, 50U));
TEST_CHECK(INTERCORE_TRANSPORT_OK ==
           intercore_transport_read_navigation(&receiver, &received, &record_sequence));
TEST_CHECK(0 == memcmp(&sent, &received, sizeof(sent)));
TEST_CHECK(INTERCORE_TRANSPORT_NO_DATA ==
           intercore_transport_read_navigation(&receiver, &received, &record_sequence));
```

Add separate cases for CRC corruption, an active index above 1, and a protocol-version mismatch. Also prove that a valid CM7_0 reinitialization increments `boot_epoch`, makes the stale CM7_1 context refuse publication, and permits publication only after CM7_1 attaches again. In a separate receiver test, change the shared epoch after receiver initialization and verify `INTERCORE_TRANSPORT_EPOCH_CHANGED`.

- [ ] **Step 2: Compile to prove the transport API is absent**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -DINTERCORE_HOST_TEST -Ilibraries/zf_common -Iproject/code project/tests/intercore_control_foundation_test.c project/code/intercore_protocol.c project/code/intercore_transport.c -o project/tests/build/intercore_control_foundation_test.exe
```

Expected: compilation fails on missing transport declarations/definitions.

- [ ] **Step 3: Implement CM7_0 layout ownership and CM7_1 attach**

`intercore_transport_cm7_0_init()` must:

1. Reject null or non-32-byte-aligned pointers.
2. Preserve `previous_epoch` only when the existing magic, version, and layout size are valid; otherwise use zero.
3. Clear all 8192 bytes through a volatile byte loop.
4. Write version, layout size, `boot_epoch = previous_epoch + 1U` with zero wrapped to 1, and `cm7_0_ready = 1U`.
5. Execute the transport memory barrier and initialize the private context.

`intercore_transport_cm7_1_attach()` rejects a missing magic/version/layout/ready flag, then records the current epoch and sets `cm7_1_ready = 1U` after a barrier.

Define the barrier exactly as:

```c
#if defined(INTERCORE_HOST_TEST)
#include <stdatomic.h>
#define INTERCORE_DMB() atomic_signal_fence(memory_order_seq_cst)
#else
#include "cy_project.h"
#define INTERCORE_DMB() __DMB()
#endif
```

- [ ] **Step 4: Implement inactive-slot publication**

Publication must build a zeroed local 256-byte slot, validate the navigation payload, increment the shared record sequence with zero skipped, prepare the record CRC, copy the complete slot into the inactive shared slot through a volatile byte loop, execute `INTERCORE_DMB()`, update `navigation_active_index`, update `navigation_sequence`, execute a second barrier, and increment `cm7_1_publish_count`.

Do not publish when the transport role is not CM7_1, attach is incomplete, or the current shared epoch differs from the context epoch.

The commit order in the implementation must be visible in this form:

```c
inactive_index = (1U == transport->shared->metadata.navigation_active_index) ? 0U : 1U;
intercore_copy_to_volatile(
    (volatile uint8 *)&transport->shared->navigation[inactive_index],
    (const uint8 *)&local_slot,
    sizeof(local_slot));
INTERCORE_DMB();
transport->shared->metadata.navigation_active_index = inactive_index;
transport->shared->metadata.navigation_sequence = local_slot.header.sequence;
INTERCORE_DMB();
```

- [ ] **Step 5: Implement stable-copy consumption**

Consumption must:

1. Check role/attach and shared magic/version/layout.
2. Return `INTERCORE_TRANSPORT_EPOCH_CHANGED` immediately when the shared epoch differs, reset `last_navigation_sequence`, and adopt the new epoch.
3. Read active index and sequence; reject active index above 1.
4. Return `NO_DATA` when sequence is zero or equals `last_navigation_sequence`.
5. Copy the selected complete slot into a local slot through a volatile byte loop.
6. Execute `INTERCORE_DMB()` and reread active index/sequence.
7. Return `NO_DATA` when either changed during the copy.
8. Validate record header and CRC, then structurally validate the payload.
9. Copy the payload out, record the sequence, and increment `cm7_0_consume_count`.

CRC/version failures increment their dedicated health counters and return `INVALID`; they do not update `last_navigation_sequence`.

Use a before/after stability check around the copy:

```c
active_before = transport->shared->metadata.navigation_active_index;
sequence_before = transport->shared->metadata.navigation_sequence;
intercore_copy_from_volatile((uint8 *)&local_slot,
                             (const volatile uint8 *)&transport->shared->navigation[active_before],
                             sizeof(local_slot));
INTERCORE_DMB();
active_after = transport->shared->metadata.navigation_active_index;
sequence_after = transport->shared->metadata.navigation_sequence;
if((active_before != active_after) || (sequence_before != sequence_after))
{
    return INTERCORE_TRANSPORT_NO_DATA;
}
```

- [ ] **Step 6: Run all transport tests**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -DINTERCORE_HOST_TEST -Ilibraries/zf_common -Iproject/code project/tests/intercore_control_foundation_test.c project/code/intercore_protocol.c project/code/intercore_transport.c -o project/tests/build/intercore_control_foundation_test.exe
project/tests/build/intercore_control_foundation_test.exe
```

Expected: `intercore_control_foundation_test: PASS` and exit code 0.

- [ ] **Step 7: Commit the transport**

```powershell
git add project/code/intercore_transport.h project/code/intercore_transport.c project/tests/intercore_control_foundation_test.c
git commit -m "Add double-buffered inter-core transport"
```

---

### Task 3: Add a Nonblocking IPC Doorbell

**Files:**
- Create: `project/code/intercore_notify.h`
- Create: `project/code/intercore_notify.c`
- Create: `project/code/intercore_notify_port.h`
- Create: `project/code/intercore_notify_port.c`
- Modify: `project/tests/intercore_control_foundation_test.c`

**Interfaces:**
- Consumes: Cypress `Cy_IPC_Pipe_Init()`, `Cy_IPC_Pipe_RegisterCallback()`, and `Cy_IPC_Pipe_SendMessage()` only through the port.
- Produces: `intercore_notify_init()`, `intercore_notify_try()`, `intercore_notify_take_pending()`, release/receive callbacks, and success/busy counters.

- [ ] **Step 1: Add failing notification state tests**

The host test supplies mock implementations of:

```c
uint8 intercore_notify_port_init(intercore_role_enum role);
uint8 intercore_notify_port_send(const intercore_doorbell_struct *message);
```

Test these transitions:

- First send succeeds and marks one message in flight.
- A second call before the release callback returns false without invoking the port again.
- `intercore_notify_release_callback()` permits the next send.
- A port-busy result clears the in-flight flag and increments the busy count.
- `intercore_notify_receive_callback(INTERCORE_NOTIFY_NAVIGATION)` sets a pending bit, and `intercore_notify_take_pending()` returns and clears it.

The success/in-flight portion must contain these assertions:

```c
mock_notify_send_result = 1U;
mock_notify_send_count = 0U;
TEST_CHECK(1U == intercore_notify_init(INTERCORE_ROLE_CM7_1));
TEST_CHECK(1U == intercore_notify_try(INTERCORE_NOTIFY_NAVIGATION));
TEST_CHECK(1U == mock_notify_send_count);
TEST_CHECK(0U == intercore_notify_try(INTERCORE_NOTIFY_HEARTBEAT));
TEST_CHECK(1U == mock_notify_send_count);
intercore_notify_release_callback();
TEST_CHECK(1U == intercore_notify_try(INTERCORE_NOTIFY_HEARTBEAT));
intercore_notify_receive_callback(INTERCORE_NOTIFY_NAVIGATION);
TEST_CHECK(INTERCORE_NOTIFY_NAVIGATION == intercore_notify_take_pending());
TEST_CHECK(0U == intercore_notify_take_pending());
```

- [ ] **Step 2: Compile to prove notification code is absent**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -DINTERCORE_HOST_TEST -Ilibraries/zf_common -Iproject/code project/tests/intercore_control_foundation_test.c project/code/intercore_protocol.c project/code/intercore_transport.c project/code/intercore_notify.c -o project/tests/build/intercore_control_foundation_test.exe
```

Expected: compilation fails on missing notification declarations/definitions.

- [ ] **Step 3: Implement the notification state machine**

Use these fixed definitions:

```c
#define INTERCORE_NOTIFY_CLIENT_ID       (0x31UL)
#define INTERCORE_NOTIFY_NAVIGATION      (0x00000001UL)
#define INTERCORE_NOTIFY_CONTROL_STATUS  (0x00000002UL)
#define INTERCORE_NOTIFY_HEARTBEAT       (0x00000004UL)

typedef struct
{
    uint32 clientId;
    uint32 data;
}intercore_doorbell_struct;

typedef struct
{
    uint32 success_count;
    uint32 busy_count;
    uint32 received_count;
    uint32 pending_bits;
    uint8 initialized;
    uint8 in_flight;
}intercore_notify_diag_struct;

uint8 intercore_notify_init(intercore_role_enum role);
uint8 intercore_notify_try(uint32 bits);
uint32 intercore_notify_take_pending(void);
void intercore_notify_release_callback(void);
void intercore_notify_receive_callback(uint32 bits);
const intercore_notify_diag_struct *intercore_notify_get_diag(void);
```

`intercore_notify_try(bits)` must return immediately. If a message is already in flight, increment busy and return false. Otherwise set the static message, mark in-flight, call the port exactly once, and return true only on accepted send. There is no retry loop, delay call, or wait on the release callback.

- [ ] **Step 4: Implement the Cypress IPC port**

`intercore_notify_port_init()` configures endpoint 0 for CM7_0 and endpoint 1 for CM7_1 using `CY_IPC_PIPE_ENDPOINTS_DEFAULT_CONFIG`, registers the receive callback for client `0x31`, clears/enables the endpoint IRQ, and stores destination endpoint 1 for CM7_0 or 0 for CM7_1.

`intercore_notify_port_send()` calls:

```c
Cy_IPC_Pipe_SendMessage(intercore_notify_destination,
                        (void *)message,
                        intercore_notify_release_callback)
```

once and maps only `CY_IPC_PIPE_SUCCESS` to true. The receive callback extracts `data` and calls `intercore_notify_receive_callback(data)`; it performs no record parsing.

- [ ] **Step 5: Run host notification tests and scan for blocking calls**

Run the updated host test. Then run:

```powershell
rg -n "while|system_delay|ipc_send_data" project/code/intercore_notify.c project/code/intercore_notify_port.c
```

Expected: host test prints PASS; the scan finds no match.

- [ ] **Step 6: Commit the doorbell**

```powershell
git add project/code/intercore_notify.h project/code/intercore_notify.c project/code/intercore_notify_port.h project/code/intercore_notify_port.c project/tests/intercore_control_foundation_test.c
git commit -m "Add nonblocking inter-core notification"
```

---

### Task 4: Add the CM7_0 Motion-command Router

**Files:**
- Create: `project/code/motion_command_router.h`
- Create: `project/code/motion_command_router.c`
- Create: `project/code/motion_command_router_port.h`
- Create: `project/code/motion_command_router_port.c`
- Modify: `project/tests/intercore_control_foundation_test.c`

**Interfaces:**
- Consumes: timestamped requests and a two-function chassis port.
- Produces: deterministic source arbitration, remote arm state, maintenance exclusion, latched emergency stop, and diagnostics.

- [ ] **Step 1: Add failing router tests with a mock chassis port**

Mock these functions and record every call:

```c
void motion_command_router_port_apply(float forward_rpm,
                                      float turn_rate_dps,
                                      uint8 enable,
                                      uint32 now_ms);
void motion_command_router_port_stop(uint32 now_ms);
```

Add independent tests proving:

1. UART local overrides wireless manual, which overrides autonomous.
2. A request is stopped when `now_ms - received_ms >= valid_for_ms`.
3. Non-finite requests and `valid_for_ms == 0U` are rejected.
4. Remote sources are rejected before `motion_command_router_arm_remote()`.
5. Entering maintenance immediately stops and clears both remote sources.
6. Leaving maintenance leaves remote disarmed; an old request cannot resume.
7. Emergency stop is latched; ordinary submit calls cannot clear it.
8. Safety fault always calls the stop port and clears remote arm.

Use concrete timestamps and requests; the priority/timeout test must include:

```c
motion_command_request_struct autonomous = {10.0f, 1.0f, 1U, 100U, 200U, 1U};
motion_command_request_struct wireless = {20.0f, 2.0f, 2U, 100U, 200U, 1U};
motion_command_request_struct uart = {30.0f, 3.0f, 3U, 100U, 500U, 1U};

motion_command_router_init();
TEST_CHECK(1U == motion_command_router_arm_remote(100U, 0U));
TEST_CHECK(1U == motion_command_router_submit(MOTION_SOURCE_AUTONOMOUS, &autonomous));
TEST_CHECK(1U == motion_command_router_submit(MOTION_SOURCE_WIRELESS_MANUAL, &wireless));
TEST_CHECK(1U == motion_command_router_submit(MOTION_SOURCE_UART_LOCAL, &uart));
motion_command_router_update(101U, 0U);
TEST_CHECK(30.0f == mock_router_last_forward_rpm);
TEST_CHECK(MOTION_SOURCE_UART_LOCAL ==
           motion_command_router_get_diag()->active_source);
motion_command_router_update(600U, 0U);
TEST_CHECK(0U < mock_router_stop_count);
TEST_CHECK(MOTION_SOURCE_NONE ==
           motion_command_router_get_diag()->active_source);
```

- [ ] **Step 2: Compile to prove router code is absent**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -DINTERCORE_HOST_TEST -Ilibraries/zf_common -Iproject/code project/tests/intercore_control_foundation_test.c project/code/intercore_protocol.c project/code/intercore_transport.c project/code/intercore_notify.c project/code/motion_command_router.c -o project/tests/build/intercore_control_foundation_test.exe
```

Expected: compilation fails on missing router declarations/definitions.

- [ ] **Step 3: Implement request storage and validation**

Store one private slot for each nonzero source. Reject null requests, unknown sources, non-finite values, `enable > 1U`, validity outside `[1, 500]`, a remote validity above 200 ms, and a sequence that is not newer than the last accepted sequence for that source. A successful submit copies the complete request and sets the slot valid; it never calls the chassis port directly.

Remote submit additionally requires `remote_armed == 1U`, `maintenance_mode == 0U`, and `emergency_stop_latched == 0U`. UART local submit remains available when remote is disarmed, but is rejected during maintenance, emergency stop, or safety fault.

Use the same finite predicate as `intercore_protocol.c`, and reject before indexing the source array:

```c
if((MOTION_SOURCE_AUTONOMOUS > source) ||
   (MOTION_SOURCE_UART_LOCAL < source) ||
   (0U == motion_request_is_valid(request, source)))
{
    motion_router_diag.rejected_count++;
    return 0U;
}
```

Compare nonzero 32-bit sequences with wrap support:

```c
if((0U != motion_slots[source].last_sequence) &&
   (0 >= (int32)(request->source_sequence - motion_slots[source].last_sequence)))
{
    motion_router_diag.rejected_count++;
    return 0U;
}
```

- [ ] **Step 4: Implement update priority and stop behavior**

At each update:

1. If `safety_fault` is true, clear remote arm and all slots, set `MOTION_STOP_SAFETY`, and call stop.
2. Else if emergency stop is latched, call stop.
3. Expire every slot whose local age reaches its validity and increment `expired_count` once.
4. In maintenance mode, keep all motion slots invalid and remain stopped.
5. Select UART local, then wireless manual, then autonomous.
6. Call apply with the selected request and local `now_ms`; otherwise call stop only when transitioning from an active source or stop reason.

Entering or leaving maintenance clears all slots and calls stop synchronously. Emergency stop clears all slots, remote arm, and calls stop synchronously. Clearing emergency stop and arming remote both reject an active safety fault.

Keep source selection explicit and independent of enum ordering:

```c
if(1U == motion_slots[MOTION_SOURCE_UART_LOCAL].valid)
{
    selected_source = MOTION_SOURCE_UART_LOCAL;
}
else if(1U == motion_slots[MOTION_SOURCE_WIRELESS_MANUAL].valid)
{
    selected_source = MOTION_SOURCE_WIRELESS_MANUAL;
}
else if(1U == motion_slots[MOTION_SOURCE_AUTONOMOUS].valid)
{
    selected_source = MOTION_SOURCE_AUTONOMOUS;
}
```

- [ ] **Step 5: Implement the only chassis adapter**

`motion_command_router_port.c` contains exactly:

```c
#include "motion_command_router_port.h"
#include "control_chassis.h"

void motion_command_router_port_apply(float forward_rpm,
                                      float turn_rate_dps,
                                      uint8 enable,
                                      uint32 now_ms)
{
    control_chassis_set_cmd(forward_rpm, turn_rate_dps, enable, now_ms);
}

void motion_command_router_port_stop(uint32 now_ms)
{
    control_chassis_stop(now_ms);
}
```

- [ ] **Step 6: Run router tests**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -DINTERCORE_HOST_TEST -Ilibraries/zf_common -Iproject/code project/tests/intercore_control_foundation_test.c project/code/intercore_protocol.c project/code/intercore_transport.c project/code/intercore_notify.c project/code/motion_command_router.c -o project/tests/build/intercore_control_foundation_test.exe
project/tests/build/intercore_control_foundation_test.exe
```

Expected: `intercore_control_foundation_test: PASS` and exit code 0.

- [ ] **Step 7: Commit the router**

```powershell
git add project/code/motion_command_router.h project/code/motion_command_router.c project/code/motion_command_router_port.h project/code/motion_command_router_port.c project/tests/intercore_control_foundation_test.c
git commit -m "Add motion command router"
```

---

### Task 5: Reserve Shared SRAM and Configure Both CM7 MPUs

**Files:**
- Create: `project/code/intercore_memory.h`
- Create: `project/code/intercore_memory.c`
- Modify: `project/iar/icf/linker_directives_tviibh.icf`
- Modify: `project/user/main_cm7_0.c`
- Modify: `project/user/main_cm7_1.c`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_0.ewp`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_1.ewp`

**Interfaces:**
- Consumes: exported linker symbols and Cypress MPU/cache APIs.
- Produces: `intercore_memory_configure()` and `intercore_memory_get_layout()` on both CM7 cores.

Use these exact declarations:

```c
uint8 intercore_memory_configure(void);
volatile intercore_shared_layout_struct *intercore_memory_get_layout(void);
```

- [ ] **Step 1: Change the linker partition without placing a section in shared memory**

In the ICF, define:

```text
define symbol intercore_shared_sram_size       = 8K;
define symbol _base_SRAM_CM7_SHARED            = sram_base_address + cm0plus_sram_reserve + cm7_0_sram_reserve;
define symbol _base_SRAM_CM7_1                 = _base_SRAM_CM7_SHARED + intercore_shared_sram_size;
define symbol _size_SRAM_CM7_1                 = sram_total_size_user - cm0plus_sram_reserve - cm7_0_sram_reserve - intercore_shared_sram_size;
define exported symbol __intercore_shared_sram_base = _base_SRAM_CM7_SHARED;
define exported symbol __intercore_shared_sram_size = intercore_shared_sram_size;
```

Keep `_base_SRAM_CM7_0` and `_size_SRAM_CM7_0` unchanged. Keep CM0+'s `SRAM_CM7_1` binary region based on the new private CM7_1 base so its vector table moves to `0x28082000`.

- [ ] **Step 2: Implement linker-symbol and MPU access**

`intercore_memory.c` declares the two exported linker symbols as `extern uint8`. It rejects a base other than `0x28080000`, size other than 8192, or a base not aligned to 8192.

Before MPU setup, call:

```c
SCB_CleanInvalidateDCache_by_Addr((volatile void *)base, (int32_t)size);
__DSB();
__ISB();
```

Configure one region using:

```c
const cy_stc_mpu_region_cfg_t shared_region =
{
    .addr = INTERCORE_SHARED_BASE_ADDRESS,
    .size = CY_MPU_SIZE_8KB,
    .permission = CY_MPU_ACCESS_P_FULL_ACCESS,
    .attribute = CY_MPU_ATTR_NORM_SHR_MEM_NC,
    .execute = CY_MPU_INST_ACCESS_DIS,
    .srd = 0U,
    .enable = CY_MPU_ENABLE
};
```

Call `Cy_MPU_Setup(&shared_region, 1U, CY_MPU_USE_DEFAULT_MAP_AS_BG, CY_MPU_DISABLED_DURING_FAULT_NMI)`, followed by `__DSB()` and `__ISB()`. Return false on any mismatch or `CY_MPU_FAILURE`.

- [ ] **Step 3: Integrate safe startup on CM7_0**

After clock/debug/LED initialization and before `app_init()`, call `intercore_memory_configure()`. On failure call `led_blink_error_code(4U)`. Do not initialize or clear the shared layout from `main`; CM7_0 application initialization owns that action in Task 6.

Insert this exact block after the LED GPIO initialization:

```c
if(1U != intercore_memory_configure())
{
    led_blink_error_code(4U);
}
```

- [ ] **Step 4: Remove CM7_1 UART0 ownership and configure its MPU**

Remove `debug_info_init()` from `main_cm7_1.c`. After `clock_init()`, call `intercore_memory_configure()`. On failure, remain in an empty infinite loop. Do not attach transport, start PIT_CH2, initialize GNSS/camera/Wi-Fi, or publish a heartbeat in this task.

The CM7_1 body for this phase is:

```c
int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);

    if(1U != intercore_memory_configure())
    {
        while(true)
        {
        }
    }

    while(true)
    {
    }
}
```

- [ ] **Step 5: Add shared files to both CM7 IAR projects**

Add protocol, transport, notify, notify port, and memory `.c/.h` files to a `code/intercore` group in both `.ewp` files. Add router and router port only to CM7_0. Do not add host tests to an IAR project.

Each file entry uses the existing project-relative form, for example:

```xml
<group>
    <name>intercore</name>
    <file>
        <name>$PROJ_DIR$\..\..\code\intercore_protocol.c</name>
    </file>
    <file>
        <name>$PROJ_DIR$\..\..\code\intercore_protocol.h</name>
    </file>
</group>
```

- [ ] **Step 6: Build all three cores and inspect maps**

From IAR 9.40.1, open `project/iar/cyt4bb7.eww`, select Debug, and rebuild these projects in order:

1. `cyt4bb7_cm_7_0`
2. `cyt4bb7_cm_7_1`
3. `cyt4bb7_cm_0_plus`

Expected: zero errors. In the CM7_1 map, the SRAM/vector start is `0x28082000`; no section spans `0x28080000-0x28081FFF`. In the CM0+ map, the embedded CM7_1 image/vector address also resolves to `0x28082000`.

- [ ] **Step 7: Commit the memory boundary**

```powershell
git add project/code/intercore_memory.h project/code/intercore_memory.c project/iar/icf/linker_directives_tviibh.icf project/user/main_cm7_0.c project/user/main_cm7_1.c project/iar/project_config/cyt4bb7_cm_7_0.ewp project/iar/project_config/cyt4bb7_cm_7_1.ewp
git commit -m "Reserve shared SRAM for inter-core transport"
```

---

### Task 6: Integrate CM7_0 Command Reception and UART Routing

**Files:**
- Create: `project/code/intercore_control.h`
- Create: `project/code/intercore_control.c`
- Modify: `project/code/app_config.h`
- Modify: `project/code/app.c`
- Modify: `project/code/app_scheduler.c`
- Modify: `project/code/host_command.c`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_0.ewp`
- Modify: `project/tests/intercore_control_foundation_test.c`

**Interfaces:**
- Consumes: target shared layout, transport results, navigation command, notification bits, safety state, and motion router.
- Produces: `intercore_control_init()` and `intercore_control_update(uint32 now_ms)`.

- [ ] **Step 1: Add CM7-local acceptance tests**

Expose a pure helper in `intercore_control.h`:

```c
uint8 intercore_control_accept_navigation(const navigation_command_struct *command,
                                          uint32 now_ms);
```

With mocked router submission, test that it rejects forward magnitude above 60 RPM, turn magnitude above 60 deg/s, repeated or older `source_sequence`, waypoint mode while remote is disarmed, and non-finite values. Test that wireless source maps to `MOTION_SOURCE_WIRELESS_MANUAL`, while vision and waypoint map to `MOTION_SOURCE_AUTONOMOUS`. Test that `enable == 0U` cancels the mapped source rather than applying zero as an armed command.

The mapping test uses a fresh receiver/router state and these concrete assertions:

```c
navigation_command_struct command = {0};

TEST_CHECK(1U == motion_command_router_arm_remote(100U, 0U));
command.forward_rpm = 25.0f;
command.turn_rate_dps = 15.0f;
command.confidence = 0.9f;
command.source_sequence = 1U;
command.valid_for_ms = 200U;
command.enable = 1U;
command.source = NAVIGATION_SOURCE_WIRELESS_MANUAL;
command.mode = NAVIGATION_MODE_MANUAL;
TEST_CHECK(1U == intercore_control_accept_navigation(&command, 100U));
motion_command_router_update(101U, 0U);
TEST_CHECK(MOTION_SOURCE_WIRELESS_MANUAL ==
           motion_command_router_get_diag()->active_source);

command.source_sequence = 2U;
command.forward_rpm = 61.0f;
TEST_CHECK(0U == intercore_control_accept_navigation(&command, 102U));
command.forward_rpm = 25.0f;
command.source_sequence = 1U;
TEST_CHECK(0U == intercore_control_accept_navigation(&command, 103U));
```

The host test must provide `intercore_memory_get_layout()` as a stub returning its aligned test layout; notification and chassis port functions remain the mocks created in Tasks 3 and 4. In `intercore_control.c`, isolate the hardware-heavy configuration include exactly as follows so the same policy code compiles under GCC:

```c
#if defined(INTERCORE_HOST_TEST)
#define APP_INTERCORE_COMMAND_TIMEOUT_MS  (200U)
#define APP_INTERCORE_FORWARD_LIMIT_RPM   (60.0f)
#define APP_INTERCORE_TURN_LIMIT_DPS      (60.0f)
#else
#include "app_config.h"
#endif
```

- [ ] **Step 2: Add exact configuration constants**

Add to `app_config.h`:

```c
#define APP_INTERCORE_COMMAND_TIMEOUT_MS      (200U)
#define APP_INTERCORE_FORWARD_LIMIT_RPM       (60.0f)
#define APP_INTERCORE_TURN_LIMIT_DPS          (60.0f)
#define APP_UART_LOCAL_COMMAND_VALID_MS       (500U)
```

The 60 RPM and 60 deg/s values intentionally match the existing non-fast chassis limits. The receiver does not call `control_chassis_set_fast_enable()`.

- [ ] **Step 3: Implement CM7_0 receiver lifecycle**

`intercore_control_init()` obtains the layout from `intercore_memory_get_layout()`, initializes it as CM7_0 owner, initializes the CM7_0 notification endpoint, and zeros per-source sequence tracking. Return false if any initialization step fails.

Declare the lifecycle API exactly as:

```c
uint8 intercore_control_init(void);
void intercore_control_update(uint32 now_ms);
```

`intercore_control_update()` always polls `intercore_transport_read_navigation()`; notification bits are diagnostic hints only. On `OK`, pass the payload to `intercore_control_accept_navigation()`. On `EPOCH_CHANGED`, cancel both remote router sources immediately. On `INVALID`, increment a receiver rejection counter and leave the last valid command to expire naturally.

Track `source_sequence` independently for wireless, vision, and waypoint sources. Use the same wrap-safe comparison as the router:

```c
if((0U != intercore_last_source_sequence[command->source]) &&
   (0 >= (int32)(command->source_sequence -
                 intercore_last_source_sequence[command->source])))
{
    return 0U;
}
```

- [ ] **Step 4: Integrate receiver and router into the 1 ms control schedule**

In `app_init()`, initialize the router after safety and application state, then initialize `intercore_control`. Return error code bit `0x08U` when the shared/IPC receiver cannot initialize so CM7_0 enters the existing safe fault path.

In `app_scheduler_run_pending()`, after `app_safety_update(now_ms)` and before leg/chassis updates, call:

```c
intercore_control_update(now_ms);
motion_command_router_update(now_ms, app_safety_is_fault());
```

The router therefore applies a valid command before the next 5 ms chassis update and stops no later than its local timeout.

- [ ] **Step 5: Route STOP and `C` through the router**

Replace the `C,<forward>,<turn>` direct call with a `MOTION_SOURCE_UART_LOCAL` request whose `received_ms` is `now_ms`, validity is 500 ms, sequence is a local monotonically increasing counter with zero skipped, and enable is true.

The submission block is:

```c
motion_command_request_struct request;

host_motion_sequence++;
if(0U == host_motion_sequence)
{
    host_motion_sequence = 1U;
}
request.forward_rpm = kp;
request.turn_rate_dps = ki;
request.source_sequence = host_motion_sequence;
request.received_ms = now_ms;
request.valid_for_ms = APP_UART_LOCAL_COMMAND_VALID_MS;
request.enable = APP_TRUE;
if(APP_TRUE != motion_command_router_submit(MOTION_SOURCE_UART_LOCAL, &request))
{
    actuator_motor_record_command_error(APP_TRUE);
    return;
}
```

STOP must first call `motion_command_router_emergency_stop(now_ms)`, then retain the existing leg lock, balance-off, and motor-stop actions. Add these exact commands before other dispatch:

- `ARM`: clear emergency stop and arm remote only when `app_safety_is_fault()` is false.
- `MAINT,1`: enter maintenance, clear remote/local motion, and stop chassis.
- `MAINT,0`: leave maintenance stopped and remote-disarmed.

Reject `C` while maintenance mode is active. An unsuccessful ARM or a malformed `MAINT,n` command records a command error.

- [ ] **Step 6: Gate direct diagnostic commands**

Process STOP, ARM, `MAINT,n`, `B,n`, and `C,f,t` before the maintenance gate. Before every remaining direct motor, leg, servo, calibration, excitation, or gain command, require `motion_command_router_get_diag()->maintenance_mode == APP_TRUE`; otherwise record a command error and return.

Place this gate immediately after the normal-command dispatch blocks and before `IMU_ZERO`, `LH`, `LHF`, `LJ`, `LIKREF`, `LXY`, `LIK`, `BZ`, `BS`, `BI`, all gain commands, `M`, `D`, `P`, `PL`, and `PR`:

```c
if(APP_TRUE != motion_command_router_get_diag()->maintenance_mode)
{
    actuator_motor_record_command_error(APP_TRUE);
    return;
}
```

Remove every direct `control_chassis_stop()` call from `host_command.c`; entering maintenance or STOP already routes the stop through the router. This leaves `motion_command_router_port.c` as the only application implementation file that invokes the chassis set/stop APIs.

- [ ] **Step 7: Add CM7_0 project membership and run static ownership scans**

Add `intercore_control.c/.h` to the CM7_0 `code/intercore` group. Run:

```powershell
rg -n "control_chassis_(set_cmd|stop)" project/code --glob "!control_chassis.c" --glob "!control_chassis.h" --glob "!motion_command_router_port.c"
rg -n "actuator_|control_balance|control_chassis" project/code -g "intercore_protocol.*" -g "intercore_transport.*" -g "intercore_notify.*" -g "intercore_memory.*"
rg -n "ipc_send_data" project/code
```

Expected: all three scans return no matches.

- [ ] **Step 8: Run host tests and rebuild all IAR cores**

Run the full GCC test and executable; expect PASS. Rebuild CM7_0, CM7_1, then CM0+ in IAR; expect zero errors. Record warnings, code size, SRAM size, and confirm the CM7_1 private start remains `0x28082000`.

- [ ] **Step 9: Commit CM7_0 integration**

```powershell
git add project/code/intercore_control.h project/code/intercore_control.c project/code/app_config.h project/code/app.c project/code/app_scheduler.c project/code/host_command.c project/iar/project_config/cyt4bb7_cm_7_0.ewp project/tests/intercore_control_foundation_test.c
git commit -m "Route inter-core motion commands on CM7_0"
```

---

### Task 7: Verify the Foundation Before Peripheral Work

**Files:**
- Verify only; do not change control gains, pin maps, scheduler periods, motor limits, or vendor drivers.

**Interfaces:**
- Consumes: all deliverables from Tasks 1 through 6.
- Produces: an evidence-backed go/no-go decision for the CM7_1 scheduler/heartbeat plan.

- [ ] **Step 1: Run the complete host suite from a clean build directory**

```powershell
Remove-Item -LiteralPath project/tests/build -Recurse -Force -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Force project/tests/build | Out-Null
gcc -std=c11 -Wall -Wextra -Werror -DINTERCORE_HOST_TEST -Ilibraries/zf_common -Iproject/code project/tests/intercore_control_foundation_test.c project/code/intercore_protocol.c project/code/intercore_transport.c project/code/intercore_notify.c project/code/motion_command_router.c project/code/intercore_control.c -o project/tests/build/intercore_control_foundation_test.exe
project/tests/build/intercore_control_foundation_test.exe
```

Expected: compilation exit code 0 and `intercore_control_foundation_test: PASS`.

- [ ] **Step 2: Run repository/static checks**

```powershell
git diff --check
rg -n "ipc_send_data|system_delay|while" project/code/intercore_notify.c project/code/intercore_notify_port.c
rg -n "control_chassis_(set_cmd|stop)" project/code --glob "!control_chassis.c" --glob "!control_chassis.h" --glob "!motion_command_router_port.c"
rg -n "actuator_|control_balance|control_chassis" project/code -g "intercore_protocol.*" -g "intercore_transport.*" -g "intercore_notify.*" -g "intercore_memory.*"
```

Expected: `git diff --check` passes and every `rg` ownership/blocking scan returns no match.

- [ ] **Step 3: Rebuild all target images**

Rebuild CM7_0, CM7_1, and CM0+ in IAR Debug configuration. Expected: zero errors; no new warning is accepted without written justification. Confirm map addresses and that shared structs fit the exact linker reservation.

- [ ] **Step 4: Run the motor-disabled boot check**

Place the robot on a stand with motor power disabled. Flash all three images and verify:

- CM7_0 LED heartbeat and UART0 telemetry still operate.
- IMU initializes and reports healthy data.
- CM7_1 does not claim UART0 and remains in its empty nonblocking loop.
- No MPU fault occurs on either CM7 core.
- CM7_0 scheduler maximum gap remains no more than 2 ms.
- No servo or motor command is emitted solely because CM7_1 boots or resets.

- [ ] **Step 5: Exercise UART router safety semantics with motors disabled**

Send this sequence over UART0 and inspect router/chassis diagnostics in the debugger:

```text
C,10,5
STOP
C,10,5
ARM
C,10,5
MAINT,1
C,10,5
MAINT,0
ARM
C,10,5
```

Expected:

1. First `C` becomes UART-local active for at most 500 ms.
2. STOP clears the target and latches emergency stop.
3. The next `C` is rejected.
4. ARM permits a fresh `C`.
5. Entering maintenance stops and rejects normal `C` motion.
6. Leaving maintenance remains stopped until ARM plus a fresh `C`.

- [ ] **Step 6: Run the CM7_0 baseline with motor power enabled only after stand checks pass**

Use the existing safe all-90-degree/locked-leg reference and low-speed chassis limits. Confirm IMU, BLDC feedback, servo output, balance mode transitions, STOP response, UART telemetry, and scheduler diagnostics show no regression. Do not inject a CM7_1 navigation command in this phase because the CM7_1 scheduler/publisher belongs to roadmap item 2.

- [ ] **Step 7: Record the final repository state**

```powershell
git status --short --branch
git log --oneline -7
```

Expected: only intentionally recorded validation evidence, if any, is uncommitted; implementation source and project files are committed. Proceed to the CM7_1 scheduler/heartbeat plan only when every check above passes.

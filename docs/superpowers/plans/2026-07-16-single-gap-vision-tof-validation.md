# Single-Gap Vision and ToF Validation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在现有 CYT4BB7 双核相机基线上实现一个默认禁用、可分阶段解锁的单间隙闭环：CM7_1 从两个锥桶的底部中心计算转向，DL1B 独立负责近障碍停车，CM7_0 保留最终运动权限。

**Architecture:** 复用现有双槽相机传输、`perception_intercore` 位姿槽、`navigation_command_struct` 和 CM7_0 `motion_command_router`，不扩展 8 KB 共享 ABI。新增三个可独立主机测试的纯模块：单间隙检测、DL1B 安全快照、单间隙状态机；CM7_1 的薄应用层只负责调度、组合输入并发布 100 ms 有效期的视觉命令。默认构建保持运动禁用，实测轮周长写入配置且通过架空轮门禁后才允许非零命令。

**Tech Stack:** Embedded C (IAR EWARM 9.40.1, CYT4BB7/Traveo II), MT9V03X 188×120 Gray8, DL1B + software I2C, PowerShell static/host test scripts, GCC C11 host tests, existing inter-core SRAM transport.

## Global Constraints

- 基线是当前 `codex/cone-perception-gap-mapping`，并保留 `7a58d6f1 Tune camera display to 40 ms` 的相机显示行为。
- 控制周期 40 ms；只消费最新完整帧，不排队处理旧帧；相机或运动请求超过 100 ms 必须停车。
- DL1B 轮询周期 50 ms；有效距离 `<=350 mm`、无效结果、初始化失败或超过 100 ms 未更新均禁止运动。
- 图像固定 188×120；MVP 场地只允许两个高置信锥桶；少于两个连续 5 帧或出现第三个候选时停车。
- 初始目标速度 0.20 m/s，最大 0.30 m/s；转向限制 `±15 deg/s`，每 40 ms 最大变化 `5 deg/s`，误差死区 3 px。
- `PASS_CANDIDATE` 需要两个底部中心连续 2 帧位于 `v>=96`；之后 0.40 s 内将转向收敛到零，正向里程达到 0.20 m才完成，1.50 s 超时或里程倒退则停车。
- `SINGLE_GAP_WHEEL_CIRCUMFERENCE_MM` 默认精确设为 `0U`，此值表示“尚未实测”，并必须让运动编译门禁失败；不得用标称轮径猜测。
- CM7_1 不直接驱动电机；CM7_0 通过现有 `ARM` 命令、运动路由、超时和急停保留最终权限。
- 运动构建中 WiFi 图传关闭；图传调试构建中运动关闭。图传连接失败不得阻止帧处理或停车命令。
- 不修改 `libraries/zf_device/zf_device_dl1b.*`；用 `project/code` 适配层隔离厂商全局变量和同步调用。
- 不使用动态内存；每个任务先得到预期失败，再做最小实现，再运行测试并单独提交。
- 不纳入开始实施时已经存在的未提交改动或采样数据；当前包括 UART/CM7_1/GNSS 相关修改和 `tmp/`。每个任务开始前重新执行 `git status --short`，只暂存任务列出的窄路径或交互式补丁。

## Planned File Map

| Path | Responsibility |
|---|---|
| `project/code/single_gap_config.h` | 单间隙容量、时间、速度、安全门限和默认禁用开关 |
| `project/code/single_gap_types.h` | 无指针观测、ToF 快照、控制状态、停止原因和输出结构 |
| `project/code/dl1b_safety.h/.c` | 20 Hz DL1B 调度、有效性与 100 ms 新鲜度快照 |
| `project/code/dl1b_safety_port.h/.c` | 对现有 `dl1b_init/get_distance` 的硬件薄适配 |
| `project/code/single_gap_detector.h/.c` | 188×120 ROI 顺序遍历、候选评分和双锥桶间隙观测 |
| `project/code/single_gap_controller.h/.c` | 5/7 获取、丢失、PD、通过和锁存停车状态机 |
| `project/code/single_gap_pose_source.h/.c` | CM7_0 将实测轮速积分为短程里程并发布既有位姿快照 |
| `project/code/single_gap_app.h/.c` | CM7_1 组合相机、ToF、里程和跨核导航发布 |
| `project/code/camera_frame_consumer.h/.c` | 在 WiFi 发送前调用非阻塞帧处理回调，WiFi 失败仍消费图像 |
| `project/user/main_cm0plus.c` | 单间隙构建中释放 P19_0 |
| `project/user/main_cm7_0.c` | 单间隙构建中释放 P19_0 并保留无 LED 的故障锁死 |
| `project/user/main_cm7_1.c` | 初始化和轮询单间隙应用 |
| `project/code/intercore_control.c` | 对视觉命令增加 100 ms、±15 deg/s 和模式门禁 |
| `project/code/app.c`, `project/code/app_scheduler.c` | 初始化并以 20 Hz 发布短程位姿 |
| `project/iar/project_config/*.ewp` | 将新增源文件加入正确核心，保留并行 GNSS 项目项 |
| `project/tests/single_gap_*_test.c` | 检测、ToF、控制器、里程和应用层主机测试 |
| `tools/test_single_gap_*` | GCC 主机测试启动器和引脚/构建静态门禁 |
| `docs/single-gap-hardware-test.md` | 轮周长、ToF、静态图像、架空轮和低速落地证据 |

---

### Task 1: Freeze the MVP contracts and compile-time safety gates

**Files:**
- Create: `project/code/single_gap_config.h`
- Create: `project/code/single_gap_types.h`
- Create: `tools/test_single_gap_integration_static.ps1`

**Interfaces:**
- Consumes: existing `perception_candidate_struct` from `project/code/perception_types.h`.
- Produces: `single_gap_observation_struct`, `single_gap_tof_snapshot_struct`, `single_gap_controller_struct`, `single_gap_output_struct`, and all constants used by later tasks.

- [ ] **Step 1: Write the failing static contract test**

Create `tools/test_single_gap_integration_static.ps1` with exact checks:

```powershell
param([string]$Root = (Split-Path -Parent $PSScriptRoot))
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Require([string]$Path, [string]$Pattern)
{
    $full = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $full)) { throw "Missing $Path" }
    if (-not (Select-String -Path $full -Pattern $Pattern -Quiet))
    { throw "Missing '$Pattern' in $Path" }
}

Require 'project/code/single_gap_config.h' 'SINGLE_GAP_CONTROL_PERIOD_MS\s+\(40U\)'
Require 'project/code/single_gap_config.h' 'SINGLE_GAP_TOF_STOP_MM\s+\(350U\)'
Require 'project/code/single_gap_config.h' 'SINGLE_GAP_SENSOR_STALE_MS\s+\(100U\)'
Require 'project/code/single_gap_config.h' 'SINGLE_GAP_MOTION_ENABLE\s+\(0U\)'
Require 'project/code/single_gap_config.h' 'SINGLE_GAP_WHEEL_CIRCUMFERENCE_MM\s+\(0U\)'
Require 'project/code/single_gap_types.h' 'SINGLE_GAP_STATE_PASS_CANDIDATE'
Require 'project/code/single_gap_types.h' 'SINGLE_GAP_STOP_TOF_NEAR'
Require 'project/code/single_gap_types.h' 'single_gap_output_struct'
Write-Output 'single-gap static contracts: PASS'
```

Run: `powershell -ExecutionPolicy Bypass -File tools/test_single_gap_integration_static.ps1`

Expected: FAIL with `Missing project/code/single_gap_config.h`.

- [ ] **Step 2: Add exact constants and compile gates**

Create `single_gap_config.h` with these defaults and guards:

```c
#ifndef _single_gap_config_h_
#define _single_gap_config_h_

#define SINGLE_GAP_ENABLE                    (0U)
#define SINGLE_GAP_MOTION_ENABLE             (0U)
#define SINGLE_GAP_IMAGE_WIDTH               (188U)
#define SINGLE_GAP_IMAGE_HEIGHT              (120U)
#define SINGLE_GAP_ROI_TOP_PX                (20U)
#define SINGLE_GAP_ROI_BOTTOM_PX             (107U)
#define SINGLE_GAP_CONTROL_PERIOD_MS          (40U)
#define SINGLE_GAP_TOF_PERIOD_MS              (50U)
#define SINGLE_GAP_SENSOR_STALE_MS            (100U)
#define SINGLE_GAP_ACQUIRE_WINDOW_FRAMES      (7U)
#define SINGLE_GAP_ACQUIRE_HITS               (5U)
#define SINGLE_GAP_LOST_FRAMES                (5U)
#define SINGLE_GAP_MIN_WIDTH_PX               (24U)
#define SINGLE_GAP_BOTTOM_ENTER_PX            (96U)
#define SINGLE_GAP_BOTTOM_CONFIRM_FRAMES      (2U)
#define SINGLE_GAP_DEADBAND_PX                (3)
#define SINGLE_GAP_TURN_LIMIT_DPS             (15.0f)
#define SINGLE_GAP_TURN_STEP_DPS              (5.0f)
#define SINGLE_GAP_TURN_DECAY_MS              (400U)
#define SINGLE_GAP_PASS_TIMEOUT_MS            (1500U)
#define SINGLE_GAP_PASS_DISTANCE_M            (0.20f)
#define SINGLE_GAP_FORWARD_SPEED_MPS           (0.20f)
#define SINGLE_GAP_MAX_SPEED_MPS               (0.30f)
#define SINGLE_GAP_FORWARD_LIMIT_RPM           (60.0f)
#define SINGLE_GAP_TOF_STOP_MM                 (350U)
#define SINGLE_GAP_WHEEL_CIRCUMFERENCE_MM      (0U)
#define SINGLE_GAP_NAV_VALID_MS                (100U)

#if (SINGLE_GAP_MOTION_ENABLE && !SINGLE_GAP_ENABLE)
#error "single-gap motion requires single-gap sensing"
#endif
#if (SINGLE_GAP_MOTION_ENABLE && (SINGLE_GAP_WHEEL_CIRCUMFERENCE_MM == 0U))
#error "measure wheel circumference before enabling motion"
#endif
#endif
```

- [ ] **Step 3: Define pointer-free data types**

Create `single_gap_types.h`; use exact names and fields:

```c
#ifndef _single_gap_types_h_
#define _single_gap_types_h_

#include "perception_types.h"

typedef enum
{
    SINGLE_GAP_STATE_DISABLED = 0,
    SINGLE_GAP_STATE_ACQUIRE,
    SINGLE_GAP_STATE_APPROACH,
    SINGLE_GAP_STATE_PASS_CANDIDATE,
    SINGLE_GAP_STATE_PASSED,
    SINGLE_GAP_STATE_FAULT_STOP
} single_gap_state_enum;

typedef enum
{
    SINGLE_GAP_STOP_NONE = 0,
    SINGLE_GAP_STOP_DISABLED,
    SINGLE_GAP_STOP_FRAME_STALE,
    SINGLE_GAP_STOP_TARGET_LOST,
    SINGLE_GAP_STOP_TARGET_AMBIGUOUS,
    SINGLE_GAP_STOP_GAP_NARROW,
    SINGLE_GAP_STOP_TOF_INVALID,
    SINGLE_GAP_STOP_TOF_STALE,
    SINGLE_GAP_STOP_TOF_NEAR,
    SINGLE_GAP_STOP_ODOMETRY,
    SINGLE_GAP_STOP_PASS_TIMEOUT,
    SINGLE_GAP_STOP_PASSED
} single_gap_stop_reason_enum;

typedef struct
{
    uint32 sequence;
    uint32 capture_ms;
    uint16 accepted_count;
    uint16 gap_center_x;
    uint16 gap_width_px;
    uint16 left_bottom_y;
    uint16 right_bottom_y;
    uint8 valid;
    uint8 ambiguous;
} single_gap_observation_struct;

typedef struct
{
    uint32 sample_ms;
    uint16 distance_mm;
    uint8 initialized;
    uint8 valid;
} single_gap_tof_snapshot_struct;

typedef struct
{
    uint32 acquire_bits;
    uint32 state_enter_ms;
    uint32 last_frame_ms;
    float pass_start_odometry_m;
    float previous_error;
    float previous_turn_dps;
    uint16 gap_center_history[3];
    uint8 acquire_count;
    uint8 lost_count;
    uint8 bottom_count;
    uint8 gap_history_count;
    uint8 gap_history_index;
    uint8 armed;
    single_gap_state_enum state;
    single_gap_stop_reason_enum stop_reason;
} single_gap_controller_struct;

typedef struct
{
    float forward_rpm;
    float turn_rate_dps;
    uint16 gap_center_x;
    uint16 gap_width_px;
    uint8 enable;
    single_gap_state_enum state;
    single_gap_stop_reason_enum stop_reason;
} single_gap_output_struct;

#endif
```

- [ ] **Step 4: Run, inspect, and commit only the contracts**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_single_gap_integration_static.ps1
git diff --check
git status --short
```

Expected: static contract test PASS; no source outside the three task files is staged.

Commit:

```powershell
git add project/code/single_gap_config.h project/code/single_gap_types.h tools/test_single_gap_integration_static.ps1
git commit -m "Define single-gap validation contracts"
```

### Task 2: Add fail-closed DL1B service and exclusive pin ownership

**Files:**
- Create: `project/code/dl1b_safety.h`
- Create: `project/code/dl1b_safety.c`
- Create: `project/code/dl1b_safety_port.h`
- Create: `project/code/dl1b_safety_port.c`
- Create: `project/tests/single_gap_dl1b_test.c`
- Create: `tools/test_single_gap_dl1b_host.ps1`
- Modify: `project/user/main_cm0plus.c`
- Modify: `project/user/main_cm7_0.c`
- Modify: `tools/test_single_gap_integration_static.ps1`

**Interfaces:**
- Consumes: vendor `dl1b_init()`, `dl1b_get_distance()`, `dl1b_finsh_flag`, and `dl1b_distance_mm` only inside `dl1b_safety_port.c`.
- Produces: `uint8 dl1b_safety_init(uint32 now_ms)`, `void dl1b_safety_update(uint32 now_ms)`, and `single_gap_tof_snapshot_struct dl1b_safety_get_snapshot(void)`.

- [ ] **Step 1: Write the failing host test with a mock port**

The test mock exposes `mock_init_result`, `mock_read_valid`, and `mock_distance_mm`. Assert these exact cases:

```c
static void test_dl1b_fail_closed(void)
{
    single_gap_tof_snapshot_struct sample;
    mock_init_result = 0U;
    TEST_CHECK(0U == dl1b_safety_init(0U));
    sample = dl1b_safety_get_snapshot();
    TEST_CHECK(0U == sample.initialized);
    TEST_CHECK(0U == sample.valid);

    mock_init_result = 1U;
    TEST_CHECK(1U == dl1b_safety_init(0U));
    mock_read_valid = 1U;
    mock_distance_mm = 500U;
    dl1b_safety_update(49U);
    TEST_CHECK(0U == dl1b_safety_get_snapshot().valid);
    dl1b_safety_update(50U);
    TEST_CHECK(500U == dl1b_safety_get_snapshot().distance_mm);

    mock_distance_mm = 350U;
    dl1b_safety_update(100U);
    TEST_CHECK(350U == dl1b_safety_get_snapshot().distance_mm);
    mock_read_valid = 0U;
    dl1b_safety_update(150U);
    TEST_CHECK(0U == dl1b_safety_get_snapshot().valid);
}
```

Run: `powershell -ExecutionPolicy Bypass -File tools/test_single_gap_dl1b_host.ps1`

Expected: FAIL because `dl1b_safety.h/.c` do not exist.

- [ ] **Step 2: Implement the port boundary and 50 ms scheduler**

Use this port contract:

```c
uint8 dl1b_safety_port_init(void);
uint8 dl1b_safety_port_read(uint16 *distance_mm);
```

`dl1b_safety_port_init()` returns `1U` only when vendor `dl1b_init()` returns `0U`. `dl1b_safety_port_read()` calls `dl1b_get_distance()` once and returns `1U` only when `dl1b_finsh_flag != 0U` and `1U <= dl1b_distance_mm <= 4000U`.

`dl1b_safety_update()` performs no transaction until 50 ms elapsed. Each failed read immediately writes `valid=0U`; each valid read atomically updates `distance_mm`, `sample_ms`, and then `valid=1U`. No I2C call is made from an ISR.

- [ ] **Step 3: Release P19_0 on CM0+ and CM7_0 when the feature is enabled**

Include `single_gap_config.h` in both main files. Wrap every P19_0 LED initialization, error blink and heartbeat access with:

```c
#if (SINGLE_GAP_ENABLE == 0U)
    /* existing P19_0 LED operation */
#endif
```

When `SINGLE_GAP_ENABLE != 0U`, `led_blink_error_code()` must ignore `code` and remain in an empty infinite loop. Extend the static script to assert that P19_0 appears only inside the disabled-feature branches and that the DL1B header still declares P19_0/P19_1/P07_2.

- [ ] **Step 4: Verify and commit**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_single_gap_dl1b_host.ps1
powershell -ExecutionPolicy Bypass -File tools/test_single_gap_integration_static.ps1
git diff --check
```

Expected: both scripts PASS; the host binary is removed from `%TEMP%` in `finally`.

Commit:

```powershell
git add project/code/dl1b_safety.c project/code/dl1b_safety.h project/code/dl1b_safety_port.c project/code/dl1b_safety_port.h project/tests/single_gap_dl1b_test.c tools/test_single_gap_dl1b_host.ps1 tools/test_single_gap_integration_static.ps1 project/user/main_cm0plus.c project/user/main_cm7_0.c
git commit -m "Add DL1B fail-closed safety service"
```

### Task 3: Port the bounded two-cone detector to C

**Files:**
- Modify: `project/code/single_gap_config.h`
- Create: `project/code/single_gap_detector.h`
- Create: `project/code/single_gap_detector.c`
- Create: `project/tests/single_gap_detector_test.c`
- Create: `tools/test_single_gap_detector_host.ps1`
- Modify: `tools/test_single_gap_integration_static.ps1`

**Interfaces:**
- Consumes: `const uint8 *pixels`, width, height, stride, sequence and capture time.
- Produces: `uint8 single_gap_detector_process(..., single_gap_observation_struct *observation)`; return `1U` means the frame shape was valid, while `observation.valid` means exactly two acceptable cones formed a usable gap.

The header exposes exactly:

```c
uint8 single_gap_detector_process(const uint8 *pixels,
                                  uint16 width,
                                  uint16 height,
                                  uint16 stride,
                                  uint32 sequence,
                                  uint32 capture_ms,
                                  single_gap_observation_struct *observation);
```

- [ ] **Step 1: Write failing deterministic image tests**

Generate images in C without external files. The fixture draws a tapered bright component and a white middle band. Cover centered pair, right-shifted pair, one cone, three cones, narrow pair, flat image and wrong dimensions:

```c
draw_cone(image, 48, 38, 82, 104);
draw_cone(image, 106, 38, 140, 104);
TEST_CHECK(1U == single_gap_detector_process(image, 188U, 120U, 188U,
                                              1U, 20U, &observation));
TEST_CHECK(1U == observation.valid);
TEST_CHECK(2U == observation.accepted_count);
TEST_CHECK(94U == observation.gap_center_x);

draw_third_cone(image, 8, 40, 34, 104);
single_gap_detector_process(image, 188U, 120U, 188U, 2U, 60U, &observation);
TEST_CHECK(0U == observation.valid);
TEST_CHECK(1U == observation.ambiguous);
```

Run: `powershell -ExecutionPolicy Bypass -File tools/test_single_gap_detector_host.ps1`

Expected: FAIL because the detector API is missing.

- [ ] **Step 2: Implement one bounded ROI traversal**

Use fixed arrays for at most 96 row runs and 24 components. Traverse only rows 20 through 107 and all 188 columns once. Foreground is `pixel >= 160` or `pixel >= five_pixel_mean + 40`. Link a row run to overlapping runs in the previous row, merge labels with a bounded union table, and reject components with height `<8`, width `<4`, or area `<24`.

Add these capacities to `single_gap_config.h` in the same change:

```c
#define SINGLE_GAP_MAX_RUNS                   (96U)
#define SINGLE_GAP_MAX_COMPONENTS             (24U)
#define SINGLE_GAP_MAX_ACCEPTED               (3U)
```

For each surviving component, compute bottom center and 3 slices for height `<12`, otherwise 5 slices at 15/30/50/70/85 percent. Use the already-frozen integer score:

```c
score = (77U * taper + 64U * band + 38U * symmetry +
         26U * aspect + 26U * base + 25U * contrast + 128U) >> 8;
```

Accept score `>=166U`. Keep only the top three accepted candidates because the MVP only distinguishes zero, one, two, or ambiguous-more-than-two. Sort the first two by `bottom_center_x`. Set `valid=1U` only when count is exactly two and their center separation is at least 24 px; set `ambiguous=1U` when at least three pass. A flat/saturated sparse-quality sample rejects all candidates.

- [ ] **Step 3: Add capacity and heap static gates**

Extend the static script to require `SINGLE_GAP_MAX_RUNS (96U)`, `SINGLE_GAP_MAX_COMPONENTS (24U)`, the exact process signature, and reject `malloc|calloc|realloc|free` in `single_gap_*.c`.

- [ ] **Step 4: Verify and commit**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_single_gap_detector_host.ps1
powershell -ExecutionPolicy Bypass -File tools/test_single_gap_integration_static.ps1
git diff --check
```

Expected: centered/right pair assertions PASS; one/three/narrow/flat/wrong-size cases remain invalid without an out-of-bounds access.

Commit:

```powershell
git add project/code/single_gap_config.h project/code/single_gap_detector.c project/code/single_gap_detector.h project/tests/single_gap_detector_test.c tools/test_single_gap_detector_host.ps1 tools/test_single_gap_integration_static.ps1
git commit -m "Add bounded single-gap cone detector"
```

### Task 4: Implement the fail-closed PD and pass state machine

**Files:**
- Create: `project/code/single_gap_controller.h`
- Create: `project/code/single_gap_controller.c`
- Create: `project/tests/single_gap_controller_test.c`
- Create: `tools/test_single_gap_controller_host.ps1`
- Modify: `tools/test_single_gap_integration_static.ps1`

**Interfaces:**
- Consumes: one `single_gap_observation_struct`, one `single_gap_tof_snapshot_struct`, current odometry in metres plus its validity flag, `forward_rpm`, `now_ms`, and explicit `armed` state.
- Produces: `single_gap_output_struct`; every call fully defines enable, forward, turn, state and stop reason.

The header exposes exactly:

```c
void single_gap_controller_init(single_gap_controller_struct *controller);
void single_gap_controller_set_armed(single_gap_controller_struct *controller,
                                      uint8 armed,
                                      uint32 now_ms);
void single_gap_controller_update(single_gap_controller_struct *controller,
                                  const single_gap_observation_struct *observation,
                                  const single_gap_tof_snapshot_struct *tof,
                                  float odometry_m,
                                  uint8 odometry_valid,
                                  float forward_rpm,
                                  uint32 now_ms,
                                  single_gap_output_struct *output);
```

- [ ] **Step 1: Write failing state-transition and safety tests**

Cover these exact sequences:

```c
single_gap_controller_init(&controller);
single_gap_controller_set_armed(&controller, 1U, 0U);

/* 5 valid observations in 7 frames acquire the target. */
for(index = 0U; index < 7U; index++)
{
    valid_observation.valid = (0U != ((0x76U >> (6U - index)) & 1U)) ? 1U : 0U;
    valid_observation.capture_ms = index * 40U;
    single_gap_controller_update(&controller, &valid_observation, &clear_tof,
                                 0.0f, 1U, 50.0f, index * 40U, &output);
}
TEST_CHECK(SINGLE_GAP_STATE_APPROACH == controller.state);

/* Right target gives positive turn; three-pixel error gives zero. */
valid_observation.gap_center_x = 120U;
single_gap_controller_update(&controller, &valid_observation, &clear_tof,
                             0.0f, 1U, 50.0f, 320U, &output);
TEST_CHECK(output.turn_rate_dps > 0.0f);
TEST_CHECK(output.turn_rate_dps <= 15.0f);

/* Near, invalid, stale ToF and stale frame each latch a stop. */
clear_tof.distance_mm = 350U;
single_gap_controller_update(&controller, &valid_observation, &clear_tof,
                             0.0f, 1U, 50.0f, 360U, &output);
TEST_CHECK(SINGLE_GAP_STOP_TOF_NEAR == output.stop_reason);
TEST_CHECK(0U == output.enable);
```

Also assert: accepted gap centres `90, 150, 92` produce a filtered centre of `92`; more than two candidates stops immediately; five lost frames stop; gap width `<24` stops; the turn step is never over 5 deg/s; invalid or stale odometry stops while armed; a normal bottom exit reaches `PASSED` only after `odometry_delta>=0.20`; 1.50 s timeout and negative odometry stop; reappearance away from the bottom cannot count as a pass; only reinitialization clears the fault latch.

Run: `powershell -ExecutionPolicy Bypass -File tools/test_single_gap_controller_host.ps1`

Expected: FAIL because the controller does not exist.

- [ ] **Step 2: Implement acquisition and safety precedence**

The update order is exact: disabled/latched state, ToF initialized-valid-fresh-near checks, odometry validity, frame freshness, ambiguity/narrow checks, state transition, then command calculation. Safety always wins over a new valid frame.

Use a seven-bit rolling history:

```c
controller->acquire_bits = ((controller->acquire_bits << 1U) |
                            ((0U != observation->valid) ? 1U : 0U)) & 0x7FU;
controller->acquire_count = single_gap_popcount7(controller->acquire_bits);
```

Enter `APPROACH` at five hits. In `ACQUIRE` output remains disabled. In `APPROACH`, five consecutive invalid frames enter `FAULT_STOP`; ambiguity and narrow gaps stop on the first frame.

- [ ] **Step 3: Implement bounded PD and pass logic**

Insert each valid `gap_center_x` into the three-element history and use the median of the available entries; after three entries this makes `90, 150, 92 -> 92`. Use image centre `93.5f`, normalized error `(median_gap_center_x-93.5f)/94.0f`, `Kp=15.0f`, and `Kd=0.0f` initially. Apply the 3 px deadband, `±15 deg/s` clamp and 5 deg/s-per-update slew clamp. Forward RPM is supplied by the caller and is emitted only in `APPROACH`/`PASS_CANDIDATE`.

After two bottom-confirm frames, record entry time and odometry. During `PASS_CANDIDATE`, preserve the last turn and linearly multiply it by `(400-elapsed_ms)/400` until zero. Mark `PASSED` only when the observation leaves through the bottom and odometry delta is at least 0.20 m. A passed or fault state emits zero RPM and remains latched.

- [ ] **Step 4: Verify and commit**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_single_gap_controller_host.ps1
powershell -ExecutionPolicy Bypass -File tools/test_single_gap_integration_static.ps1
git diff --check
```

Expected: all transition, sign, clamp, timeout and latch tests PASS.

Commit:

```powershell
git add project/code/single_gap_controller.c project/code/single_gap_controller.h project/tests/single_gap_controller_test.c tools/test_single_gap_controller_host.ps1 tools/test_single_gap_integration_static.ps1
git commit -m "Add single-gap safety controller"
```

### Task 5: Publish short-range wheel odometry through the existing pose slots

**Files:**
- Create: `project/code/single_gap_pose_source.h`
- Create: `project/code/single_gap_pose_source.c`
- Create: `project/tests/single_gap_pose_source_test.c`
- Create: `tools/test_single_gap_pose_source_host.ps1`
- Modify: `project/code/app.c`
- Modify: `project/code/app_scheduler.c`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_0.ewp`
- Modify: `tools/test_single_gap_integration_static.ps1`

**Interfaces:**
- Consumes: `control_balance_get_diag()->wheel_speed_rpm`, configured wheel circumference, and existing `perception_intercore_publish_pose()`.
- Produces: a 20 Hz `perception_pose_snapshot_struct` whose `position_x_m` is cumulative forward distance and whose validity includes `PERCEPTION_POSE_VALID_WHEEL | PERCEPTION_POSE_VALID_ODOMETRY` only when feedback and circumference are valid.

The header exposes exactly:

```c
uint8 single_gap_pose_source_init(void);
void single_gap_pose_source_update(uint32 now_ms);
float single_gap_pose_integrate_distance(float previous_m,
                                         float wheel_rpm,
                                         float circumference_m,
                                         uint32 dt_ms);
```

- [ ] **Step 1: Write the failing pure integration test**

Expose a pure helper:

```c
float single_gap_pose_integrate_distance(float previous_m,
                                         float wheel_rpm,
                                         float circumference_m,
                                         uint32 dt_ms);
```

Assert `60 rpm`, `0.20 m/rev`, `1000 ms` produces `0.20 m`; `-60 rpm` produces `-0.20 m`; zero or negative circumference returns the unchanged distance; non-finite input invalidates the sample.

Run: `powershell -ExecutionPolicy Bypass -File tools/test_single_gap_pose_source_host.ps1`

Expected: FAIL because the helper is missing.

- [ ] **Step 2: Implement CM7_0 publisher without changing the shared ABI**

`single_gap_pose_source_init()` obtains the existing shared layout and calls `perception_intercore_cm7_0_init()`. `single_gap_pose_source_update(now_ms)` runs at 20 Hz, integrates measured average wheel RPM using:

```c
distance_delta_m = wheel_rpm * ((float)dt_ms / 60000.0f) *
                   ((float)SINGLE_GAP_WHEEL_CIRCUMFERENCE_MM / 1000.0f);
```

Set unused position Y and attitude fields to zero. Set `timestamp_us=now_ms*1000U`, increment a nonzero sequence, and publish. With circumference `0.0f`, publish a snapshot without the odometry-valid bit so CM7_1 remains stopped.

- [ ] **Step 3: Wire the 20 Hz call and preserve concurrent project edits**

Initialize after `intercore_control_init()` in `app.c`. Add a dedicated `pose_last_ms` in `app_scheduler.c`, run every 50 ms, and add only `single_gap_pose_source.c/.h` entries to CM7_0 IAR project. Before editing the `.ewp`, preserve every existing GNSS/perception entry and review the narrow XML diff.

- [ ] **Step 4: Verify and commit**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_single_gap_pose_source_host.ps1
powershell -ExecutionPolicy Bypass -File tools/test_perception_intercore_host.ps1
powershell -ExecutionPolicy Bypass -File tools/test_single_gap_integration_static.ps1
git diff --check
```

Expected: pure integration and existing perception transport tests PASS; the shared layout remains exactly 8192 bytes.

Commit only these paths after confirming the `.ewp` contains no unrelated deletion:

```powershell
git add project/code/single_gap_pose_source.c project/code/single_gap_pose_source.h project/tests/single_gap_pose_source_test.c tools/test_single_gap_pose_source_host.ps1 tools/test_single_gap_integration_static.ps1 project/code/app.c project/code/app_scheduler.c project/iar/project_config/cyt4bb7_cm_7_0.ewp
git commit -m "Publish single-gap wheel odometry"
```

### Task 6: Integrate CM7_1 perception before optional WiFi display

**Files:**
- Create: `project/code/single_gap_app.h`
- Create: `project/code/single_gap_app.c`
- Create: `project/tests/single_gap_app_test.c`
- Create: `tools/test_single_gap_app_host.ps1`
- Modify: `project/code/camera_frame_consumer.h`
- Modify: `project/code/camera_frame_consumer.c`
- Modify: `project/code/camera_debug_config.h`
- Modify: `project/user/main_cm7_1.c`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_1.ewp`
- Modify: `tools/test_single_gap_integration_static.ps1`

**Interfaces:**
- Consumes: `camera_vision_frame_view_struct`, `dl1b_safety_get_snapshot()`, `perception_intercore_read_pose()`, and the controller output.
- Produces: a `NAVIGATION_SOURCE_VISION` / `NAVIGATION_MODE_VISION_ASSIST` command through `intercore_transport_publish_navigation()` every 40 ms; invalid/finished states publish `enable=0U` with the corresponding stop reason.

The header exposes exactly:

```c
uint8 single_gap_app_init(void); /* 0U success, 1U fail-closed init */
void single_gap_app_on_frame(const camera_vision_frame_view_struct *frame);
void single_gap_app_service(uint32 now_ms);
float single_gap_speed_mps_to_rpm(float speed_mps, float circumference_m);
```

- [ ] **Step 1: Write failing app-layer tests with mocked ports**

Use injected test ports for navigation publish, pose read, ToF snapshot and frame processing. Assert:

```c
TEST_CHECK(0U == single_gap_app_init());
single_gap_app_on_frame(&frame);
single_gap_app_service(40U);
TEST_CHECK(1U == publish_count);
TEST_CHECK(NAVIGATION_SOURCE_VISION == published.source);
TEST_CHECK(NAVIGATION_MODE_VISION_ASSIST == published.mode);
TEST_CHECK(100U == published.valid_for_ms);

/* Motion default remains disabled even with a valid target. */
TEST_CHECK(0U == published.enable);

/* A stale frame and a 350 mm ToF sample publish an explicit stop. */
```

Also assert that 0.20 m/s converts to RPM only when circumference is positive:

```c
rpm = single_gap_speed_mps_to_rpm(0.20f, 0.20f);
TEST_CHECK(float_near(60.0f, rpm, 0.001f));
TEST_CHECK(0.0f == single_gap_speed_mps_to_rpm(0.20f, 0.0f));
```

Add a third assertion that a conversion above `SINGLE_GAP_FORWARD_LIMIT_RPM` produces a disabled `NAVIGATION_STOP_INVALID` command rather than relying on CM7_0 to clamp it.

Run: `powershell -ExecutionPolicy Bypass -File tools/test_single_gap_app_host.ps1`

Expected: FAIL because `single_gap_app` and the frame hook do not exist.

- [ ] **Step 2: Add a non-blocking frame callback before WiFi send**

Add this API to `camera_frame_consumer.h`:

```c
typedef void (*camera_frame_handler_fn)(const camera_vision_frame_view_struct *frame);
void camera_frame_consumer_set_handler(camera_frame_handler_fn handler);
```

After acquiring and freshness-checking a frame, invoke the handler before `seekfree_assistant_camera_send()`. Refactor network handling so a disconnected socket attempts retry but does not return before frame acquisition. Guard only the send section with a connected-socket check. Always release the camera slot.

In `camera_debug_config.h` enforce mutually exclusive modes:

```c
#if SINGLE_GAP_MOTION_ENABLE
#undef APP_CAMERA_WIFI_ENABLE
#define APP_CAMERA_WIFI_ENABLE (0U)
#endif
```

- [ ] **Step 3: Implement CM7_1 orchestration and explicit stop publication**

`single_gap_app_init()` attaches both existing transports, initializes DL1B, initializes the controller, and registers `single_gap_app_on_frame`. `on_frame()` only runs the detector and stores the latest pointer-free observation. `service()` polls ToF every loop, consumes the newest pose, runs the controller every 40 ms, converts speed to RPM, and publishes.

Build the command exactly as:

```c
command.forward_rpm = output.enable ? output.forward_rpm : 0.0f;
command.turn_rate_dps = output.enable ? output.turn_rate_dps : 0.0f;
command.confidence = output.enable ? 1.0f : 0.0f;
command.source_sequence = next_nonzero_sequence();
command.valid_for_ms = SINGLE_GAP_NAV_VALID_MS;
command.enable = output.enable;
command.source = NAVIGATION_SOURCE_VISION;
command.mode = NAVIGATION_MODE_VISION_ASSIST;
command.stop_reason = map_single_gap_stop_reason(output.stop_reason);
command.reserved[0] = 0U;
command.reserved[1] = 0U;
```

Map controller stop reasons exactly: `NONE -> NAVIGATION_STOP_NONE`; `DISABLED`, `PASSED` and `STOP_PASSED -> NAVIGATION_STOP_DISABLED`; frame/ToF stale -> `NAVIGATION_STOP_STALE`; ToF near -> `NAVIGATION_STOP_EMERGENCY`; all remaining faults -> `NAVIGATION_STOP_INVALID`. Initialize the controller armed flag to `SINGLE_GAP_MOTION_ENABLE`; CM7_0 still requires the operator's independent `ARM` command before accepting enabled remote motion.

Even when `SINGLE_GAP_ENABLE==0U`, initialize no sensor and publish no command. When sensing is enabled but motion is disabled, publish only disabled diagnostic commands.

- [ ] **Step 4: Wire CM7_1 and preserve the dirty GNSS project entries**

Call `single_gap_app_init()` after camera consumer initialization and call `single_gap_app_service(camera_frame_consumer_now_ms())` before `camera_frame_consumer_service()` in the main loop. Add the new source files to CM7_1 `.ewp` without staging or reverting the pre-existing GNSS XML changes owned by another task; if the `.ewp` cannot be separated cleanly, commit all already-reviewed GNSS entries together only after identifying their originating commits.

- [ ] **Step 5: Verify and commit**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_single_gap_app_host.ps1
powershell -ExecutionPolicy Bypass -File tools/test_camera_capture_service_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_single_gap_integration_static.ps1
git diff --check
```

Expected: application tests PASS; camera slots release with WiFi connected or disconnected; sensing-only build emits no enabled navigation command.

Commit:

```powershell
git add project/code/single_gap_app.c project/code/single_gap_app.h project/tests/single_gap_app_test.c tools/test_single_gap_app_host.ps1 project/code/camera_frame_consumer.c project/code/camera_frame_consumer.h project/code/camera_debug_config.h project/user/main_cm7_1.c tools/test_single_gap_integration_static.ps1
git add -p project/iar/project_config/cyt4bb7_cm_7_1.ewp
git commit -m "Integrate single-gap sensing on CM7_1"
```

### Task 7: Harden CM7_0 vision-command acceptance

**Files:**
- Modify: `project/code/intercore_control.c`
- Modify: `project/code/app_config.h`
- Modify: `project/tests/intercore_control_foundation_test.c`
- Modify: `tools/test_single_gap_integration_static.ps1`

**Interfaces:**
- Consumes: existing `navigation_command_struct` from CM7_1.
- Produces: a CM7_0 policy that accepts single-gap motion only for vision source + vision-assist mode + maximum 100 ms validity + `|turn|<=15 deg/s`; a disabled vision command cancels the autonomous source immediately.

- [ ] **Step 1: Add failing policy tests**

Extend the existing host test with these cases:

```c
command.source = NAVIGATION_SOURCE_VISION;
command.mode = NAVIGATION_MODE_VISION_ASSIST;
command.valid_for_ms = 100U;
command.turn_rate_dps = 15.0f;
TEST_CHECK(1U == intercore_control_accept_navigation(&command, 100U));

command.source_sequence++;
command.turn_rate_dps = 15.1f;
TEST_CHECK(0U == intercore_control_accept_navigation(&command, 101U));

command.source_sequence++;
command.turn_rate_dps = 0.0f;
command.valid_for_ms = 101U;
TEST_CHECK(0U == intercore_control_accept_navigation(&command, 102U));

command.source_sequence++;
command.valid_for_ms = 100U;
command.enable = 0U;
TEST_CHECK(1U == intercore_control_accept_navigation(&command, 103U));
```

Run: `powershell -ExecutionPolicy Bypass -File tools/test_intercore_control_foundation.ps1`

Expected: the 15.1 deg/s and 101 ms cases are currently accepted, so the test FAILS.

- [ ] **Step 2: Add source-specific validation before routing**

In `intercore_control_accept_navigation()`, before mapping the source, reject a vision command unless:

```c
(NAVIGATION_MODE_VISION_ASSIST == command->mode) &&
(SINGLE_GAP_NAV_VALID_MS >= command->valid_for_ms) &&
(SINGLE_GAP_TURN_LIMIT_DPS >= command->turn_rate_dps) &&
(-SINGLE_GAP_TURN_LIMIT_DPS <= command->turn_rate_dps)
```

Do not require `enable=1U` for validation: a structurally valid disabled command must reach the existing cancellation path. Keep global finite and RPM limits unchanged as the second safety layer.

Include `single_gap_config.h` from `app_config.h` and define the camera-only actuator gate from the motion switch:

```c
#if SINGLE_GAP_MOTION_ENABLE
#define APP_CAMERA_DEBUG_ONLY           (0U)
#else
#define APP_CAMERA_DEBUG_ONLY           (1U)
#endif
```

The static script must reject a build configuration that leaves both `SINGLE_GAP_MOTION_ENABLE` and `APP_CAMERA_DEBUG_ONLY` equal to `1U` after preprocessing.

- [ ] **Step 3: Verify and commit**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_intercore_control_foundation.ps1
powershell -ExecutionPolicy Bypass -File tools/test_single_gap_integration_static.ps1
git diff --check
```

Expected: all existing manual/waypoint tests and new vision-specific tests PASS.

Commit:

```powershell
git add project/code/intercore_control.c project/code/app_config.h project/tests/intercore_control_foundation_test.c tools/test_single_gap_integration_static.ps1
git commit -m "Harden single-gap vision command policy"
```

### Task 8: Build, measure the wheel, and execute hardware gates in order

**Files:**
- Create: `docs/single-gap-hardware-test.md`
- Modify after measurement: `project/code/single_gap_config.h`
- Modify only when IAR source membership requires it: `project/iar/project_config/cyt4bb7_cm_0_plus.ewp`, `cyt4bb7_cm_7_0.ewp`, `cyt4bb7_cm_7_1.ewp`

**Interfaces:**
- Consumes: all prior task deliverables and physical wheel/ToF/camera evidence.
- Produces: one measured sensing-only commit, one separately reviewed motion-enable commit, and a signed hardware report; motion remains disabled if any gate fails.

- [ ] **Step 1: Run the complete software gate**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_single_gap_dl1b_host.ps1
powershell -ExecutionPolicy Bypass -File tools/test_single_gap_detector_host.ps1
powershell -ExecutionPolicy Bypass -File tools/test_single_gap_controller_host.ps1
powershell -ExecutionPolicy Bypass -File tools/test_single_gap_pose_source_host.ps1
powershell -ExecutionPolicy Bypass -File tools/test_single_gap_app_host.ps1
powershell -ExecutionPolicy Bypass -File tools/test_intercore_control_foundation.ps1
powershell -ExecutionPolicy Bypass -File tools/test_single_gap_integration_static.ps1
git diff --check
```

Expected: all commands PASS and `git diff --check` is silent.

- [ ] **Step 2: Build all three cores with motion disabled**

Run from an IAR command prompt:

```powershell
IarBuild.exe project/iar/project_config/cyt4bb7_cm_0_plus.ewp -build Debug
IarBuild.exe project/iar/project_config/cyt4bb7_cm_7_0.ewp -build Debug
IarBuild.exe project/iar/project_config/cyt4bb7_cm_7_1.ewp -build Debug
```

Expected: zero errors. Record code/RAM sizes and confirm the shared layout linker check remains 8192 bytes.

- [ ] **Step 3: Measure wheel circumference and commit the measured value**

Keep motors off. Mark a drive wheel and the floor, roll exactly 10 complete wheel revolutions under vehicle load, measure travel in metres three times, and compute:

```text
circumference_mm = round(1000 * median(travel_1_m, travel_2_m, travel_3_m) / 10)
```

The three circumference estimates must differ by no more than 1 percent. Replace only `SINGLE_GAP_WHEEL_CIRCUMFERENCE_MM (0U)` with the measured integer millimetres, leave `SINGLE_GAP_MOTION_ENABLE (0U)`, rerun all software tests and rebuild all cores.

Commit:

```powershell
git add project/code/single_gap_config.h docs/single-gap-hardware-test.md
git commit -m "Record measured wheel circumference"
```

- [ ] **Step 4: Validate DL1B and static vision with no motion authority**

Set `SINGLE_GAP_ENABLE (1U)` and keep motion at `0U`. Build and flash all affected cores. Record DL1B readings at approximately 0.20, 0.35, 0.50, 1.0 and 2.0 m; verify 350 mm or less, a disconnected module, and stale data all produce the expected stop reason. Save at least 20 stationary two-cone frames and verify gap-center peak-to-peak jitter is at most 6 px. Add a third cone and verify ambiguity.

Commit the sensing-enable configuration and evidence separately:

```powershell
git add project/code/single_gap_config.h docs/single-gap-hardware-test.md
git commit -m "Validate single-gap sensing hardware"
```

- [ ] **Step 5: Enable only the raised-wheel gate**

Set `SINGLE_GAP_MOTION_ENABLE (1U)`, which forces WiFi display off. Build and flash, support the chassis so wheels cannot touch the ground, send `ARM`, and move the visual gap left/right. Confirm right image error produces right correction, left error produces left correction, deadband produces zero turn, and all injected sensor failures stop within 100 ms. If any sign is wrong, disable motion before changing code.

Commit only after every raised-wheel row is signed:

```powershell
git add project/code/single_gap_config.h docs/single-gap-hardware-test.md
git commit -m "Enable raised-wheel single-gap validation"
```

- [ ] **Step 6: Run low-speed ground acceptance**

Use an open area, a wide two-cone gate and a person holding physical power cutoff. Confirm measured steady speed is `0.20 m/s ±10%`; if not, correct only the measured circumference or explicitly lower the speed target. Run a flat obstacle insertion to prove the 350 mm ToF stop. Then complete 10 consecutive single-gate runs without collision, manual steering or unintended continuation, and stop within 0.50 m after the gate line. Do not raise the target to 0.30 m/s in this plan; that begins the next validation stage.

- [ ] **Step 7: Final audit and evidence commit**

Run:

```powershell
git status --short --branch
git log --oneline --decorate -12
git diff 2f605f4..HEAD --stat
```

Confirm every implementation task has exactly one scoped commit, except the intentionally separate measurement/sensing/motion hardware commits. Confirm no BMP, IAR output, `tmp/` data or unrelated GNSS changes were accidentally staged. Commit only the completed report:

```powershell
git add docs/single-gap-hardware-test.md
git commit -m "Record single-gap hardware acceptance"
```

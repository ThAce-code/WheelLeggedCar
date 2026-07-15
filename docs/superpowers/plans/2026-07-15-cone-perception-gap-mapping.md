# Cone Perception and Gap Mapping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 CYT4BB7 双核固件上实现 MT9V03X `188x120 @ 50 FPS` 锥桶检测、地面投影、静态跟踪、`N-1` 间隙地图和安全目标快照，并用逐飞助手保存的 BMP、离线回放及分阶段硬件试验证明实时性和正确性。

**Architecture:** CM7_1 独占相机、检测、投影、跟踪和间隙状态机；CM7_0 发布 IMU/轮速/腿高融合位姿并消费只读感知目标。算法先在 Python 参考实现中冻结几何与判定，再移植为无动态内存的定点/有限浮点 C 模块。双核只经 8 KB 无初始化双缓冲共享区交换带序号、时间戳和 CRC 的结构化快照，WiFi-SPI 仅用于可丢帧调试。

**Tech Stack:** Embedded C (IAR EWARM 9.40.1, CYT4BB7/Traveo II), PowerShell static checks, Python 3 `unittest` + NumPy + OpenCV for calibration/reference replay, MT9V03X grayscale global-shutter camera, SeekFree WiFi-SPI assistant.

## Global Constraints

- 固件遵循仓库现有 4 空格风格、`uint8/uint16/uint32` 类型和 `0U/1U` 后缀；不修改 `libraries` 中的厂商代码，MT9V03X 适配层放在 `project/code`。
- 不使用 `malloc/calloc/realloc/free`，所有图像缓冲、候选、轨迹和共享快照均为静态定长数组。
- 固定容量：图像 `188x120`，候选最多 24，已确认锥桶最多 32，间隙最多 31；超限时丢弃最低分候选并增加诊断计数。
- 检测每帧处理跟踪窗，偶数帧额外处理全局发现 ROI；相机 50 Hz，全局发现 25 Hz，地图/间隙输出 20 Hz。
- CM7_1 不直接驱动电机；CM7_0 对感知目标保留最终有效性检查和 SAFE_STOP 权限。
- 无线发送不得处于识别关键路径；调试队列满时丢调试帧而不是阻塞相机或算法。
- 每个任务先写失败测试，再写最小实现，再运行任务列出的验证命令；不得把尚未通过的硬件门禁表述为已完成。
- 不纳入已有用户改动和采样数据：`tools/calibration/samples/*`、`tools/calibration/camera_calib.*`、`data/*.csv`、`tmp/`。
- 每次提交只包含当前任务列出的文件。提交前执行 `git diff --check` 和对应测试。

## Planned File Map

| Path | Responsibility |
|---|---|
| `tools/calibration/camera_calibrate.py` | 接收逐飞助手 BMP，标准/鱼眼模型比较，输出 JSON/NPZ/C 头文件 |
| `tools/calibration/camera_utils.py` | 单点去畸变及模型无关投影函数 |
| `tools/calibration/tests/test_camera_calibration.py` | BMP、分辨率、模型选择、C 导出回归测试 |
| `tools/perception/reference_detector.py` | 离线 ROI、候选、256 权重打分参考实现 |
| `tools/perception/reference_geometry.py` | 地平线、单点去畸变、射线落地参考实现 |
| `tools/perception/reference_tracker.py` | 马氏门控、轨迹生命周期、排序与间隙参考实现 |
| `tools/perception/replay.py` | BMP 序列 + 位姿 CSV 回放、CSV/叠图/统计输出 |
| `tools/perception/tests/` | 参考模型和边界条件单元测试 |
| `tools/test_perception_integration_static.ps1` | 容量、接口、无动态内存、IAR 成员和安全边界静态检查 |
| `project/code/perception_config.h` | 容量、频率、门限、评分权重和超时的唯一固件定义 |
| `project/code/perception_types.h` | 不含指针的跨模块/跨核数据结构和枚举 |
| `project/code/perception_crc.h/.c` | 共享快照 CRC32 |
| `project/code/perception_shared.h/.c` | 位姿/感知快照双缓冲发布和一致读取 |
| `project/code/perception_calibration.h/.c` | 标定常量、单点去畸变、相机射线 |
| `project/code/perception_roi.h/.c` | IMU 地平线曲线、发现区和跟踪窗 |
| `project/code/perception_detector.h/.c` | 一次顺序遍历、行程连通、截面特征和打分 |
| `project/code/perception_projection.h/.c` | 底部中心像素到地面坐标及协方差 |
| `project/code/perception_tracker.h/.c` | 数据关联、确认/丢失、地图合并和固定 ID |
| `project/code/perception_gap.h/.c` | 纵向排序、`N-1` 间隙、返程目标和状态机 |
| `project/code/perception_camera.h/.c` | MT9V03X 帧适配、VSYNC 时间戳、双缓冲和曝光质量 |
| `project/code/perception_app.h/.c` | CM7_1 感知流水线和时间预算管理 |
| `project/code/perception_client.h/.c` | CM7_0 位姿发布、目标消费、超时和安全门禁 |
| `project/user/main_cm7_1.c` | CM7_1 初始化与非阻塞循环 |
| `project/user/cm7_1_isr.c` | 相机/时间戳 ISR 的最小转发入口 |
| `project/code/app.c`, `project/code/app_scheduler.c` | CM7_0 周期调用与任务状态接入 |
| `project/code/app_config.h` | CM7_0 感知超时和任务门限 |
| `project/iar/icf/linker_directives_tviibh.icf` | 预留 8 KB `.perception_shared` 区域 |
| `project/iar/project_config/cyt4bb7_cm_7_0.ewp` | 加入共享/客户端源文件 |
| `project/iar/project_config/cyt4bb7_cm_7_1.ewp` | 加入完整感知源文件和 MT9V03X 依赖 |
| `docs/cone-perception-hardware-test.md` | 可重复的静态、开环和场地验收记录模板 |

---

### Task 1: Make SeekFree BMP calibration reproducible

**Files:**
- Modify: `tools/calibration/camera_calibrate.py`
- Modify: `tools/calibration/camera_utils.py`
- Modify: `tools/calibration/tests/test_camera_calibration.py`
- Create: `tools/calibration/tests/test_camera_export.py`

- [ ] **Step 1: Add failing tests for BMP input, model selection, and C export**

Add tests that generate 188×120 synthetic checkerboards in a temporary directory, save them as BMP, and assert:

```python
def test_bmp_glob_is_loaded_at_native_resolution(self):
    result = calibrate_from_images(str(self.sample_dir / "*.bmp"), (9, 6), 25.0)
    self.assertEqual(result["image_size"], [188, 120])

def test_mixed_resolution_images_are_rejected(self):
    with self.assertRaisesRegex(ValueError, "same resolution"):
        calibrate_from_images(str(self.sample_dir / "*.bmp"), (9, 6), 25.0)

def test_best_model_must_improve_rms_by_five_percent(self):
    self.assertEqual(select_model(0.80, 0.78), "standard")
    self.assertEqual(select_model(0.80, 0.74), "fisheye")

def test_export_header_contains_resolution_crc_and_fixed_point_coefficients(self):
    export_c_header(self.result, self.header_path)
    text = self.header_path.read_text(encoding="utf-8")
    self.assertIn("CAMERA_CALIBRATION_WIDTH (188U)", text)
    self.assertIn("CAMERA_CALIBRATION_HEIGHT (120U)", text)
    self.assertIn("camera_calibration_crc32", text)
```

Run: `python -m unittest tools.calibration.tests.test_camera_calibration tools.calibration.tests.test_camera_export -v`

Expected: FAIL because fisheye comparison and C export do not exist.

- [ ] **Step 2: Implement explicit model comparison and deterministic export**

Add CLI options:

```text
--model auto|standard|fisheye     default: auto
--output-header PATH              optional generated C header
--expected-width 188              reject mismatched images
--expected-height 120             reject mismatched images
```

Use `cv2.calibrateCamera` for `standard`, `cv2.fisheye.calibrate` for `fisheye`, and select fisheye only when its RMS is at least 5% lower. JSON must contain `schema_version`, `model`, `image_size`, `K`, `D`, `rms`, `per_view_rms`, `board`, and CRC32 of the canonical JSON calibration payload. Export Q16.16 values for `fx/fy/cx/cy` and Q2.29 values for distortion coefficients; never export the nominal 120° FOV as calibration.

Implement model-aware single-point undistortion:

```python
def undistort_points(points_px, calibration):
    points = np.asarray(points_px, dtype=np.float64).reshape(-1, 1, 2)
    k = np.asarray(calibration["K"], dtype=np.float64)
    d = np.asarray(calibration["D"], dtype=np.float64)
    if calibration["model"] == "fisheye":
        return cv2.fisheye.undistortPoints(points, k, d, P=k).reshape(-1, 2)
    return cv2.undistortPoints(points, k, d, P=k).reshape(-1, 2)
```

- [ ] **Step 3: Run calibration tests and one real BMP dry run**

Run:

```powershell
python -m unittest discover -s tools/calibration/tests -v
python tools/calibration/camera_calibrate.py --images "tmp/perception_calibration/*.bmp" --cols 9 --rows 6 --square-size-mm 25 --model auto --expected-width 188 --expected-height 120 --output-json tmp/mt9v03x_calibration.json --output-npz tmp/mt9v03x_calibration.npz --output-header tmp/mt9v03x_calibration_generated.h
```

Expected: tests PASS. Dry run either produces all three outputs or rejects the dataset with an explicit reason such as insufficient detected views; it must not silently resize images.

- [ ] **Step 4: Commit calibration tooling only**

```powershell
git add tools/calibration/camera_calibrate.py tools/calibration/camera_utils.py tools/calibration/tests/test_camera_calibration.py tools/calibration/tests/test_camera_export.py
git commit -m "Add MT9V03X BMP calibration export"
```

### Task 2: Freeze perception contracts and constants

**Files:**
- Create: `project/code/perception_config.h`
- Create: `project/code/perception_types.h`
- Create: `tools/test_perception_integration_static.ps1`

- [ ] **Step 1: Write a failing static contract test**

The PowerShell test must assert exact constants, reject heap calls, and verify that shared structures contain no pointer fields:

```powershell
Require-Pattern 'PERCEPTION_IMAGE_WIDTH\s+\(188U\)' 'project/code/perception_config.h'
Require-Pattern 'PERCEPTION_IMAGE_HEIGHT\s+\(120U\)' 'project/code/perception_config.h'
Require-Pattern 'PERCEPTION_SCORE_ACCEPT\s+\(166U\)' 'project/code/perception_config.h'
Require-Pattern 'PERCEPTION_SCORE_HIGH\s+\(204U\)' 'project/code/perception_config.h'
Require-Pattern 'PERCEPTION_MAX_TRACKS\s+\(32U\)' 'project/code/perception_config.h'
Require-Pattern 'PERCEPTION_GATE_CHI2_Q16\s+\(392561UL\)' 'project/code/perception_config.h'
Reject-Pattern '\b(malloc|calloc|realloc|free)\s*\(' 'project/code/perception_*.c'
```

Run: `powershell -ExecutionPolicy Bypass -File tools/test_perception_integration_static.ps1`

Expected: FAIL because the headers do not exist.

- [ ] **Step 2: Add the single source of truth for capacities and thresholds**

Define these exact groups in `perception_config.h`:

```c
#define PERCEPTION_IMAGE_WIDTH                 (188U)
#define PERCEPTION_IMAGE_HEIGHT                (120U)
#define PERCEPTION_FRAME_RATE_HZ               (50U)
#define PERCEPTION_DISCOVERY_DIVIDER           (2U)
#define PERCEPTION_MAP_RATE_HZ                 (20U)
#define PERCEPTION_MAX_CANDIDATES              (24U)
#define PERCEPTION_MAX_TRACKS                  (32U)
#define PERCEPTION_MAX_GAPS                    (31U)
#define PERCEPTION_SCORE_ACCEPT                (166U)
#define PERCEPTION_SCORE_HIGH                  (204U)
#define PERCEPTION_WEIGHT_TAPER                (77U)
#define PERCEPTION_WEIGHT_BAND                 (64U)
#define PERCEPTION_WEIGHT_SYMMETRY             (38U)
#define PERCEPTION_WEIGHT_ASPECT               (26U)
#define PERCEPTION_WEIGHT_BASE                 (26U)
#define PERCEPTION_WEIGHT_CONTRAST             (25U)
#define PERCEPTION_GATE_CHI2_Q16               (392561UL) /* 5.99 */
#define PERCEPTION_IMU_STALE_MS                (30U)
#define PERCEPTION_WHEEL_STALE_MS              (100U)
#define PERCEPTION_CAMERA_PREDICT_MS           (100U)
#define PERCEPTION_CAMERA_INVALID_MS           (300U)
#define PERCEPTION_FRAME_BUDGET_US             (20000U)
#define PERCEPTION_P99_TARGET_US               (15000U)
```

Add a preprocessor sum check requiring the six weights to equal 256.

- [ ] **Step 3: Define pointer-free public structures**

Use fixed-width fields and explicit validity bits. At minimum define:

```c
typedef enum
{
    PERCEPTION_STATE_BOOT = 0,
    PERCEPTION_STATE_READY,
    PERCEPTION_STATE_OUTBOUND_MAP,
    PERCEPTION_STATE_TURN_LOCK,
    PERCEPTION_STATE_RETURN_GAPS,
    PERCEPTION_STATE_FINISH,
    PERCEPTION_STATE_SAFE_STOP
} perception_state_enum;

typedef struct
{
    uint32 sequence;
    uint32 timestamp_us;
    float32 position_x_m;
    float32 position_y_m;
    float32 yaw_rad;
    float32 roll_rad;
    float32 pitch_rad;
    float32 speed_mps;
    float32 camera_height_m;
    uint16 validity_flags;
    uint16 reserved;
    uint32 crc32;
} perception_pose_snapshot_struct;

typedef struct
{
    uint16 gap_id;
    uint16 left_cone_id;
    uint16 right_cone_id;
    uint16 validity_flags;
    float32 center_x_m;
    float32 center_y_m;
    float32 heading_rad;
    float32 approach_x_m;
    float32 approach_y_m;
    float32 exit_x_m;
    float32 exit_y_m;
} perception_gap_target_struct;
```

Also define candidate, ground observation, track summary, diagnostics, and `perception_snapshot_struct`. Keep full internal covariance out of the cross-core snapshot; publish only the target, up to 32 compact map summaries, counters, state, sequence, timestamp, validity flags and CRC.

- [ ] **Step 4: Run static test and commit**

Run: `powershell -ExecutionPolicy Bypass -File tools/test_perception_integration_static.ps1`

Expected: PASS for contract section; later-file checks print SKIP until their tasks create the files.

Commit:

```powershell
git add project/code/perception_config.h project/code/perception_types.h tools/test_perception_integration_static.ps1
git commit -m "Define perception contracts and limits"
```

### Task 3: Implement offline geometry and ROI reference models

**Files:**
- Create: `tools/perception/__init__.py`
- Create: `tools/perception/reference_geometry.py`
- Create: `tools/perception/tests/__init__.py`
- Create: `tools/perception/tests/test_reference_geometry.py`
- Create: `tools/perception/tests/test_reference_roi.py`

- [ ] **Step 1: Add numeric tests for horizon and ray-ground intersection**

Cover level pose, ±10° roll, ±8° pitch, dynamic camera height, a ray parallel to the ground, and bottom-center projection. Assert that a level horizon is horizontal, roll tilts it with the expected sign, increasing pitch moves it consistently, and `abs(dz) < 0.05` or `lambda <= 0` returns invalid.

```python
def test_ground_projection_rejects_near_parallel_ray(self):
    result = project_pixel_to_ground(self.calib, self.pose, (94.0, 60.0))
    self.assertFalse(result.valid)

def test_dynamic_height_changes_range_not_bearing(self):
    low = project_pixel_to_ground(self.calib, pose(height=0.20), self.bottom_pixel)
    high = project_pixel_to_ground(self.calib, pose(height=0.30), self.bottom_pixel)
    self.assertAlmostEqual(low.bearing_rad, high.bearing_rad, places=4)
    self.assertGreater(high.range_m, low.range_m)
```

Run: `python -m unittest discover -s tools/perception/tests -p "test_reference_*.py" -v`

Expected: FAIL because reference modules do not exist.

- [ ] **Step 2: Implement the exact geometry pipeline**

Implement:

```text
raw pixel -> model-aware single-point undistortion -> K^-1 ray
-> R_vehicle_camera -> R_world_vehicle(roll,pitch,yaw) -> world ray d
-> lambda = -pc.z / d.z -> p = pc + lambda*d
```

The raw-image horizon must be generated by sampling 16 points of the ground-parallel ray boundary, applying distortion, and linearly interpolating a 188-column `horizon_y[x]`. It is not a single y-value. Discovery ROI begins at `clamp(horizon_y[x] + 4, 0, 119)` and excludes the configured bottom vehicle mask. Tracking windows are clipped to the same horizon and vehicle mask.

- [ ] **Step 3: Verify and commit**

Run: `python -m unittest discover -s tools/perception/tests -v`

Expected: all geometry/ROI tests PASS with no OpenCV full-frame remap.

Commit:

```powershell
git add tools/perception/__init__.py tools/perception/reference_geometry.py tools/perception/tests
git commit -m "Add perception geometry reference model"
```

### Task 4: Implement the offline detector and tuneable score output

**Files:**
- Create: `tools/perception/reference_detector.py`
- Create: `tools/perception/tests/test_reference_detector.py`
- Create: `tools/perception/replay.py`

- [ ] **Step 1: Add synthetic and BMP-level detector tests**

Generate 188×120 grayscale fixtures for a tapered cone, rectangle, vertical pole, floor seam, saturated frame and underexposed frame. Assert:

- the cone score is at least 166;
- rectangle and pole scores are below 166;
- score components sum to the returned integer score and never exceed 256;
- only 3–5 internal height slices are inspected after a connected candidate exists;
- discovery runs on even frames and tracking windows run every frame;
- saturated/underexposed quality gate suppresses map observations.

Run: `python -m unittest tools.perception.tests.test_reference_detector -v`

Expected: FAIL because detector functions do not exist.

- [ ] **Step 2: Implement the one-pass detector reference**

For each active ROI pixel, update a 5-pixel horizontal rolling mean, local contrast, and thresholded row runs in one left-to-right traversal. Link row runs to previous-row runs when horizontal overlap is non-empty, using a bounded union table. For each component compute bounding box, area, bottom center, foreground/background means, 3 slices for height `< 12`, otherwise 5 slices at 15/30/50/70/85% height.

Compute each feature in `[0, 256]` and score with integer rounding:

```python
score = (
    77 * taper + 64 * white_band + 38 * symmetry
    + 26 * aspect + 26 * base + 25 * contrast + 128
) >> 8
```

Return the six components in replay CSV so field data can change feature curves without changing the 256 weight contract.

- [ ] **Step 3: Add deterministic replay output**

`replay.py` accepts `--images`, `--pose-csv`, `--calibration`, `--output-csv`, and `--overlay-dir`. It sorts frames by numeric stem, rejects missing pose timestamps, and writes per-frame quality, ROI pixel count, candidates, component scores, accepted detections and elapsed host time.

- [ ] **Step 4: Run tests and commit**

```powershell
python -m unittest discover -s tools/perception/tests -v
git add tools/perception/reference_detector.py tools/perception/replay.py tools/perception/tests/test_reference_detector.py
git commit -m "Add offline cone detector replay"
```

### Task 5: Implement offline tracker, map, and gap state machine

**Files:**
- Create: `tools/perception/reference_tracker.py`
- Create: `tools/perception/tests/test_reference_tracker.py`
- Create: `tools/perception/tests/test_reference_gap.py`

- [ ] **Step 1: Add failing association and lifecycle tests**

Tests must cover `D² <= 5.99`, Euclidean hard gate, nearest-neighbor conflict, 3/5 confirmation, far/small 4/6 confirmation, 8-frame prediction-only hold, deletion after 20 frames, max 32 tracks, and overlapping-track merge preserving the older ID.

- [ ] **Step 2: Add failing `N-1` and return-order tests**

Create non-collinear cone fixtures. Freeze the outbound start-to-turn unit axis, sort by longitudinal projection, assert exactly `N-1` adjacent gaps, then assert return selection is far-to-near. For every gap verify approach and exit are 0.6 m on opposite sides of the center along the return heading. Assert that a turn lock freezes IDs and order even if later detections jitter.

- [ ] **Step 3: Implement bounded tracking and gap logic**

Use a 2-D constant-position static process model with vehicle-motion compensation. Data association first applies Euclidean range based on covariance and then 2-D Mahalanobis distance. Resolve collisions by lowest `D²`, then higher detector score, then lower track ID. New tracks consume the first free slot; if full, count overflow and do not evict a confirmed track.

Implement exact state transitions:

```text
BOOT -> READY: calibration, camera, pose and shared memory valid
READY -> OUTBOUND_MAP: explicit task-start edge
OUTBOUND_MAP -> TURN_LOCK: explicit turnaround-entry edge
TURN_LOCK -> RETURN_GAPS: frozen map has >=2 confirmed cones and generated gaps
RETURN_GAPS -> FINISH: every frozen gap reports exit crossing
any active state -> SAFE_STOP: critical stale/CRC/calibration/state invariant failure
```

- [ ] **Step 4: Run tests and commit**

```powershell
python -m unittest discover -s tools/perception/tests -v
git add tools/perception/reference_tracker.py tools/perception/tests/test_reference_tracker.py tools/perception/tests/test_reference_gap.py
git commit -m "Add cone map and gap reference model"
```

### Task 6: Add shared memory, CRC, and linker reservation

**Files:**
- Create: `project/code/perception_crc.h`
- Create: `project/code/perception_crc.c`
- Create: `project/code/perception_shared.h`
- Create: `project/code/perception_shared.c`
- Modify: `project/iar/icf/linker_directives_tviibh.icf`
- Modify: `tools/test_perception_integration_static.ps1`

- [ ] **Step 1: Extend static tests for an exact 8 KB no-init section**

Require `.perception_shared`, `0x2000`, `do not initialize`, two slots per direction, CRC before publish, and a memory barrier. Reject raw image arrays and pointer fields from the shared container.

Run the static script and confirm it fails only the new shared-memory checks.

- [ ] **Step 2: Implement CRC32 and double-buffer protocol**

Use polynomial `0xEDB88320`, initial/final xor `0xFFFFFFFF`. Expose:

```c
void perception_shared_owner_init(void);
uint8 perception_shared_publish_pose(const perception_pose_snapshot_struct *snapshot);
uint8 perception_shared_read_pose(perception_pose_snapshot_struct *snapshot);
uint8 perception_shared_publish_perception(const perception_snapshot_struct *snapshot);
uint8 perception_shared_read_perception(perception_snapshot_struct *snapshot);
```

Writer sequence: copy to inactive slot with `crc32=0`, compute CRC, write CRC, execute `__DMB()`, publish slot index and sequence. Reader sequence: read publication word, copy slot, `__DMB()`, reread publication word, then accept only if unchanged, monotonic and CRC-valid. CM7_0 alone initializes the owner header at boot; CM7_1 waits for the magic/schema before publishing.

- [ ] **Step 3: Reserve the linker section without overlapping CM7_1 SRAM**

Define a shared reserve of `0x2000`, reduce `_size_SRAM_CM7_1` by that amount, define `SRAM_PERCEPTION_SHARED` at the old top of CM7_1 SRAM, and add:

```text
do not initialize { section .perception_shared };
place at start of SRAM_PERCEPTION_SHARED { section .perception_shared };
```

Place one `__root` shared container in this section. Add compile-time size checks that the container is `<= 8192` and both snapshot structs are 4-byte aligned.

- [ ] **Step 4: Run static checks and commit**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_perception_integration_static.ps1
git diff --check
git add project/code/perception_crc.h project/code/perception_crc.c project/code/perception_shared.h project/code/perception_shared.c project/iar/icf/linker_directives_tviibh.icf tools/test_perception_integration_static.ps1
git commit -m "Add perception shared snapshot transport"
```

### Task 7: Port calibration, horizon, ROI, and projection to firmware

**Files:**
- Create: `project/code/perception_calibration.h`
- Create: `project/code/perception_calibration.c`
- Create: `project/code/perception_roi.h`
- Create: `project/code/perception_roi.c`
- Create: `project/code/perception_projection.h`
- Create: `project/code/perception_projection.c`
- Modify: `tools/test_perception_integration_static.ps1`

- [ ] **Step 1: Add static API and forbidden-operation checks**

Require the following interfaces and reject full-frame undistortion buffers:

```c
uint8 perception_calibration_validate(void);
uint8 perception_calibration_ray(float32 raw_u, float32 raw_v, float32 ray_camera[3]);
uint8 perception_roi_build_horizon(const perception_pose_snapshot_struct *pose, int16 horizon_y[PERCEPTION_IMAGE_WIDTH]);
void perception_roi_build(uint32 frame_sequence, const int16 horizon_y[PERCEPTION_IMAGE_WIDTH], const perception_track_window_struct *windows, uint8 window_count, perception_roi_struct *roi);
uint8 perception_project_candidate(const perception_candidate_struct *candidate, const perception_pose_snapshot_struct *pose, perception_ground_observation_struct *observation);
```

- [ ] **Step 2: Port the tested reference equations**

Use generated calibration constants checked by schema/resolution/CRC. Convert only the candidate bottom-center and 16 horizon sample points; never remap all 22,560 pixels. Projection rejects stale pose, invalid calibration, `abs(dz)<0.05`, `lambda<=0`, points behind the vehicle, and range outside the configured 0.25–8.0 m band.

Propagate covariance numerically with ±1 pixel in u/v, pose roll/pitch uncertainty, and camera-height uncertainty; store a symmetric 2×2 covariance in the ground observation.

- [ ] **Step 3: Add a host parity fixture**

Extend the static script to parse a small C-generated CSV fixture containing raw pixels, pose, projected x/y and validity. Compare 20 golden cases against `reference_geometry.py` with absolute tolerance 0.03 m and matching validity flags.

- [ ] **Step 4: Verify and commit**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_perception_integration_static.ps1
python -m unittest tools.perception.tests.test_reference_geometry tools.perception.tests.test_reference_roi -v
git add project/code/perception_calibration.* project/code/perception_roi.* project/code/perception_projection.* tools/test_perception_integration_static.ps1
git commit -m "Port perception geometry to firmware"
```

### Task 8: Port bounded detector, tracker, and gap logic to firmware

**Files:**
- Create: `project/code/perception_detector.h`
- Create: `project/code/perception_detector.c`
- Create: `project/code/perception_tracker.h`
- Create: `project/code/perception_tracker.c`
- Create: `project/code/perception_gap.h`
- Create: `project/code/perception_gap.c`
- Modify: `tools/test_perception_integration_static.ps1`

- [ ] **Step 1: Add static checks for bounded storage and exact APIs**

Require compile-time arrays sized from config constants, 3/5 and 4/6 confirmation masks, eight-frame prediction hold, 20-frame deletion, and no recursion or heap usage. Require:

```c
uint8 perception_detector_run(const uint8 image[PERCEPTION_IMAGE_HEIGHT][PERCEPTION_IMAGE_WIDTH], const perception_roi_struct *roi, perception_candidate_struct candidates[PERCEPTION_MAX_CANDIDATES], perception_detector_diag_struct *diag);
void perception_tracker_init(perception_tracker_struct *tracker);
void perception_tracker_step(perception_tracker_struct *tracker, const perception_ground_observation_struct *observations, uint8 observation_count, const perception_pose_snapshot_struct *pose);
uint8 perception_gap_lock_map(const perception_tracker_struct *tracker, const perception_pose_snapshot_struct *pose, perception_gap_map_struct *gap_map);
void perception_gap_step(perception_gap_map_struct *gap_map, const perception_pose_snapshot_struct *pose, uint32 event_flags, perception_gap_target_struct *target);
```

- [ ] **Step 2: Port detector with deterministic overflow behavior**

Use 188-entry previous/current row-run arrays and a 24-entry component table. When more than 24 candidates qualify, retain the 24 highest scores using insertion into a fixed sorted index array; ties prefer larger pixel height, then smaller x. Record ROI pixels, runs, components, candidate overflow, quality rejection and maximum elapsed cycles.

- [ ] **Step 3: Port tracker and state machine**

Match the Python reference tie-breakers and lifecycle exactly. Use stable 16-bit cone IDs that never change after confirmation and are not recycled during the task. At TURN_LOCK copy only confirmed tracks into a frozen array, project onto the start-to-turn axis, stable-sort ascending, generate adjacent pairs, then expose gaps in reverse order.

- [ ] **Step 4: Add replay parity comparison**

Add a host-readable firmware trace schema. For a fixed synthetic sequence, compare accepted candidate IDs/scores, projected positions, confirmed IDs, frozen order and current gap ID against Python reference output. Exact integer fields must match; positions may differ by at most 0.03 m.

- [ ] **Step 5: Verify and commit**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_perception_integration_static.ps1
python -m unittest discover -s tools/perception/tests -v
git add project/code/perception_detector.* project/code/perception_tracker.* project/code/perception_gap.* tools/test_perception_integration_static.ps1
git commit -m "Add bounded cone perception pipeline"
```

### Task 9: Integrate MT9V03X acquisition and non-blocking debug output on CM7_1

**Files:**
- Create: `project/code/perception_camera.h`
- Create: `project/code/perception_camera.c`
- Create: `project/code/perception_app.h`
- Create: `project/code/perception_app.c`
- Modify: `project/user/main_cm7_1.c`
- Modify: `project/user/cm7_1_isr.c`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_1.ewp`
- Modify: `tools/test_perception_integration_static.ps1`

- [ ] **Step 1: Import and identify the exact working camera symbols**

Copy the already hardware-proven MT9V03X driver/configuration used for the live `188x120 @ 50 FPS` image into the branch through the existing SeekFree library integration; do not reimplement MIPI timing. Record the actual init function, frame-ready flag, image symbol, VSYNC hook and WiFi-SPI image-send API at the top of `perception_camera.c`. The adapter must fail compilation if width or height is not 188×120.

- [ ] **Step 2: Define a narrow camera adapter and failing static checks**

Expose:

```c
uint8 perception_camera_init(void);
uint8 perception_camera_try_acquire(perception_frame_struct *frame);
void perception_camera_release(uint32 frame_sequence);
void perception_camera_vsync_isr(uint32 timestamp_us);
void perception_camera_debug_step(void);
```

`try_acquire` returns immediately when no complete frame exists. ISR only timestamps and swaps producer index; it does not detect cones or send WiFi data. Two static 22,560-byte image buffers are owned only by CM7_1. Debug output uses a one-frame mailbox; a new debug request overwrites/drops an old pending request and increments `debug_drop_count`.

- [ ] **Step 3: Implement image quality and exposure diagnostics**

Sample every fourth pixel to compute p05, p50, p95 and clipped-dark/clipped-bright ratios. Reject mapping observations when clipped fraction exceeds 30% or dynamic range `p95-p05 < 24`; still run timestamp and prediction logic. Report exposure/gain if the camera driver exposes them, but do not let automatic exposure change more often than once per 10 frames.

- [ ] **Step 4: Integrate the CM7_1 pipeline**

`perception_app_step()` performs at most one bounded unit per call: acquire pose, acquire frame, build ROI, detect, project, track, update map at 20 Hz, publish snapshot, or service one debug chunk. Measure DWT cycles for every frame. After three consecutive frames above 20 ms, disable global discovery and debug streaming until ten consecutive frames are below 15 ms; tracking windows remain active.

`main_cm7_1.c` initializes clocks, timing, shared reader, camera and app, then repeatedly calls `perception_app_step()` without blocking delays.

- [ ] **Step 5: Add every new source/header to the CM7_1 IAR project**

Place public headers and source files in the existing `code` group. The static test must verify every expected path appears once in `cyt4bb7_cm_7_1.ewp` and that no CM7_0 motor/control source was added to CM7_1.

- [ ] **Step 6: Build and run the camera-only hardware gate**

From an IAR command shell:

```powershell
IarBuild.exe project/iar/project_config/cyt4bb7_cm_7_1.ewp -build Debug
```

If the installed configuration name differs, use the configuration shown in the IAR workspace and record it in the hardware test document. Expected: zero compiler/linker errors, shared section <=8 KB, static RAM leaves both image buffers plus at least 16 KB stack/working headroom.

Flash CM7_1 with motors disabled. Observe 10 minutes: camera remains 50 FPS, no buffer overwrite, WiFi disconnect does not change recognition cycle count, and quality flags react to covering/overexposing the lens.

- [ ] **Step 7: Commit CM7_1 acquisition integration**

```powershell
git add project/code/perception_camera.* project/code/perception_app.* project/user/main_cm7_1.c project/user/cm7_1_isr.c project/iar/project_config/cyt4bb7_cm_7_1.ewp tools/test_perception_integration_static.ps1
git commit -m "Integrate MT9V03X perception on CM7_1"
```

### Task 10: Integrate CM7_0 pose publication and safe target consumption

**Files:**
- Create: `project/code/perception_client.h`
- Create: `project/code/perception_client.c`
- Modify: `project/code/app_config.h`
- Modify: `project/code/app_scheduler.c`
- Modify: `project/code/app.c`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_0.ewp`
- Modify: `tools/test_perception_integration_static.ps1`

- [ ] **Step 1: Add failing checks for ownership and staleness**

Require CM7_0 to be the only shared owner initializer. Require pose publishing at 100 Hz and target reading at 50 Hz. Require stale checks at 30 ms IMU, 100 ms wheel, 100 ms prediction-only camera and 300 ms invalid camera. Reject any call from `perception_client.c` to motor PWM, servo PWM, balance output or direct chassis actuator functions.

- [ ] **Step 2: Implement the client API**

Expose:

```c
void perception_client_init(void);
void perception_client_publish_pose_100hz(uint32 now_us);
void perception_client_update_50hz(uint32 now_us);
uint8 perception_client_get_target(perception_gap_target_struct *target);
uint8 perception_client_is_safe(void);
```

Build camera height from fixed camera mount height plus the existing validated leg-height estimate. Capture IMU, wheel speed, local odometry and height from one scheduler tick; set validity bits per sensor age rather than substituting zeros. Target getter returns false for CRC failure, sequence regression, invalid state, target invalid flag or age >300 ms.

- [ ] **Step 3: Connect scheduler without authorizing motion**

Add the periodic publish/read calls to existing scheduler slots. In `app.c`, expose the perception state and target through telemetry/diagnostic accessors only. Do not feed the target into steering, balance, motor or servo commands in this task. On perception unsafe, set a perception fault/status bit that the existing task supervisor can later use; do not overwrite unrelated current faults.

- [ ] **Step 4: Add CM7_0 project membership, build all affected cores, and commit**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_perception_integration_static.ps1
IarBuild.exe project/iar/project_config/cyt4bb7_cm_7_0.ewp -build Debug
IarBuild.exe project/iar/project_config/cyt4bb7_cm_7_1.ewp -build Debug
```

Expected: both cores build with zero errors; map file shows one identical `.perception_shared` address; CM7_0 actuator behavior is unchanged when perception is absent.

Commit:

```powershell
git add project/code/perception_client.* project/code/app_config.h project/code/app_scheduler.c project/code/app.c project/iar/project_config/cyt4bb7_cm_7_0.ewp tools/test_perception_integration_static.ps1
git commit -m "Add safe perception client on CM7_0"
```

### Task 11: Add field replay, diagnostics, and hardware acceptance protocol

**Files:**
- Modify: `tools/perception/replay.py`
- Create: `tools/perception/evaluate.py`
- Create: `tools/perception/tests/test_evaluate.py`
- Create: `docs/cone-perception-hardware-test.md`

- [ ] **Step 1: Define the dataset and metric schema in tests**

Each labeled frame record contains timestamp, image path, pose, zero or more cone bottom pixels, world cone IDs/positions, task state and visibility flags. Evaluation returns precision, recall, false positives per frame, bottom-pixel error, ground-position error by distance bin, ID switches, map duplicates, gap-count correctness, gap-order correctness and frame-time percentiles.

Tests must prove zero-denominator handling, distance bin edges, ID-switch counting and percentile computation.

- [ ] **Step 2: Implement evaluator and replay exports**

`evaluate.py` accepts label JSONL and replay CSV, performs one-to-one matching with configurable pixel/world gates, and writes machine-readable JSON plus a concise Markdown table. `replay.py` additionally writes a per-frame trace compatible with the evaluator and a parameter fingerprint containing calibration CRC, config constants and Git commit.

- [ ] **Step 3: Write the staged hardware protocol**

The document must contain checkboxes and tables for:

1. **Static camera:** 20 checkerboard views, native 188×120, calibration RMS and held-out edge error.
2. **Ground projection:** marks at 0.5/1/2/3/5 m and left/center/right; median error ≤0.10 m through 3 m and ≤0.20 m at 5 m.
3. **Static cones:** bright, dark, shadow, paving seams and partial occlusion; first acceptance target precision ≥90%, recall ≥85% on at least 300 labeled frames.
4. **Moving open loop:** motors/steering perception output disconnected; 0.5, 0.75 and 1.0 m/s logs; no map ID reorder after TURN_LOCK and exactly `N-1` gaps.
5. **Fault injection:** cover lens, freeze frames, stop pose publisher, corrupt CRC, disconnect WiFi; verify prediction-only at 100 ms and invalid/SAFE_STOP signal at 300 ms without actuator surprises.
6. **Timing:** at least 10,000 frames; P99 ≤15 ms, maximum <20 ms in normal mode, zero camera buffer overruns. If the initial target is missed, record the measured value and keep the hardware gate open rather than weakening the threshold silently.

- [ ] **Step 4: Verify and commit**

```powershell
python -m unittest discover -s tools/perception/tests -v
powershell -ExecutionPolicy Bypass -File tools/test_perception_integration_static.ps1
git diff --check
git add tools/perception/replay.py tools/perception/evaluate.py tools/perception/tests/test_evaluate.py docs/cone-perception-hardware-test.md
git commit -m "Add cone perception field evaluation"
```

### Task 12: End-to-end release gate

**Files:**
- Modify only if a verified failure requires a fix; otherwise no source changes.

- [ ] **Step 1: Run all host and static tests from a clean shell**

```powershell
python -m unittest discover -s tools/calibration/tests -v
python -m unittest discover -s tools/perception/tests -v
powershell -ExecutionPolicy Bypass -File tools/test_perception_integration_static.ps1
git diff --check
```

Expected: all tests PASS, static script has no SKIP entries, `git diff --check` is silent.

- [ ] **Step 2: Build all three cores**

```powershell
IarBuild.exe project/iar/project_config/cyt4bb7_cm_0_plus.ewp -build Debug
IarBuild.exe project/iar/project_config/cyt4bb7_cm_7_0.ewp -build Debug
IarBuild.exe project/iar/project_config/cyt4bb7_cm_7_1.ewp -build Debug
```

Expected: zero errors. Record configuration name, code/RAM sizes and `.perception_shared` address in `docs/cone-perception-hardware-test.md`.

- [ ] **Step 3: Run offline acceptance on the frozen dataset**

```powershell
python tools/perception/replay.py --images "tmp/perception_dataset/frames/*.bmp" --pose-csv "tmp/perception_dataset/pose.csv" --calibration "tmp/perception_dataset/calibration.json" --output-csv tmp/perception_replay.csv --overlay-dir tmp/perception_overlay
python tools/perception/evaluate.py --labels "tmp/perception_dataset/labels.jsonl" --replay tmp/perception_replay.csv --output-json tmp/perception_metrics.json --output-md tmp/perception_metrics.md
```

Expected: metric report states dataset size and parameter fingerprint and meets the thresholds in Task 11. Dataset paths are operator inputs; generated images and metrics remain untracked artifacts.

- [ ] **Step 4: Execute and sign the hardware gates in order**

Do not enable target-to-chassis control as part of this plan. Finish when static camera, ground projection, static-cone, moving open-loop, fault-injection and timing sections have measured results and all perception acceptance boxes are signed. Any later closed-loop path-following work starts from a separate design because it changes actuator authority and safety scope.

- [ ] **Step 5: Final repository audit**

```powershell
git status --short --branch
git log --oneline --decorate -12
git diff origin/codex/leg-control-speed-assist...HEAD --stat
```

Confirm no BMP, NPZ, JSON calibration output, replay overlay, CSV log, generated IAR output or pre-existing user data was committed. If all gates pass, commit only the completed hardware test record:

```powershell
git add docs/cone-perception-hardware-test.md
git commit -m "Record cone perception hardware validation"
```

# Balance Fast Mode Servo Control Transfer Design

日期：2026-07-18

源分支：`codex/leg-control-speed-assist`

目标分支：`codex/balance-fast-mode-spec`

## 1. 目标

把轮腿分支中已经形成完整依赖链的 servo/leg 控制成果移入快速平衡分支，为后续更快的车体速度试验提供腿部执行基础，同时保持现有电机、平衡和腿部安全门限。

本次迁移只证明代码与主机侧回归测试完成，不把 300 Hz PWM、腿部运动或高速平衡标记为硬件验收通过。

## 2. 已确认的分支关系

`codex/balance-fast-mode-spec` 当前位于 `da1ebe5`，并且是 `codex/leg-control-speed-assist` 的祖先。`4f057a1a` 比 `da1ebe5` 多 84 个提交，形成一条可直接快进的历史：

```text
da1ebe5  Guard validated fast speed cap
   |
   | 84 commits: leg control, 300 Hz servo, validation and fixes
   v
4f057a1a  Fix measured five-bar IK geometry
```

`4f057a1a` 之前的 leg 历史已经包含 cross-circle/PC 相机辅助 IK 标定工具；这些工具随线性历史一同保留。`4f057a1a` 之后开始混入 CM7_1 cone perception 运行时历史，直到 `c07702a` 才再次出现物理坐标标定改动。因此本次明确以 `4f057a1a` 为迁移边界，不引入其后的 cone-perception 运行时提交，也不直接移动到 `c07702a`。

## 3. 采用方案

在目标工作区中执行：

```powershell
git switch codex/balance-fast-mode-spec
git merge --ff-only 4f057a1a
```

采用快进而不是 cherry-pick 或文件复制，原因如下：

- servo 控制依赖前序 leg 配置、IK、调度、遥测和测试契约；
- 完整历史已经线性建立，不需要制造重复提交；
- `--ff-only` 会在分支关系发生变化时失败关闭，避免无意生成合并提交；
- 现有目标工作区里的 `.superpowers/`、`data/` 和 `fast_tune_kit.zip` 保持未跟踪状态，不加入迁移提交。

设计文档和后续实施计划单独提交。执行快进后，再把这两个纯文档提交接到 `4f057a1a` 之后，使代码历史仍完整包含该干净里程碑。

## 4. 迁移能力边界

迁移到目标分支的 servo/leg 关键能力包括：

- 300 Hz TCPWM servo 输出与约 3333 us 执行 tick；
- `servo_motion` 一阶低通执行单元；
- 四路同步 S7 腿部轨迹；
- 正常和快速模式的最终角速度护栏；
- servo settled 连续确认和底盘运动门控；
- 非阻塞 servo/leg 遥测；
- `Cy_Tcpwm_Pwm_SetCompare0_Buff()` 后调用 `Cy_Tcpwm_TriggerCapture0()`，确保缓冲 compare 值切换到有效输出；
- 五连杆 IK、已测几何和腿部转换控制；
- IMU 标定重试、时序和噪声完整性修复，这些修复与 servo/leg 安全链共享调度和启动路径。

本次不迁移：

- `4f057a1a` 之后的 CM7_1 cone perception、gap mapping 和 shared snapshot 提交；
- `c07702a` 中依赖相机标记测量的物理坐标标定尾部；
- 工作区中的采集 CSV、相机样本、临时目录或压缩包；
- 新的平衡增益、RPM 上限、转向增益或自动高速使能逻辑。

因为采用线性快进，`8807176` 到 `98edcdd` 的 cross-circle 标定工具提交会保留；它们是离线标定支持，不在 CM7_1 上启用实时 cone perception。

## 5. 安全要求

- 不提高 `APP_CHASSIS_FAST_FORWARD_RPM_LIMIT`、`APP_BALANCE_RPM_LIMIT` 或任何电机执行上限。
- 不修改 fast-mode 的显式使能、姿态限制、IMU 新鲜度、BLDC 在线状态和 fault gate。
- 不把软件中的 `output_angle_deg` 描述为实测 servo 角度；当前 servo 仍是 PWM-only 开环执行器。
- 不因主机侧测试通过而自动进行带载高速运动。
- 上板验证必须先架空或可靠支撑车辆，保持轮电机停用，从四路 PWM 周期和 90 度脉宽开始检查。

## 6. 验证策略

### 6.1 迁移边界

执行前后验证：

```powershell
git merge-base codex/balance-fast-mode-spec 4f057a1a
git rev-list --left-right --count codex/balance-fast-mode-spec...4f057a1a
git log --oneline codex/balance-fast-mode-spec..4f057a1a
```

执行后代码提交必须包含 `4f057a1a`，且不得包含 `5c216e4` 到 `c07702a` 的 cone-perception 运行时尾部。

### 6.2 主机侧测试

至少运行以下测试：

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_servo_motion_numeric.ps1
powershell -ExecutionPolicy Bypass -File tools/test_servo_300hz_integration_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_servo_pwm_frequency_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_servo_pwm_resolution_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_leg_transition_numeric.ps1
powershell -ExecutionPolicy Bypass -File tools/test_ik_height_control_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_timing_noise_regressions.ps1
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
```

同时运行与这些脚本关联的汇总入口（若存在），并执行 `git diff --check`。已知 `tools/calibration/geometry_utils.py` 在迁移历史中报告文件末尾新增空行；它是既有非功能性差异，不应扩展为无关的批量格式化。

### 6.3 IAR 和硬件门槛

- 构建受影响的 `cyt4bb7_cm_7_0`；若当前环境无法启动 IAR，明确记录为未验证，不以静态测试替代。
- 示波器确认四路 servo PWM 周期约 3.333 ms，90 度脉宽约 1.5 ms。
- 先验证安全参考、保持力、温升和 fault，再进行小范围腿高动作。
- 只有在腿部执行与平衡分别通过硬件门槛后，才逐级提高底盘速度；本次迁移本身不授权带载高速测试。

## 7. 回退与发布

快进前记录目标 SHA `da1ebe5`。如果主机侧回归失败，不推送目标分支，并在修复前保留失败状态和日志。若已在本地完成快进但尚未推送，可通过建立恢复分支保留现场，再由用户明确授权是否回退；不使用 `git reset --hard` 清理用户工作区。

本次默认只完成本地迁移、文档和验证。除非用户再次明确要求，不推送 `codex/balance-fast-mode-spec`，也不创建 PR。

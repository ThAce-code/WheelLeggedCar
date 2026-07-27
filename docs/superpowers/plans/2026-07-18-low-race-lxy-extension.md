# Low-race LXY Extension Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let the existing `LXY` command reach the hardware-tested left-leg `40 deg, 140 deg` low-race pose without adding a command or globally disabling workspace validation.

**Architecture:** Keep the calibrated eight-vertex inset hull as the default validator and add one configuration-backed experimental acceptance window centred on the model estimate `(-18.831, 25.076) mm`. Lower the private model Y bound from `25.0 mm` to `23.0 mm`, just enough for that pose, while retaining servo limits, finite checks, IK branches, S7 planning, motor-stop behaviour, and all legacy rejection contracts.

**Tech Stack:** Embedded C99, PowerShell host harnesses, GCC numeric harness, IAR Embedded Workbench 9.40.1, CYT4BB7/Traveo II.

## Global Constraints

- Do not add `LRACE` or change the UART wire syntax.
- Do not change `LIK`; its hardware-tested `40,140` path remains available.
- Preserve the per-servo `10 deg .. 175 deg` limits.
- Preserve the calibrated hull and its `2.0f mm` inset for all non-experimental points.
- Keep `LXY,0,55` and raw hull vertices rejected.
- The low-race point is a command/model estimate, not measured wheel-centre feedback.
- Do not enable balance fast mode, wheel motors, direct servo bypass, or dynamic speed-to-shift mapping.
- Preserve unrelated untracked files under `data/` and `fast_tune_kit.zip`.

---

## File map

- `project/code/leg_config.h`: owns the experimental low-race acceptance-window fields.
- `project/code/leg_config.c`: owns the enabled target, `0.5 mm` tolerances, and private model Y lower bound.
- `project/code/leg_kinematics.c`: validates the narrow experimental target in addition to the unchanged inset hull.
- `tools/test_leg_ik_zero_calibration_static.ps1`: compiles the real kinematics implementation and proves low-race acceptance, mirrored command mapping, and legacy rejection.
- `tools/test_leg_physical_ik_static.ps1`: locks the configuration and non-global-bypass source contract.
- `docs/leg-ik-zero-calibration-hardware-test.md`: records the supported static test command and its experimental status.

### Task 1: Add the failing low-race numeric contract

**Files:**
- Modify: `tools/test_leg_ik_zero_calibration_static.ps1:54-107`
- Modify: `tools/test_leg_ik_zero_calibration_static.ps1:134-171`
- Modify: `tools/test_leg_physical_ik_static.ps1:40-105`

**Interfaces:**
- Consumes: `leg_kinematics_target_valid`, `leg_kinematics_solve`, `leg_kinematics_forward_command`, and `leg_kinematics_map_target_pose`.
- Produces: a regression contract for the configured target `(-18.831f, 25.076f)` and mirrored logical commands `[40, 140, 140, 40]`.

- [ ] **Step 1: Extend the GCC numeric harness before changing production code**

Add these variables near the other harness locals:

```c
    float low_left_x;
    float low_left_y;
    float low_right_x;
    float low_right_y;
```

Insert the following block after the existing `LXY,0,55` rejection check and before `return 0;`:

```c
    if((APP_TRUE != leg_kinematics_target_valid(-18.831f, 25.076f)) ||
       (APP_TRUE != leg_kinematics_target_valid(-18.83f, 25.08f)))
    {
        return 21;
    }
    if((APP_TRUE == leg_kinematics_target_valid(-18.20f, 25.08f)) ||
       (APP_TRUE == leg_kinematics_target_valid(-18.83f, 24.50f)))
    {
        return 22;
    }
    if((APP_TRUE != leg_kinematics_solve(APP_FALSE, -18.831f, 25.076f,
                                         &left_ref, &left_target)) ||
       (APP_TRUE != leg_kinematics_solve(APP_TRUE, -18.831f, 25.076f,
                                         &right_ref, &right_target)) ||
       (APP_TRUE != leg_kinematics_map_target_pose(&left_ref, &right_ref,
                                                    &left_target, &right_target,
                                                    target_cmd)))
    {
        return 23;
    }
    if((fabsf(target_cmd[LEG_SERVO_FL] - 40.0f) > 0.05f) ||
       (fabsf(target_cmd[LEG_SERVO_FR] - 140.0f) > 0.05f) ||
       (fabsf(target_cmd[LEG_SERVO_RL] - 140.0f) > 0.05f) ||
       (fabsf(target_cmd[LEG_SERVO_RR] - 40.0f) > 0.05f))
    {
        return 24;
    }
    if((APP_TRUE != leg_kinematics_forward_command(APP_FALSE, 40.0f, 140.0f,
                                                    &low_left_x, &low_left_y)) ||
       (APP_TRUE != leg_kinematics_forward_command(APP_TRUE, 140.0f, 40.0f,
                                                    &low_right_x, &low_right_y)))
    {
        return 25;
    }
    if((fabsf(low_left_x - (-18.831f)) > 0.01f) ||
       (fabsf(low_left_y - 25.076f) > 0.01f) ||
       (fabsf(low_right_x - low_left_x) > 0.01f) ||
       (fabsf(low_right_y - low_left_y) > 0.01f))
    {
        return 26;
    }
    if(APP_TRUE == leg_kinematics_forward_command(APP_FALSE, 9.0f, 140.0f,
                                                   &low_left_x, &low_left_y))
    {
        return 27;
    }
```

Add source assertions to `tools/test_leg_physical_ik_static.ps1`:

```powershell
Require-Pattern $configHeader 'experimental_race_enable' `
    "experimental low-race enable missing"
Require-Pattern $configHeader 'experimental_race_x_mm' `
    "experimental low-race target X missing"
Require-Pattern $configHeader 'experimental_race_y_mm' `
    "experimental low-race target Y missing"
Require-Pattern $configHeader 'experimental_race_tolerance_x_mm' `
    "experimental low-race X tolerance missing"
Require-Pattern $configHeader 'experimental_race_tolerance_y_mm' `
    "experimental low-race Y tolerance missing"
Require-Pattern $configHeader 'experimental_race_ik_min_margin' `
    "experimental low-race IK margin missing"
Require-Pattern $configSource '\.experimental_race_enable\s*=\s*1U' `
    "experimental low-race target must be explicitly enabled"
Require-Pattern $configSource '\.experimental_race_x_mm\s*=\s*-18\.831f' `
    "experimental low-race target X mismatch"
Require-Pattern $configSource '\.experimental_race_y_mm\s*=\s*25\.076f' `
    "experimental low-race target Y mismatch"
Require-Pattern $configSource '\.experimental_race_tolerance_x_mm\s*=\s*0\.5f' `
    "experimental low-race X tolerance mismatch"
Require-Pattern $configSource '\.experimental_race_tolerance_y_mm\s*=\s*0\.5f' `
    "experimental low-race Y tolerance mismatch"
Require-Pattern $configSource '\.experimental_race_ik_min_margin\s*=\s*0\.02f' `
    "experimental low-race IK margin mismatch"
Require-Pattern $kinematicsSource 'leg_kinematics_experimental_race_target_valid' `
    "narrow experimental target validator missing"
Reject-Pattern $kinematicsSource 'return\s+APP_TRUE;\s*/\*.*bypass' `
    "physical target validation must not be globally bypassed"
```

- [ ] **Step 2: Run the focused tests and verify RED**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\test_leg_ik_zero_calibration_static.ps1
powershell -ExecutionPolicy Bypass -File tools\test_leg_physical_ik_static.ps1
```

Expected: both commands fail because the low-race target is still outside the hull and the experimental configuration fields/helper do not exist. The numeric harness must not fail because of a compiler or script syntax error.

### Task 2: Implement the narrow configuration-backed extension

**Files:**
- Modify: `project/code/leg_config.h:43-71`
- Modify: `project/code/leg_config.c:20-52`
- Modify: `project/code/leg_kinematics.c:292-338`

**Interfaces:**
- Consumes: `leg_kinematics_config_struct` and the existing physical hull.
- Produces: private `static uint8 leg_kinematics_experimental_race_target_valid(...)`; the public `leg_kinematics_target_valid(float,float)` signature is unchanged.

- [ ] **Step 1: Add explicit configuration fields**

Add these fields immediately after `physical_workspace_inset_mm` in `leg_kinematics_config_struct`:

```c
    uint8 experimental_race_enable;
    float experimental_race_x_mm;
    float experimental_race_y_mm;
    float experimental_race_tolerance_x_mm;
    float experimental_race_tolerance_y_mm;
    float experimental_race_ik_min_margin;
    leg_ik_branch_enum experimental_race_alpha_branch;
    leg_ik_branch_enum experimental_race_beta_branch;
```

Change only the private model lower bound and initialize the new fields in `leg_config.c`:

```c
        .y_min_mm = 23.0f,
```

```c
        .physical_workspace_inset_mm = 2.0f,
        .experimental_race_enable = 1U,
        .experimental_race_x_mm = -18.831f,
        .experimental_race_y_mm = 25.076f,
        .experimental_race_tolerance_x_mm = 0.5f,
        .experimental_race_tolerance_y_mm = 0.5f,
        .experimental_race_ik_min_margin = 0.02f,
        .experimental_race_alpha_branch = LEG_IK_BRANCH_PLUS,
        .experimental_race_beta_branch = LEG_IK_BRANCH_PLUS,
```

- [ ] **Step 2: Add the fail-closed experimental validator**

Insert this helper immediately before `leg_kinematics_target_valid`:

```c
static uint8 leg_kinematics_experimental_race_target_valid(
    const leg_kinematics_config_struct *cfg,
    float x_mm,
    float y_mm)
{
    if((NULL == cfg) ||
       (0U == cfg->experimental_race_enable) ||
       (APP_FALSE == leg_kinematics_is_finite(cfg->experimental_race_x_mm)) ||
       (APP_FALSE == leg_kinematics_is_finite(cfg->experimental_race_y_mm)) ||
       (APP_FALSE == leg_kinematics_is_finite(cfg->experimental_race_tolerance_x_mm)) ||
       (APP_FALSE == leg_kinematics_is_finite(cfg->experimental_race_tolerance_y_mm)) ||
       (0.0f > cfg->experimental_race_tolerance_x_mm) ||
       (0.0f > cfg->experimental_race_tolerance_y_mm))
    {
        return APP_FALSE;
    }
    if((leg_kinematics_absf(x_mm - cfg->experimental_race_x_mm) >
        cfg->experimental_race_tolerance_x_mm) ||
       (leg_kinematics_absf(y_mm - cfg->experimental_race_y_mm) >
        cfg->experimental_race_tolerance_y_mm))
    {
        return APP_FALSE;
    }
    return APP_TRUE;
}
```

After the existing finite/config precondition block in `leg_kinematics_target_valid`, add:

```c
    if(APP_TRUE == leg_kinematics_experimental_race_target_valid(cfg, x_mm, y_mm))
    {
        return APP_TRUE;
    }
```

Do not alter the existing hull loop.

In `leg_kinematics_solve_model`, retain `profile->ik_min_margin` for normal
targets.  Convert the model point back to physical coordinates and select
`cfg->experimental_race_ik_min_margin` only when the same narrow experimental
validator accepts that physical point.  Reject invalid configured margins and
keep the existing `result->singularity_margin` telemetry value unchanged.
For the same experimental point, select the configured experimental alpha and
beta branches and pass `NULL` instead of `previous` to
`leg_kinematics_select_angle`; normal targets retain their existing branch and
nearest-root continuity behaviour.

- [ ] **Step 3: Run focused tests and verify GREEN**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\test_leg_ik_zero_calibration_static.ps1
powershell -ExecutionPolicy Bypass -File tools\test_leg_physical_ik_static.ps1
```

Expected: both scripts exit `0`; the numeric harness resolves approximately `[40,140,140,40]`; `LXY,0,55` and points outside the `0.5 mm` window remain rejected.

- [ ] **Step 4: Commit the tested implementation**

```powershell
git add -- project/code/leg_config.h project/code/leg_config.c project/code/leg_kinematics.c tools/test_leg_ik_zero_calibration_static.ps1 tools/test_leg_physical_ik_static.ps1
git commit -m "Allow experimental low-race LXY target"
```

### Task 3: Document and run the full software gate

**Files:**
- Modify: `docs/leg-ik-zero-calibration-hardware-test.md:20-35`
- Modify: `tools/test_leg_coordinate_contract_static.ps1:80-100`

**Interfaces:**
- Consumes: the new experimental configuration and unchanged UART command path.
- Produces: a hardware command sequence and regression evidence suitable for IAR flashing.

- [ ] **Step 1: Add a failing documentation contract**

Add this assertion to `tools/test_leg_coordinate_contract_static.ps1` before changing the document:

```powershell
Require-Pattern $ikDoc 'experimental low-race[\s\S]*LXY,-18\.83,25\.08[\s\S]*LIK.*40.*140' `
    'Hardware procedure must distinguish the experimental low-race LXY point from measured calibration.'
```

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools\test_leg_coordinate_contract_static.ps1
```

Expected: FAIL because the hardware document does not yet contain the experimental command.

- [ ] **Step 2: Add the static hardware procedure**

Add this section to `docs/leg-ik-zero-calibration-hardware-test.md` after the coordinate-contract explanation:

````markdown
### Experimental low-race target

The hardware-tested mirrored command `LIK,40,140,140,40` places the left-leg
logical pair at `40,140` and maps by
the current model to approximately `BODY_WHEEL=(-18.831,25.076) mm`.  This is
an experimental command estimate, not a measured wheel-centre calibration
sample.  With the vehicle supported and wheel-motor power disconnected, run:

```text
STOP
LIKREF
LXY,-18.83,25.08
```

Require telemetry `servo_settled=1`, `fault_reason=0`, and target commands near
`[40,140,140,40]`.  Stop immediately for mechanical interference.  Passing
this static gate does not enable dynamic wheel shift or balance fast mode.
````

- [ ] **Step 3: Run the complete relevant host regression set**

Run each command and require exit code `0`:

```powershell
powershell -ExecutionPolicy Bypass -File tools\test_leg_coordinate_contract_static.ps1
powershell -ExecutionPolicy Bypass -File tools\test_leg_ik_zero_calibration_static.ps1
powershell -ExecutionPolicy Bypass -File tools\test_leg_physical_ik_static.ps1
powershell -ExecutionPolicy Bypass -File tools\test_leg_transition_numeric.ps1
powershell -ExecutionPolicy Bypass -File tools\test_servo_300hz_integration_static.ps1
powershell -ExecutionPolicy Bypass -File tools\test_ik_height_control_static.ps1
```

Expected: six scripts pass with no assertion or compiler failure.

- [ ] **Step 4: Build all three IAR Debug projects**

Run from the worktree root:

```powershell
& 'D:\IAR\common\bin\IarBuild.exe' project\iar\cyt4bb7.eww -build cyt4bb7_cm_7_1 -log all
& 'D:\IAR\common\bin\IarBuild.exe' project\iar\cyt4bb7.eww -build cyt4bb7_cm_7_0 -log all
& 'D:\IAR\common\bin\IarBuild.exe' project\iar\cyt4bb7.eww -build cyt4bb7_cm_0_plus -log all
```

Expected: all three builds report `0 errors`; retain and report CM7_0 warnings separately rather than calling them clean.

- [ ] **Step 5: Commit documentation and its contract**

```powershell
git add -- docs/leg-ik-zero-calibration-hardware-test.md tools/test_leg_coordinate_contract_static.ps1
git commit -m "Document experimental low-race LXY test"
```

- [ ] **Step 6: Inspect final scope**

Run:

```powershell
git status --short
git diff HEAD~2 --check
git diff HEAD~2 --stat
```

Expected: only the listed source, test, and documentation files are committed; pre-existing untracked data remains untouched.

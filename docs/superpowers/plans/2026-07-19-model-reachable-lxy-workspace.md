# Model-Reachable LXY Workspace Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make every `LXY` target use five-bar geometry, a global `0.02` singularity margin, and real left/right servo limits instead of the measured hull or low-race coordinate exception.

**Architecture:** A private candidate engine in `leg_kinematics.c` enumerates all four alpha/beta root pairs, maps them through side-specific servo calibration, rejects invalid commands, and selects one deterministically. Public validation, inverse solving, and forward-kinematics root matching share those semantics.

**Tech Stack:** Embedded C, PowerShell static/numeric harnesses, host GCC with `-std=c99 -Wall -Werror`, and IAR Embedded Workbench 9.40.1.

## Global Constraints

- `LXY` remains an absolute `BODY_WHEEL` millimetre command: `+X` forward and `+Y` down.
- Keep the fitted similarity transform and all-90-degree `LIKREF` reference unchanged.
- Reachability requires real roots, model-space `Y > 0`, margin `>= 0.02`, and an in-range solution for both sides.
- Remove the hull, 2 mm inset, all `experimental_race_*` fields, and X/Y rectangle gates from production reachability.
- Prefer previous-result continuity; without previous state prefer configured branches, then proximity to the calibrated reference.
- Keep the low-race coordinate valid without requiring beta to equal exactly `140 deg`.
- Preserve wheel/balance stop gating, S7 smoothing, 300 Hz execution, diagnostics, and fail-closed behavior.
- Do not flash, enable powered wheel motion, change balance/LQR, or touch unrelated untracked data.

## File Map

- `project/code/leg_config.h/.c`: set one `0.02` margin, then remove obsolete workspace/race data after all runtime references are gone.
- `project/code/leg_kinematics.c`: candidate engine, shared validation/solve, unbounded-by-rectangle FK matching.
- `tools/test_leg_ik_zero_calibration_static.ps1`: full-reachability and reference numeric gate.
- `tools/test_leg_transition_numeric.ps1`: grid round trips and branch continuity.
- `tools/test_leg_physical_ik_static.ps1`: source/config contract.
- `tools/test_leg_coordinate_contract_static.ps1` and `docs/leg-ik-zero-calibration-hardware-test.md`: operator contract.

---

### Task 1: Unified candidate engine and reachability

**Files:**
- Modify: `tools/test_leg_ik_zero_calibration_static.ps1`
- Modify: `project/code/leg_config.h`
- Modify: `project/code/leg_config.c`
- Modify: `project/code/leg_kinematics.c`

**Interfaces:**
- Consumes: `leg_kinematics_map_one()`, `leg_config_get_servo()`, transform/reference configuration.
- Produces: unchanged `leg_kinematics_target_valid()` and `leg_kinematics_solve()` APIs using one private candidate engine.

- [ ] **Step 1: Write a failing numeric reachability test**

Add this helper to the generated C harness in `tools/test_leg_ik_zero_calibration_static.ps1` and call it from `main()`:

```c
static int check_model_reachable_commands(void)
{
    static const float commands[][2] =
    {
        {40.0f, 140.0f}, {45.0f, 135.0f},
        {50.0f, 130.0f}, {60.0f, 120.0f}
    };
    uint32 i;
    for(i = 0U; i < (sizeof(commands) / sizeof(commands[0])); i++)
    {
        float x_mm;
        float y_mm;
        leg_ik_result_struct left;
        leg_ik_result_struct right;
        if((APP_TRUE != leg_kinematics_forward_command(APP_FALSE,
                                                       commands[i][0], commands[i][1],
                                                       &x_mm, &y_mm)) ||
           (APP_TRUE != leg_kinematics_target_valid(x_mm, y_mm)) ||
           (APP_TRUE != leg_kinematics_solve(APP_FALSE, x_mm, y_mm, NULL, &left)) ||
           (APP_TRUE != leg_kinematics_solve(APP_TRUE, x_mm, y_mm, NULL, &right)))
        {
            printf("model-reachable command rejected at index %u\n", (unsigned int)i);
            return 1;
        }
    }
    return 0;
}
```

Keep explicit rejection of `NAN`, `INFINITY`, and `(1000,1000)`.

- [ ] **Step 2: Verify RED**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\test_leg_ik_zero_calibration_static.ps1
```

Expected: nonzero exit containing `model-reachable command rejected`, proving the hull/race window still blocks at least one valid forward-derived target.

- [ ] **Step 3: Select the unified margin while preserving build compatibility**

Set the single active solver margin:

```c
.ik_min_margin = 0.02f,
```

Keep the old hull, rectangle, and `experimental_race_*` configuration fields temporarily so Task 1 remains build-compatible with forward kinematics and the old static harness. The candidate engine added below must not consult those fields. Task 3 removes them after Task 2 removes the last runtime references.

- [ ] **Step 4: Add a command-valid candidate type and mapper**

Add this private type:

```c
typedef struct
{
    float alpha_rad;
    float beta_rad;
    float alpha_command_deg;
    float beta_command_deg;
    float score;
    leg_ik_branch_enum alpha_branch;
    leg_ik_branch_enum beta_branch;
    uint8 valid;
}leg_ik_candidate_struct;
```

Implement `leg_kinematics_map_candidate(uint8 right_side, float alpha_rad, float beta_rad, float *alpha_command_deg, float *beta_command_deg)`. Select `FL/RL` or `FR/RR`, call `leg_kinematics_map_one()` with `alpha_reference_deg` and `beta_reference_deg`, and return false when either mapped command violates its servo limit.

- [ ] **Step 5: Enumerate and score all inverse roots**

Refactor `leg_kinematics_solve_model()` to reject non-finite inputs and `Y <= 0`, calculate plus/minus roots once, publish the smaller alpha/beta margin, and reject margin below `profile->ik_min_margin`. Enumerate `{PLUS,MINUS} x {PLUS,MINUS}` and discard candidates rejected by `leg_kinematics_map_candidate()`.

Use this scoring contract:

```c
if((NULL != previous) && (APP_TRUE == previous->valid))
{
    score = leg_kinematics_wrapped_distance(alpha_rad, previous->alpha_rad) +
            leg_kinematics_wrapped_distance(beta_rad, previous->beta_rad);
}
else if((preferred_alpha == alpha_branch) && (preferred_beta == beta_branch))
{
    score = 0.0f;
}
else
{
    score = 1.0f +
            leg_kinematics_wrapped_distance(alpha_rad, reference_alpha_rad) +
            leg_kinematics_wrapped_distance(beta_rad, reference_beta_rad);
}
```

Retain the first candidate on an exact score tie. Delete point-specific race logic and `leg_kinematics_select_angle()`. Leave rectangular helpers only for the existing forward-kinematics path until Task 2.

- [ ] **Step 6: Share the solver with public validation**

Make `leg_kinematics_target_valid()` transform physical to model once and call `leg_kinematics_solve_model()` for both left and right with `previous=NULL`. Make `leg_kinematics_solve()` transform once and call the private solver directly, avoiding recursive validation.

- [ ] **Step 7: Verify GREEN**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\test_leg_ik_zero_calibration_static.ps1
```

Expected: exit `0`; reference remains all-90 and the forward-derived targets solve on both sides. Do not run the old physical static contract as a Task 1 gate because it still describes the temporary hull/race implementation that Task 3 replaces.

- [ ] **Step 8: Commit**

```powershell
git add -- project/code/leg_config.c project/code/leg_kinematics.c tools/test_leg_ik_zero_calibration_static.ps1
git commit -m "Use model reachability for LXY targets"
```

---

### Task 2: Forward matching and branch continuity

**Files:**
- Modify: `tools/test_leg_transition_numeric.ps1`
- Modify: `project/code/leg_kinematics.c`

**Interfaces:**
- Consumes: Task 1 candidate engine.
- Produces: forward/inverse round trips without rectangle clamps and bounded adjacent-target motion.

- [ ] **Step 1: Write a failing grid-continuity test**

Add a host C loop over physical `Y=20..100 mm` and `X=-60..20 mm` in `1 mm` steps. For every accepted point require both sides to solve, margin `>=0.02`, FK round-trip error `<=0.5 mm`, and wrapped change from the previous adjacent accepted solution `<=12 deg` per joint. Clear `previous.valid` across rejected gaps.

The central assertion is:

```c
if((APP_TRUE == previous.valid) &&
   ((12.0f < wrapped_delta_deg(result.servo_deg[0], previous.servo_deg[0])) ||
    (12.0f < wrapped_delta_deg(result.servo_deg[1], previous.servo_deg[1]))))
{
    printf("model branch discontinuity %.1f %.1f\n", physical_x, physical_y);
    return 1;
}
```

- [ ] **Step 2: Verify RED**

Run `powershell -NoProfile -ExecutionPolicy Bypass -File tools\test_leg_transition_numeric.ps1`.

Expected: nonzero exit because FK still references deleted rectangle fields or an adjacent branch jump is detected.

- [ ] **Step 3: Remove rectangular FK filtering**

In `leg_kinematics_forward()`, remove all workspace clamps and X/Y-box checks. Initially accept each circle intersection only when X/Y are finite and `Y > 0`. Build a valid `input_pose` from the input geometric angles and pass it as `previous` when matching each intersection through `leg_kinematics_solve_model()`:

```c
input_pose.servo_deg[0] = servo_a_deg;
input_pose.servo_deg[1] = servo_b_deg;
input_pose.alpha_rad = alpha_rad;
input_pose.beta_rad = beta_rad;
input_pose.singularity_margin = 1.0f;
input_pose.valid = APP_TRUE;
```

Retain `LEG_KINEMATICS_FK_MATCH_EPS`; if both intersections match, keep the deterministic choice nearest `model_reference_x_mm`.

- [ ] **Step 4: Verify GREEN**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\test_leg_transition_numeric.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tools\test_leg_ik_zero_calibration_static.ps1
```

Expected: both exit `0`, grid round trips stay within `0.5 mm`, and adjacent motion stays within `12 deg`.

- [ ] **Step 5: Commit**

```powershell
git add -- project/code/leg_kinematics.c tools/test_leg_transition_numeric.ps1
git commit -m "Keep model workspace branches continuous"
```

---

### Task 3: Static contracts and operator documentation

**Files:**
- Modify: `tools/test_leg_physical_ik_static.ps1`
- Modify: `tools/test_leg_coordinate_contract_static.ps1`
- Modify: `tools/test_leg_ik_zero_calibration_static.ps1`
- Modify: `tools/test_leg_transition_numeric.ps1`
- Modify: `project/code/leg_config.h`
- Modify: `project/code/leg_config.c`
- Modify: `docs/leg-ik-zero-calibration-hardware-test.md`

**Interfaces:**
- Consumes: Tasks 1-2 behavior.
- Produces: regression contracts that reject reintroduction of empirical gates.

- [ ] **Step 1: Write failing static assertions**

Replace positive hull/race assertions with:

```powershell
Reject-Pattern $configHeader 'physical_workspace|experimental_race_|x_min_mm|x_max_mm|y_min_mm|y_max_mm' `
    "legacy workspace fields must be removed"
Reject-Pattern $kinematicsSource 'leg_kinematics_experimental_race_target_valid|leg_kinematics_model_workspace_valid' `
    "coordinate and rectangle gates must be removed"
Require-Pattern $configSource '\.ik_min_margin\s*=\s*0\.02f' `
    "model workspace must use margin 0.02"
Require-Pattern $kinematicsSource 'leg_kinematics_map_candidate' `
    "candidate reachability must use servo mapping"
```

Require the hardware guide to contain `model-reachable`, `0.02`, `servo limits`, and `command/model estimate`, and reject text claiming the hull remains the active boundary.

- [ ] **Step 2: Verify RED**

Run both static scripts. Expected: nonzero exit until legacy production symbols and old documentation are gone.

- [ ] **Step 3: Remove obsolete configuration now that runtime references are gone**

Make `leg_kinematics_config_struct` contain only link lengths, physical/model references, similarity-transform values, and four branch preferences. Delete `LEG_PHYSICAL_WORKSPACE_VERTEX_COUNT`, X/Y bounds, `physical_workspace`, `physical_workspace_inset_mm`, and every `experimental_race_*` member and initializer. Update the generated host config structs in both numeric PowerShell harnesses to exactly match production field order.

- [ ] **Step 4: Update hardware documentation**

State that `LXY` accepts lower-half-plane points with real roots, margin at least `0.02`, and a valid mapped command on both sides. Retain `STOP`, `LIKREF`, `LXY,-18.83,25.08`. Specify wheel power disconnected, supported chassis, one-axis manual increments no larger than `2 mm`, waiting for settled output, and immediate stop on interference, branch surprise, invalid IK, or leg fault.

- [ ] **Step 5: Verify GREEN**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\test_leg_physical_ik_static.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tools\test_leg_coordinate_contract_static.ps1
```

Expected: both exit `0` with existing pass messages.

- [ ] **Step 6: Commit**

```powershell
git add -- project/code/leg_config.h project/code/leg_config.c tools/test_leg_ik_zero_calibration_static.ps1 tools/test_leg_transition_numeric.ps1 tools/test_leg_physical_ik_static.ps1 tools/test_leg_coordinate_contract_static.ps1 docs/leg-ik-zero-calibration-hardware-test.md
git commit -m "Document model-reachable LXY workspace"
```

---

### Task 4: Full regression and three-core build

**Files:** Verify only unless a gate exposes a scoped defect.

**Interfaces:**
- Consumes: Tasks 1-3.
- Produces: software evidence only; hardware acceptance remains separate.

- [ ] **Step 1: Run all six leg tests with per-test exit checks**

```powershell
$tests=@('tools\test_leg_coordinate_contract_static.ps1','tools\test_leg_ik_zero_calibration_static.ps1','tools\test_leg_physical_ik_static.ps1','tools\test_leg_transition_numeric.ps1','tools\test_servo_300hz_integration_static.ps1','tools\test_ik_height_control_static.ps1')
foreach($test in $tests) {
    & powershell -NoProfile -ExecutionPolicy Bypass -File $test
    if(0 -ne $LASTEXITCODE) { throw ("FAILED {0}" -f $test) }
}
```

Expected: six pass messages and outer exit `0`.

- [ ] **Step 2: Build CM7_1, CM7_0, and CM0+ separately**

```powershell
$iar='D:\IAR\common\bin\IarBuild.exe'
foreach($name in @('cyt4bb7_cm_7_1','cyt4bb7_cm_7_0','cyt4bb7_cm_0_plus')) {
    $project=(Resolve-Path ("project\iar\project_config\{0}.ewp" -f $name)).Path
    & $iar $project -build Debug -log warnings
    if(0 -ne $LASTEXITCODE) { throw ("BUILD FAILED {0}" -f $name) }
}
```

Expected: three `Build succeeded` summaries and zero errors. Record, but do not conceal, existing unrelated `control_leg.c` warnings.

- [ ] **Step 3: Verify final scope and artifacts**

```powershell
git diff --check
git status --short --branch
git log -6 --oneline
Get-ChildItem project\iar -Recurse -File -Filter 'cyt4bb7_cm_*.hex' |
    Where-Object { $_.FullName -match 'Debug_' } |
    Select-Object FullName,Length,LastWriteTime
```

Expected: no intended tracked changes left uncommitted, only pre-existing untracked data artifacts, and three fresh HEX files.

- [ ] **Step 4: Report the separate hardware gate**

Report the first supported, wheel-power-disconnected sequence exactly as `STOP`, `LIKREF`, `LXY,-18.83,25.08`. Do not claim the physical workspace accepted until the user reports settled output, no fault, and no mechanical interference.

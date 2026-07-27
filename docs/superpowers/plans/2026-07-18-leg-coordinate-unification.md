# Leg Coordinate Unification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `BODY_WHEEL` the only public wheel-center position frame, preserve existing `LH/LHF` standing behavior as explicitly nonphysical legacy stance units, and report calibrated command-position estimates instead of fabricated or measured-looking positions.

**Architecture:** `leg_kinematics` owns every physical/model transform, convex-hull check, IK/FK operation, and logical-servo-command-to-physical-pose conversion. `control_leg` keeps the legacy stance planner numerically unchanged but publishes physical pose estimates derived from actuator `output_deg`; balance and chassis keep their existing scheduling values under explicit `legacy_stance_*` names. The 55-float telemetry frame keeps its bandwidth and repurposes obsolete/duplicate height fields for four physical coordinates plus a pose-status bitmask.

**Tech Stack:** Embedded C for CYT4BB7/Traveo II, IAR EWARM 9.40.1, PowerShell static/numeric regression scripts, Python 3 for transform checks, VOFA+ JustFloat telemetry.

## Global Constraints

- Work only in `D:\smartcar\WheelLeggedCar_cyt4bb7_v1\.worktree\balance-fast-mode-spec` on `codex/balance-fast-mode-spec`.
- The public physical frame is `BODY_WHEEL`: origin at the chassis-fixed cross-circle marker, millimetres, `+X` forward, `+Y` downward.
- The calibrated reference is exactly `P0 = (-20.766667f, 47.356667f) mm`; it is a point, not the coordinate origin.
- `LXY` consumes absolute `BODY_WHEEL` millimetres and `LIKREF` targets `P0`; `LXY,0,55` must be rejected.
- Model coordinates and similarity-transform helpers remain private to `project/code/leg_kinematics.c`.
- The physical workspace is the fitted eight-vertex convex hull with a `2.0f mm` inward margin, never an independent-X/Y rectangle.
- The system is PWM-only/open-loop: runtime X/Y values are command estimates, never actual or measured wheel-center positions.
- Keep the current logical `LH/LHF` servo trajectories and the numeric `30/55/80` legacy stance range unchanged.
- Keep current balance gain scheduling, chassis limits, S7 timing, 300 Hz PWM, LPF/rate limiting, and safety gates numerically unchanged.
- Apply the calibrated servo-0 pulse trim `-11.111111f us`; servo 1-3 trims remain `0.0f us`.
- Left-leg physical calibration source is measured; right-leg source is a mirror assumption until independent hardware validation.
- Do not add low-race stance targets, dynamic wheel shift, speed-to-shift mapping, LQR changes, speed-limit changes, or virtual-lean changes.
- Preserve all pre-existing untracked `.superpowers/`, `data/*.csv`, and `fast_tune_kit.zip` files; stage only paths named by each task.

---

## File Structure

- `project/code/leg_config.h/.c`: physical calibration constants and explicitly named legacy stance profile.
- `project/code/leg_kinematics.h/.c`: public physical IK/FK API; private model-space solver and transforms.
- `project/code/app_config.h`, `project/code/actuator_servo.c`: calibrated pulse trims at the hardware write boundary.
- `project/code/app_types.h`: physical command-pose type, provenance enum, pose-status flags, and legacy stance diagnostics.
- `project/code/control_leg.h/.c`: legacy planner compatibility and physical command-pose publication.
- `project/code/host_command.c`: retain `LH/LHF/LXY/LIKREF` wire syntax while calling semantically correct APIs.
- `project/code/control_balance.c`, `project/code/control_chassis.c`: name-only migration to legacy stance scheduling.
- `project/code/telemetry.c`: fixed 55-float v2 coordinate contract.
- `tools/collect_balance_data.ps1`, `tools/calib_ik_servo.ps1`: decode the v2 pose status and physical positions.
- `tools/test_leg_physical_ik_static.ps1`: physical transform, hull, and command-FK contract.
- `tools/test_leg_coordinate_contract_static.ps1`: reject mixed coordinate semantics and fake position publication.
- Existing `tools/test_*.ps1`: update exact symbols and preserve numeric behavior.
- `docs/leg-ik-zero-calibration-hardware-test.md`, `docs/leg-height-phase1-hardware-test.md`: physical-frame and open-loop hardware handoff.

---

### Task 1: Port the calibrated physical frame and command-space FK

**Files:**
- Create: `tools/test_leg_physical_ik_static.ps1`
- Modify: `project/code/leg_config.h`
- Modify: `project/code/leg_config.c`
- Modify: `project/code/leg_kinematics.h`
- Modify: `project/code/leg_kinematics.c`

**Interfaces:**
- Consumes: existing five-bar `leg_kinematics_solve`/`leg_kinematics_forward` implementation and servo configuration.
- Produces: `uint8 leg_kinematics_target_valid(float x_mm, float y_mm)`, physical `leg_kinematics_solve`, physical `leg_kinematics_forward`, and `uint8 leg_kinematics_forward_command(uint8 right_side, float servo_a_command_deg, float servo_b_command_deg, float *x_mm, float *y_mm)`.

- [ ] **Step 1: Add the failing physical-coordinate static and numeric test**

Create `tools/test_leg_physical_ik_static.ps1` as a coordinate-only test. Use these exact file bindings and contract checks (the embedded Python block below supplies the transform/hull numeric check; no camera or perception dependency is allowed):

```powershell
Require-Pattern $kinematicsHeader `
    'leg_kinematics_forward_command\(uint8 right_side,' `
    'logical-servo-command FK API missing'
Require-Pattern $kinematicsSource `
    'cfg->alpha_reference_deg' `
    'command FK must reconstruct alpha from the calibrated reference'
Require-Pattern $kinematicsSource `
    'cfg->beta_reference_deg' `
    'command FK must reconstruct beta from the calibrated reference'
Require-Pattern $kinematicsSource `
    'leg_config_get_servo\(servo_a_index\)' `
    'command FK must use the per-servo logical direction'
Require-Pattern $configHeader `
    'LEG_PHYSICAL_WORKSPACE_VERTEX_COUNT\s*=\s*8' `
    'physical workspace must contain the fitted eight-vertex hull'
Require-Pattern $configHeader `
    'physical_workspace\[LEG_PHYSICAL_WORKSPACE_VERTEX_COUNT\]\[2\]' `
    'kinematics config must store the physical hull'
Require-Pattern $configSource `
    '\.physical_reference_x_mm\s*=\s*-20\.766667f' `
    'physical reference X must be the three-reference mean'
Require-Pattern $configSource `
    '\.physical_reference_y_mm\s*=\s*47\.356667f' `
    'physical reference Y must be the three-reference mean'
Require-Pattern $configSource `
    '\.model_to_physical_scale\s*=\s*0\.955219899f' `
    'fitted uniform scale missing'
Require-Pattern $configSource `
    '\.physical_workspace_inset_mm\s*=\s*2\.0f' `
    'physical hull must be inset by 2 mm'
Require-Pattern $kinematicsHeader `
    'leg_kinematics_target_valid\(float x_mm,\s*float y_mm\)' `
    'public physical target validation API missing'
Require-Pattern $kinematicsSource `
    'leg_kinematics_physical_to_model' `
    'physical-to-model inverse similarity missing'
Require-Pattern $kinematicsSource `
    'leg_kinematics_model_to_physical' `
    'model-to-physical similarity missing'
Require-Pattern $kinematicsSource `
    'leg_kinematics_solve_model' `
    'five-bar model solver must be private'
Require-Pattern $kinematicsSource `
    'leg_kinematics_target_valid\(x_mm, y_mm\)' `
    'public IK solve must reject targets outside the physical hull'
Reject-Pattern $kinematicsSource `
    'x\s*=\s*x_mm\s*\+\s*cfg->x_offset_mm' `
    'obsolete additive X offset must not survive'
Reject-Pattern $kinematicsSource `
    'y\s*=\s*y_mm\s*\+\s*cfg->y_offset_mm' `
    'obsolete additive Y offset must not survive'
```

The script begins with the following helpers and file variables:

```powershell
$ErrorActionPreference = "Stop"

function Require-Pattern([string]$Path, [string]$Pattern, [string]$Message) {
    $text = Get-Content $Path -Raw
    if($text -notmatch $Pattern) { throw $Message }
}
function Reject-Pattern([string]$Path, [string]$Pattern, [string]$Message) {
    $text = Get-Content $Path -Raw
    if($text -match $Pattern) { throw $Message }
}

$configHeader = "project/code/leg_config.h"
$configSource = "project/code/leg_config.c"
$kinematicsHeader = "project/code/leg_kinematics.h"
$kinematicsSource = "project/code/leg_kinematics.c"
```

After the static assertions, embed and execute this complete numeric check:

```python
import math

pref = (-20.766667, 47.356667)
lref = (22.830129, 46.929213)
scale = 0.955219899
m = (-0.996313812, 0.085783378, 0.085783378, 0.996313812)
hull = [
    (-40.620, 47.370), (-30.910, 39.630),
    (-20.380, 32.170), (-15.040, 47.600),
    (-22.030, 88.490), (-31.420, 74.120),
    (-37.940, 59.340), (-39.580, 53.010),
]

def model_to_physical(point):
    dx, dy = point[0] - lref[0], point[1] - lref[1]
    return (pref[0] + scale * (m[0] * dx + m[1] * dy),
            pref[1] + scale * (m[2] * dx + m[3] * dy))

def physical_to_model(point):
    dx, dy = (point[0] - pref[0]) / scale, (point[1] - pref[1]) / scale
    return (lref[0] + m[0] * dx + m[2] * dy,
            lref[1] + m[1] * dx + m[3] * dy)

def valid(point, inset=2.0):
    for index, first in enumerate(hull):
        second = hull[(index + 1) % len(hull)]
        ex, ey = second[0] - first[0], second[1] - first[1]
        distance = (ex * (point[1] - first[1]) - ey * (point[0] - first[0])) / math.hypot(ex, ey)
        if distance < inset:
            return False
    return True

def command_to_geometric(command, neutral, offset, direction, reference):
    return reference + ((command - neutral - offset) / direction)

assert max(abs(a-b) for a,b in zip(model_to_physical(lref), pref)) < 1e-6
assert abs(command_to_geometric(90.0, 90.0, 0.0, -1.0, 170.536799) - 170.536799) < 1e-9
assert abs(command_to_geometric(90.0, 90.0, 0.0, -1.0, -4.081158) - (-4.081158)) < 1e-9
probe = (20.0, 70.0)
assert max(abs(a-b) for a,b in zip(physical_to_model(model_to_physical(probe)), probe)) < 1e-5
assert valid(pref)
assert not valid((0.0, 55.0))
```

Wrap it with a PowerShell here-string piped to `python -`, throw on nonzero `$LASTEXITCODE`, and finish with:

```powershell
Write-Host "physical-coordinate leg IK static test passed"
```

- [ ] **Step 2: Run the test and verify the old coordinate contract fails**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_leg_physical_ik_static.ps1
```

Expected: FAIL on missing `LEG_PHYSICAL_WORKSPACE_VERTEX_COUNT`, `physical_reference_x_mm`, or `leg_kinematics_forward_command`.

- [ ] **Step 3: Replace the temporary kinematics configuration with the calibrated physical contract**

In `project/code/leg_config.h`, add the hull constant and replace additive offsets/cross-band fields with the following exact fields:

```c
typedef enum
{
    LEG_PHYSICAL_WORKSPACE_VERTEX_COUNT = 8
}leg_physical_workspace_constant_enum;

typedef struct
{
    float l1_mm;
    float l2_mm;
    float l3_mm;
    float l4_mm;
    float l5_mm;
    float x_min_mm;
    float x_max_mm;
    float y_min_mm;
    float y_max_mm;
    float physical_reference_x_mm;
    float physical_reference_y_mm;
    float alpha_reference_deg;
    float beta_reference_deg;
    float model_reference_x_mm;
    float model_reference_y_mm;
    float model_to_physical_scale;
    float model_to_physical_m00;
    float model_to_physical_m01;
    float model_to_physical_m10;
    float model_to_physical_m11;
    float physical_workspace[LEG_PHYSICAL_WORKSPACE_VERTEX_COUNT][2];
    float physical_workspace_inset_mm;
    leg_ik_branch_enum left_alpha_branch;
    leg_ik_branch_enum left_beta_branch;
    leg_ik_branch_enum right_alpha_branch;
    leg_ik_branch_enum right_beta_branch;
}leg_kinematics_config_struct;
```

In `project/code/leg_config.c`, use the calibrated servo directions and exact physical block:

```c
        {0, 90.0f, 90.0f, 10.0f, 175.0f, -1.0f, 0.0f,  1.0f,  1.0f},
        {1, 90.0f, 90.0f, 10.0f, 175.0f,  1.0f, 0.0f,  1.0f, -1.0f},
        {2, 90.0f, 90.0f, 10.0f, 175.0f, -1.0f, 0.0f, -1.0f,  1.0f},
        {3, 90.0f, 90.0f, 10.0f, 175.0f,  1.0f, 0.0f, -1.0f, -1.0f}
```

```c
        .x_min_mm = 10.0f,
        .x_max_mm = 50.0f,
        .y_min_mm = 25.0f,
        .y_max_mm = 100.0f,
        .physical_reference_x_mm = -20.766667f,
        .physical_reference_y_mm = 47.356667f,
        .alpha_reference_deg = 170.536799f,
        .beta_reference_deg = -4.081158f,
        .model_reference_x_mm = 22.830129f,
        .model_reference_y_mm = 46.929213f,
        .model_to_physical_scale = 0.955219899f,
        .model_to_physical_m00 = -0.996313812f,
        .model_to_physical_m01 = 0.085783378f,
        .model_to_physical_m10 = 0.085783378f,
        .model_to_physical_m11 = 0.996313812f,
        .physical_workspace =
        {
            {-40.620f, 47.370f}, {-30.910f, 39.630f},
            {-20.380f, 32.170f}, {-15.040f, 47.600f},
            {-22.030f, 88.490f}, {-31.420f, 74.120f},
            {-37.940f, 59.340f}, {-39.580f, 53.010f}
        },
        .physical_workspace_inset_mm = 2.0f,
```

- [ ] **Step 4: Make the model solver private and wrap it with the physical transform**

In `project/code/leg_kinematics.c`, rename the current solve body to `leg_kinematics_solve_model`, rename rectangular helpers to `leg_kinematics_model_workspace_valid*`, and remove `x_offset_mm/y_offset_mm`. Add the following exact hull and similarity functions before the private model solver:

```c
uint8 leg_kinematics_target_valid(float x_mm, float y_mm)
{
    const leg_kinematics_config_struct *cfg;
    uint8 i;

    cfg = leg_config_get_kinematics();
    if((NULL == cfg) ||
       (APP_FALSE == leg_kinematics_is_finite(x_mm)) ||
       (APP_FALSE == leg_kinematics_is_finite(y_mm)) ||
       (APP_FALSE == leg_kinematics_is_finite(cfg->physical_workspace_inset_mm)) ||
       (0.0f > cfg->physical_workspace_inset_mm))
    {
        return APP_FALSE;
    }
    for(i = 0U; i < LEG_PHYSICAL_WORKSPACE_VERTEX_COUNT; i++)
    {
        uint8 next;
        float first_x;
        float first_y;
        float edge_x;
        float edge_y;
        float edge_length;
        float edge_cross;

        next = (uint8)((i + 1U) % LEG_PHYSICAL_WORKSPACE_VERTEX_COUNT);
        first_x = cfg->physical_workspace[i][0];
        first_y = cfg->physical_workspace[i][1];
        edge_x = cfg->physical_workspace[next][0] - first_x;
        edge_y = cfg->physical_workspace[next][1] - first_y;
        edge_length = sqrtf((edge_x * edge_x) + (edge_y * edge_y));
        if((APP_FALSE == leg_kinematics_is_finite(edge_length)) ||
           (LEG_KINEMATICS_EPS > edge_length))
        {
            return APP_FALSE;
        }
        edge_cross = (edge_x * (y_mm - first_y)) -
                     (edge_y * (x_mm - first_x));
        if(edge_cross < (cfg->physical_workspace_inset_mm * edge_length))
        {
            return APP_FALSE;
        }
    }
    return APP_TRUE;
}

static uint8 leg_kinematics_physical_to_model(float physical_x_mm,
                                               float physical_y_mm,
                                               float *model_x_mm,
                                               float *model_y_mm)
{
    const leg_kinematics_config_struct *cfg;
    float delta_x;
    float delta_y;

    if((NULL == model_x_mm) || (NULL == model_y_mm))
    {
        return APP_FALSE;
    }
    cfg = leg_config_get_kinematics();
    if((NULL == cfg) ||
       (APP_FALSE == leg_kinematics_is_finite(physical_x_mm)) ||
       (APP_FALSE == leg_kinematics_is_finite(physical_y_mm)) ||
       (APP_FALSE == leg_kinematics_is_finite(cfg->model_to_physical_scale)) ||
       (LEG_KINEMATICS_EPS > cfg->model_to_physical_scale))
    {
        return APP_FALSE;
    }
    delta_x = (physical_x_mm - cfg->physical_reference_x_mm) /
              cfg->model_to_physical_scale;
    delta_y = (physical_y_mm - cfg->physical_reference_y_mm) /
              cfg->model_to_physical_scale;
    *model_x_mm = cfg->model_reference_x_mm +
                  (cfg->model_to_physical_m00 * delta_x) +
                  (cfg->model_to_physical_m10 * delta_y);
    *model_y_mm = cfg->model_reference_y_mm +
                  (cfg->model_to_physical_m01 * delta_x) +
                  (cfg->model_to_physical_m11 * delta_y);
    return ((APP_TRUE == leg_kinematics_is_finite(*model_x_mm)) &&
            (APP_TRUE == leg_kinematics_is_finite(*model_y_mm))) ? APP_TRUE : APP_FALSE;
}

static uint8 leg_kinematics_model_to_physical(float model_x_mm,
                                               float model_y_mm,
                                               float *physical_x_mm,
                                               float *physical_y_mm)
{
    const leg_kinematics_config_struct *cfg;
    float delta_x;
    float delta_y;

    if((NULL == physical_x_mm) || (NULL == physical_y_mm))
    {
        return APP_FALSE;
    }
    cfg = leg_config_get_kinematics();
    if((NULL == cfg) ||
       (APP_FALSE == leg_kinematics_is_finite(model_x_mm)) ||
       (APP_FALSE == leg_kinematics_is_finite(model_y_mm)) ||
       (APP_FALSE == leg_kinematics_is_finite(cfg->model_to_physical_scale)) ||
       (LEG_KINEMATICS_EPS > cfg->model_to_physical_scale))
    {
        return APP_FALSE;
    }
    delta_x = model_x_mm - cfg->model_reference_x_mm;
    delta_y = model_y_mm - cfg->model_reference_y_mm;
    *physical_x_mm = cfg->physical_reference_x_mm +
                     cfg->model_to_physical_scale *
                     ((cfg->model_to_physical_m00 * delta_x) +
                      (cfg->model_to_physical_m01 * delta_y));
    *physical_y_mm = cfg->physical_reference_y_mm +
                     cfg->model_to_physical_scale *
                     ((cfg->model_to_physical_m10 * delta_x) +
                      (cfg->model_to_physical_m11 * delta_y));
    return ((APP_TRUE == leg_kinematics_is_finite(*physical_x_mm)) &&
            (APP_TRUE == leg_kinematics_is_finite(*physical_y_mm))) ? APP_TRUE : APP_FALSE;
}
```

The public wrapper must be exactly:

```c
uint8 leg_kinematics_solve(uint8 right_side,
                           float x_mm,
                           float y_mm,
                           const leg_ik_result_struct *previous,
                           leg_ik_result_struct *result)
{
    float model_x_mm;
    float model_y_mm;

    if(NULL == result)
    {
        return APP_FALSE;
    }
    result->valid = APP_FALSE;
    if((APP_FALSE == leg_kinematics_target_valid(x_mm, y_mm)) ||
       (APP_FALSE == leg_kinematics_physical_to_model(x_mm, y_mm,
                                                       &model_x_mm,
                                                       &model_y_mm)))
    {
        return APP_FALSE;
    }
    return leg_kinematics_solve_model(right_side,
                                       model_x_mm,
                                       model_y_mm,
                                       previous,
                                       result);
}
```

`leg_kinematics_target_valid` must iterate all eight counter-clockwise edges and require:

```c
edge_cross = (edge_x * (y_mm - first_y)) -
             (edge_y * (x_mm - first_x));
if(edge_cross < (cfg->physical_workspace_inset_mm * edge_length))
{
    return APP_FALSE;
}
```

Make `leg_kinematics_forward` return `BODY_WHEEL` through `leg_kinematics_model_to_physical`; do not expose model X/Y.

- [ ] **Step 5: Add logical command to physical pose conversion**

Declare in `project/code/leg_kinematics.h`:

```c
uint8 leg_kinematics_forward_command(uint8 right_side,
                                     float servo_a_command_deg,
                                     float servo_b_command_deg,
                                     float *x_mm,
                                     float *y_mm);
```

Implement it without modifying outputs on failure:

```c
uint8 leg_kinematics_forward_command(uint8 right_side,
                                     float servo_a_command_deg,
                                     float servo_b_command_deg,
                                     float *x_mm,
                                     float *y_mm)
{
    const leg_kinematics_config_struct *cfg;
    const leg_servo_config_struct *servo_a;
    const leg_servo_config_struct *servo_b;
    uint8 servo_a_index;
    uint8 servo_b_index;
    float alpha_deg;
    float beta_deg;
    float physical_x_mm;
    float physical_y_mm;

    if((NULL == x_mm) || (NULL == y_mm))
    {
        return APP_FALSE;
    }
    cfg = leg_config_get_kinematics();
    servo_a_index = (APP_TRUE == right_side) ? LEG_SERVO_FR : LEG_SERVO_FL;
    servo_b_index = (APP_TRUE == right_side) ? LEG_SERVO_RR : LEG_SERVO_RL;
    servo_a = leg_config_get_servo(servo_a_index);
    servo_b = leg_config_get_servo(servo_b_index);
    if((NULL == cfg) || (NULL == servo_a) || (NULL == servo_b) ||
       (0.0f == servo_a->direction) || (0.0f == servo_b->direction) ||
       (APP_FALSE == leg_kinematics_is_finite(servo_a_command_deg)) ||
       (APP_FALSE == leg_kinematics_is_finite(servo_b_command_deg)))
    {
        return APP_FALSE;
    }

    alpha_deg = cfg->alpha_reference_deg +
                ((servo_a_command_deg - servo_a->neutral_deg - servo_a->ik_offset_deg) /
                 servo_a->direction);
    beta_deg = cfg->beta_reference_deg +
               ((servo_b_command_deg - servo_b->neutral_deg - servo_b->ik_offset_deg) /
                servo_b->direction);
    if(APP_TRUE != leg_kinematics_forward(right_side,
                                           alpha_deg,
                                           beta_deg,
                                           &physical_x_mm,
                                           &physical_y_mm))
    {
        return APP_FALSE;
    }
    *x_mm = physical_x_mm;
    *y_mm = physical_y_mm;
    return APP_TRUE;
}
```

- [ ] **Step 6: Run the physical-coordinate test**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_leg_physical_ik_static.ps1
```

Expected: `physical-coordinate leg IK static test passed`.

- [ ] **Step 7: Commit the physical kinematics slice**

```powershell
git add -- project/code/leg_config.h project/code/leg_config.c project/code/leg_kinematics.h project/code/leg_kinematics.c tools/test_leg_physical_ik_static.ps1
git diff --cached --check
git commit -m "Unify leg kinematics on physical coordinates"
```

Expected: one commit containing only the five listed files.

---

### Task 2: Apply the physical calibration pulse trim at the PWM boundary

**Files:**
- Modify: `project/code/app_config.h`
- Modify: `project/code/actuator_servo.c`
- Modify: `tools/test_servo_pwm_resolution_static.ps1`

**Interfaces:**
- Consumes: logical actuator `output_deg` and the existing untrimmed public helper `actuator_servo_angle_to_duty`.
- Produces: per-channel trim applied only by hardware writes; public untrimmed conversion remains stable for generic tests and callers.

- [ ] **Step 1: Extend the failing PWM test with exact trim behavior**

Add these assertions to `tools/test_servo_pwm_resolution_static.ps1`:

```powershell
Assert-Contains "project/code/app_config.h" `
    '#define APP_SERVO0_PULSE_TRIM_US\s+\(-11\.111111f\)' `
    'servo0 physical calibration trim missing'
Assert-Contains "project/code/actuator_servo.c" `
    'actuator_servo_angle_to_duty_with_trim' `
    'hardware write path must apply channel trim'
Assert-Contains "project/code/actuator_servo.c" `
    'actuator_servo_pulse_trim_us\[index\]' `
    'hardware write must select the active channel trim'
```

Update its numeric check so 90 degrees stays `9000` for the public untrimmed helper and servo 0 hardware output is `8933` at `PWM_DUTY_MAX = 20000`, 300 Hz.

- [ ] **Step 2: Run the PWM test and verify it fails on missing trim**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_servo_pwm_resolution_static.ps1
```

Expected: FAIL with `servo0 physical calibration trim missing`.

- [ ] **Step 3: Add exact trim constants and trimmed hardware conversion**

In `project/code/app_config.h`:

```c
#define APP_SERVO0_PULSE_TRIM_US        (-11.111111f)
#define APP_SERVO1_PULSE_TRIM_US        (0.0f)
#define APP_SERVO2_PULSE_TRIM_US        (0.0f)
#define APP_SERVO3_PULSE_TRIM_US        (0.0f)
```

In `project/code/actuator_servo.c`, add the four-element constant array from `c07702a`. Route only `actuator_servo_write` through:

```c
static uint32 actuator_servo_angle_to_duty_with_trim(float angle_deg,
                                                      float pulse_trim_us)
{
    float limited_angle;
    float pulse_us;

    limited_angle = actuator_servo_limit(angle_deg);
    pulse_us = (float)APP_SERVO_MIN_PULSE_US +
               (limited_angle - APP_SERVO_MIN_DEG) *
               (float)(APP_SERVO_MAX_PULSE_US - APP_SERVO_MIN_PULSE_US) /
               (APP_SERVO_MAX_DEG - APP_SERVO_MIN_DEG);
    pulse_us += pulse_trim_us;
    if((float)APP_SERVO_MIN_PULSE_US > pulse_us)
    {
        pulse_us = (float)APP_SERVO_MIN_PULSE_US;
    }
    else if((float)APP_SERVO_MAX_PULSE_US < pulse_us)
    {
        pulse_us = (float)APP_SERVO_MAX_PULSE_US;
    }
    return (uint32)(pulse_us * (float)PWM_DUTY_MAX *
                    (float)APP_SERVO_PWM_FREQ_HZ / 1000000.0f);
}
```

Keep:

```c
uint32 actuator_servo_angle_to_duty(float angle_deg)
{
    return actuator_servo_angle_to_duty_with_trim(angle_deg, 0.0f);
}
```

- [ ] **Step 4: Run PWM and 300 Hz integration tests**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_servo_pwm_resolution_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_servo_300hz_integration_static.ps1
```

Expected: both scripts print their `passed` message; 300 Hz and S7/LPF contracts remain unchanged.

- [ ] **Step 5: Commit the pulse calibration slice**

```powershell
git add -- project/code/app_config.h project/code/actuator_servo.c tools/test_servo_pwm_resolution_static.ps1
git diff --cached --check
git commit -m "Apply calibrated servo pulse trim"
```

---

### Task 3: Demote the old height domain to explicit legacy stance units

**Files:**
- Create: `tools/test_leg_coordinate_contract_static.ps1`
- Modify: `project/code/leg_config.h`
- Modify: `project/code/leg_config.c`
- Modify: `project/code/control_leg.h`
- Modify: `project/code/control_leg.c`
- Modify: `project/code/host_command.c`
- Modify: `project/code/app_config.h`
- Modify: `project/code/app_types.h`
- Modify: `project/code/control_balance.c`
- Modify: `project/code/control_chassis.c`
- Modify: `tools/test_leg_transition_numeric.ps1`
- Modify: `tools/test_ik_height_control_static.ps1`
- Modify: `tools/test_leg_first_height_frame_static.ps1`

**Interfaces:**
- Consumes: existing numeric legacy planner and its `30/55/80`, rate, acceleration, jerk, settle, gain, and drive-limit values.
- Produces: `leg_stance_profile_struct`, `leg_config_get_stance_profile`, `control_leg_set_legacy_stance`, `control_leg_set_fast_legacy_stance`, and `legacy_stance_*` diagnostics without any numeric control change.

- [ ] **Step 1: Add a failing mixed-domain rejection test**

Create `tools/test_leg_coordinate_contract_static.ps1` with:

```powershell
$ErrorActionPreference = "Stop"

function Require-Pattern([string]$Path, [string]$Pattern, [string]$Message) {
    $text = Get-Content $Path -Raw
    if($text -notmatch $Pattern) { throw $Message }
}
function Reject-Pattern([string]$Path, [string]$Pattern, [string]$Message) {
    $text = Get-Content $Path -Raw
    if($text -match $Pattern) { throw $Message }
}

$types = "project/code/app_types.h"
$config = "project/code/leg_config.h"
$leg = "project/code/control_leg.c"
$balance = "project/code/control_balance.c"
$chassis = "project/code/control_chassis.c"

Require-Pattern $config 'leg_stance_profile_struct' 'legacy stance profile type missing'
Require-Pattern $config 'legacy_low_units' 'legacy stance bounds must not claim millimetres'
Require-Pattern $leg 'control_leg_apply_legacy_stance' 'legacy planner name missing'
Require-Pattern $balance 'legacy_stance_norm' 'balance schedule must identify legacy input'
Require-Pattern $chassis 'legacy_stance_norm' 'chassis schedule must identify legacy input'
Reject-Pattern $types 'actual_height_mm' 'open-loop diagnostics must not claim actual height'
Reject-Pattern $types 'target_height_mm|height_ref_mm|height_norm' 'legacy scalar diagnostics must not claim physical height'
Reject-Pattern $leg 'Y_real\s*~=' 'legacy empirical scalar must not be documented as physical Y'

Write-Host "leg coordinate naming contract passed"
```

- [ ] **Step 2: Run the naming test and verify it fails**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_leg_coordinate_contract_static.ps1
```

Expected: FAIL on `legacy stance profile type missing`.

- [ ] **Step 3: Apply the exact semantic rename table without changing values**

Apply this mapping consistently in configuration, controller, balance, chassis, host command, and tests:

| Old symbol | New symbol |
|---|---|
| `leg_height_profile_struct` | `leg_stance_profile_struct` |
| `height_profile` | `stance_profile` |
| `leg_config_get_height_profile` | `leg_config_get_stance_profile` |
| `low_height_mm` | `legacy_low_units` |
| `high_height_mm` | `legacy_high_units` |
| `default_height_mm` | `legacy_default_units` |
| `max_height_speed_mm_s` | `legacy_max_rate_units_s` |
| `max_height_accel_mm_s2` | `legacy_max_accel_units_s2` |
| `max_height_jerk_mm_s3` | `legacy_max_jerk_units_s3` |
| `height_position_kp_s` | `legacy_position_kp_s` |
| `height_rate_kp_s` | `legacy_rate_kp_s` |
| `height_settle_error_mm` | `legacy_settle_error_units` |
| `height_settle_ms` | `legacy_settle_ms` |
| `fast_height_transition_ms` | `fast_stance_transition_ms` |
| `safe_support_height_mm` | `legacy_safe_support_units` |
| `height_min` | `legacy_body_min_units` |
| `height_max` | `legacy_body_max_units` |
| `LEG_MODE_HEIGHT` | `LEG_MODE_LEGACY_STANCE` |
| `LEG_MODE_FAST_HEIGHT` | `LEG_MODE_FAST_LEGACY_STANCE` |
| `LEG_MODE_DIRECT_STEP` | `LEG_MODE_DIRECT_LEGACY_STANCE` |
| `control_leg_set_height` | `control_leg_set_legacy_stance` |
| `control_leg_set_fast_height` | `control_leg_set_fast_legacy_stance` |
| `control_leg_set_direct_step_height` | `control_leg_set_direct_legacy_stance` |
| `target_height_mm` | `legacy_stance_target_units` |
| `height_ref_mm` | `legacy_stance_ref_units` |
| `height_rate_mm_s` | `legacy_stance_rate_units_s` |
| `height_norm` | `legacy_stance_norm` |
| `leg_height_norm` | `legacy_stance_norm` |

Rename internal planner variables and helpers by the same rule, including `control_leg_apply_empirical_height` to `control_leg_apply_legacy_stance`, `CONTROL_LEG_EMPIRICAL_CENTER_HEIGHT_MM` to `CONTROL_LEG_LEGACY_CENTER_UNITS`, and `CONTROL_LEG_EMPIRICAL_MM_PER_DELTA_DEG` to `CONTROL_LEG_LEGACY_UNITS_PER_DELTA_DEG`.

Also rename `control_leg_set_body_cmd(float height_cmd, ...)`'s parameter to `legacy_stance_cmd_units`, `control_leg_fast_height_blend` to `control_leg_fast_stance_blend`, and every internal `*_height_*` planner variable to its `*_legacy_stance_*` equivalent. Functionality and numeric constants do not change.

The helper formula must remain numerically identical:

```c
delta_deg = (stance_units - CONTROL_LEG_LEGACY_CENTER_UNITS) /
            CONTROL_LEG_LEGACY_UNITS_PER_DELTA_DEG;
```

Keep the host protocol unchanged:

```c
if(APP_TRUE == control_leg_set_fast_legacy_stance(value, now_ms)) /* LHF */
if(APP_TRUE == control_leg_set_legacy_stance(value, now_ms))      /* LH */
```

Rename `APP_LEG_VERIFY_HEIGHT_CMD` to `APP_LEG_VERIFY_STANCE_CMD_UNITS`; its value remains `0.0f`.

- [ ] **Step 4: Remove the false physical-height diagnostic**

Delete `actual_height_mm` from `leg_diag_struct`. At this task, the relevant legacy portion must be:

```c
typedef struct
{
    float legacy_stance_target_units;
    float legacy_stance_ref_units;
    float legacy_stance_rate_units_s;
    float legacy_stance_norm;
    float ik_margin;
    /* physical command-pose members are added in Task 4 */
```

Do not introduce a replacement named `actual`, `measured`, or `_mm` for the legacy scalar.

- [ ] **Step 5: Update numeric tests to prove behavior did not change**

Update the configuration parser and C harness type in `tools/test_leg_transition_numeric.ps1` to the new exact names. Keep these assertions and numeric values:

```powershell
Assert-Equal $config["legacy_low_units"] 30.0 "Extended legacy low stance"
Assert-Equal $config["legacy_high_units"] 80.0 "Extended legacy high stance"
Assert-Equal $config["legacy_default_units"] 55.0 "Legacy default stance"
Assert-Equal $config["legacy_max_rate_units_s"] 20.0 "Legacy maximum stance rate"
Assert-Equal $config["legacy_max_accel_units_s2"] 20.0 "Legacy maximum stance acceleration"
Assert-Equal $config["legacy_max_jerk_units_s3"] 80.0 "Legacy maximum stance jerk"
Assert-Equal $config["legacy_settle_error_units"] 1.0 "Legacy settle error"
Assert-Equal $config["fast_stance_transition_ms"] 500.0 "Fast stance transition duration"
Assert-Equal $config["legacy_safe_support_units"] 55.0 "Legacy safe support stance"
```

Keep the same 45-to-65, 55-to-30, and 55-to-80 S7 trajectory inputs; rename test parameters from `*Mm` to `*Units` only.

Update static expectations in `tools/test_ik_height_control_static.ps1` and `tools/test_leg_first_height_frame_static.ps1` to the new symbols while retaining their fail-closed, first-frame, nonblocking, and scheduling assertions.

- [ ] **Step 6: Run naming and numeric compatibility tests**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_leg_coordinate_contract_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_leg_transition_numeric.ps1
powershell -ExecutionPolicy Bypass -File tools/test_ik_height_control_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_leg_first_height_frame_static.ps1
```

Expected: all four scripts print their `passed` message. The numeric test must still report the same 30/55/80 bounds and 500 ms S7 duration.

- [ ] **Step 7: Commit the legacy-domain rename**

```powershell
git add -- project/code/leg_config.h project/code/leg_config.c project/code/control_leg.h project/code/control_leg.c project/code/host_command.c project/code/app_config.h project/code/app_types.h project/code/control_balance.c project/code/control_chassis.c tools/test_leg_coordinate_contract_static.ps1 tools/test_leg_transition_numeric.ps1 tools/test_ik_height_control_static.ps1 tools/test_leg_first_height_frame_static.ps1
git diff --cached --check
git commit -m "Clarify legacy leg stance units"
```

---

### Task 4: Publish physical command-pose estimates from actuator output

**Files:**
- Modify: `project/code/app_types.h`
- Modify: `project/code/control_leg.c`
- Modify: `project/code/control_leg.h`
- Modify: `tools/test_leg_coordinate_contract_static.ps1`
- Modify: `tools/test_leg_ik_zero_calibration_static.ps1`

**Interfaces:**
- Consumes: `leg_kinematics_forward_command`, actuator `output_deg[4]`, calibrated reference, and physical hull validator.
- Produces: `leg_pose_command_estimate_struct` for each leg, provenance, validity, physical `LIKREF`, and absolute physical `LXY`.

- [ ] **Step 1: Extend the contract test with failing physical-pose assertions**

Add:

```powershell
Require-Pattern $types 'leg_pose_command_estimate_struct' 'physical command-pose type missing'
Require-Pattern $types 'LEG_POSE_SOURCE_MEASURED_CALIBRATION' 'measured calibration provenance missing'
Require-Pattern $types 'LEG_POSE_SOURCE_MIRROR_ASSUMPTION' 'right-leg mirror provenance missing'
Require-Pattern $leg 'control_leg_update_command_pose_estimates' 'actuator-output FK publication missing'
Require-Pattern $leg 'control_leg_actuator_diag\.output_deg\[LEG_SERVO_FL\]' 'left pose must use actuator output command'
Require-Pattern $leg 'control_leg_actuator_diag\.output_deg\[LEG_SERVO_FR\]' 'right pose must use actuator output command'
Require-Pattern $leg 'kinematics->physical_reference_x_mm' 'LIKREF must use physical X'
Require-Pattern $leg 'leg_kinematics_target_valid\(x_mm, y_mm\)' 'LXY must use the physical hull'
Reject-Pattern $leg 'left_x_mm\s*=\s*0\.0f' 'physical X must not be fabricated as zero'
Reject-Pattern $leg 'left_y_mm\s*=\s*control_leg_' 'legacy stance must not be copied into physical Y'
```

- [ ] **Step 2: Run the contract test and verify pose publication is missing**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_leg_coordinate_contract_static.ps1
```

Expected: FAIL with `physical command-pose type missing`.

- [ ] **Step 3: Define explicit command-estimate and provenance types**

In `project/code/app_types.h`:

```c
typedef enum
{
    LEG_POSE_SOURCE_NONE = 0,
    LEG_POSE_SOURCE_MEASURED_CALIBRATION = 1,
    LEG_POSE_SOURCE_MIRROR_ASSUMPTION = 2
}leg_pose_source_enum;

typedef struct
{
    float x_mm;
    float y_mm;
    leg_pose_source_enum source;
    uint8 valid;
}leg_pose_command_estimate_struct;
```

Add to `leg_diag_struct` immediately after `ik_margin`:

```c
    leg_pose_command_estimate_struct left_command_pose_body_mm;
    leg_pose_command_estimate_struct right_command_pose_body_mm;
```

Delete the old flat `left_x_mm`, `left_y_mm`, `right_x_mm`, and `right_y_mm` fields.

- [ ] **Step 4: Derive pose estimates from executed logical servo commands**

Add to `project/code/control_leg.c`:

```c
static void control_leg_update_command_pose_estimates(void)
{
    float x_mm;
    float y_mm;

    control_leg_diag.left_command_pose_body_mm.source =
        LEG_POSE_SOURCE_MEASURED_CALIBRATION;
    control_leg_diag.left_command_pose_body_mm.valid = APP_FALSE;
    if(APP_TRUE == leg_kinematics_forward_command(
                       APP_FALSE,
                       control_leg_actuator_diag.output_deg[LEG_SERVO_FL],
                       control_leg_actuator_diag.output_deg[LEG_SERVO_RL],
                       &x_mm,
                       &y_mm))
    {
        control_leg_diag.left_command_pose_body_mm.x_mm = x_mm;
        control_leg_diag.left_command_pose_body_mm.y_mm = y_mm;
        control_leg_diag.left_command_pose_body_mm.valid = APP_TRUE;
    }

    control_leg_diag.right_command_pose_body_mm.source =
        LEG_POSE_SOURCE_MIRROR_ASSUMPTION;
    control_leg_diag.right_command_pose_body_mm.valid = APP_FALSE;
    if(APP_TRUE == leg_kinematics_forward_command(
                       APP_TRUE,
                       control_leg_actuator_diag.output_deg[LEG_SERVO_FR],
                       control_leg_actuator_diag.output_deg[LEG_SERVO_RR],
                       &x_mm,
                       &y_mm))
    {
        control_leg_diag.right_command_pose_body_mm.x_mm = x_mm;
        control_leg_diag.right_command_pose_body_mm.y_mm = y_mm;
        control_leg_diag.right_command_pose_body_mm.valid = APP_TRUE;
    }
}
```

Call this once in `control_leg_publish_diag` after the actuator snapshot fields are copied. On FK failure, retain the previous X/Y and set `valid = APP_FALSE`; never substitute `(0, legacy_stance_ref_units)`.

Remove every mode-specific assignment to the old flat X/Y diagnostics. This includes init, legacy stance, fast legacy stance, direct stance, IK reference, and IK validate paths.

- [ ] **Step 5: Convert `LIKREF` and `LXY` to the physical contract**

Port only the relevant `control_leg.c` changes from `c07702a`:

```c
leg_kinematics_solve(APP_FALSE,
                     kinematics->physical_reference_x_mm,
                     kinematics->physical_reference_y_mm,
                     NULL,
                     &left_reference)
```

Use the same physical reference for the right side and set the stored target to those physical values. Remove `control_leg_ik_validation_point_valid`; `control_leg_set_xy` must contain:

```c
if(APP_FALSE == leg_kinematics_target_valid(x_mm, y_mm))
{
    return APP_FALSE;
}
control_leg_ik_target_x_mm = x_mm;
control_leg_ik_target_y_mm = y_mm;
```

The command remains absolute millimetres. Do not subtract `P0` and do not clamp invalid points.

- [ ] **Step 6: Update zero-calibration static checks**

In `tools/test_leg_ik_zero_calibration_static.ps1`, replace `(0,55)` reference expectations with:

```powershell
Assert-Contains "project/code/leg_config.c" `
    '\.physical_reference_x_mm\s*=\s*-20\.766667f' `
    'LIKREF physical X missing'
Assert-Contains "project/code/leg_config.c" `
    '\.physical_reference_y_mm\s*=\s*47\.356667f' `
    'LIKREF physical Y missing'
Assert-Contains "project/code/control_leg.c" `
    'leg_kinematics_target_valid\(x_mm, y_mm\)' `
    'LXY must delegate to the physical hull'
```

Retain the assertion that `LIKREF` maps to four logical 90-degree servo commands.

- [ ] **Step 7: Run physical pose and legacy regression tests**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_leg_coordinate_contract_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_leg_physical_ik_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_leg_ik_zero_calibration_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_leg_transition_numeric.ps1
```

Expected: all pass; `LH=55` still maps to four logical 90-degree commands, while command FK maps those commands near `P0`.

- [ ] **Step 8: Commit physical command-pose publication**

```powershell
git add -- project/code/app_types.h project/code/control_leg.c project/code/control_leg.h tools/test_leg_coordinate_contract_static.ps1 tools/test_leg_ik_zero_calibration_static.ps1
git diff --cached --check
git commit -m "Report physical leg command poses"
```

---

### Task 5: Migrate the fixed-bandwidth telemetry and data collectors

**Files:**
- Modify: `project/code/app_types.h`
- Modify: `project/code/telemetry.c`
- Modify: `tools/collect_balance_data.ps1`
- Modify: `tools/test_collect_balance_data.ps1`
- Modify: `tools/calib_ik_servo.ps1`
- Modify: `tools/test_ik_height_control_static.ps1`
- Modify: `tools/test_balance_drive_v2_static.ps1`
- Modify: `tools/test_servo_300hz_integration_static.ps1`
- Modify: `tools/test_timing_noise_regressions.ps1`

**Interfaces:**
- Consumes: two `leg_pose_command_estimate_struct` values, `ik_valid`, and provenance.
- Produces: unchanged `float vofa_data[55]`, a pose-status bitmask at index 16, physical X/Y at indices 33-36, and decoded CSV fields.

- [ ] **Step 1: Define the exact 55-float v2 mapping in the collector test first**

Keep indices `0-12`, `17-32`, and `38-54` semantically unchanged except for the documented mapping below. Replace the leg slots with:

```text
13 legacy_stance_target_units
14 legacy_stance_ref_units
15 legacy_stance_norm
16 leg_pose_status_flags
17 leg_output_enable
33 leg_left_command_x_mm
34 leg_left_command_y_mm
35 leg_right_command_x_mm
36 leg_right_command_y_mm
37 leg_ik_margin
```

Define status bits in `project/code/app_types.h`:

```c
typedef enum
{
    LEG_POSE_STATUS_IK_VALID = (1U << 0),
    LEG_POSE_STATUS_LEFT_VALID = (1U << 1),
    LEG_POSE_STATUS_RIGHT_VALID = (1U << 2),
    LEG_POSE_STATUS_LEFT_MEASURED = (1U << 3),
    LEG_POSE_STATUS_RIGHT_MIRROR = (1U << 4)
}leg_pose_status_flag_enum;
```

Update the synthetic frame in `tools/test_collect_balance_data.ps1` so index 16 is `31.0`, positions are `-20.75,47.35,-20.80,47.40`, and index 37 remains `0.42`. Assert all decoded validity/source fields and assert the wire float count remains 55.

- [ ] **Step 2: Run collector and timing tests and verify they fail on the old layout**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_collect_balance_data.ps1
powershell -ExecutionPolicy Bypass -File tools/test_timing_noise_regressions.ps1
```

Expected: collector FAIL on the new physical X/Y or status assertions; timing must still recognize the 55-float constraint.

- [ ] **Step 3: Build the status word and repurpose only obsolete leg slots**

In `project/code/telemetry.c`, build `uint32 leg_pose_status_flags` before filling the frame:

```c
leg_pose_status_flags = 0U;
if(APP_TRUE == leg->ik_valid)
{
    leg_pose_status_flags |= LEG_POSE_STATUS_IK_VALID;
}
if(APP_TRUE == leg->left_command_pose_body_mm.valid)
{
    leg_pose_status_flags |= LEG_POSE_STATUS_LEFT_VALID;
}
if(APP_TRUE == leg->right_command_pose_body_mm.valid)
{
    leg_pose_status_flags |= LEG_POSE_STATUS_RIGHT_VALID;
}
if(LEG_POSE_SOURCE_MEASURED_CALIBRATION == leg->left_command_pose_body_mm.source)
{
    leg_pose_status_flags |= LEG_POSE_STATUS_LEFT_MEASURED;
}
if(LEG_POSE_SOURCE_MIRROR_ASSUMPTION == leg->right_command_pose_body_mm.source)
{
    leg_pose_status_flags |= LEG_POSE_STATUS_RIGHT_MIRROR;
}
```

Write exact indices:

```c
vofa_data[13] = leg->legacy_stance_target_units;
vofa_data[14] = leg->legacy_stance_ref_units;
vofa_data[15] = leg->legacy_stance_norm;
vofa_data[16] = (float)leg_pose_status_flags;
vofa_data[17] = (float)leg->output_enable;
vofa_data[33] = leg->left_command_pose_body_mm.x_mm;
vofa_data[34] = leg->left_command_pose_body_mm.y_mm;
vofa_data[35] = leg->right_command_pose_body_mm.x_mm;
vofa_data[36] = leg->right_command_pose_body_mm.y_mm;
vofa_data[37] = leg->ik_margin;
```

Keep motion state at 38, fault at 39, safety/trajectory at 40-45, and timing at 46-54. Do not increase `vofa_data[55]`.

- [ ] **Step 4: Decode pose status and physical coordinates in both collectors**

In `tools/collect_balance_data.ps1`, parse:

```powershell
$poseStatus = [int]$values[16]
leg_pose_status_flags = $poseStatus
leg_ik_valid = [int](($poseStatus -band 1) -ne 0)
leg_left_pose_valid = [int](($poseStatus -band 2) -ne 0)
leg_right_pose_valid = [int](($poseStatus -band 4) -ne 0)
leg_left_pose_source = $(if(($poseStatus -band 8) -ne 0) { "measured_calibration" } else { "none" })
leg_right_pose_source = $(if(($poseStatus -band 16) -ne 0) { "mirror_assumption" } else { "none" })
leg_left_command_x_mm = $values[33]
leg_left_command_y_mm = $values[34]
leg_right_command_x_mm = $values[35]
leg_right_command_y_mm = $values[36]
leg_ik_margin = $values[37]
```

Use `Convert-CsvField` for the source strings. Remove CSV names `leg_target_height_mm`, `leg_height_cmd_est_mm`, `leg_height_norm`, `leg_height_ref_mm`, and `leg_height_rate_mm_s`. The CSV contains the 55 wire values plus five decoded pose columns (`leg_ik_valid`, two side-valid fields, and two source fields), so the exact header count is `4 metadata + 55 wire + 5 decoded + 1 note = 65`.

Apply the same wire-index mapping in `tools/calib_ik_servo.ps1`. Its calibration CSV must record the four physical command coordinates, pose status, `ik_valid`, and provenance; remove the legacy height-rate column.

- [ ] **Step 5: Update static telemetry contracts without relaxing timing**

In the four static test scripts, require:

```powershell
Assert-Contains "project/code/telemetry.c" 'float vofa_data\[55\]' 'Telemetry must remain 55 floats.'
Assert-Contains "project/code/telemetry.c" 'vofa_data\[33\]\s*=\s*leg->left_command_pose_body_mm\.x_mm' 'Left physical X missing.'
Assert-Contains "project/code/telemetry.c" 'vofa_data\[36\]\s*=\s*leg->right_command_pose_body_mm\.y_mm' 'Right physical Y missing.'
Assert-Contains "project/code/telemetry.c" 'LEG_POSE_STATUS_RIGHT_MIRROR' 'Right provenance flag missing.'
Assert-NotContains "project/code/telemetry.c" 'actual_height_mm' 'Telemetry must not claim measured height.'
```

Keep the existing nonblocking `Cy_SCB_WriteArray`, drop counter, frame sequence, servo tick, IMU counters, and line-utilization assertions unchanged.

- [ ] **Step 6: Run telemetry, timing, and integration tests**

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_collect_balance_data.ps1
powershell -ExecutionPolicy Bypass -File tools/test_ik_height_control_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_servo_300hz_integration_static.ps1
powershell -ExecutionPolicy Bypass -File tools/test_timing_noise_regressions.ps1
```

Expected: all pass; the timing test still reports less than 50 percent line utilization because the wire frame remains 55 floats.

- [ ] **Step 7: Commit the telemetry contract**

```powershell
git add -- project/code/app_types.h project/code/telemetry.c tools/collect_balance_data.ps1 tools/test_collect_balance_data.ps1 tools/calib_ik_servo.ps1 tools/test_ik_height_control_static.ps1 tools/test_balance_drive_v2_static.ps1 tools/test_servo_300hz_integration_static.ps1 tools/test_timing_noise_regressions.ps1
git diff --cached --check
git commit -m "Expose physical leg poses in telemetry"
```

---

### Task 6: Complete documentation, whole-suite regression, and hardware gates

**Files:**
- Modify: `docs/leg-ik-zero-calibration-hardware-test.md`
- Modify: `docs/leg-height-phase1-hardware-test.md`
- Modify: `tools/test_leg_coordinate_contract_static.ps1`
- Test: all affected PowerShell scripts and CM7_0 IAR project.

**Interfaces:**
- Consumes: completed physical frame, legacy compatibility, pose telemetry, and provenance.
- Produces: an evidence-backed software handoff and explicit motor-disabled left/right hardware acceptance gates.

- [ ] **Step 1: Update the hardware documents with the exact coordinate contract**

Add this block to `docs/leg-ik-zero-calibration-hardware-test.md`:

```markdown
## BODY_WHEEL coordinate contract

- Origin: chassis-fixed cross-circle marker center.
- Unit: millimetres.
- +X: vehicle forward.
- +Y: downward.
- Reference command: `LIKREF = (-20.766667, 47.356667) mm`.
- Runtime X/Y: calibrated command estimate from actuator output commands; no position feedback is fitted.
- Left source: measured calibration.
- Right source: mirror assumption until the independent checks below pass.

`LXY,0,55` is outside the calibrated physical workspace and must be rejected.
```

Add a motor-disabled table requiring target X/Y, measured X/Y, component error, pose-status flags, and servo outputs for `LIKREF` plus two inset physical points. Require the right leg to be measured independently at the same three points before dynamic wheel shift is allowed.

In `docs/leg-height-phase1-hardware-test.md`, replace claims that `LH` values are millimetres with “legacy stance units”; preserve the existing immutable bench, stationary support, balance-in-place, straight, turn, and stop gate order.

- [ ] **Step 2: Add final stale-name and forbidden-pattern checks**

Extend `tools/test_leg_coordinate_contract_static.ps1` with:

```powershell
$runtimeFiles = @(
    "project/code/app_types.h",
    "project/code/control_leg.h",
    "project/code/control_leg.c",
    "project/code/control_balance.c",
    "project/code/control_chassis.c",
    "project/code/telemetry.c"
)
foreach($path in $runtimeFiles) {
    Reject-Pattern $path 'actual_height_mm|target_height_mm|height_ref_mm|height_norm' `
        "stale mixed-domain height symbol remains in $path"
}
Reject-Pattern "project/code/control_leg.c" `
    'left_command_pose_body_mm\.x_mm\s*=\s*0\.0f' `
    'invalid pose must not fabricate X=0'
Reject-Pattern "project/code/control_leg.c" `
    'left_command_pose_body_mm\.y_mm\s*=\s*control_leg_legacy' `
    'legacy stance must not fabricate physical Y'
Require-Pattern "docs/leg-ik-zero-calibration-hardware-test.md" `
    'right leg[\s\S]*mirror assumption[\s\S]*independent' `
    'right-leg independent validation gate missing'
```

- [ ] **Step 3: Run the complete coordinate/servo/balance regression set**

```powershell
$tests = @(
    "tools/test_leg_physical_ik_static.ps1",
    "tools/test_leg_coordinate_contract_static.ps1",
    "tools/test_servo_pwm_resolution_static.ps1",
    "tools/test_servo_300hz_integration_static.ps1",
    "tools/test_leg_ik_zero_calibration_static.ps1",
    "tools/test_leg_transition_numeric.ps1",
    "tools/test_leg_first_height_frame_static.ps1",
    "tools/test_ik_height_control_static.ps1",
    "tools/test_collect_balance_data.ps1",
    "tools/test_balance_drive_v2_static.ps1",
    "tools/test_timing_noise_regressions.ps1"
)
foreach($test in $tests) {
    powershell -ExecutionPolicy Bypass -File $test
    if(0 -ne $LASTEXITCODE) { throw "$test failed" }
}
```

Expected: every script prints its `passed` message and the loop exits 0.

- [ ] **Step 4: Audit the diff and build boundary**

```powershell
git diff --check
git status --short
rg -n "actual_height_mm|target_height_mm|height_ref_mm|height_norm|reference_x_mm|reference_y_mm|control_leg_ik_validation_point_valid" project/code tools docs
git diff --stat a044d17..HEAD
```

Expected: `git diff --check` is silent; `rg` finds only historical explanations or explicitly updated test rejection patterns, not runtime symbols; untracked user data remains untouched.

Open `project/iar/cyt4bb7.eww` in IAR EWARM 9.40.1 and build `cyt4bb7_cm_7_0`. Expected: zero errors and no new warnings. Record the build result; do not claim an IAR pass if the IDE/toolchain is unavailable.

- [ ] **Step 5: Commit documentation and final test gates**

```powershell
git add -- docs/leg-ik-zero-calibration-hardware-test.md docs/leg-height-phase1-hardware-test.md tools/test_leg_coordinate_contract_static.ps1
git diff --cached --check
git commit -m "Document physical leg coordinate gates"
```

- [ ] **Step 6: Perform the motor-disabled hardware acceptance before race work**

With wheel-motor output disabled:

1. Run `LIKREF`; confirm logical servo outputs are near 90 degrees and telemetry reports command estimates near `P0` with left measured/right mirror provenance.
2. Measure left wheel center at `LIKREF` and two targets at least 2 mm inside the hull; record target-minus-measured X/Y errors.
3. Repeat independently for the right wheel; do not infer right measurements from left values.
4. Verify `LXY,0,55` is rejected and does not move the servos.
5. Verify an invalid FK sets the relevant validity bit to 0 without replacing X/Y with legacy stance values.

Expected software status before hardware: implemented and regression-tested. Expected hardware status remains “pending” until the measurements above are recorded. Do not enable dynamic wheel shift, lower race stance, or higher speed from software evidence alone.

---

## Final Success Criteria

- Runtime code has one public physical wheel-center frame and no fake `x=0/y=legacy` publication.
- `LH/LHF` keep the same servo trajectories and scheduling behavior under `legacy_stance_*` names.
- `LIKREF` uses `(-20.766667,47.356667) mm`; `LXY` is absolute physical millimetres and uses the inset convex hull.
- Physical pose telemetry follows actuator `output_deg`, so it represents the currently executed open-loop command through S7/LPF transitions.
- Left measured/right mirror provenance and validity are machine-readable without enlarging the 55-float wire frame.
- All relevant PowerShell regressions pass; CM7_0 IAR and left/right hardware results are reported separately and honestly.

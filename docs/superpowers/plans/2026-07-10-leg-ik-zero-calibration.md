# Leg IK Zero Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a calibrated mapping from five-bar IK joint angles to the four physical PWM servo commands, then expose safe reference-pose and limited XY validation commands without altering `LH` or `LHF`.

**Architecture:** The existing `leg_kinematics` solver continues to calculate mathematical active-joint angles.  A new mapping stage subtracts the solver angle at the reference point `(0 mm, 55 mm)`, applies each servo's installation direction and offset, then adds the measured reference-pose PWM midpoint.  `control_leg` owns the accepted previous IK solutions and only emits a new XY command if both legs, all four mapped servo limits, and the restricted validation workspace are valid.

**Tech Stack:** C99 firmware for CYT4BB/Traveo II, existing `leg_kinematics`, `control_leg`, UART host commands, PowerShell static/numeric tests, IAR CM7_0 hardware build.

## Global Constraints

- Reference pose is level, left/right symmetric, approximately `X = 0 mm`, `Y = 55 mm`.
- `LH` and `LHF` keep the current empirical 30–80 mm height mapping unchanged.
- Initial `LXY` acceptance range is `x = [-10, 10] mm`, `y = [50, 60] mm`.
- No automatic physical X/Y feedback exists; `LXY` is an open-loop, bench-only validation path.
- Reject invalid IK, insufficient margin, non-finite values, or mapped servo-limit violations while preserving the last safe command.
- Do not change PWM timing, servo rate limits, pins, or motor/balance parameters.
- Current kinematic dimensions in `leg_config.c` are explicitly temporary; no claim of metric XY accuracy is permitted before geometry calibration.

---

### Task 1: Add reference-pose calibration data and a pure IK-to-servo mapping API

**Files:**
- Modify: `project/code/leg_config.h:20-55`
- Modify: `project/code/leg_config.c:8-32`
- Modify: `project/code/leg_kinematics.h:12-30`
- Modify: `project/code/leg_kinematics.c:247-358`
- Create: `tools/test_leg_ik_zero_calibration_static.ps1`

**Interfaces:**
- Consumes: `leg_ik_result_struct` mathematical pair angles from `leg_kinematics_solve`.
- Produces: `uint8 leg_kinematics_map_reference_pose(const leg_ik_result_struct *left, const leg_ik_result_struct *right, float servo_deg[LEG_SERVO_COUNT]);`.
- Produces: `uint8 leg_kinematics_map_target_pose(const leg_ik_result_struct *left_reference, const leg_ik_result_struct *right_reference, const leg_ik_result_struct *left_target, const leg_ik_result_struct *right_target, float servo_deg[LEG_SERVO_COUNT]);`.

- [ ] **Step 1: Write the failing static/numeric test**

Create `tools/test_leg_ik_zero_calibration_static.ps1`.  It must copy `leg_config.c` and `leg_kinematics.c` into a temporary GCC harness and compile the following assertion program:

```c
int main(void)
{
    leg_ik_result_struct left_ref = {0};
    leg_ik_result_struct right_ref = {0};
    leg_ik_result_struct left_target = {0};
    leg_ik_result_struct right_target = {0};
    float reference_cmd[LEG_SERVO_COUNT];
    float target_cmd[LEG_SERVO_COUNT];

    if((APP_TRUE != leg_kinematics_solve(APP_FALSE, 0.0f, 55.0f, NULL, &left_ref)) ||
       (APP_TRUE != leg_kinematics_solve(APP_TRUE, 0.0f, 55.0f, NULL, &right_ref)) ||
       (APP_TRUE != leg_kinematics_map_reference_pose(&left_ref, &right_ref, reference_cmd)))
    {
        return 1;
    }
    if((fabsf(reference_cmd[LEG_SERVO_FL] - leg_config_get_servo(LEG_SERVO_FL)->neutral_deg) > 0.001f) ||
       (fabsf(reference_cmd[LEG_SERVO_FR] - leg_config_get_servo(LEG_SERVO_FR)->neutral_deg) > 0.001f) ||
       (fabsf(reference_cmd[LEG_SERVO_RL] - leg_config_get_servo(LEG_SERVO_RL)->neutral_deg) > 0.001f) ||
       (fabsf(reference_cmd[LEG_SERVO_RR] - leg_config_get_servo(LEG_SERVO_RR)->neutral_deg) > 0.001f))
    {
        return 2;
    }
    if((APP_TRUE != leg_kinematics_solve(APP_FALSE, 5.0f, 55.0f, &left_ref, &left_target)) ||
       (APP_TRUE != leg_kinematics_solve(APP_TRUE, 5.0f, 55.0f, &right_ref, &right_target)) ||
       (APP_TRUE != leg_kinematics_map_target_pose(&left_ref, &right_ref, &left_target, &right_target, target_cmd)))
    {
        return 3;
    }
    return 0;
}
```

Run: `powershell -ExecutionPolicy Bypass -File .\tools\test_leg_ik_zero_calibration_static.ps1`

Expected: FAIL because the mapping APIs and calibration fields do not exist.

- [ ] **Step 2: Add configuration fields with safe initial values**

Extend `leg_servo_config_struct` immediately after `direction`:

```c
float ik_offset_deg;
```

Keep `neutral_deg` as `mid_deg`: it is the physical PWM command of the reference pose.  Initialize it to the current safe value `90.0f`, initialize `ik_offset_deg` to `0.0f`, and preserve every existing min/max limit and direction value.  Add these explicit restricted-command values to `leg_kinematics_config_struct`:

```c
float validate_x_min_mm;
float validate_x_max_mm;
float validate_y_min_mm;
float validate_y_max_mm;
float reference_x_mm;
float reference_y_mm;
```

Initialize them to `-10.0f`, `10.0f`, `50.0f`, `60.0f`, `0.0f`, and `55.0f` respectively.

- [ ] **Step 3: Implement the mapping API**

In `leg_kinematics.c`, add a private mapper and the two public wrappers:

```c
static uint8 leg_kinematics_map_one(uint8 servo_index, float reference_deg,
                                    float target_deg, float *command_deg)
{
    const leg_servo_config_struct *cfg = leg_config_get_servo(servo_index);
    float value;
    if((NULL == cfg) || (NULL == command_deg) ||
       (APP_FALSE == leg_kinematics_is_finite(reference_deg)) ||
       (APP_FALSE == leg_kinematics_is_finite(target_deg)))
    {
        return APP_FALSE;
    }
    value = cfg->neutral_deg + cfg->direction * (target_deg - reference_deg) + cfg->ik_offset_deg;
    if(APP_FALSE == leg_kinematics_servo_valid(servo_index, value))
    {
        return APP_FALSE;
    }
    *command_deg = value;
    return APP_TRUE;
}
```

Map left result indexes `[0,1]` to `LEG_SERVO_FL, LEG_SERVO_RL`, right result indexes `[0,1]` to `LEG_SERVO_FR, LEG_SERVO_RR`.  The reference wrapper passes equal reference/target values; therefore it must exactly produce all four `neutral_deg` values.  The target wrapper calls the private helper four times and returns false without partially accepting an invalid target.

- [ ] **Step 4: Run static/numeric tests**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\test_leg_ik_zero_calibration_static.ps1
powershell -ExecutionPolicy Bypass -File .\tools\test_ik_height_control_static.ps1
```

Expected: both print their success messages.  The existing height-control test proves the empirical `LH/LHF` path is still represented in source and its numeric behavior was not altered.

- [ ] **Step 5: Commit Task 1**

```powershell
git add -- project/code/leg_config.h project/code/leg_config.c project/code/leg_kinematics.h project/code/leg_kinematics.c tools/test_leg_ik_zero_calibration_static.ps1
git commit -m "Add calibrated IK servo mapping"
```

### Task 2: Add a hold-safe reference pose and restricted XY controller mode

**Files:**
- Modify: `project/code/control_leg.h:12-36`
- Modify: `project/code/control_leg.c:12-40,301-643,645-813`

**Interfaces:**
- Consumes: Task 1 mapping APIs and `leg_kinematics_solve`.
- Produces: `uint8 control_leg_set_ik_reference(uint32 now_ms);`.
- Produces: `uint8 control_leg_set_xy(float x_mm, float y_mm, uint32 now_ms);`.
- Produces: two new modes, `LEG_MODE_IK_REFERENCE` and `LEG_MODE_IK_VALIDATE`.

- [ ] **Step 1: Extend the failing static test for controller-source requirements**

Append these checks to `tools/test_leg_ik_zero_calibration_static.ps1` before compiling its temporary harness:

```powershell
Assert-Contains "project/code/control_leg.h" "control_leg_set_ik_reference" "Missing reference-pose controller API."
Assert-Contains "project/code/control_leg.h" "control_leg_set_xy" "Missing XY controller API."
Assert-Contains "project/code/control_leg.c" "LEG_MODE_IK_REFERENCE" "Missing safe IK reference mode."
Assert-Contains "project/code/control_leg.c" "LEG_MODE_IK_VALIDATE" "Missing restricted XY validation mode."
Assert-Contains "project/code/control_leg.c" "leg_kinematics_map_target_pose" "XY mode must use calibrated mapping."
```

Run: `powershell -ExecutionPolicy Bypass -File .\tools\test_leg_ik_zero_calibration_static.ps1`

Expected: FAIL because controller support does not exist.

- [ ] **Step 2: Add persistent controller state and public APIs**

Add these controller state variables near the existing height state:

```c
static float control_leg_ik_target_x_mm;
static float control_leg_ik_target_y_mm;
static leg_ik_result_struct control_leg_ik_reference_left;
static leg_ik_result_struct control_leg_ik_reference_right;
static leg_ik_result_struct control_leg_ik_previous_left;
static leg_ik_result_struct control_leg_ik_previous_right;
static uint8 control_leg_ik_reference_valid;
```

`control_leg_set_ik_reference` must solve both sides at configured reference XY with no previous solution, map using `leg_kinematics_map_reference_pose`, require all four mapped limits, then enter `LEG_MODE_IK_REFERENCE`.  It returns false and leaves the current output untouched if any check fails.

`control_leg_set_xy` must reject a faulted controller, non-finite inputs, and any input outside `validate_*`.  It stores the target and enters `LEG_MODE_IK_VALIDATE`; it must not modify `LH/LHF` state.

- [ ] **Step 3: Implement execution behavior that preserves the last safe command**

Add switch cases with the following behavior:

```c
case LEG_MODE_IK_REFERENCE:
    /* Re-map stored reference; output may be enabled only when app state is runnable. */
    control_leg_motion_state = LEG_MOTION_STABLE;
    control_leg_fault_reason = LEG_FAULT_NONE;
    control_leg_diag.left_x_mm = cfg->reference_x_mm;
    control_leg_diag.left_y_mm = cfg->reference_y_mm;
    control_leg_diag.right_x_mm = cfg->reference_x_mm;
    control_leg_diag.right_y_mm = cfg->reference_y_mm;
    break;

case LEG_MODE_IK_VALIDATE:
    /* Solve both target legs using their own previously accepted solution. */
    /* Map only after both solver results are valid. */
    /* On failure call control_leg_enter_fault(LEG_FAULT_IK_INVALID) and do not publish a new target. */
    break;
```

When a mapped result is valid, copy its four output angles to `control_leg_servo_cmd`, update both `control_leg_ik_previous_*` values, set `ik_margin` to the lower side margin, set `ik_valid = APP_TRUE`, and mark `LEG_MOTION_STABLE`.  When a mapped limit fails, enter `LEG_FAULT_SERVO_LIMIT`; when either solver rejects, enter `LEG_FAULT_IK_INVALID`.  In neither failure case may the previous XY servo command be overwritten before the fault-safe command is selected.

Initialize `control_leg_ik_reference_valid = APP_FALSE` in `control_leg_init`; clear previous solutions when `STOP` selects lock through `control_leg_set_mode`.

- [ ] **Step 4: Run regression and source tests**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\test_leg_ik_zero_calibration_static.ps1
powershell -ExecutionPolicy Bypass -File .\tools\test_ik_height_control_static.ps1
powershell -ExecutionPolicy Bypass -File .\tools\test_leg_transition_numeric.ps1
```

Expected: all commands exit `0`.  The first verifies the new APIs and mapping call; the two existing tests guard the current height trajectory and numerical IK solver.

- [ ] **Step 5: Commit Task 2**

```powershell
git add -- project/code/control_leg.h project/code/control_leg.c tools/test_leg_ik_zero_calibration_static.ps1
git commit -m "Add restricted calibrated leg XY validation"
```

### Task 3: Add explicit UART commands that disable drive and balance before IK validation

**Files:**
- Modify: `project/code/host_command.c:298-387`
- Modify: `tools/test_leg_ik_zero_calibration_static.ps1`

**Interfaces:**
- Consumes: `control_leg_set_ik_reference`, `control_leg_set_xy`, and existing `host_command_parse_two_numbers`.
- Produces: `LIKREF` and `LXY,<x_mm>,<y_mm>` host commands.

- [ ] **Step 1: Add failing parser/source checks**

Append:

```powershell
Assert-Contains "project/code/host_command.c" "'L' == line[0]) && ('I' == line[1]) && ('K' == line[2]) && ('R' == line[3]" "Missing LIKREF command parser."
Assert-Contains "project/code/host_command.c" "'L' == line[0]) && ('X' == line[1]) && ('Y' == line[2]" "Missing LXY command parser."
Assert-Contains "project/code/host_command.c" "control_chassis_stop(now_ms);" "IK bench commands must stop chassis first."
Assert-Contains "project/code/host_command.c" "control_balance_set_mode(BALANCE_MODE_OFF);" "IK bench commands must disable balance first."
```

Run: `powershell -ExecutionPolicy Bypass -File .\tools\test_leg_ik_zero_calibration_static.ps1`

Expected: FAIL because neither command parser exists.

- [ ] **Step 2: Implement `LIKREF`**

Place the exact-match `LIKREF` handler before the generic `LIK,` handler:

```c
if(('L' == line[0]) && ('I' == line[1]) && ('K' == line[2]) &&
   ('R' == line[3]) && ('E' == line[4]) && ('F' == line[5]) && ('\0' == line[6]))
{
    control_chassis_stop(now_ms);
    control_balance_set_mode(BALANCE_MODE_OFF);
    actuator_motor_set_mode_stop();
    if(APP_TRUE == control_leg_set_ik_reference(now_ms))
    {
        actuator_motor_record_command_error(APP_FALSE);
        return;
    }
}
```

- [ ] **Step 3: Implement `LXY,x_mm,y_mm`**

Use `host_command_parse_two_numbers(&line[4], &value, &period_ms_f)` only when the prefix is `LXY,`.  `value` is X, `period_ms_f` is Y; name a local `y_mm` if doing so improves clarity.  Before calling `control_leg_set_xy`, invoke the same chassis stop, balance-off, and motor-stop sequence as `LIKREF`.  Clear the command error and return only if the controller accepts the target; otherwise fall through to the existing error path.

- [ ] **Step 4: Run all static tests**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\test_leg_ik_zero_calibration_static.ps1
powershell -ExecutionPolicy Bypass -File .\tools\test_ik_height_control_static.ps1
powershell -ExecutionPolicy Bypass -File .\tools\test_servo_pwm_frequency_static.ps1
```

Expected: all print success.  The PWM frequency test protects the validated production 50 Hz setting.

- [ ] **Step 5: Commit Task 3**

```powershell
git add -- project/code/host_command.c tools/test_leg_ik_zero_calibration_static.ps1
git commit -m "Add safe leg IK validation commands"
```

### Task 4: Document the one-time hardware zero calibration and restricted bring-up

**Files:**
- Create: `docs/leg-ik-zero-calibration-hardware-test.md`
- Modify: `docs/leg-height-phase1-hardware-test.md`
- Test: `tools/test_leg_ik_zero_calibration_static.ps1`

**Interfaces:**
- Consumes: `LIK`, `LIKREF`, `LXY` and telemetry fields `servo_target_deg`, `ik_valid`, `ik_margin`, `fault_reason`.
- Produces: a reproducible operator procedure and required evidence filenames.

- [ ] **Step 1: Write documentation source test**

Add checks:

```powershell
Assert-Contains "docs/leg-ik-zero-calibration-hardware-test.md" "LIKREF" "Hardware procedure must include LIKREF."
Assert-Contains "docs/leg-ik-zero-calibration-hardware-test.md" "LXY,0,55" "Hardware procedure must include reference XY check."
Assert-Contains "docs/leg-ik-zero-calibration-hardware-test.md" "LXY,5,55" "Hardware procedure must include positive X check."
Assert-Contains "docs/leg-ik-zero-calibration-hardware-test.md" "LXY,-5,55" "Hardware procedure must include negative X check."
Assert-Contains "docs/leg-ik-zero-calibration-hardware-test.md" "LXY,0,52" "Hardware procedure must include lower Y check."
Assert-Contains "docs/leg-ik-zero-calibration-hardware-test.md" "LXY,0,58" "Hardware procedure must include higher Y check."
```

Run: `powershell -ExecutionPolicy Bypass -File .\tools\test_leg_ik_zero_calibration_static.ps1`

Expected: FAIL because the procedure file is missing.

- [ ] **Step 2: Write the hardware procedure**

Document these mandatory gates in `docs/leg-ik-zero-calibration-hardware-test.md`:

1. Vehicle supported so wheels cannot fall or drive; wheels stopped and balance disabled.
2. Use `LIK,a0,a1,a2,a3` to physically set a level, symmetric `Y≈55 mm` reference pose.
3. Record the four final values and replace only the four `neutral_deg` initializers in `leg_config.c`; retain `ik_offset_deg = 0` initially.
4. Rebuild the affected CM7_0 IAR project and flash.
5. Log 3 seconds of `LIKREF`; require `fault_reason=0`, stable targets, and visual reference-pose match.
6. Run and log each command separately: `LXY,0,55`, `LXY,5,55`, `LXY,-5,55`, `LXY,0,52`, `LXY,0,58`.
7. Stop on a wrong direction, unexpected motion, nonzero fault, or `ik_valid=0`; do not enlarge range or enable driving.

State explicitly that this validates command consistency and X direction only, not measured millimeter X accuracy.

- [ ] **Step 3: Run static and diff checks**

Run:

```powershell
git diff --check
powershell -ExecutionPolicy Bypass -File .\tools\test_leg_ik_zero_calibration_static.ps1
powershell -ExecutionPolicy Bypass -File .\tools\test_ik_height_control_static.ps1
powershell -ExecutionPolicy Bypass -File .\tools\test_leg_transition_numeric.ps1
powershell -ExecutionPolicy Bypass -File .\tools\test_servo_pwm_resolution_static.ps1
powershell -ExecutionPolicy Bypass -File .\tools\test_servo_pwm_frequency_static.ps1
```

Expected: `git diff --check` has no output and every PowerShell test exits `0`.  IAR build and hardware verification remain required because this repository has no command-line IAR build tool.

- [ ] **Step 4: Commit Task 4**

```powershell
git add -- docs/leg-ik-zero-calibration-hardware-test.md docs/leg-height-phase1-hardware-test.md tools/test_leg_ik_zero_calibration_static.ps1
git commit -m "Document calibrated leg IK bring-up"
```

$ErrorActionPreference = "Stop"

function Require-Pattern {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if(-not (Test-Path $Path)) {
        throw ("Missing file: {0}" -f $Path)
    }
    if((Get-Content -Raw $Path) -notmatch $Pattern) {
        throw $Message
    }
}

function Reject-Pattern {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if((Get-Content -Raw $Path) -match $Pattern) {
        throw $Message
    }
}

$header = "project/code/control_leg.h"
$source = "project/code/control_leg.c"
$types = "project/code/app_types.h"
$chassis = "project/code/control_chassis.c"
$supervisor = "project/code/control_race_assist.c"
$config = "project/code/app_config.h"
$balance = "project/code/control_balance.c"
$hostCommand = "project/code/host_command.c"
$telemetry = "project/code/telemetry.c"
$collector = "tools/collect_balance_data.ps1"
$calibration = "tools/calib_ik_servo.ps1"
$hardwareDoc = "docs/low-race-leg-assist-hardware-test.md"

Require-Pattern $hardwareDoc 'BRA,0' 'Procedure must start from assist disabled.'
Require-Pattern $hardwareDoc 'LXY,-18\.83,25\.08' 'Procedure must establish the approved low-race zero.'
Require-Pattern $hardwareDoc 'BRG,0\.005,0\.005,0\.15' 'Level-1 test must set explicit conservative gains.'
Require-Pattern $hardwareDoc 'BRA,1' 'Level-1 assist must be enabled explicitly.'
Require-Pattern $hardwareDoc '250 RPM' 'Level-1 ground gate missing.'
Require-Pattern $hardwareDoc '390--410 RPM' 'Final 400 RPM acceptance band missing.'
Require-Pattern $hardwareDoc 'three seconds' 'Final dwell requirement missing.'
Require-Pattern $hardwareDoc 'command estimate' 'Open-loop pose meaning must be explicit.'
Require-Pattern $hardwareDoc 'motor power physically disconnected' 'Gate 0 must physically disconnect motor power.'
Require-Pattern $hardwareDoc 'LXY,-20\.83,27\.08' 'Gate 0 must check the rearward manual endpoint.'
Require-Pattern $hardwareDoc 'LXY,-16\.83,27\.08' 'Gate 0 must check the forward manual endpoint.'
Require-Pattern $hardwareDoc 'IK valid' 'Gate 0 must require valid IK before accepting each manual endpoint.'
Require-Pattern $hardwareDoc '(?s)STOP\s*->\s*LIKREF\s*->\s*settled\s*->\s*LXY,-18\.83,25\.08\s*->\s*settled\s*->\s*BRA,0\s*->\s*BRG,0,0,0\s*->\s*BRA,1\s*->\s*B,3' `
    'Gate 1 must re-enter low race completely before the A run.'
Require-Pattern $hardwareDoc 'APP_CHASSIS_CMD_TIMEOUT_MS=500' 'Gate 1 must document the chassis command timeout.'
Require-Pattern $hardwareDoc '100--200 ms heartbeat' 'Gate 1 must require a command heartbeat.'
Require-Pattern $hardwareDoc '(?s)single `C,250,0`.*30 RPM' 'Gate 1 must reject a one-shot 250 RPM command.'
Require-Pattern $hardwareDoc 'NOT RUN' 'Hardware status must remain explicit.'
Require-Pattern $hardwareDoc 'on-target WCET: NOT RUN' `
    'The DWT preflight WCET status must remain explicit until measured on CM7_0.'
Require-Pattern $hardwareDoc 'APP_RACE_ASSIST_PREFLIGHT_WCET_ENABLE=1' `
    'The hardware handoff must show how to enable the default-off DWT hook.'
Require-Pattern $hardwareDoc 'control_leg_race_preflight_max_cycles' `
    'The hardware handoff must name the DWT high-water diagnostic.'
Reject-Pattern $hardwareDoc 'u=0,\+1,0,-1,0' `
    'Gate 0 must not claim an unavailable direct-u supervisor sequence.'

function Require-TextPattern {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )

    if($Text -notmatch $Pattern) {
        throw $Message
    }
}

$hardwareText = Get-Content -Raw $hardwareDoc
$gate0Start = $hardwareText.IndexOf("## Gate 0:")
$gate1Start = $hardwareText.IndexOf("## Gate 1:", $gate0Start)
if(($gate0Start -lt 0) -or ($gate1Start -lt 0)) {
    throw "Unable to isolate Gate 0 hardware procedure."
}
$gate0Text = $hardwareText.Substring($gate0Start, $gate1Start - $gate0Start)
$gate1Text = $hardwareText.Substring($gate1Start)

Require-TextPattern $gate0Text '(?s)channel 16.*IK valid' `
    'Gate 0 must use channel 16 for manual IK validity.'
Require-TextPattern $gate0Text '(?s)channel 37.*common IK\s+margin.*>=0\.02' `
    'Gate 0 must use channel 37 common IK margin.'
Require-TextPattern $gate0Text '(?s)channels\s+22--25.*planner target' `
    'Gate 0 must record planner targets.'
Require-TextPattern $gate0Text '(?s)channels\s+18--21.*PWM command' `
    'Gate 0 must record PWM command outputs.'
Require-TextPattern $gate0Text '(?s)channel 31.*servo_settled=1' `
    'Gate 0 must require the settled flag.'
Require-TextPattern $gate0Text 'large or unexpected command jump' `
    'Gate 0 must reject any large or unexpected manual command jump.'
Require-TextPattern $gate0Text '(?s)branch\s+continuity.*software regression' `
    'Gate 0 must defer branch continuity proof to software regression.'
if($gate0Text -match '69/70|channel-71|branch flags') {
    throw "Gate 0 must not use race-only margin or branch telemetry."
}
Require-TextPattern $gate1Text '(?s)margins 69/70.*channel-71 branch bits' `
    'Gate 1 automatic path must retain per-side margins and branch bits.'
Require-Pattern $hardwareDoc 'wheel-motor power stage' `
    'Physical disconnection must name the isolated motor power stage.'
Require-Pattern $hardwareDoc '(?s)BLDC logic,\s*UART, and speed feedback remain powered' `
    'Gate 0 must retain logic and feedback after power-stage isolation.'
Require-Pattern $hardwareDoc 'channel 7 remains 1' `
    'Gate 0 must require live wheel feedback when the power stage is isolated.'
Require-Pattern $hardwareDoc 'firmware/driver output-disabled bench mode' `
    'Gate 0 needs an executable fallback when power isolation loses feedback.'
Require-Pattern $hardwareDoc '(?s)Only\s+Gate 0 may waive channel 7' `
    'Feedback-waiver scope must be limited to Gate 0.'

Require-Pattern $header 'LEG_MODE_RACE_ASSIST' `
    "Race assist leg mode missing."
Require-Pattern $header 'control_leg_set_race_assist_request\(float u_request,\s*float dx_mm,\s*float dy_mm,\s*uint32 now_ms\)' `
    "Race assist request API missing."
Require-Pattern $header 'control_leg_disable_race_assist\(uint32 now_ms\)' `
    "Race assist disable API missing."
Require-Pattern $types 'LEG_MOTION_RACE_ASSIST' `
    "Drive-allowed race motion state missing."
Require-Pattern $types 'LEG_MOTION_RACE_FAULT_HOLD' `
    "Race-specific fault hold state missing."
Require-Pattern $types 'float race_assist_request;' `
    "Race request diagnostic missing."
Require-Pattern $types 'float race_assist_actual;' `
    "Race actual diagnostic missing."
Require-Pattern $types 'float race_target_x_mm;' `
    "Race Cartesian X diagnostic missing."
Require-Pattern $types 'float race_target_y_mm;' `
    "Race Cartesian Y diagnostic missing."
Require-Pattern $types 'float left_ik_margin;' `
    "Left IK margin diagnostic missing."
Require-Pattern $types 'float right_ik_margin;' `
    "Right IK margin diagnostic missing."
Require-Pattern $types 'uint8 ik_branch_flags;' `
    "IK branch diagnostic flags missing."
Require-Pattern $types 'uint8 race_path_valid;' `
    "Race path-valid diagnostic missing."
Require-Pattern $types 'uint8 held_command_valid;' `
    "Race fault hold must distinguish a validated held command from a failed new target."
Require-Pattern $config 'APP_RACE_ASSIST_PREFLIGHT_WCET_ENABLE\s+\(0U\)' `
    "Preflight WCET instrumentation must be default-off."
Require-Pattern $config 'APP_BALANCE_RPM_LIMIT\s+\(460\.0f\)' `
    "Hard balance ceiling must leave 400 RPM correction reserve."
Require-Pattern $config 'APP_BALANCE_DEFAULT_RUNTIME_RPM_LIMIT\s+\(300\.0f\)' `
    "Non-assisted balance must retain the validated 300 RPM runtime cap."
Require-Pattern $types 'balance_output_limit_rpm' `
    "Active balance cap must be observable."
Require-Pattern $telemetry 'float vofa_data\[72\]' `
    "Race diagnostics require the exact 72-float frame."
Require-Pattern $telemetry 'vofa_data\[55\]\s*=\s*\(float\)chassis->race_assist_enable' `
    "Race enable must occupy telemetry index 55."
Require-Pattern $telemetry 'vofa_data\[56\]\s*=\s*\(float\)chassis->race_assist_level' `
    "Race level must occupy telemetry index 56."
Require-Pattern $telemetry 'vofa_data\[57\]\s*=\s*\(float\)chassis->race_assist_state' `
    "Race state must occupy telemetry index 57."
Require-Pattern $telemetry 'vofa_data\[58\]\s*=\s*\(float\)chassis->race_assist_fault_reason' `
    "Race fault must occupy telemetry index 58."
Require-Pattern $telemetry 'vofa_data\[59\]\s*=\s*chassis->race_u_request' `
    "Race request must occupy telemetry index 59."
Require-Pattern $telemetry 'vofa_data\[60\]\s*=\s*leg->race_assist_actual' `
    "Race actual must occupy telemetry index 60."
Require-Pattern $telemetry 'vofa_data\[61\]\s*=\s*chassis->requested_accel_rpm_s' `
    "Requested acceleration must occupy telemetry index 61."
Require-Pattern $telemetry 'vofa_data\[62\]\s*=\s*chassis->forward_target_rpm' `
    "Forward target must occupy telemetry index 62."
Require-Pattern $telemetry 'vofa_data\[63\]\s*=\s*chassis->forward_ramped_rpm' `
    "Ramped forward target must occupy telemetry index 63."
Require-Pattern $telemetry 'vofa_data\[64\]\s*=\s*chassis->wheel_speed_measured_rpm' `
    "Measured wheel speed must occupy telemetry index 64."
Require-Pattern $telemetry 'vofa_data\[65\]\s*=\s*chassis->speed_error_rpm' `
    "Speed error must occupy telemetry index 65."
Require-Pattern $telemetry 'vofa_data\[66\]\s*=\s*balance->pitch_setpoint_deg' `
    "Pitch setpoint must occupy telemetry index 66."
Require-Pattern $telemetry 'vofa_data\[67\]\s*=\s*balance->balance_output_limit_rpm' `
    "Balance output limit must occupy telemetry index 67."
Require-Pattern $telemetry 'vofa_data\[68\]\s*=\s*chassis->race_turn_scale' `
    "Race turn scale must occupy telemetry index 68."
Require-Pattern $telemetry 'vofa_data\[69\]\s*=\s*leg->left_ik_margin' `
    "Left IK margin must occupy telemetry index 69."
Require-Pattern $telemetry 'vofa_data\[70\]\s*=\s*leg->right_ik_margin' `
    "Right IK margin must occupy telemetry index 70."
Require-Pattern $telemetry 'vofa_data\[71\]\s*=\s*\(float\)leg->ik_branch_flags' `
    "IK branch flags must occupy telemetry index 71."
Require-Pattern $collector '\$FloatCount\s*=\s*72' `
    "Race collector must parse the 72-float frame."
Require-Pattern $calibration '\$FloatCount\s*=\s*72' `
    "Race calibration parser must parse the 72-float frame."

Require-Pattern $source 'LEG_TRAJECTORY_RACE_ASSIST' `
    "Dedicated Cartesian race trajectory mode missing."
Require-Pattern $source 'APP_RACE_ASSIST_ZERO_X_MM\s*-\s*\(control_leg_race_dx_mm\s*\*\s*control_leg_race_u_actual\)' `
    "Race X trajectory must originate at BODY_WHEEL (-18.83, 25.08)."
Require-Pattern $source 'APP_RACE_ASSIST_ZERO_Y_MM\s*\+\s*\(control_leg_race_dy_mm\s*\*\s*control_leg_absf\(control_leg_race_u_actual\)\)' `
    "Race Y trajectory must use the signed-path absolute lift."
Require-Pattern $source '2\.1875f\s*\*\s*max_delta_deg.*APP_RACE_ASSIST_SERVO_SPEED_DPS' `
    "Race trajectory must use the S7 90 deg/s duration limit."
Require-Pattern $source '(?s)control_leg_race_solve_target\(.*&control_leg_ik_previous_left' `
    "Race trajectory must solve the left persisted IK branch."
Require-Pattern $source '(?s)control_leg_race_solve_target\(.*&control_leg_ik_previous_right' `
    "Race trajectory must solve the right persisted IK branch."
Require-Pattern $source 'control_leg_enter_race_fault_hold' `
    "Race failures must enter the dedicated fault hold."
Require-Pattern $source 'LEG_MOTION_RACE_FAULT_HOLD' `
    "Race fault hold must preserve planned commands."
Require-Pattern $source 'LEG_MOTION_RACE_ASSIST' `
    "Race trajectory must publish a drive-allowed motion state."
Require-Pattern $source 'APP_RACE_ASSIST_REQUEST_DEADBAND' `
    "Race request changes must use the specified deadband."
Require-Pattern $source 'control_leg_race_preflight_cache_valid' `
    "Race path preflight must cache a successful profile."
Require-Pattern $source 'control_leg_race_preflight_branch_flags' `
    "Race path preflight cache must include persisted branch identity."
Require-Pattern $source 'control_leg_race_preflight_cache_invalidate' `
    "Race path preflight cache must support explicit reset/fault invalidation."
Require-Pattern $source 'CONTROL_LEG_DWT_CYCCNT' `
    "The default-off CM7_0 preflight WCET hook must use DWT CYCCNT."
Require-Pattern $source 'control_leg_race_held_command_valid' `
    "Race runtime IK failure must retain only an already-validated finite command."

Require-Pattern $supervisor 'control_race_assist_entry_pose_required' `
    "Low-pose readiness must be an entry gate, not an operational self-fault."
Require-Pattern $supervisor '(?s)RACE_ASSIST_DISABLED.*RACE_ASSIST_LOW_RACE.*RACE_ASSIST_ARMED' `
    "Only disabled/low-race/armed states may require neutral-pose readiness."

Require-Pattern $chassis 'control_race_assist_update' `
    "Chassis must run the race supervisor."
Require-Pattern $chassis 'control_leg_set_race_assist_request' `
    "Chassis must hand the signed request to the leg controller."
Require-Pattern $chassis 'LEG_MOTION_RACE_ASSIST' `
    "Race leg motion must bypass only the generic transition limit."
Require-Pattern $chassis 'APP_RACE_ASSIST_PITCH_OFFSET_LIMIT_DEG' `
    "Race mode must use the initial 7 degree virtual-pitch cap."
Require-Pattern $chassis 'race_input\.leg_path_fault\s*=\s*\(\(LEG_MOTION_RACE_FAULT_HOLD == leg->motion_state\)\s*\|\|\s*\(\(LEG_MOTION_RACE_ASSIST == leg->motion_state\)\s*&&\s*\(APP_FALSE == leg->race_path_valid\)\)\)' `
    "An unplanned non-race pose must not fault race assist before the entry request."
Require-Pattern $balance 'race_balance_limit_rpm' `
    "Balance must consume the runtime race cap."
Require-Pattern $balance 'LEG_MOTION_RACE_FAULT_HOLD' `
    "Race fault hold must preserve balance while the target ramps down."
Require-Pattern $balance 'LEG_MOTION_RACE_ASSIST' `
    "Race assist must select the low-pose gain schedule."
Require-Pattern $balance '(?s)LEG_MOTION_RACE_FAULT_HOLD.*held_command_valid' `
    "Balance may continue race fault deceleration only with a validated held command."
Require-Pattern $balance 'legacy_stance_norm\s*=\s*0\.0f' `
    "Race pose must use the low-pose gain schedule rather than a stale legacy scalar."
Require-Pattern $chassis 'control_chassis_output\.race_balance_limit_rpm\s*=\s*APP_BALANCE_DEFAULT_RUNTIME_RPM_LIMIT' `
    "Disabled or unhealthy chassis output must publish the default balance cap."
Require-Pattern $chassis 'forward_before_ramp_rpm\s*=\s*control_chassis_cmd\.actual_forward_rpm' `
    "Race fault deceleration must retain the pre-ramp forward target."
Require-Pattern $chassis '(?s)RACE_ASSIST_RECENTER == race_output->state.*race_output->forward_limit_rpm' `
    "BRA0 recenter must consume the supervisor 200 RPM cap instead of legacy 220 RPM."
Require-Pattern $chassis '(?s)RACE_ASSIST_FAULT_HOLD == race_output->state.*target_turn_dps\s*=\s*0\.0f' `
    "Race fault hold must not increase turn authority."
Reject-Pattern $hostCommand 'control_leg_set_xy[\s\S]{0,400}control_leg_set_race_assist_request' `
    "Manual LXY must not become the moving assist path."
Require-Pattern $hostCommand "'B' == line\[0\].*'R' == line\[1\].*'A' == line\[2\]" `
    "BRA command parser missing."
Require-Pattern $hostCommand 'control_chassis_set_race_assist_level' `
    "BRA must call the level setter."
Require-Pattern $hostCommand 'static uint8 host_command_parse_race_assist_level' `
    "BRA must use a dedicated lexical level parser."
Require-Pattern $hostCommand 'host_command_parse_race_assist_level\(&line\[4\], &race_level\)' `
    "BRA must validate the lexical level before calling the setter."
Require-Pattern $hostCommand "host_command_parse_race_assist_level[\s\S]{0,300}\('\\0' != text\[1\]\)[\s\S]{0,200}\('0' > text\[0\]\)[\s\S]{0,200}\('4' < text\[0\]\)" `
    "BRA lexical parser must accept exactly one digit from 0 through 4."
Reject-Pattern $hostCommand "'A' == line\[2\][\s\S]{0,300}host_command_parse_number\(&line\[4\], &value\)" `
    "BRA must not use float equality to recognize an integer level."
Require-Pattern $hostCommand "'B' == line\[0\].*'R' == line\[1\].*'G' == line\[2\]" `
    "BRG command parser missing."
Require-Pattern $hostCommand 'control_chassis_set_race_assist_gains' `
    "BRG must call the bounded gain setter."
Require-Pattern $hostCommand 'host_command_match_stop\(line\)[\s\S]*control_chassis_set_race_assist_level\(0U' `
    "STOP must disarm race assist."

$raceFaultStart = (Get-Content -Raw $chassis).IndexOf("if(RACE_ASSIST_FAULT_HOLD == race_output->state)")
$raceFaultEnd = (Get-Content -Raw $chassis).IndexOf("if(APP_TRUE == race_output->enable)", $raceFaultStart)
if(($raceFaultStart -lt 0) -or ($raceFaultEnd -lt 0)) {
    throw "Unable to isolate race fault deceleration policy."
}
$raceFaultPolicy = (Get-Content -Raw $chassis).Substring($raceFaultStart, $raceFaultEnd - $raceFaultStart)
if($raceFaultPolicy -notmatch 'control_chassis_cmd\.target_forward_rpm\s*=\s*0\.0f') {
    throw "Race fault must remove forward authority in the detection cycle."
}
if($raceFaultPolicy -match 'control_chassis_cmd\.actual_forward_rpm\s*=\s*0\.0f') {
    throw "Race fault must not hard-zero the ramped forward target."
}
if($raceFaultPolicy -notmatch 'control_chassis_ramp_toward\(\s*forward_before_ramp_rpm,\s*0\.0f,\s*forward_max_delta\s*\)') {
    throw "Race fault must ramp the forward target down rather than hard-zero it."
}
if($raceFaultPolicy -notmatch 'APP_RACE_ASSIST_PITCH_OFFSET_LIMIT_DEG') {
    throw "Race fault deceleration must retain the 7 degree virtual-pitch cap."
}

$xyStart = (Get-Content -Raw $source).IndexOf("uint8 control_leg_set_xy")
$nextApi = (Get-Content -Raw $source).IndexOf("uint8 control_leg_set_race_assist_request", $xyStart)
if(($xyStart -lt 0) -or ($nextApi -lt 0)) {
    throw "Unable to isolate LXY API."
}
$xyApi = (Get-Content -Raw $source).Substring($xyStart, $nextApi - $xyStart)
if($xyApi -notmatch 'LEG_MODE_IK_VALIDATE') {
    throw "Manual LXY must remain stopped-only IK validation mode."
}
if($xyApi -match 'LEG_MODE_RACE_ASSIST') {
    throw "Manual LXY must not enter the drive-allowed race mode."
}
$disableStart = (Get-Content -Raw $source).IndexOf("void control_leg_disable_race_assist")
$nextDisableApi = (Get-Content -Raw $source).IndexOf("const leg_diag_struct *control_leg_get_diag", $disableStart)
if(($disableStart -lt 0) -or ($nextDisableApi -lt 0)) {
    throw "Unable to isolate race disable API."
}
$disableApi = (Get-Content -Raw $source).Substring($disableStart, $nextDisableApi - $disableStart)
if($disableApi -match 'control_leg_write_safe_angles|legacy_safe_support_units|control_leg_set_ik_reference') {
    throw "Race disable must recenter at u=0, not substitute the 90-degree or legacy reference."
}

function Write-ControllerHarness {
    param([string]$Path)

    @'
#include "app_state.h"
#include "app_safety.h"
#include "actuator_servo.h"
#include "leg_kinematics.h"

static actuator_servo_diag_struct host_diag;
uint32 host_ik_solve_count = 0U;

uint8 __real_leg_kinematics_solve(uint8 right_side,
                                  float x_mm,
                                  float y_mm,
                                  const leg_ik_result_struct *previous,
                                  leg_ik_result_struct *result);

uint8 __wrap_leg_kinematics_solve(uint8 right_side,
                                  float x_mm,
                                  float y_mm,
                                  const leg_ik_result_struct *previous,
                                  leg_ik_result_struct *result)
{
    host_ik_solve_count++;
    return __real_leg_kinematics_solve(right_side, x_mm, y_mm, previous, result);
}

void app_state_init(void) {}
void app_state_set(app_run_state_enum state) { (void)state; }
app_run_state_enum app_state_get(void) { return APP_STATE_STANDBY; }
uint8 app_state_is_run_enabled(void) { return APP_TRUE; }

void app_safety_init(void) {}
void app_safety_update(uint32 now_ms) { (void)now_ms; }
uint8 app_safety_is_fault(void) { return APP_FALSE; }
void app_safety_force_fault(void) {}

void actuator_servo_init(void) {}
void actuator_servo_enable(void) {}
void actuator_servo_disable(void) {}
uint32 actuator_servo_angle_to_duty(float angle_deg) { (void)angle_deg; return 0U; }
float actuator_servo_get_current_angle(uint8 index) { return host_diag.output_deg[index]; }
void actuator_servo_publish_cmd(const servo_cmd_struct *cmd,
                                float speed_limit_dps,
                                uint8 direct_bypass)
{
    uint8 i;
    (void)speed_limit_dps;
    (void)direct_bypass;
    for(i = 0U; i < 4U; i++)
    {
        host_diag.target_deg[i] = cmd->angle_deg[i];
        host_diag.output_deg[i] = cmd->angle_deg[i];
        host_diag.filtered_deg[i] = cmd->angle_deg[i];
    }
    host_diag.settled = APP_TRUE;
}
void actuator_servo_tick_300hz(void) {}
void actuator_servo_get_diag(actuator_servo_diag_struct *diag)
{
    *diag = host_diag;
}
uint8 actuator_servo_is_settled(void) { return APP_TRUE; }
uint32 actuator_servo_get_tick_count(void) { return 0U; }
'@ | Set-Content (Join-Path $Path "host_support.c") -NoNewline

    @'
#include <math.h>
#include <stdio.h>

#include "control_leg.h"

int main(void)
{
    const leg_diag_struct *diag;
    uint32 now_ms;
    uint32 solve_count_after_first_preflight;
    uint8 request_index;
    uint8 saw_zero_endpoint = APP_FALSE;

    extern uint32 host_ik_solve_count;

    control_leg_init();
    if(APP_TRUE != control_leg_set_ik_reference(0U))
    {
        fprintf(stderr, "LIKREF setup rejected\n");
        return 1;
    }
    for(now_ms = 0U; now_ms <= 1000U; now_ms++)
    {
        control_leg_update(now_ms);
    }
    diag = control_leg_get_diag();
    if((LEG_MODE_IK_REFERENCE != (leg_mode_enum)diag->mode) ||
       (APP_TRUE == control_leg_set_race_assist_request(0.0f, 2.0f, 2.0f, 1001U)) ||
       (LEG_MODE_IK_REFERENCE != (leg_mode_enum)control_leg_get_diag()->mode))
    {
        fprintf(stderr, "LIKREF race entry was not fail-closed\n");
        return 2;
    }
    if(APP_TRUE != control_leg_set_xy(-18.0f, 47.3567f, 1002U))
    {
        fprintf(stderr, "LXY setup rejected\n");
        return 3;
    }
    for(now_ms = 1002U; now_ms <= 3000U; now_ms++)
    {
        control_leg_update(now_ms);
    }
    diag = control_leg_get_diag();
    if((LEG_MODE_IK_VALIDATE != (leg_mode_enum)diag->mode) ||
       (APP_TRUE == control_leg_set_race_assist_request(0.0f, 2.0f, 2.0f, 3001U)) ||
       (LEG_MODE_IK_VALIDATE != (leg_mode_enum)control_leg_get_diag()->mode))
    {
        fprintf(stderr, "completed LXY race entry was not fail-closed\n");
        return 4;
    }
    if(APP_TRUE != control_leg_set_xy(-18.13f, 25.78f, 3002U))
    {
        fprintf(stderr, "radially-outside LXY setup rejected\n");
        return 5;
    }
    for(now_ms = 3002U; now_ms <= 5000U; now_ms++)
    {
        control_leg_update(now_ms);
    }
    diag = control_leg_get_diag();
    if((LEG_MODE_IK_VALIDATE != (leg_mode_enum)diag->mode) ||
       (APP_TRUE == control_leg_set_race_assist_request(0.0f, 2.0f, 2.0f, 5001U)) ||
       (LEG_MODE_IK_VALIDATE != (leg_mode_enum)control_leg_get_diag()->mode))
    {
        fprintf(stderr, "radially-outside low-race entry was not fail-closed\n");
        return 6;
    }
    if(APP_TRUE != control_leg_set_xy(-18.33f, 25.58f, 5002U))
    {
        fprintf(stderr, "radially-inside LXY setup rejected\n");
        return 7;
    }
    for(now_ms = 5002U; now_ms <= 7000U; now_ms++)
    {
        control_leg_update(now_ms);
    }
    if(APP_TRUE != control_leg_set_race_assist_request(0.01f, 2.0f, 2.0f, 7001U))
    {
        fprintf(stderr, "radially-inside low-race entry rejected\n");
        return 8;
    }
    solve_count_after_first_preflight = host_ik_solve_count;
    for(request_index = 0U; request_index < 20U; request_index++)
    {
        if(APP_TRUE != control_leg_set_race_assist_request(0.01f,
                                                           2.0f,
                                                           2.0f,
                                                           7002U + request_index))
        {
            fprintf(stderr, "steady race request rejected at repeat %u\n", request_index);
            return 13;
        }
    }
    if(host_ik_solve_count != solve_count_after_first_preflight)
    {
        fprintf(stderr,
                "steady request repeated path preflight: solve count %lu -> %lu\n",
                (unsigned long)solve_count_after_first_preflight,
                (unsigned long)host_ik_solve_count);
        return 14;
    }
    for(now_ms = 7001U; now_ms <= 8000U; now_ms++)
    {
        control_leg_update(now_ms);
    }
    diag = control_leg_get_diag();
    if((LEG_MODE_RACE_ASSIST != (leg_mode_enum)diag->mode) ||
       (0.005f > diag->race_assist_actual))
    {
        fprintf(stderr, "positive race setup did not settle: %.6f\n", diag->race_assist_actual);
        return 9;
    }
    if(APP_TRUE != control_leg_set_race_assist_request(-1.0f, 2.0f, 2.0f, 8001U))
    {
        fprintf(stderr, "negative race request rejected\n");
        return 10;
    }
    for(now_ms = 8001U; now_ms <= 10000U; now_ms++)
    {
        control_leg_update(now_ms);
        diag = control_leg_get_diag();
        if((0.0001f >= fabsf(diag->race_assist_actual)) &&
           (0.01f >= diag->servo_s7_progress))
        {
            saw_zero_endpoint = APP_TRUE;
        }
        if(-0.001f > diag->race_assist_actual)
        {
            if(APP_FALSE == saw_zero_endpoint)
            {
                fprintf(stderr, "opposite-sign request crossed zero without a zero endpoint\n");
                return 11;
            }
            return 0;
        }
    }
    fprintf(stderr, "race reversal did not reach negative request\n");
    return 12;
}
'@ | Set-Content (Join-Path $Path "race_leg_controller.c") -NoNewline
}

$controllerTempPath = Join-Path ([System.IO.Path]::GetTempPath()) ("race-leg-controller-" + [Guid]::NewGuid().ToString())
$controllerOriginalPath = $null
New-Item -ItemType Directory -Path $controllerTempPath | Out-Null
try {
    @'
#ifndef _zf_common_headfile_h_
#define _zf_common_headfile_h_
#include <stddef.h>
#include <stdint.h>
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef int16_t int16;
#endif
'@ | Set-Content (Join-Path $controllerTempPath "zf_common_headfile.h") -NoNewline
    Write-ControllerHarness $controllerTempPath
    Copy-Item "project/code/app_types.h" $controllerTempPath
    Copy-Item "project/code/app_config.h" $controllerTempPath
    Copy-Item "project/code/app_state.h" $controllerTempPath
    Copy-Item "project/code/app_safety.h" $controllerTempPath
    Copy-Item "project/code/actuator_servo.h" $controllerTempPath
    Copy-Item "project/code/control_leg.h" $controllerTempPath
    Copy-Item "project/code/control_leg.c" $controllerTempPath
    Copy-Item "project/code/leg_config.h" $controllerTempPath
    Copy-Item "project/code/leg_config.c" $controllerTempPath
    Copy-Item "project/code/leg_kinematics.h" $controllerTempPath
    Copy-Item "project/code/leg_kinematics.c" $controllerTempPath

    $compiler = (Get-Command gcc -ErrorAction Stop).Source
    $compilerDirectory = Split-Path $compiler
    $controllerOriginalPath = $env:PATH
    $env:PATH = $compilerDirectory + [System.IO.Path]::PathSeparator + $env:PATH
    $wcetObject = Join-Path $controllerTempPath "control_leg_wcet_enabled.o"
    $wcetCompile = & $compiler -std=c99 -Wall -Wextra -Werror `
        -DAPP_RACE_ASSIST_PREFLIGHT_WCET_ENABLE=1 `
        -I $controllerTempPath -c (Join-Path $controllerTempPath "control_leg.c") `
        -o $wcetObject 2>&1
    if(0 -ne $LASTEXITCODE) {
        $wcetCompile | Write-Host
        throw "Default-off CM7_0 DWT preflight hook did not compile when enabled."
    }
    $controllerBinary = Join-Path $controllerTempPath "race_leg_controller.exe"
    $controllerErrorAction = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    $controllerCompile = & $compiler -std=c99 -Wall -Wextra -Werror -I $controllerTempPath `
        (Join-Path $controllerTempPath "leg_config.c") `
        (Join-Path $controllerTempPath "leg_kinematics.c") `
        (Join-Path $controllerTempPath "control_leg.c") `
        (Join-Path $controllerTempPath "host_support.c") `
        (Join-Path $controllerTempPath "race_leg_controller.c") `
        '-Wl,--wrap=leg_kinematics_solve' -lm -o $controllerBinary 2>&1
    $controllerCompileExit = $LASTEXITCODE
    $ErrorActionPreference = $controllerErrorAction
    if(0 -ne $controllerCompileExit) {
        $controllerCompile | Write-Host
        throw "Unable to compile race leg controller harness."
    }
    & $controllerBinary
    if(0 -ne $LASTEXITCODE) {
        throw "Race leg controller reversal harness failed."
    }
}
finally {
    if($null -ne $controllerOriginalPath) {
        $env:PATH = $controllerOriginalPath
    }
    if(Test-Path $controllerTempPath) {
        Remove-Item -Recurse -Force $controllerTempPath
    }
}

Write-Host "low-race leg assist static checks passed"

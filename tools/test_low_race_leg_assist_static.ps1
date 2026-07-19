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
$config = "project/code/app_config.h"
$balance = "project/code/control_balance.c"
$hostCommand = "project/code/host_command.c"

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
Require-Pattern $config 'APP_BALANCE_RPM_LIMIT\s+\(460\.0f\)' `
    "Hard balance ceiling must leave 400 RPM correction reserve."
Require-Pattern $config 'APP_BALANCE_DEFAULT_RUNTIME_RPM_LIMIT\s+\(300\.0f\)' `
    "Non-assisted balance must retain the validated 300 RPM runtime cap."
Require-Pattern $types 'balance_output_limit_rpm' `
    "Active balance cap must be observable."

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
Require-Pattern $balance 'legacy_stance_norm\s*=\s*0\.0f' `
    "Race pose must use the low-pose gain schedule rather than a stale legacy scalar."
Require-Pattern $chassis 'control_chassis_output\.race_balance_limit_rpm\s*=\s*APP_BALANCE_DEFAULT_RUNTIME_RPM_LIMIT' `
    "Disabled or unhealthy chassis output must publish the default balance cap."
Require-Pattern $chassis 'forward_before_ramp_rpm\s*=\s*control_chassis_cmd\.actual_forward_rpm' `
    "Race fault deceleration must retain the pre-ramp forward target."
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

static actuator_servo_diag_struct host_diag;

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
    uint8 saw_zero_endpoint = APP_FALSE;

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
    $controllerBinary = Join-Path $controllerTempPath "race_leg_controller.exe"
    $controllerErrorAction = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    $controllerCompile = & $compiler -std=c99 -Wall -Wextra -Werror -I $controllerTempPath `
        (Join-Path $controllerTempPath "leg_config.c") `
        (Join-Path $controllerTempPath "leg_kinematics.c") `
        (Join-Path $controllerTempPath "control_leg.c") `
        (Join-Path $controllerTempPath "host_support.c") `
        (Join-Path $controllerTempPath "race_leg_controller.c") `
        -lm -o $controllerBinary 2>&1
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

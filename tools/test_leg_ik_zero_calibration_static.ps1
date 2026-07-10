$ErrorActionPreference = "Stop"

function Assert-Contains {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if(-not (Test-Path $Path)) {
        throw ("Missing file: {0}" -f $Path)
    }
    if((Get-Content $Path -Raw) -notmatch $Pattern) {
        throw $Message
    }
}

function Write-HostHeaders {
    param([string]$Path)

    @'
#ifndef _zf_common_headfile_h_
#define _zf_common_headfile_h_
#include <stddef.h>
typedef unsigned char uint8;
typedef unsigned int uint32;
typedef enum { APP_FALSE = 0, APP_TRUE = 1 } app_bool_enum;
#endif
'@ | Set-Content (Join-Path $Path "zf_common_headfile.h") -NoNewline

    @'
#ifndef _app_types_h_
#define _app_types_h_
#include "zf_common_headfile.h"
#endif
'@ | Set-Content (Join-Path $Path "app_types.h") -NoNewline

    @'
#ifndef _app_config_h_
#define _app_config_h_
#define APP_BALANCE_FINITE_ABS_LIMIT (100000.0f)
#endif
'@ | Set-Content (Join-Path $Path "app_config.h") -NoNewline
}

function Write-NumericHarness {
    param([string]$Path)

    @'
#include "leg_kinematics.h"
#include "leg_config.h"
#include <math.h>

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
'@ | Set-Content (Join-Path $Path "test_leg_ik_zero_calibration.c") -NoNewline
}

Assert-Contains "project/code/leg_config.h" "ik_offset_deg" "Missing per-servo IK offset configuration."
Assert-Contains "project/code/leg_config.h" "validate_x_min_mm" "Missing restricted IK validation workspace configuration."
Assert-Contains "project/code/leg_kinematics.h" "leg_kinematics_map_reference_pose" "Missing reference-pose mapping API."
Assert-Contains "project/code/leg_kinematics.h" "leg_kinematics_map_target_pose" "Missing target-pose mapping API."
Assert-Contains "project/code/control_leg.h" "control_leg_set_ik_reference" "Missing reference-pose controller API."
Assert-Contains "project/code/control_leg.h" "control_leg_set_xy" "Missing XY controller API."
Assert-Contains "project/code/control_leg.c" "LEG_MODE_IK_REFERENCE" "Missing safe IK reference mode."
Assert-Contains "project/code/control_leg.c" "LEG_MODE_IK_VALIDATE" "Missing restricted XY validation mode."
Assert-Contains "project/code/control_leg.c" "leg_kinematics_map_target_pose" "XY mode must use calibrated mapping."

$tempPath = Join-Path ([System.IO.Path]::GetTempPath()) ("leg-ik-zero-" + [Guid]::NewGuid().ToString())
$originalPath = $null
New-Item -ItemType Directory -Path $tempPath | Out-Null
try {
    Write-HostHeaders $tempPath
    Write-NumericHarness $tempPath
    Copy-Item "project/code/leg_config.h" (Join-Path $tempPath "leg_config.h")
    Copy-Item "project/code/leg_config.c" (Join-Path $tempPath "leg_config.c")
    Copy-Item "project/code/leg_kinematics.h" (Join-Path $tempPath "leg_kinematics.h")
    Copy-Item "project/code/leg_kinematics.c" (Join-Path $tempPath "leg_kinematics.c")

    $compiler = (Get-Command gcc -ErrorAction Stop).Source
    $compilerDirectory = Split-Path $compiler
    $originalPath = $env:PATH
    $env:PATH = $compilerDirectory + [System.IO.Path]::PathSeparator + $env:PATH
    $binary = Join-Path $tempPath "test_leg_ik_zero_calibration.exe"
    $compileOutput = & $compiler -std=c99 -Wall -Werror -I $tempPath `
        (Join-Path $tempPath "leg_config.c") `
        (Join-Path $tempPath "leg_kinematics.c") `
        (Join-Path $tempPath "test_leg_ik_zero_calibration.c") `
        -lm -o $binary 2>&1
    if(0 -ne $LASTEXITCODE) {
        $compileOutput | Write-Host
        throw ("Unable to compile zero-calibration harness (exit {0})." -f $LASTEXITCODE)
    }
    & $binary
    if(0 -ne $LASTEXITCODE) {
        throw ("Zero-calibration numeric harness failed (exit {0})." -f $LASTEXITCODE)
    }
}
finally {
    if($null -ne $originalPath) {
        $env:PATH = $originalPath
    }
    if(Test-Path $tempPath) {
        Remove-Item -Recurse -Force $tempPath
    }
}

Write-Host "leg IK zero calibration static checks passed"

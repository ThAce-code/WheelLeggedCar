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
#include <stdio.h>

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

int main(void)
{
    leg_ik_result_struct left_ref = {0};
    leg_ik_result_struct right_ref = {0};
    leg_ik_result_struct left_target = {0};
    leg_ik_result_struct right_target = {0};
    float reference_cmd[LEG_SERVO_COUNT];
    float target_cmd[LEG_SERVO_COUNT];
    static const float physical_points[][2] = {
        {-18.0000f, 47.3567f},
        {-23.5000f, 47.3567f},
        {-20.7667f, 44.0000f},
        {-20.7667f, 51.0000f}
    };
    unsigned int i;

    if((APP_TRUE != leg_kinematics_solve(APP_FALSE,
                                         -20.766667f, 47.356667f,
                                         NULL, &left_ref)) ||
       (APP_TRUE != leg_kinematics_solve(APP_TRUE,
                                         -20.766667f, 47.356667f,
                                         NULL, &right_ref)) ||
       (APP_TRUE != leg_kinematics_map_reference_pose(&left_ref, &right_ref, reference_cmd)))
    {
        return 1;
    }
    if((fabsf(reference_cmd[LEG_SERVO_FL] - 90.0f) > 0.001f) ||
       (fabsf(reference_cmd[LEG_SERVO_FR] - 90.0f) > 0.001f) ||
       (fabsf(reference_cmd[LEG_SERVO_RL] - 90.0f) > 0.001f) ||
       (fabsf(reference_cmd[LEG_SERVO_RR] - 90.0f) > 0.001f))
    {
        return 2;
    }
    if((APP_TRUE != leg_kinematics_solve(APP_FALSE, -18.0f, 47.3567f, &left_ref, &left_target)) ||
       (APP_TRUE != leg_kinematics_solve(APP_TRUE, -18.0f, 47.3567f, &right_ref, &right_target)) ||
       (APP_TRUE != leg_kinematics_map_target_pose(&left_ref, &right_ref, &left_target, &right_target, target_cmd)))
    {
        return 3;
    }
    for(i = 0U; i < (sizeof(physical_points) / sizeof(physical_points[0])); i++)
    {
        if((APP_TRUE != leg_kinematics_solve(APP_FALSE, physical_points[i][0], physical_points[i][1], &left_ref, &left_target)) ||
           (APP_TRUE != leg_kinematics_solve(APP_TRUE, physical_points[i][0], physical_points[i][1], &right_ref, &right_target)) ||
           (APP_TRUE != leg_kinematics_map_target_pose(&left_ref, &right_ref, &left_target, &right_target, target_cmd)))
        {
            return (int)(10U + i);
        }
    }
    if(0 != check_model_reachable_commands())
    {
        return 30;
    }
    if((APP_TRUE == leg_kinematics_target_valid(NAN, 47.3567f)) ||
       (APP_TRUE == leg_kinematics_target_valid(INFINITY, 47.3567f)) ||
       (APP_TRUE == leg_kinematics_target_valid(1000.0f, 1000.0f)))
    {
        return 31;
    }
    return 0;
}
'@ | Set-Content (Join-Path $Path "test_leg_ik_zero_calibration.c") -NoNewline
}

Assert-Contains "project/code/leg_config.h" "ik_offset_deg" "Missing per-servo IK offset configuration."
Assert-Contains "project/code/leg_config.c" '\{0,\s*90\.0f,\s*90\.0f' "FL safe and neutral must be literal 90 degrees."
Assert-Contains "project/code/leg_config.c" '\{1,\s*90\.0f,\s*90\.0f' "FR safe and neutral must be literal 90 degrees."
Assert-Contains "project/code/leg_config.c" '\{2,\s*90\.0f,\s*90\.0f' "RL safe and neutral must be literal 90 degrees."
Assert-Contains "project/code/leg_config.c" '\{3,\s*90\.0f,\s*90\.0f' "RR safe and neutral must be literal 90 degrees."
Assert-Contains "project/code/leg_config.c" '\.physical_reference_x_mm\s*=\s*-20\.766667f' "LIKREF physical X missing."
Assert-Contains "project/code/leg_config.c" '\.physical_reference_y_mm\s*=\s*47\.356667f' "LIKREF physical Y missing."
Assert-Contains "project/code/leg_kinematics.h" "leg_kinematics_target_valid" "Missing physical target validation API."
Assert-Contains "project/code/control_leg.c" "leg_kinematics_target_valid" "LXY validation must enforce model reachability."
Assert-Contains "project/code/leg_kinematics.h" "leg_kinematics_map_reference_pose" "Missing reference-pose mapping API."
Assert-Contains "project/code/leg_kinematics.h" "leg_kinematics_map_target_pose" "Missing target-pose mapping API."
Assert-Contains "project/code/control_leg.h" "control_leg_set_ik_reference" "Missing reference-pose controller API."
Assert-Contains "project/code/control_leg.h" "control_leg_set_xy" "Missing XY controller API."
Assert-Contains "project/code/control_leg.c" "LEG_MODE_IK_REFERENCE" "Missing safe IK reference mode."
Assert-Contains "project/code/control_leg.c" "LEG_MODE_IK_VALIDATE" "Missing restricted XY validation mode."
Assert-Contains "project/code/control_leg.c" "leg_kinematics_map_target_pose" "XY mode must use calibrated mapping."
Assert-Contains "project/code/host_command.c" "(?s)'L' == line\[0\].*'I' == line\[1\].*'K' == line\[2\].*'R' == line\[3\]" "Missing LIKREF command parser."
Assert-Contains "project/code/host_command.c" "'L' == line\[0\].*'X' == line\[1\].*'Y' == line\[2\]" "Missing LXY command parser."
Assert-Contains "project/code/host_command.c" "control_leg_set_ik_reference" "LIKREF must enter the reference controller mode."
Assert-Contains "project/code/host_command.c" "control_leg_set_xy" "LXY must enter the restricted XY controller mode."
Assert-Contains "docs/leg-ik-zero-calibration-hardware-test.md" "LIKREF" "Hardware procedure must include LIKREF."
Assert-Contains "docs/leg-ik-zero-calibration-hardware-test.md" "BODY_WHEEL" "Hardware procedure must name the public physical frame."
Assert-Contains "docs/leg-ik-zero-calibration-hardware-test.md" "LIKREF = \(-20\.766667, 47\.356667\)" "Hardware procedure must record the exact P0 reference."
Assert-Contains "docs/leg-ik-zero-calibration-hardware-test.md" "absolute.*BODY_WHEEL.*millimetres" "LXY must be documented as an absolute physical command."
Assert-Contains "docs/leg-ik-zero-calibration-hardware-test.md" "model-reachable" "Hardware procedure must define model reachability."
Assert-Contains "docs/leg-ik-zero-calibration-hardware-test.md" "0.02" "Hardware procedure must state the minimum IK margin."
Assert-Contains "docs/leg-ik-zero-calibration-hardware-test.md" "servo limits" "Hardware procedure must require mapped servo limits."
Assert-Contains "docs/leg-ik-zero-calibration-hardware-test.md" "LXY,0,55.*must be rejected" "The legacy model-space target must be documented as rejected."
Assert-Contains "docs/leg-ik-zero-calibration-hardware-test.md" "motor-disabled" "Hardware procedure must require motor-disabled acceptance."
Assert-Contains "docs/leg-ik-zero-calibration-hardware-test.md" "Target X.*Measured X.*Error X.*Pose-status flags.*Servo outputs" "Hardware procedure must record physical-coordinate acceptance evidence."
Assert-Contains "docs/leg-ik-zero-calibration-hardware-test.md" "right leg[\s\S]*measured independently" "Hardware procedure must gate right-leg provenance on independent measurement."

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

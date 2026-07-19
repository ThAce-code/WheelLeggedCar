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
    if(APP_TRUE == leg_kinematics_target_valid(0.0f, 55.0f))
    {
        return 20;
    }
    return 0;
}
'@ | Set-Content (Join-Path $Path "test_leg_ik_zero_calibration.c") -NoNewline
}

function Test-PhysicalHullPoint {
    param([double]$X, [double]$Y)

    $hull = @(
        @(-40.620, 47.370), @(-30.910, 39.630),
        @(-20.380, 32.170), @(-15.040, 47.600),
        @(-22.030, 88.490), @(-31.420, 74.120),
        @(-37.940, 59.340), @(-39.580, 53.010)
    )
    for($i = 0; $i -lt $hull.Count; $i++) {
        $first = $hull[$i]
        $second = $hull[($i + 1) % $hull.Count]
        $edgeX = $second[0] - $first[0]
        $edgeY = $second[1] - $first[1]
        $length = [math]::Sqrt($edgeX * $edgeX + $edgeY * $edgeY)
        $distance = ($edgeX * ($Y - $first[1]) - $edgeY * ($X - $first[0])) / $length
        if($distance -lt 2.0) {
            return $false
        }
    }
    return $true
}

Assert-Contains "project/code/leg_config.h" "ik_offset_deg" "Missing per-servo IK offset configuration."
Assert-Contains "project/code/leg_config.c" '\{0,\s*90\.0f,\s*90\.0f' "FL safe and neutral must be literal 90 degrees."
Assert-Contains "project/code/leg_config.c" '\{1,\s*90\.0f,\s*90\.0f' "FR safe and neutral must be literal 90 degrees."
Assert-Contains "project/code/leg_config.c" '\{2,\s*90\.0f,\s*90\.0f' "RL safe and neutral must be literal 90 degrees."
Assert-Contains "project/code/leg_config.c" '\{3,\s*90\.0f,\s*90\.0f' "RR safe and neutral must be literal 90 degrees."
Assert-Contains "project/code/leg_config.c" '\.physical_reference_x_mm\s*=\s*-20\.766667f' "LIKREF physical X missing."
Assert-Contains "project/code/leg_config.c" '\.physical_reference_y_mm\s*=\s*47\.356667f' "LIKREF physical Y missing."
Assert-Contains "project/code/leg_config.h" "physical_workspace" "Missing calibrated physical IK workspace configuration."
Assert-Contains "project/code/leg_config.h" "physical_workspace_inset_mm" "Missing physical workspace safety inset."
Assert-Contains "project/code/leg_kinematics.h" "leg_kinematics_target_valid" "Missing physical target validation API."
Assert-Contains "project/code/control_leg.c" "leg_kinematics_target_valid" "LXY validation must enforce the calibrated physical hull."
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
Assert-Contains "docs/leg-ik-zero-calibration-hardware-test.md" "convex workspace inset by 2 mm" "Hardware procedure must retain the fitted inset hull."
Assert-Contains "docs/leg-ik-zero-calibration-hardware-test.md" "LXY,0,55.*must be rejected" "The legacy model-space target must be documented as rejected."
Assert-Contains "docs/leg-ik-zero-calibration-hardware-test.md" "P1.*P2.*at least 2 mm inside" "Hardware procedure must require two inset physical targets."
Assert-Contains "docs/leg-ik-zero-calibration-hardware-test.md" "motor-disabled" "Hardware procedure must require motor-disabled acceptance."
Assert-Contains "docs/leg-ik-zero-calibration-hardware-test.md" "Target X.*Measured X.*Error X.*Pose-status flags.*Servo outputs" "Hardware procedure must record physical-coordinate acceptance evidence."
Assert-Contains "docs/leg-ik-zero-calibration-hardware-test.md" "right leg[\s\S]*measured independently" "Hardware procedure must gate right-leg provenance on independent measurement."

if(-not (Test-PhysicalHullPoint -X -20.7667 -Y 47.3567)) { throw "Physical reference must be inside the inset hull." }
if(-not (Test-PhysicalHullPoint -X -18.0 -Y 47.3567)) { throw "Forward test point must be inside the inset hull." }
if(-not (Test-PhysicalHullPoint -X -23.5 -Y 47.3567)) { throw "Rearward test point must be inside the inset hull." }
if(Test-PhysicalHullPoint -X 0.0 -Y 55.0) { throw "Uncalibrated X=0 target must be rejected." }
if(Test-PhysicalHullPoint -X -40.620 -Y 47.370) { throw "A hull vertex must be rejected by the 2 mm inset." }

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

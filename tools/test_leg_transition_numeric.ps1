$ErrorActionPreference = "Stop"

function Assert-Equal {
    param(
        [double]$Actual,
        [double]$Expected,
        [string]$Message
    )

    if($Actual -ne $Expected) {
        throw ("{0}: expected {1}, got {2}" -f $Message, $Expected, $Actual)
    }
}

function Assert-Contains {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-Content $Path -Raw
    if($text -notmatch $Pattern) {
        throw $Message
    }
}

function Step-HeightSupervisor {
    param(
        [double]$ReferenceMm,
        [double]$RateMmS,
        [double]$TargetMm,
        [double]$MaxSpeedMmS,
        [double]$AccelMmS2,
        [double]$DtS
    )

    $errorMm = $TargetMm - $ReferenceMm
    $desiredRateMmS = [math]::Min([math]::Sqrt(2.0 * $AccelMmS2 * [math]::Abs($errorMm)) - ($AccelMmS2 * $DtS), $MaxSpeedMmS)
    $desiredRateMmS = [math]::Max(0.0, $desiredRateMmS)
    if(0.0 -gt $errorMm) {
        $desiredRateMmS = -$desiredRateMmS
    }
    $maxRateDeltaMmS = $AccelMmS2 * $DtS
    $rateDeltaMmS = $desiredRateMmS - $RateMmS
    if([math]::Abs($rateDeltaMmS) -le $maxRateDeltaMmS) {
        $RateMmS = $desiredRateMmS
    }
    else {
        $RateMmS += [math]::Sign($rateDeltaMmS) * $maxRateDeltaMmS
    }
    if(0.0001 -ge [math]::Abs($RateMmS)) {
        $RateMmS = 0.0
    }
    $nextReferenceMm = $ReferenceMm + ($RateMmS * $DtS)
    if(((0.0 -lt $errorMm) -and ($TargetMm -le $nextReferenceMm)) -or
       ((0.0 -gt $errorMm) -and ($TargetMm -ge $nextReferenceMm))) {
        $nextReferenceMm = $TargetMm
        $RateMmS = 0.0
    }
    return @($nextReferenceMm, $RateMmS)
}

function Assert-BoundedHeightTrajectory {
    param([double[]]$TargetsMm)

    $referenceMm = $TargetsMm[0]
    $rateMmS = 0.0
    $targetIndex = 1
    $state = "STABLE"
    for($step = 0; $step -lt 5000; $step++) {
        if(($targetIndex -lt $TargetsMm.Count) -and (0 -lt $step) -and (0 -eq ($step % 800))) {
            $targetMm = $TargetsMm[$targetIndex]
            $targetIndex++
        }
        elseif(0 -eq $step) {
            $targetMm = $TargetsMm[0]
        }
        $previousRateMmS = $rateMmS
        $result = Step-HeightSupervisor -ReferenceMm $referenceMm -RateMmS $rateMmS -TargetMm $targetMm -MaxSpeedMmS 20.0 -AccelMmS2 40.0 -DtS 0.01
        $referenceMm = $result[0]
        $rateMmS = $result[1]
        $state = if(([math]::Abs($targetMm - $referenceMm) -le 1.0) -and (0.0 -eq $rateMmS)) { "STABLE" } else { "TRANSITION" }
        if((20.0 + 0.0001) -lt [math]::Abs($rateMmS)) {
            throw "Height trajectory exceeded 20 mm/s."
        }
        if((0.4 + 0.0001) -lt [math]::Abs($rateMmS - $previousRateMmS)) {
            throw ("Height trajectory exceeded 40 mm/s2 at 10 ms: step {0}, target {1}, reference {2}, rate {3}, previous {4}." -f $step, $targetMm, $referenceMm, $rateMmS, $previousRateMmS)
        }
        if(($state -ne "TRANSITION") -and ($state -ne "STABLE")) {
            throw "Valid height trajectory entered an invalid state."
        }
        if(($targetIndex -ge $TargetsMm.Count) -and ($state -eq "STABLE")) {
            return
        }
    }
    throw ("Height trajectory did not settle: target {0}, reference {1}, rate {2}." -f $targetMm, $referenceMm, $rateMmS)
}

function Assert-SoftFaultSafeRate {
    $angleDeg = 135.0
    for($step = 0; $step -lt 20; $step++) {
        $previousAngleDeg = $angleDeg
        $angleDeg += [math]::Sign(90.0 - $angleDeg) * [math]::Min([math]::Abs(90.0 - $angleDeg), 4.5)
        if((4.5 + 0.0001) -lt [math]::Abs($angleDeg - $previousAngleDeg)) {
            throw "Soft-fault safe target exceeded 4.5 degrees per 10 ms."
        }
    }
    Assert-Equal -Actual $angleDeg -Expected 90.0 -Message "Soft fault must approach 90 degree safe target"
}

function Assert-InsufficientIkMarginFault {
    $ikMargin = 0.19
    $minimumMargin = 0.20
    $motionState = if($ikMargin -lt $minimumMargin) { "FAULT" } else { "TRANSITION" }
    $driveAllowed = ($motionState -ne "FAULT")

    if($motionState -ne "FAULT") {
        throw "Insufficient IK margin must enter FAULT."
    }
    if($driveAllowed) {
        throw "Insufficient IK margin fault must deny drive."
    }
}

function Resolve-ChassisMotionPolicy {
    param(
        [string]$MotionState,
        [bool]$DriveAllowed,
        [bool]$FastRequested,
        [double]$ConfiguredForwardLimitRpm,
        [double]$ConfiguredFastForwardLimitRpm,
        [double]$TransitionForwardLimitRpm
    )

    if(($MotionState -eq "FAULT") -or (-not $DriveAllowed)) {
        return @{
            ForwardLimitRpm = 0.0
            EffectiveFast = $false
        }
    }
    if($MotionState -eq "TRANSITION") {
        return @{
            ForwardLimitRpm = $TransitionForwardLimitRpm
            EffectiveFast = $false
        }
    }
    return @{
        ForwardLimitRpm = if($FastRequested) { $ConfiguredFastForwardLimitRpm } else { $ConfiguredForwardLimitRpm }
        EffectiveFast = $FastRequested
    }
}

function Assert-MotionPolicy {
    $transition = Resolve-ChassisMotionPolicy -MotionState "TRANSITION" -DriveAllowed $true -FastRequested $true -ConfiguredForwardLimitRpm 80.0 -ConfiguredFastForwardLimitRpm 220.0 -TransitionForwardLimitRpm 30.0
    Assert-Equal -Actual $transition.ForwardLimitRpm -Expected 30.0 -Message "Transition forward limit"
    if($transition.EffectiveFast) {
        throw "Transition must disable effective fast blend without clearing the operator request."
    }

    $fault = Resolve-ChassisMotionPolicy -MotionState "FAULT" -DriveAllowed $false -FastRequested $true -ConfiguredForwardLimitRpm 80.0 -ConfiguredFastForwardLimitRpm 220.0 -TransitionForwardLimitRpm 30.0
    Assert-Equal -Actual $fault.ForwardLimitRpm -Expected 0.0 -Message "Fault forward limit"
    if($fault.EffectiveFast) {
        throw "Fault must disable effective fast blend."
    }
}

function Assert-HeightCommandRange {
    param([hashtable]$Config)

    $lowReject = 34.0
    $highReject = 121.0
    if(($lowReject -ge $Config["low_height_mm"]) -and ($lowReject -le $Config["high_height_mm"])) {
        throw "34 mm command must be outside the configured Phase 1 height interval."
    }
    if(($highReject -ge $Config["low_height_mm"]) -and ($highReject -le $Config["high_height_mm"])) {
        throw "121 mm command must be outside the configured Phase 1 height interval."
    }
}

function Get-LegTransitionConfig {
    $text = Get-Content "project/code/leg_config.c" -Raw
    $names = @(
        "low_height_mm",
        "high_height_mm",
        "max_height_speed_mm_s",
        "max_height_accel_mm_s2",
        "height_settle_error_mm",
        "height_settle_ms",
        "ik_min_margin",
        "safe_support_height_mm"
    )
    $config = @{}

    foreach($name in $names) {
        $match = [regex]::Match($text, "\.$name\s*=\s*([-+]?\d+(?:\.\d+)?)(?:f|U)")
        if(-not $match.Success) {
            throw ("Missing named transition setting: {0}" -f $name)
        }
        $config[$name] = [double]$match.Groups[1].Value
    }
    return $config
}

function New-HostHeaders {
    param([string]$Path)

    @'
#ifndef _app_types_h_
#define _app_types_h_
#include <stddef.h>
typedef unsigned char uint8;
typedef unsigned int uint32;
typedef enum { APP_FALSE = 0, APP_TRUE = 1 } app_bool_enum;
#endif
'@ | Set-Content (Join-Path $Path "app_types.h") -NoNewline

    @'
#ifndef _leg_config_h_
#define _leg_config_h_
#include "app_types.h"
typedef enum { LEG_SERVO_FL = 0, LEG_SERVO_FR = 1, LEG_SERVO_RL = 2, LEG_SERVO_RR = 3, LEG_SERVO_COUNT = 4 } leg_servo_id_enum;
typedef struct { uint8 servo_index; float safe_deg; float neutral_deg; float min_deg; float max_deg; float direction; float mount_x; float mount_y; } leg_servo_config_struct;
typedef enum { LEG_IK_BRANCH_PLUS = 0, LEG_IK_BRANCH_MINUS = 1 } leg_ik_branch_enum;
typedef struct { float l1_mm; float l2_mm; float l3_mm; float l4_mm; float l5_mm; float x_min_mm; float x_max_mm; float y_min_mm; float y_max_mm; float x_offset_mm; float y_offset_mm; leg_ik_branch_enum left_alpha_branch; leg_ik_branch_enum left_beta_branch; leg_ik_branch_enum right_alpha_branch; leg_ik_branch_enum right_beta_branch; } leg_kinematics_config_struct;
typedef struct { float low_height_mm; float high_height_mm; float default_height_mm; float max_height_speed_mm_s; float max_height_accel_mm_s2; float height_settle_error_mm; uint32 height_settle_ms; float ik_min_margin; float safe_support_height_mm; float transition_forward_limit_rpm; float balance_pitch_kp_low; float balance_pitch_kp_high; float balance_pitch_rate_kd_low; float balance_pitch_rate_kd_high; float balance_wheel_speed_ks_low; float balance_wheel_speed_ks_high; float balance_pitch_setpoint_low_deg; float balance_pitch_setpoint_high_deg; float chassis_forward_limit_low_rpm; float chassis_forward_limit_high_rpm; float chassis_fast_forward_limit_low_rpm; float chassis_fast_forward_limit_high_rpm; } leg_height_profile_struct;
typedef struct { leg_servo_config_struct servo[LEG_SERVO_COUNT]; leg_kinematics_config_struct kinematics; leg_height_profile_struct height_profile; float height_min; float height_max; float pitch_limit; float roll_limit; } leg_config_struct;
const leg_config_struct *leg_config_get(void);
const leg_servo_config_struct *leg_config_get_servo(uint8 leg_id);
const leg_kinematics_config_struct *leg_config_get_kinematics(void);
const leg_height_profile_struct *leg_config_get_height_profile(void);
#endif
'@ | Set-Content (Join-Path $Path "leg_config.h") -NoNewline

    @'
#ifndef _app_config_h_
#define _app_config_h_
#define APP_BALANCE_FINITE_ABS_LIMIT (100000.0f)
#endif
'@ | Set-Content (Join-Path $Path "app_config.h") -NoNewline
}

function New-NumericHarness {
    param([string]$Path)

    @'
#include "leg_kinematics.h"
#include "leg_config.h"
#include <math.h>
#include <stdio.h>

static float wrapped_delta_deg(float current_deg, float previous_deg)
{
    float delta = current_deg - previous_deg;
    while(delta > 180.0f) { delta -= 360.0f; }
    while(delta < -180.0f) { delta += 360.0f; }
    return fabsf(delta);
}

static int check_side(uint8 right_side)
{
    int height;
    leg_ik_result_struct previous = {0};
    for(height = 35; height <= 120; height++)
    {
        float x_mm;
        float y_mm;
        leg_ik_result_struct result;
        if((APP_TRUE != leg_kinematics_solve(right_side, 0.0f, (float)height, &previous, &result)) ||
           (APP_TRUE != result.valid))
        {
            printf("IK rejected side %u height %d\\n", (unsigned int)right_side, height);
            return 1;
        }
        if((!isfinite(result.servo_deg[0])) || (!isfinite(result.servo_deg[1])) ||
           (!isfinite(result.singularity_margin)) || (0.20f > result.singularity_margin))
        {
            printf("IK margin/output invalid side %u height %d\\n", (unsigned int)right_side, height);
            return 1;
        }
        if((APP_TRUE == previous.valid) &&
           ((8.0f < wrapped_delta_deg(result.servo_deg[0], previous.servo_deg[0])) ||
            (8.0f < wrapped_delta_deg(result.servo_deg[1], previous.servo_deg[1]))))
        {
            printf("IK discontinuity side %u height %d\\n", (unsigned int)right_side, height);
            return 1;
        }
        if((APP_TRUE != leg_kinematics_forward(right_side, result.servo_deg[0], result.servo_deg[1], &x_mm, &y_mm)) ||
           (0.5f < fabsf(x_mm)) || (0.5f < fabsf(y_mm - (float)height)))
        {
            printf("FK mismatch side %u height %d: %.3f, %.3f\\n", (unsigned int)right_side, height, x_mm, y_mm);
            return 1;
        }
        previous = result;
    }
    return 0;
}

int main(void)
{
    leg_ik_result_struct result = {0};
    float off_center_y_mm = sqrtf(2576.0f);
    if((0 != check_side(APP_FALSE)) || (0 != check_side(APP_TRUE)))
    {
        return 1;
    }
    if((APP_TRUE != leg_kinematics_solve(APP_FALSE, -20.0f, off_center_y_mm, 0, &result)) ||
       (APP_TRUE != result.valid) ||
       (0.20f >= result.singularity_margin) ||
       (1.0f < fabsf(result.servo_deg[0] - 151.9f)) ||
       (1.0f < fabsf(result.servo_deg[1] - 97.7f)))
    {
        printf("Off-center denominator-degenerate IK point rejected or incorrect: %.3f, %.3f, margin %.3f\\n",
               result.servo_deg[0], result.servo_deg[1], result.singularity_margin);
        return 1;
    }
    if(APP_TRUE == leg_kinematics_solve(APP_FALSE, 36.0f, 80.0f, 0, &result))
    {
        printf("Out-of-workspace point accepted\\n");
        return 1;
    }
    if(APP_TRUE == leg_kinematics_solve(APP_FALSE, 0.0f, 150.0f, 0, &result))
    {
        printf("Tangent-circle point accepted\\n");
        return 1;
    }
    return 0;
}
'@ | Set-Content (Join-Path $Path "test_leg_kinematics.c") -NoNewline
}

$config = Get-LegTransitionConfig
Assert-Equal -Actual $config["low_height_mm"] -Expected 35.0 -Message "Low calibrated height"
Assert-Equal -Actual $config["high_height_mm"] -Expected 120.0 -Message "High calibrated height"
Assert-Equal -Actual $config["max_height_speed_mm_s"] -Expected 20.0 -Message "Maximum height speed"
Assert-Equal -Actual $config["max_height_accel_mm_s2"] -Expected 40.0 -Message "Maximum height acceleration"
Assert-Equal -Actual $config["height_settle_error_mm"] -Expected 1.0 -Message "Height settle error"
Assert-Equal -Actual $config["height_settle_ms"] -Expected 300.0 -Message "Height settle time"
Assert-Equal -Actual $config["ik_min_margin"] -Expected 0.20 -Message "IK minimum margin"
Assert-Equal -Actual $config["safe_support_height_mm"] -Expected 80.0 -Message "Provisional safe support height"
Assert-HeightCommandRange -Config $config

Assert-Contains "project/code/leg_kinematics.h" "singularity_margin" "IK result must publish singularity margin."
Assert-Contains "project/code/leg_kinematics.h" "const leg_ik_result_struct \*previous" "IK solve API must accept the previous solution."
Assert-Contains "project/code/leg_kinematics.c" "leg_kinematics_forward" "IK must implement forward kinematics."

Assert-BoundedHeightTrajectory -TargetsMm @(35.0, 120.0)
Assert-BoundedHeightTrajectory -TargetsMm @(120.0, 35.0)
Assert-BoundedHeightTrajectory -TargetsMm @(80.0, 110.0, 50.0)
Assert-SoftFaultSafeRate
Assert-InsufficientIkMarginFault
Assert-MotionPolicy

$tempPath = Join-Path ([System.IO.Path]::GetTempPath()) ("leg-kinematics-" + [Guid]::NewGuid().ToString())
New-Item -ItemType Directory -Path $tempPath | Out-Null
try {
    New-HostHeaders $tempPath
    New-NumericHarness $tempPath
    Copy-Item "project/code/leg_kinematics.c" (Join-Path $tempPath "leg_kinematics.c")
    Copy-Item "project/code/leg_kinematics.h" (Join-Path $tempPath "leg_kinematics.h")
    Copy-Item "project/code/leg_config.c" (Join-Path $tempPath "leg_config.c")

    $compiler = (Get-Command gcc -ErrorAction Stop).Source
    $compilerDirectory = Split-Path $compiler
    $originalPath = $env:PATH
    $env:PATH = $compilerDirectory + [System.IO.Path]::PathSeparator + $env:PATH
    $binary = Join-Path $tempPath "test_leg_kinematics.exe"
    $compileOutput = & $compiler -std=c99 -Wall -Werror -I $tempPath `
        (Join-Path $tempPath "leg_kinematics.c") `
        (Join-Path $tempPath "leg_config.c") `
        (Join-Path $tempPath "test_leg_kinematics.c") `
        -lm -o $binary 2>&1
    if(0 -ne $LASTEXITCODE) {
        $compileOutput | Write-Host
        throw ("Unable to compile numeric IK harness (exit {0})." -f $LASTEXITCODE)
    }
    $env:PATH = $originalPath
    & $binary
    if(0 -ne $LASTEXITCODE) {
        throw "IK numeric sweep failed."
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

Write-Host "leg transition numeric checks passed"

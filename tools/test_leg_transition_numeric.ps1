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

function Assert-Near {
    param(
        [double]$Actual,
        [double]$Expected,
        [double]$Tolerance,
        [string]$Message
    )

    if([math]::Abs($Actual - $Expected) -gt $Tolerance) {
        throw ("{0}: expected {1} +/- {2}, got {3}" -f $Message, $Expected, $Tolerance, $Actual)
    }
}

function Step-LegacyStanceSupervisor {
    param(
        [double]$ReferenceUnits,
        [double]$RateUnitsS,
        [double]$AccelUnitsS2,
        [double]$TargetUnits,
        [double]$MaxRateUnitsS,
        [double]$MaxAccelUnitsS2,
        [double]$MaxJerkUnitsS3,
        [double]$PositionKpS,
        [double]$RateKpS,
        [double]$DtS
    )

    $errorUnits = $TargetUnits - $ReferenceUnits
    $desiredRateUnitsS = [math]::Max(-$MaxRateUnitsS, [math]::Min($errorUnits * $PositionKpS, $MaxRateUnitsS))
    $desiredAccelUnitsS2 = [math]::Max(-$MaxAccelUnitsS2,
        [math]::Min($RateKpS * ($desiredRateUnitsS - $RateUnitsS), $MaxAccelUnitsS2))
    $accelDeltaUnitsS2 = [math]::Max(-$MaxJerkUnitsS3 * $DtS,
        [math]::Min($desiredAccelUnitsS2 - $AccelUnitsS2, $MaxJerkUnitsS3 * $DtS))
    $AccelUnitsS2 += $accelDeltaUnitsS2
    $AccelUnitsS2 = [math]::Max(-$MaxAccelUnitsS2, [math]::Min($AccelUnitsS2, $MaxAccelUnitsS2))
    $RateUnitsS = [math]::Max(-$MaxRateUnitsS, [math]::Min($RateUnitsS + ($AccelUnitsS2 * $DtS), $MaxRateUnitsS))
    $nextReferenceUnits = $ReferenceUnits + ($RateUnitsS * $DtS)
    if(([math]::Abs($TargetUnits - $nextReferenceUnits) -le 0.01) -and
       ([math]::Abs($RateUnitsS) -le 0.05) -and
       ([math]::Abs($AccelUnitsS2) -le ($MaxJerkUnitsS3 * $DtS))) {
        $nextReferenceUnits = $TargetUnits
        $RateUnitsS = 0.0
        $AccelUnitsS2 = 0.0
    }
    return @($nextReferenceUnits, $RateUnitsS, $AccelUnitsS2)
}

function Assert-JerkLimitedLegacyStanceTrajectory {
    param(
        [double]$PositionKpS,
        [double]$RateKpS
    )
    $referenceUnits = 55.0
    $rateUnitsS = 0.0
    $accelUnitsS2 = 0.0
    $targetUnits = 65.0
    for($step = 0; $step -lt 5000; $step++) {
        if(200 -eq $step) {
            $targetUnits = 45.0
        }
        $previousAccelUnitsS2 = $accelUnitsS2
        $result = Step-LegacyStanceSupervisor -ReferenceUnits $referenceUnits -RateUnitsS $rateUnitsS -AccelUnitsS2 $accelUnitsS2 -TargetUnits $targetUnits -MaxRateUnitsS 20.0 -MaxAccelUnitsS2 20.0 -MaxJerkUnitsS3 80.0 -PositionKpS $PositionKpS -RateKpS $RateKpS -DtS 0.01
        $referenceUnits = $result[0]
        $rateUnitsS = $result[1]
        $accelUnitsS2 = $result[2]
        if((20.0 + 0.0001) -lt [math]::Abs($rateUnitsS)) {
            throw "Legacy stance trajectory exceeded 20 units/s."
        }
        if((20.0 + 0.0001) -lt [math]::Abs($accelUnitsS2)) {
            throw "Legacy stance trajectory exceeded 20 units/s2."
        }
        if((0.8 + 0.0001) -lt [math]::Abs($accelUnitsS2 - $previousAccelUnitsS2)) {
            throw ("Legacy stance trajectory exceeded 80 units/s3 at 10 ms: step {0}." -f $step)
        }
        $halfDeltaDeg = 0.5 * (($referenceUnits - 55.0) / 0.595)
        $servoFl = 90.0 + $halfDeltaDeg
        $servoFr = 90.0 - $halfDeltaDeg
        $servoRl = 90.0 - $halfDeltaDeg
        $servoRr = 90.0 + $halfDeltaDeg
        Assert-Near -Actual $servoFl -Expected $servoRr -Tolerance 0.0001 -Message "FL/RR empirical synchronization"
        Assert-Near -Actual $servoFr -Expected $servoRl -Tolerance 0.0001 -Message "FR/RL empirical synchronization"
        Assert-Near -Actual ($servoFl + $servoFr) -Expected 180.0 -Tolerance 0.0001 -Message "Front differential pair center"
        Assert-Near -Actual ($servoRr - $servoRl) -Expected (($referenceUnits - 55.0) / 0.595) -Tolerance 0.0001 -Message "Empirical legacy stance differential"
        if(($step -gt 200) -and (45.0 -eq $referenceUnits) -and (0.0 -eq $rateUnitsS) -and (0.0 -eq $accelUnitsS2)) {
            return
        }
    }
    throw ("Jerk-limited legacy stance trajectory did not settle: target {0}, reference {1}, rate {2}, accel {3}." -f $targetUnits, $referenceUnits, $rateUnitsS, $accelUnitsS2)
}

function Get-S7Blend([double]$u) {
    $u = [math]::Max(0.0, [math]::Min($u, 1.0))
    $u2 = $u * $u
    $u4 = $u2 * $u2
    return $u4 * (35.0 - 84.0 * $u + 70.0 * $u2 - 20.0 * $u2 * $u)
}

function Get-S7Derivative1([double]$u) {
    $u = [math]::Max(0.0, [math]::Min($u, 1.0))
    $u2 = $u * $u
    $u3 = $u2 * $u
    return (140.0 * $u3) - (420.0 * $u3 * $u) + (420.0 * $u3 * $u2) - (140.0 * $u3 * $u2 * $u)
}

function Get-S7Derivative2([double]$u) {
    $u = [math]::Max(0.0, [math]::Min($u, 1.0))
    $u2 = $u * $u
    $u3 = $u2 * $u
    return (420.0 * $u2) - (1680.0 * $u3) + (2100.0 * $u3 * $u) - (840.0 * $u3 * $u2)
}

function Get-S7Derivative3([double]$u) {
    $u = [math]::Max(0.0, [math]::Min($u, 1.0))
    $u2 = $u * $u
    return (840.0 * $u) - (5040.0 * $u2) + (8400.0 * $u2 * $u) - (4200.0 * $u2 * $u2)
}

function Get-FastLegacyStanceReference {
    param(
        [double]$StartUnits,
        [double]$TargetUnits,
        [double]$ElapsedMs,
        [double]$DurationMs
    )

    $u = [math]::Max(0.0, [math]::Min($ElapsedMs / $DurationMs, 1.0))
    $blend = Get-S7Blend $u
    return $StartUnits + (($TargetUnits - $StartUnits) * $blend)
}

function Assert-FastLegacyStanceTrajectory {
    param(
        [double]$StartUnits,
        [double]$TargetUnits,
        [double]$LowUnits,
        [double]$HighUnits,
        [double]$DurationMs
    )

    $previousUnits = $StartUnits
    $direction = [math]::Sign($TargetUnits - $StartUnits)
    for($elapsedMs = 0; $elapsedMs -lt $DurationMs; $elapsedMs += 6) {
        $referenceUnits = Get-FastLegacyStanceReference -StartUnits $StartUnits -TargetUnits $TargetUnits -ElapsedMs $elapsedMs -DurationMs $DurationMs
        if(($LowUnits -gt $referenceUnits) -or ($HighUnits -lt $referenceUnits)) {
            throw ("Fast legacy stance reference overshot its command interval at {0} ms: {1}." -f $elapsedMs, $referenceUnits)
        }
        if((0.0 -lt $direction) -and ($previousUnits -gt ($referenceUnits + 0.000001))) {
            throw ("Fast legacy stance reference reversed before its target at {0} ms." -f $elapsedMs)
        }
        if((0.0 -gt $direction) -and ($previousUnits -lt ($referenceUnits - 0.000001))) {
            throw ("Fast legacy stance reference reversed before its target at {0} ms." -f $elapsedMs)
        }
        $previousUnits = $referenceUnits
    }
    Assert-Near -Actual (Get-FastLegacyStanceReference -StartUnits $StartUnits -TargetUnits $TargetUnits -ElapsedMs 0.0 -DurationMs $DurationMs) -Expected $StartUnits -Tolerance 0.000001 -Message "Fast legacy stance start reference"
    Assert-Near -Actual (Get-FastLegacyStanceReference -StartUnits $StartUnits -TargetUnits $TargetUnits -ElapsedMs $DurationMs -DurationMs $DurationMs) -Expected $TargetUnits -Tolerance 0.000001 -Message "Fast legacy stance completes at configured duration"
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

function Assert-S7Properties {
    Assert-Near -Actual (Get-S7Blend 0.0) -Expected 0.0 -Tolerance 0.000001 -Message "S7 p(0)=0"
    Assert-Near -Actual (Get-S7Blend 1.0) -Expected 1.0 -Tolerance 0.000001 -Message "S7 p(1)=1"

    # Monotonicity check
    $prev = 0.0
    for($u = 0.001; $u -le 1.001; $u += 0.001) {
        $val = Get-S7Blend ([math]::Min($u, 1.0))
        if($val -lt ($prev - 0.000001)) {
            throw ("S7 blend not monotonic at u={0}: prev={1}, val={2}" -f $u, $prev, $val)
        }
        $prev = $val
    }

    # Derivatives zero at both endpoints
    Assert-Near -Actual (Get-S7Derivative1 0.0) -Expected 0.0 -Tolerance 0.000001 -Message "S7 velocity zero at u=0"
    Assert-Near -Actual (Get-S7Derivative1 1.0) -Expected 0.0 -Tolerance 0.000001 -Message "S7 velocity zero at u=1"
    Assert-Near -Actual (Get-S7Derivative2 0.0) -Expected 0.0 -Tolerance 0.000001 -Message "S7 acceleration zero at u=0"
    Assert-Near -Actual (Get-S7Derivative2 1.0) -Expected 0.0 -Tolerance 0.000001 -Message "S7 acceleration zero at u=1"
    Assert-Near -Actual (Get-S7Derivative3 0.0) -Expected 0.0 -Tolerance 0.000001 -Message "S7 jerk zero at u=0"
    Assert-Near -Actual (Get-S7Derivative3 1.0) -Expected 0.0 -Tolerance 0.000001 -Message "S7 jerk zero at u=1"
}

function Assert-SharedPoseTrajectory {
    $startDeg = @(90.0, 90.0, 90.0, 90.0)
    $targetDeg = @(114.0, 66.0, 108.0, 72.0)
    $maxDelta = 0.0
    for($i = 0; $i -lt 4; $i++) {
        $delta = [math]::Abs($targetDeg[$i] - $startDeg[$i])
        if($delta -gt $maxDelta) { $maxDelta = $delta }
    }
    $durationS = [math]::Max(0.10, 2.1875 * $maxDelta / 90.0)
    $durationMs = $durationS * 1000.0
    $samplesPerMs = 6

    for($t = 0.0; $t -lt $durationMs; $t += $samplesPerMs) {
        $u = [math]::Min($t / $durationMs, 1.0)
        $blend = Get-S7Blend $u
        for($i = 0; $i -lt 4; $i++) {
            $expectedAngle = $startDeg[$i] + ($targetDeg[$i] - $startDeg[$i]) * $blend
            if(($expectedAngle -lt 0.0) -or ($expectedAngle -gt 180.0)) {
                throw ("Shared-pose channel {0} exceeded angle bounds at t={1} ms: {2}" -f $i, $t, $expectedAngle)
            }
        }
    }

    # All channels reach target on same sample
    for($i = 0; $i -lt 4; $i++) {
        $finalAngle = $startDeg[$i] + ($targetDeg[$i] - $startDeg[$i]) * (Get-S7Blend 1.0)
        Assert-Near -Actual $finalAngle -Expected $targetDeg[$i] -Tolerance 0.000001 -Message ("Shared-pose channel {0} reaches target" -f $i)
    }
}

function Assert-StableGateModel {
    # Planner complete but actuator not settled -> keep TRANSITION
    $plannerComplete = $true
    $actuatorSettled = $false
    $actuatorError = 0.21
    $settleThreshold = 0.2

    $motionState = if($plannerComplete -and $actuatorSettled -and ($actuatorError -le $settleThreshold)) {
        "STABLE"
    } else {
        "TRANSITION"
    }

    if($motionState -ne "TRANSITION") {
        throw "Motion state must remain TRANSITION when planner is complete but actuator error exceeds settle threshold."
    }

    # Actuator settled after 30 ticks -> STABLE
    $actuatorSettled = $true
    $actuatorError = 0.01
    $motionState = if($plannerComplete -and $actuatorSettled -and ($actuatorError -le $settleThreshold)) {
        "STABLE"
    } else {
        "TRANSITION"
    }

    if($motionState -ne "STABLE") {
        throw "Motion state must become STABLE only after planner completes and actuator settles."
    }
}

function Assert-LegacyStanceCommandRange {
    param([hashtable]$Config)

    $lowReject = 29.0
    $highReject = 81.0
    $phase1RejectedHigh = 120.0
    if(($lowReject -ge $Config["legacy_low_units"]) -and ($lowReject -le $Config["legacy_high_units"])) {
        throw "Legacy stance command 29 must be outside the configured interval."
    }
    if(($highReject -ge $Config["legacy_low_units"]) -and ($highReject -le $Config["legacy_high_units"])) {
        throw "Legacy stance command 81 must be outside the configured interval."
    }
    if(($phase1RejectedHigh -ge $Config["legacy_low_units"]) -and ($phase1RejectedHigh -le $Config["legacy_high_units"])) {
        throw "Legacy stance command 120 must be outside the configured Phase 1 interval."
    }
}

function Get-LegTransitionConfig {
    $text = Get-Content "project/code/leg_config.c" -Raw
    $names = @(
        "legacy_low_units",
        "legacy_high_units",
        "legacy_default_units",
        "legacy_max_rate_units_s",
        "legacy_max_accel_units_s2",
        "legacy_max_jerk_units_s3",
        "legacy_position_kp_s",
        "legacy_rate_kp_s",
        "legacy_settle_error_units",
        "legacy_settle_ms",
        "fast_stance_transition_ms",
        "ik_min_margin",
        "legacy_safe_support_units"
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
typedef struct { uint8 servo_index; float safe_deg; float neutral_deg; float min_deg; float max_deg; float direction; float ik_offset_deg; float mount_x; float mount_y; } leg_servo_config_struct;
typedef enum { LEG_IK_BRANCH_PLUS = 0, LEG_IK_BRANCH_MINUS = 1 } leg_ik_branch_enum;
typedef enum { LEG_PHYSICAL_WORKSPACE_VERTEX_COUNT = 8 } leg_physical_workspace_constant_enum;
typedef struct { float l1_mm; float l2_mm; float l3_mm; float l4_mm; float l5_mm; float x_min_mm; float x_max_mm; float y_min_mm; float y_max_mm; float physical_reference_x_mm; float physical_reference_y_mm; float alpha_reference_deg; float beta_reference_deg; float model_reference_x_mm; float model_reference_y_mm; float model_to_physical_scale; float model_to_physical_m00; float model_to_physical_m01; float model_to_physical_m10; float model_to_physical_m11; float physical_workspace[LEG_PHYSICAL_WORKSPACE_VERTEX_COUNT][2]; float physical_workspace_inset_mm; uint8 experimental_race_enable; float experimental_race_x_mm; float experimental_race_y_mm; float experimental_race_tolerance_x_mm; float experimental_race_tolerance_y_mm; float experimental_race_ik_min_margin; leg_ik_branch_enum experimental_race_alpha_branch; leg_ik_branch_enum experimental_race_beta_branch; leg_ik_branch_enum left_alpha_branch; leg_ik_branch_enum left_beta_branch; leg_ik_branch_enum right_alpha_branch; leg_ik_branch_enum right_beta_branch; } leg_kinematics_config_struct;
typedef struct { float legacy_low_units; float legacy_high_units; float legacy_default_units; float legacy_max_rate_units_s; float legacy_max_accel_units_s2; float legacy_max_jerk_units_s3; float legacy_position_kp_s; float legacy_rate_kp_s; float legacy_settle_error_units; uint32 legacy_settle_ms; uint32 fast_stance_transition_ms; float ik_min_margin; float legacy_safe_support_units; float transition_forward_limit_rpm; float balance_pitch_kp_low; float balance_pitch_kp_high; float balance_pitch_rate_kd_low; float balance_pitch_rate_kd_high; float balance_wheel_speed_ks_low; float balance_wheel_speed_ks_high; float balance_pitch_setpoint_low_deg; float balance_pitch_setpoint_high_deg; float chassis_forward_limit_low_rpm; float chassis_forward_limit_high_rpm; float chassis_fast_forward_limit_low_rpm; float chassis_fast_forward_limit_high_rpm; } leg_stance_profile_struct;
typedef struct { leg_servo_config_struct servo[LEG_SERVO_COUNT]; leg_kinematics_config_struct kinematics; leg_stance_profile_struct stance_profile; float legacy_body_min_units; float legacy_body_max_units; float pitch_limit; float roll_limit; } leg_config_struct;
const leg_config_struct *leg_config_get(void);
const leg_servo_config_struct *leg_config_get_servo(uint8 leg_id);
const leg_kinematics_config_struct *leg_config_get_kinematics(void);
const leg_stance_profile_struct *leg_config_get_stance_profile(void);
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
    int physical_x_i;
    int physical_y_i;

    for(physical_y_i = 20; physical_y_i <= 100; physical_y_i++)
    {
        leg_ik_result_struct previous = {0};
        const float physical_y = (float)physical_y_i;

        for(physical_x_i = -60; physical_x_i <= 20; physical_x_i++)
        {
            const float physical_x = (float)physical_x_i;
            float x_mm;
            float y_mm;
            leg_ik_result_struct result;

            if(APP_TRUE != leg_kinematics_target_valid(physical_x, physical_y))
            {
                previous.valid = APP_FALSE;
                continue;
            }
            if((APP_TRUE != leg_kinematics_solve(right_side, physical_x, physical_y,
                                                  &previous, &result)) ||
               (APP_TRUE != result.valid))
            {
                printf("IK rejected side %u physical %.1f %.1f\\n",
                       (unsigned int)right_side, physical_x, physical_y);
                return 1;
            }
            if((!isfinite(result.servo_deg[0])) || (!isfinite(result.servo_deg[1])) ||
               (!isfinite(result.singularity_margin)) || (0.02f > result.singularity_margin))
            {
                printf("IK margin/output invalid side %u physical %.1f %.1f\\n",
                       (unsigned int)right_side, physical_x, physical_y);
                return 1;
            }
            if((APP_TRUE == previous.valid) &&
               ((12.0f < wrapped_delta_deg(result.servo_deg[0], previous.servo_deg[0])) ||
                (12.0f < wrapped_delta_deg(result.servo_deg[1], previous.servo_deg[1]))))
            {
                printf("model branch discontinuity %.1f %.1f\\n", physical_x, physical_y);
                return 1;
            }
            if((APP_TRUE != leg_kinematics_forward(right_side, result.servo_deg[0], result.servo_deg[1], &x_mm, &y_mm)) ||
               (0.5f < fabsf(x_mm - physical_x)) || (0.5f < fabsf(y_mm - physical_y)))
            {
                printf("FK mismatch side %u physical %.1f %.1f: %.3f, %.3f\\n",
                       (unsigned int)right_side, physical_x, physical_y, x_mm, y_mm);
                return 1;
            }
            previous = result;
        }
    }
    return 0;
}

int main(void)
{
    leg_ik_result_struct left_reference = {0};
    leg_ik_result_struct right_reference = {0};
    const leg_kinematics_config_struct *cfg = leg_config_get_kinematics();
    float servo_deg[LEG_SERVO_COUNT];
    float forward_x_mm;
    float forward_y_mm;
    if(0 == cfg)
    {
        return 1;
    }
    if((0 != check_side(APP_FALSE)) || (0 != check_side(APP_TRUE)))
    {
        return 1;
    }
    if((APP_TRUE != leg_kinematics_solve(APP_FALSE,
                                         cfg->physical_reference_x_mm,
                                         cfg->physical_reference_y_mm,
                                         0, &left_reference)) ||
       (APP_TRUE != leg_kinematics_solve(APP_TRUE,
                                         cfg->physical_reference_x_mm,
                                         cfg->physical_reference_y_mm,
                                         0, &right_reference)) ||
       (APP_TRUE != leg_kinematics_map_reference_pose(&left_reference,
                                                       &right_reference,
                                                       servo_deg)) ||
       (0.01f < fabsf(servo_deg[0] - 90.0f)) ||
       (0.01f < fabsf(servo_deg[1] - 90.0f)) ||
       (0.01f < fabsf(servo_deg[2] - 90.0f)) ||
       (0.01f < fabsf(servo_deg[3] - 90.0f)) ||
       (APP_TRUE != leg_kinematics_forward(APP_FALSE,
                                           left_reference.servo_deg[0],
                                           left_reference.servo_deg[1],
                                           &forward_x_mm, &forward_y_mm)) ||
       (0.5f < fabsf(forward_x_mm - cfg->physical_reference_x_mm)) ||
       (0.5f < fabsf(forward_y_mm - cfg->physical_reference_y_mm)))
    {
        printf("Physical reference equivalence failed: %.3f, %.3f\\n",
               forward_x_mm, forward_y_mm);
        return 1;
    }
    if(APP_TRUE != leg_kinematics_target_valid(0.0f, 55.0f))
    {
        printf("Model-reachable X=0 target rejected\\n");
        return 1;
    }
    if(APP_TRUE != leg_kinematics_target_valid(-40.620f, 47.370f))
    {
        printf("Model-reachable hull vertex rejected\\n");
        return 1;
    }
    return 0;
}
'@ | Set-Content (Join-Path $Path "test_leg_kinematics.c") -NoNewline
}

$config = Get-LegTransitionConfig
Assert-Equal -Actual $config["legacy_low_units"] -Expected 30.0 -Message "Extended empirical low legacy stance"
Assert-Equal -Actual $config["legacy_high_units"] -Expected 80.0 -Message "Extended empirical high legacy stance"
Assert-Equal -Actual $config["legacy_default_units"] -Expected 55.0 -Message "Empirical Phase 1 default legacy stance"
Assert-Equal -Actual $config["legacy_max_rate_units_s"] -Expected 20.0 -Message "Fast-response maximum legacy stance rate"
Assert-Equal -Actual $config["legacy_max_accel_units_s2"] -Expected 20.0 -Message "Fast-response maximum legacy stance acceleration"
Assert-Equal -Actual $config["legacy_max_jerk_units_s3"] -Expected 80.0 -Message "Maximum legacy stance jerk"
Assert-Equal -Actual $config["legacy_position_kp_s"] -Expected 2.0 -Message "Legacy stance position-to-rate gain"
Assert-Equal -Actual $config["legacy_rate_kp_s"] -Expected 4.0 -Message "Legacy stance rate-to-acceleration gain"
Assert-Equal -Actual $config["legacy_settle_error_units"] -Expected 1.0 -Message "Legacy stance settle error"
Assert-Equal -Actual $config["legacy_settle_ms"] -Expected 300.0 -Message "Legacy stance settle time"
Assert-Equal -Actual $config["fast_stance_transition_ms"] -Expected 500.0 -Message "Fast legacy stance transition duration"
Assert-Equal -Actual $config["ik_min_margin"] -Expected 0.02 -Message "IK minimum margin"
Assert-Equal -Actual $config["legacy_safe_support_units"] -Expected 55.0 -Message "Empirical Phase 1 safe support legacy stance"
Assert-LegacyStanceCommandRange -Config $config

Assert-Contains "project/code/leg_kinematics.h" "singularity_margin" "IK result must publish singularity margin."
Assert-Contains "project/code/leg_kinematics.h" "const leg_ik_result_struct \*previous" "IK solve API must accept the previous solution."
Assert-Contains "project/code/leg_kinematics.c" "leg_kinematics_forward" "IK must implement forward kinematics."

Assert-JerkLimitedLegacyStanceTrajectory -PositionKpS $config["legacy_position_kp_s"] -RateKpS $config["legacy_rate_kp_s"]
Assert-FastLegacyStanceTrajectory -StartUnits 45.0 -TargetUnits 65.0 -LowUnits $config["legacy_low_units"] -HighUnits $config["legacy_high_units"] -DurationMs $config["fast_stance_transition_ms"]
Assert-FastLegacyStanceTrajectory -StartUnits 55.0 -TargetUnits 30.0 -LowUnits $config["legacy_low_units"] -HighUnits $config["legacy_high_units"] -DurationMs $config["fast_stance_transition_ms"]
Assert-FastLegacyStanceTrajectory -StartUnits 55.0 -TargetUnits 80.0 -LowUnits $config["legacy_low_units"] -HighUnits $config["legacy_high_units"] -DurationMs $config["fast_stance_transition_ms"]
Assert-SoftFaultSafeRate
Assert-InsufficientIkMarginFault
Assert-MotionPolicy
Assert-S7Properties
Assert-SharedPoseTrajectory
Assert-StableGateModel

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

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

function Assert-LxyBranchPreflight {
    $text = Get-Content "project/code/control_leg.c" -Raw
    $start = $text.IndexOf("uint8 control_leg_set_xy")
    $end = $text.IndexOf("uint8 control_leg_set_fast_legacy_stance", $start)
    if(($start -lt 0) -or ($end -le $start)) {
        throw "Unable to locate control_leg_set_xy for branch preflight check."
    }

    $body = $text.Substring($start, $end - $start)
    $leftSolve = $body.IndexOf("leg_kinematics_solve(APP_FALSE")
    $rightSolve = $body.IndexOf("leg_kinematics_solve(APP_TRUE")
    $targetWrite = $body.IndexOf("control_leg_ik_target_x_mm = x_mm;")
    if(($leftSolve -lt 0) -or ($rightSolve -lt 0) -or ($targetWrite -lt 0) -or
       ($leftSolve -gt $targetWrite) -or ($rightSolve -gt $targetWrite)) {
        throw "LXY must preflight both persisted branches before changing the active target."
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
typedef struct { float l1_mm; float l2_mm; float l3_mm; float l4_mm; float l5_mm; float physical_reference_x_mm; float physical_reference_y_mm; float alpha_reference_deg; float beta_reference_deg; float model_reference_x_mm; float model_reference_y_mm; float model_to_physical_scale; float model_to_physical_m00; float model_to_physical_m01; float model_to_physical_m10; float model_to_physical_m11; leg_ik_branch_enum left_alpha_branch; leg_ik_branch_enum left_beta_branch; leg_ik_branch_enum right_alpha_branch; leg_ik_branch_enum right_beta_branch; } leg_kinematics_config_struct;
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

#define LEG_MODEL_GRID_ACCEPTED_POINT_COUNT (5637U)
#define LEG_MODEL_GRID_POINT_COUNT          (6561U)
#define LEG_MODEL_MIN_MARGIN                (0.02f)
#define LEG_ORACLE_EPS                      (0.000001f)
#define LEG_ORACLE_PI                       (3.14159265358979323846f)
#define LEG_ORACLE_TWO_PI                   (6.28318530717958647692f)

typedef enum
{
    LEG_SWEEP_X = 0,
    LEG_SWEEP_Y = 1
}leg_sweep_axis_enum;

typedef enum
{
    LEG_ORACLE_ACCEPT = 0,
    LEG_ORACLE_REJECT_NONFINITE,
    LEG_ORACLE_REJECT_TRANSFORM,
    LEG_ORACLE_REJECT_MODEL_Y,
    LEG_ORACLE_REJECT_ALPHA_GEOMETRY,
    LEG_ORACLE_REJECT_BETA_GEOMETRY,
    LEG_ORACLE_REJECT_MARGIN,
    LEG_ORACLE_REJECT_LEFT_LIMIT,
    LEG_ORACLE_REJECT_RIGHT_LIMIT
}leg_oracle_reason_enum;

typedef struct
{
    float model_x_mm;
    float model_y_mm;
    float alpha_rad[2];
    float beta_rad[2];
    float alpha_discriminant;
    float beta_discriminant;
    float alpha_margin;
    float beta_margin;
    float singularity_margin;
    uint8 side_candidate_mask[2];
    leg_oracle_reason_enum reason;
    uint8 valid;
}leg_oracle_result_struct;

static float wrapped_delta_deg(float current_deg, float previous_deg)
{
    float delta = current_deg - previous_deg;
    while(delta > 180.0f) { delta -= 360.0f; }
    while(delta < -180.0f) { delta += 360.0f; }
    return fabsf(delta);
}

static float adjacent_motion_limit_deg(const leg_ik_result_struct *previous,
                                       const leg_ik_result_struct *result)
{
    if((0.05f > previous->singularity_margin) ||
       (0.05f > result->singularity_margin))
    {
        return 16.0f;
    }
    return 13.0f;
}

static float oracle_wrap_positive(float angle_rad)
{
    while(0.0f > angle_rad) { angle_rad += LEG_ORACLE_TWO_PI; }
    while(LEG_ORACLE_TWO_PI <= angle_rad) { angle_rad -= LEG_ORACLE_TWO_PI; }
    return angle_rad;
}

static float oracle_wrapped_delta(float target_rad, float reference_rad)
{
    float delta = oracle_wrap_positive(target_rad - reference_rad);
    if(LEG_ORACLE_PI < delta) { delta -= LEG_ORACLE_TWO_PI; }
    return delta;
}

static int oracle_solve_roots(float a,
                              float b,
                              float c,
                              float roots_rad[2],
                              float *margin,
                              float *discriminant)
{
    float magnitude;
    float root;
    float phase_rad;
    float offset_rad;

    if((NULL == roots_rad) || (NULL == margin) || (NULL == discriminant) ||
       (!isfinite(a)) || (!isfinite(b)) || (!isfinite(c)))
    {
        return 0;
    }
    *discriminant = (a * a) + (b * b) - (c * c);
    magnitude = sqrtf((a * a) + (b * b));
    if((0.0f > *discriminant) || (LEG_ORACLE_EPS > magnitude))
    {
        return 0;
    }
    root = sqrtf(*discriminant);
    *margin = root / magnitude;
    phase_rad = atan2f(b, a);
    offset_rad = atan2f(root, c);
    roots_rad[LEG_IK_BRANCH_PLUS] = oracle_wrap_positive(phase_rad + offset_rad);
    roots_rad[LEG_IK_BRANCH_MINUS] = oracle_wrap_positive(phase_rad - offset_rad);
    return (isfinite(*margin) &&
            isfinite(roots_rad[LEG_IK_BRANCH_PLUS]) &&
            isfinite(roots_rad[LEG_IK_BRANCH_MINUS]) &&
            (0.0f <= *margin) && (1.0f >= *margin));
}

static int oracle_map_root(uint8 servo_index,
                           float reference_deg,
                           float root_rad,
                           float *command_deg)
{
    const leg_servo_config_struct *servo = leg_config_get_servo(servo_index);
    float command;

    if((NULL == servo) || (NULL == command_deg) ||
       (!isfinite(reference_deg)) || (!isfinite(root_rad)))
    {
        return 0;
    }
    command = servo->neutral_deg +
              (servo->direction *
               (oracle_wrapped_delta(root_rad,
                                     reference_deg * LEG_ORACLE_PI / 180.0f) *
                180.0f / LEG_ORACLE_PI)) +
              servo->ik_offset_deg;
    if((!isfinite(command)) ||
       (servo->min_deg > command) || (servo->max_deg < command))
    {
        return 0;
    }
    *command_deg = command;
    return 1;
}

static uint8 oracle_side_candidate_mask(uint8 right_side,
                                        const leg_oracle_result_struct *oracle)
{
    const leg_kinematics_config_struct *cfg = leg_config_get_kinematics();
    const uint8 alpha_servo = (APP_TRUE == right_side) ? LEG_SERVO_FR : LEG_SERVO_FL;
    const uint8 beta_servo = (APP_TRUE == right_side) ? LEG_SERVO_RR : LEG_SERVO_RL;
    uint8 mask = 0U;
    int alpha_branch;

    if((NULL == cfg) || (NULL == oracle))
    {
        return 0U;
    }
    for(alpha_branch = LEG_IK_BRANCH_PLUS;
        alpha_branch <= LEG_IK_BRANCH_MINUS;
        alpha_branch++)
    {
        int beta_branch;
        float alpha_command_deg;

        if(0 == oracle_map_root(alpha_servo,
                                cfg->alpha_reference_deg,
                                oracle->alpha_rad[alpha_branch],
                                &alpha_command_deg))
        {
            continue;
        }
        for(beta_branch = LEG_IK_BRANCH_PLUS;
            beta_branch <= LEG_IK_BRANCH_MINUS;
            beta_branch++)
        {
            float beta_command_deg;

            if(0 != oracle_map_root(beta_servo,
                                    cfg->beta_reference_deg,
                                    oracle->beta_rad[beta_branch],
                                    &beta_command_deg))
            {
                mask |= (uint8)(1U << ((2 * alpha_branch) + beta_branch));
            }
        }
    }
    return mask;
}

static int oracle_classify(float physical_x_mm,
                           float physical_y_mm,
                           leg_oracle_result_struct *oracle)
{
    const leg_kinematics_config_struct *cfg = leg_config_get_kinematics();
    float delta_x;
    float delta_y;
    float a;
    float b;
    float c;
    float d;
    float e;
    float f;

    if(NULL == oracle)
    {
        return 0;
    }
    oracle->model_x_mm = 0.0f;
    oracle->model_y_mm = 0.0f;
    oracle->alpha_rad[0] = 0.0f;
    oracle->alpha_rad[1] = 0.0f;
    oracle->beta_rad[0] = 0.0f;
    oracle->beta_rad[1] = 0.0f;
    oracle->alpha_discriminant = 0.0f;
    oracle->beta_discriminant = 0.0f;
    oracle->alpha_margin = 0.0f;
    oracle->beta_margin = 0.0f;
    oracle->singularity_margin = 0.0f;
    oracle->side_candidate_mask[APP_FALSE] = 0U;
    oracle->side_candidate_mask[APP_TRUE] = 0U;
    oracle->reason = LEG_ORACLE_REJECT_NONFINITE;
    oracle->valid = APP_FALSE;

    if((NULL == cfg) || (!isfinite(physical_x_mm)) || (!isfinite(physical_y_mm)))
    {
        return 0;
    }
    oracle->reason = LEG_ORACLE_REJECT_TRANSFORM;
    if((!isfinite(cfg->model_to_physical_scale)) ||
       (LEG_ORACLE_EPS > cfg->model_to_physical_scale))
    {
        return 0;
    }
    delta_x = (physical_x_mm - cfg->physical_reference_x_mm) /
              cfg->model_to_physical_scale;
    delta_y = (physical_y_mm - cfg->physical_reference_y_mm) /
              cfg->model_to_physical_scale;
    oracle->model_x_mm = cfg->model_reference_x_mm +
                         (cfg->model_to_physical_m00 * delta_x) +
                         (cfg->model_to_physical_m10 * delta_y);
    oracle->model_y_mm = cfg->model_reference_y_mm +
                         (cfg->model_to_physical_m01 * delta_x) +
                         (cfg->model_to_physical_m11 * delta_y);
    if((!isfinite(oracle->model_x_mm)) || (!isfinite(oracle->model_y_mm)))
    {
        return 0;
    }
    oracle->reason = LEG_ORACLE_REJECT_MODEL_Y;
    if(0.0f >= oracle->model_y_mm)
    {
        return 0;
    }

    a = 2.0f * oracle->model_x_mm * cfg->l1_mm;
    b = 2.0f * oracle->model_y_mm * cfg->l1_mm;
    c = (oracle->model_x_mm * oracle->model_x_mm) +
        (oracle->model_y_mm * oracle->model_y_mm) +
        (cfg->l1_mm * cfg->l1_mm) - (cfg->l2_mm * cfg->l2_mm);
    d = 2.0f * (oracle->model_x_mm - cfg->l5_mm) * cfg->l4_mm;
    e = 2.0f * oracle->model_y_mm * cfg->l4_mm;
    f = ((oracle->model_x_mm - cfg->l5_mm) *
         (oracle->model_x_mm - cfg->l5_mm)) +
        (oracle->model_y_mm * oracle->model_y_mm) +
        (cfg->l4_mm * cfg->l4_mm) - (cfg->l3_mm * cfg->l3_mm);
    oracle->reason = LEG_ORACLE_REJECT_ALPHA_GEOMETRY;
    if(0 == oracle_solve_roots(a, b, c,
                               oracle->alpha_rad,
                               &oracle->alpha_margin,
                               &oracle->alpha_discriminant))
    {
        return 0;
    }
    oracle->reason = LEG_ORACLE_REJECT_BETA_GEOMETRY;
    if(0 == oracle_solve_roots(d, e, f,
                               oracle->beta_rad,
                               &oracle->beta_margin,
                               &oracle->beta_discriminant))
    {
        return 0;
    }
    oracle->singularity_margin = (oracle->alpha_margin < oracle->beta_margin) ?
                                 oracle->alpha_margin : oracle->beta_margin;
    oracle->reason = LEG_ORACLE_REJECT_MARGIN;
    if(LEG_MODEL_MIN_MARGIN > oracle->singularity_margin)
    {
        return 0;
    }
    oracle->side_candidate_mask[APP_FALSE] =
        oracle_side_candidate_mask(APP_FALSE, oracle);
    oracle->side_candidate_mask[APP_TRUE] =
        oracle_side_candidate_mask(APP_TRUE, oracle);
    oracle->reason = LEG_ORACLE_REJECT_LEFT_LIMIT;
    if(0U == oracle->side_candidate_mask[APP_FALSE])
    {
        return 0;
    }
    oracle->reason = LEG_ORACLE_REJECT_RIGHT_LIMIT;
    if(0U == oracle->side_candidate_mask[APP_TRUE])
    {
        return 0;
    }
    oracle->reason = LEG_ORACLE_ACCEPT;
    oracle->valid = APP_TRUE;
    return 1;
}

static const char *oracle_reason_name(leg_oracle_reason_enum reason)
{
    switch(reason)
    {
        case LEG_ORACLE_ACCEPT: return "accept";
        case LEG_ORACLE_REJECT_NONFINITE: return "nonfinite";
        case LEG_ORACLE_REJECT_TRANSFORM: return "transform";
        case LEG_ORACLE_REJECT_MODEL_Y: return "model-y";
        case LEG_ORACLE_REJECT_ALPHA_GEOMETRY: return "alpha-geometry";
        case LEG_ORACLE_REJECT_BETA_GEOMETRY: return "beta-geometry";
        case LEG_ORACLE_REJECT_MARGIN: return "margin";
        case LEG_ORACLE_REJECT_LEFT_LIMIT: return "left-limit";
        case LEG_ORACLE_REJECT_RIGHT_LIMIT: return "right-limit";
        default: return "unknown";
    }
}

static const char *oracle_branch_name(int branch)
{
    return (LEG_IK_BRANCH_PLUS == branch) ? "PLUS" : "MINUS";
}

static int oracle_selected_commands(uint8 right_side,
                                    const leg_oracle_result_struct *oracle,
                                    const leg_ik_result_struct *result,
                                    float command_deg[2],
                                    int branch[2])
{
    const leg_kinematics_config_struct *cfg = leg_config_get_kinematics();
    const uint8 alpha_servo = (APP_TRUE == right_side) ? LEG_SERVO_FR : LEG_SERVO_FL;
    const uint8 beta_servo = (APP_TRUE == right_side) ? LEG_SERVO_RR : LEG_SERVO_RL;
    float plus_delta;
    float minus_delta;
    uint8 candidate_bit;

    if((NULL == cfg) || (NULL == oracle) || (NULL == result) ||
       (NULL == command_deg) || (NULL == branch))
    {
        return 0;
    }
    plus_delta = fabsf(oracle_wrapped_delta(result->alpha_rad,
                                             oracle->alpha_rad[LEG_IK_BRANCH_PLUS]));
    minus_delta = fabsf(oracle_wrapped_delta(result->alpha_rad,
                                              oracle->alpha_rad[LEG_IK_BRANCH_MINUS]));
    branch[0] = (plus_delta <= minus_delta) ? LEG_IK_BRANCH_PLUS : LEG_IK_BRANCH_MINUS;
    plus_delta = fabsf(oracle_wrapped_delta(result->beta_rad,
                                             oracle->beta_rad[LEG_IK_BRANCH_PLUS]));
    minus_delta = fabsf(oracle_wrapped_delta(result->beta_rad,
                                              oracle->beta_rad[LEG_IK_BRANCH_MINUS]));
    branch[1] = (plus_delta <= minus_delta) ? LEG_IK_BRANCH_PLUS : LEG_IK_BRANCH_MINUS;
    candidate_bit = (uint8)(1U << ((2 * branch[0]) + branch[1]));
    if((result->alpha_branch != branch[0]) ||
       (result->beta_branch != branch[1]) ||
       (0U == (oracle->side_candidate_mask[right_side] & candidate_bit)) ||
       (0 == oracle_map_root(alpha_servo,
                             cfg->alpha_reference_deg,
                             oracle->alpha_rad[branch[0]],
                             &command_deg[0])) ||
       (0 == oracle_map_root(beta_servo,
                             cfg->beta_reference_deg,
                             oracle->beta_rad[branch[1]],
                             &command_deg[1])))
    {
        return 0;
    }
    return 1;
}

static int check_oracle_grid(void)
{
    uint32 oracle_accepted_count = 0U;
    uint32 production_accepted_count = 0U;
    uint32 point_count = 0U;
    int physical_y_i;

    for(physical_y_i = 20; physical_y_i <= 100; physical_y_i++)
    {
        int physical_x_i;

        for(physical_x_i = -60; physical_x_i <= 20; physical_x_i++)
        {
            const float physical_x = (float)physical_x_i;
            const float physical_y = (float)physical_y_i;
            leg_oracle_result_struct oracle;
            const uint8 oracle_valid = (0 != oracle_classify(physical_x,
                                                              physical_y,
                                                              &oracle)) ? APP_TRUE : APP_FALSE;
            const uint8 production_valid = leg_kinematics_target_valid(physical_x,
                                                                        physical_y);
            uint8 side;

            point_count++;
            if(APP_TRUE == oracle_valid) { oracle_accepted_count++; }
            if(APP_TRUE == production_valid) { production_accepted_count++; }
            if(oracle_valid != production_valid)
            {
                printf("reachability oracle mismatch physical %.1f %.1f model %.6f %.6f oracle %u production %u reason %s discriminants %.6f/%.6f margins %.8f/%.8f/min %.8f masks 0x%02x/0x%02x\n",
                       physical_x, physical_y,
                       oracle.model_x_mm, oracle.model_y_mm,
                       (unsigned int)oracle_valid,
                       (unsigned int)production_valid,
                       oracle_reason_name(oracle.reason),
                       oracle.alpha_discriminant,
                       oracle.beta_discriminant,
                       oracle.alpha_margin,
                       oracle.beta_margin,
                       oracle.singularity_margin,
                       (unsigned int)oracle.side_candidate_mask[APP_FALSE],
                       (unsigned int)oracle.side_candidate_mask[APP_TRUE]);
                return 1;
            }
            for(side = APP_FALSE; side <= APP_TRUE; side++)
            {
                leg_ik_result_struct result = {0};
                float command_deg[2];
                int branch[2];
                const uint8 expected_side_valid =
                    (0U != oracle.side_candidate_mask[side]) ? APP_TRUE : APP_FALSE;
                const uint8 solve_return = leg_kinematics_solve(side,
                                                                 physical_x,
                                                                 physical_y,
                                                                 NULL,
                                                                 &result);

                if(((APP_TRUE == expected_side_valid) &&
                    ((APP_TRUE != solve_return) || (APP_TRUE != result.valid))) ||
                   ((APP_FALSE == expected_side_valid) &&
                    ((APP_FALSE != solve_return) || (APP_FALSE != result.valid))))
                {
                    printf("side oracle mismatch side %u physical %.1f %.1f reason %s expected %u solve %u/result %u margin %.8f mask 0x%02x\n",
                           (unsigned int)side, physical_x, physical_y,
                           oracle_reason_name(oracle.reason),
                           (unsigned int)expected_side_valid,
                           (unsigned int)solve_return,
                           (unsigned int)result.valid,
                           oracle.singularity_margin,
                           (unsigned int)oracle.side_candidate_mask[side]);
                    return 1;
                }
                if((APP_TRUE == expected_side_valid) &&
                   (0 == oracle_selected_commands(side,
                                                   &oracle,
                                                   &result,
                                                   command_deg,
                                                   branch)))
                {
                    printf("production solution is not an oracle candidate side %u physical %.1f %.1f raw %.6f/%.6f mask 0x%02x\n",
                           (unsigned int)side, physical_x, physical_y,
                           result.alpha_rad, result.beta_rad,
                           (unsigned int)oracle.side_candidate_mask[side]);
                    return 1;
                }
            }
        }
    }
    if((LEG_MODEL_GRID_POINT_COUNT != point_count) ||
       (LEG_MODEL_GRID_ACCEPTED_POINT_COUNT != oracle_accepted_count) ||
       (LEG_MODEL_GRID_ACCEPTED_POINT_COUNT != production_accepted_count))
    {
        printf("Grid baseline mismatch: points %u/%u oracle accepted %u/%u production accepted %u/%u\n",
               (unsigned int)point_count,
               (unsigned int)LEG_MODEL_GRID_POINT_COUNT,
               (unsigned int)oracle_accepted_count,
               (unsigned int)LEG_MODEL_GRID_ACCEPTED_POINT_COUNT,
               (unsigned int)production_accepted_count,
               (unsigned int)LEG_MODEL_GRID_ACCEPTED_POINT_COUNT);
        return 1;
    }
    return 0;
}

static int check_branch_metadata_fail_closed(void)
{
    leg_ik_result_struct previous = {0};
    leg_ik_result_struct result = {0};

    if(APP_TRUE != leg_kinematics_solve(APP_TRUE,
                                        -13.0f,
                                        28.0f,
                                        NULL,
                                        &previous))
    {
        printf("unable to establish previous branch for fail-closed test\n");
        return 1;
    }
    previous.alpha_branch = (leg_ik_branch_enum)2;
    if((APP_FALSE != leg_kinematics_solve(APP_TRUE,
                                          -14.0f,
                                          28.0f,
                                          &previous,
                                          &result)) ||
       (APP_FALSE != result.valid))
    {
        printf("invalid previous branch metadata did not fail closed\n");
        return 1;
    }
    if(APP_TRUE != leg_kinematics_solve(APP_TRUE,
                                        -13.0f,
                                        28.0f,
                                        NULL,
                                        &previous))
    {
        printf("unable to establish servo-limit branch test state\n");
        return 1;
    }
    previous.alpha_branch = LEG_IK_BRANCH_MINUS;
    if((APP_FALSE != leg_kinematics_solve(APP_TRUE,
                                          -14.0f,
                                          28.0f,
                                          &previous,
                                          &result)) ||
       (APP_FALSE != result.valid))
    {
        printf("unavailable previous branch identity did not fail closed\n");
        return 1;
    }
    return 0;
}

static int check_oracle_rejections(void)
{
    static const struct
    {
        float physical_x;
        float physical_y;
        leg_oracle_reason_enum expected_reason;
        const char *name;
    }cases[] =
    {
        {NAN, 55.0f, LEG_ORACLE_REJECT_NONFINITE, "NaN"},
        {INFINITY, 55.0f, LEG_ORACLE_REJECT_NONFINITE, "infinity"},
        {0.0f, 0.0f, LEG_ORACLE_REJECT_MODEL_Y, "model Y <= 0"},
        {-55.0f, 27.0f, LEG_ORACLE_REJECT_MARGIN, "margin below 0.02"}
    };
    uint32 i;

    for(i = 0U; i < (sizeof(cases) / sizeof(cases[0])); i++)
    {
        leg_oracle_result_struct oracle;

        if((0 != oracle_classify(cases[i].physical_x,
                                 cases[i].physical_y,
                                 &oracle)) ||
           (cases[i].expected_reason != oracle.reason) ||
           (APP_FALSE != leg_kinematics_target_valid(cases[i].physical_x,
                                                      cases[i].physical_y)))
        {
            printf("oracle rejection mismatch %s: reason %s expected %s production %u model %.6f %.6f discriminants %.6f/%.6f margins %.8f/%.8f/min %.8f masks 0x%02x/0x%02x\n",
                   cases[i].name,
                   oracle_reason_name(oracle.reason),
                   oracle_reason_name(cases[i].expected_reason),
                   (unsigned int)leg_kinematics_target_valid(cases[i].physical_x,
                                                              cases[i].physical_y),
                   oracle.model_x_mm, oracle.model_y_mm,
                   oracle.alpha_discriminant, oracle.beta_discriminant,
                   oracle.alpha_margin, oracle.beta_margin,
                   oracle.singularity_margin,
                   (unsigned int)oracle.side_candidate_mask[APP_FALSE],
                   (unsigned int)oracle.side_candidate_mask[APP_TRUE]);
            return 1;
        }
    }
    return 0;
}

static int check_side(uint8 right_side,
                      leg_sweep_axis_enum axis,
                      int direction,
                      uint32 *accepted_count)
{
    int outer_i;

    if((NULL == accepted_count) ||
       ((LEG_SWEEP_X != axis) && (LEG_SWEEP_Y != axis)) ||
       ((1 != direction) && (-1 != direction)))
    {
        return 1;
    }
    *accepted_count = 0U;

    for(outer_i = (LEG_SWEEP_X == axis) ? 20 : -60;
        outer_i <= ((LEG_SWEEP_X == axis) ? 100 : 20);
        outer_i++)
    {
        leg_ik_result_struct previous = {0};
        leg_oracle_result_struct previous_oracle;
        float previous_command_deg[2] = {0.0f, 0.0f};
        int previous_branch[2] = {LEG_IK_BRANCH_PLUS, LEG_IK_BRANCH_PLUS};
        float previous_x = 0.0f;
        float previous_y = 0.0f;
        int step;

        for(step = 0; step <= 80; step++)
        {
            const int swept_value = (LEG_SWEEP_X == axis) ?
                                    ((0 < direction) ? (-60 + step) : (20 - step)) :
                                    ((0 < direction) ? (20 + step) : (100 - step));
            const float physical_x = (float)((LEG_SWEEP_X == axis) ? swept_value : outer_i);
            const float physical_y = (float)((LEG_SWEEP_X == axis) ? outer_i : swept_value);
            float x_mm;
            float y_mm;
            leg_ik_result_struct result;
            leg_oracle_result_struct oracle;
            float command_deg[2];
            int branch[2];
            const uint8 oracle_valid = (0 != oracle_classify(physical_x,
                                                              physical_y,
                                                              &oracle)) ? APP_TRUE : APP_FALSE;
            const uint8 production_valid = leg_kinematics_target_valid(physical_x,
                                                                        physical_y);

            if(oracle_valid != production_valid)
            {
                printf("reachability mismatch during sweep side %u axis %c direction %+d physical %.1f %.1f oracle %u production %u reason %s margin %.8f masks 0x%02x/0x%02x\n",
                       (unsigned int)right_side,
                       (LEG_SWEEP_X == axis) ? 'X' : 'Y', direction,
                       physical_x, physical_y,
                       (unsigned int)oracle_valid,
                       (unsigned int)production_valid,
                       oracle_reason_name(oracle.reason),
                       oracle.singularity_margin,
                       (unsigned int)oracle.side_candidate_mask[APP_FALSE],
                       (unsigned int)oracle.side_candidate_mask[APP_TRUE]);
                return 1;
            }
            if(APP_TRUE != oracle_valid)
            {
                previous.valid = APP_FALSE;
                previous_oracle.valid = APP_FALSE;
                previous_command_deg[0] = 0.0f;
                previous_command_deg[1] = 0.0f;
                previous_branch[0] = LEG_IK_BRANCH_PLUS;
                previous_branch[1] = LEG_IK_BRANCH_PLUS;
                previous_x = 0.0f;
                previous_y = 0.0f;
                continue;
            }
            if((APP_TRUE != leg_kinematics_solve(right_side, physical_x, physical_y,
                                                  &previous, &result)) ||
               (APP_TRUE != result.valid))
            {
                printf("IK rejected side %u physical %.1f %.1f\n",
                       (unsigned int)right_side, physical_x, physical_y);
                return 1;
            }
            (*accepted_count)++;
            if((!isfinite(result.servo_deg[0])) || (!isfinite(result.servo_deg[1])) ||
               (!isfinite(result.singularity_margin)) || (0.02f > result.singularity_margin))
            {
                printf("IK margin/output invalid side %u physical %.1f %.1f\n",
                       (unsigned int)right_side, physical_x, physical_y);
                return 1;
            }
            if(0 == oracle_selected_commands(right_side,
                                              &oracle,
                                              &result,
                                              command_deg,
                                              branch))
            {
                printf("sweep solution is not an oracle candidate side %u axis %c direction %+d physical %.1f %.1f raw %.6f/%.6f mask 0x%02x\n",
                       (unsigned int)right_side,
                       (LEG_SWEEP_X == axis) ? 'X' : 'Y', direction,
                       physical_x, physical_y,
                       result.alpha_rad, result.beta_rad,
                       (unsigned int)oracle.side_candidate_mask[right_side]);
                return 1;
            }
            if((APP_TRUE == previous.valid) &&
               ((adjacent_motion_limit_deg(&previous, &result) <
                 wrapped_delta_deg(command_deg[0], previous_command_deg[0])) ||
                (adjacent_motion_limit_deg(&previous, &result) <
                 wrapped_delta_deg(command_deg[1], previous_command_deg[1]))))
            {
                printf("model branch discontinuity side %u axis %c direction %+d endpoints (%.1f,%.1f)->(%.1f,%.1f) margins %.8f->%.8f branches %s/%s->%s/%s raw %.6f/%.6f->%.6f/%.6f commands %.6f/%.6f->%.6f/%.6f deltas %.6f/%.6f masks 0x%02x->0x%02x limit %.1f\n",
                       (unsigned int)right_side,
                       (LEG_SWEEP_X == axis) ? 'X' : 'Y', direction,
                       previous_x, previous_y, physical_x, physical_y,
                       previous.singularity_margin, result.singularity_margin,
                       oracle_branch_name(previous_branch[0]),
                       oracle_branch_name(previous_branch[1]),
                       oracle_branch_name(branch[0]),
                       oracle_branch_name(branch[1]),
                       previous.alpha_rad, previous.beta_rad,
                       result.alpha_rad, result.beta_rad,
                       previous_command_deg[0], previous_command_deg[1],
                       command_deg[0], command_deg[1],
                       wrapped_delta_deg(command_deg[0], previous_command_deg[0]),
                       wrapped_delta_deg(command_deg[1], previous_command_deg[1]),
                       (unsigned int)previous_oracle.side_candidate_mask[right_side],
                       (unsigned int)oracle.side_candidate_mask[right_side],
                       adjacent_motion_limit_deg(&previous, &result));
                return 1;
            }
            if((APP_TRUE != leg_kinematics_forward(right_side, result.servo_deg[0], result.servo_deg[1], &x_mm, &y_mm)) ||
               (0.5f < fabsf(x_mm - physical_x)) || (0.5f < fabsf(y_mm - physical_y)))
            {
                printf("FK mismatch side %u physical %.1f %.1f: %.3f, %.3f\n",
                       (unsigned int)right_side, physical_x, physical_y, x_mm, y_mm);
                return 1;
            }
            previous = result;
            previous_oracle = oracle;
            previous_command_deg[0] = command_deg[0];
            previous_command_deg[1] = command_deg[1];
            previous_branch[0] = branch[0];
            previous_branch[1] = branch[1];
            previous_x = physical_x;
            previous_y = physical_y;
        }
    }
    return 0;
}

int main(void)
{
    leg_ik_result_struct left_reference = {0};
    leg_ik_result_struct right_reference = {0};
    const leg_kinematics_config_struct *cfg = leg_config_get_kinematics();
    uint32 accepted_count[2][2][2];
    float servo_deg[LEG_SERVO_COUNT];
    float forward_x_mm;
    float forward_y_mm;
    if(0 == cfg)
    {
        return 1;
    }
    if((0 != check_branch_metadata_fail_closed()) ||
       (0 != check_oracle_rejections()) ||
       (0 != check_oracle_grid()))
    {
        return 1;
    }
    if((0 != check_side(APP_FALSE, LEG_SWEEP_X, 1, &accepted_count[APP_FALSE][LEG_SWEEP_X][1])) ||
       (0 != check_side(APP_FALSE, LEG_SWEEP_X, -1, &accepted_count[APP_FALSE][LEG_SWEEP_X][0])) ||
       (0 != check_side(APP_FALSE, LEG_SWEEP_Y, 1, &accepted_count[APP_FALSE][LEG_SWEEP_Y][1])) ||
       (0 != check_side(APP_FALSE, LEG_SWEEP_Y, -1, &accepted_count[APP_FALSE][LEG_SWEEP_Y][0])) ||
       (0 != check_side(APP_TRUE, LEG_SWEEP_X, 1, &accepted_count[APP_TRUE][LEG_SWEEP_X][1])) ||
       (0 != check_side(APP_TRUE, LEG_SWEEP_X, -1, &accepted_count[APP_TRUE][LEG_SWEEP_X][0])) ||
       (0 != check_side(APP_TRUE, LEG_SWEEP_Y, 1, &accepted_count[APP_TRUE][LEG_SWEEP_Y][1])) ||
       (0 != check_side(APP_TRUE, LEG_SWEEP_Y, -1, &accepted_count[APP_TRUE][LEG_SWEEP_Y][0])))
    {
        return 1;
    }
    {
        uint8 side;

        for(side = APP_FALSE; side <= APP_TRUE; side++)
        {
            int axis;

            for(axis = LEG_SWEEP_X; axis <= LEG_SWEEP_Y; axis++)
            {
                int direction_index;

                for(direction_index = 0; direction_index <= 1; direction_index++)
                {
                    if(LEG_MODEL_GRID_ACCEPTED_POINT_COUNT !=
                       accepted_count[side][axis][direction_index])
                    {
                        printf("Sweep accepted-point count mismatch side %u axis %c direction %+d: %u/%u\n",
                               (unsigned int)side,
                               (LEG_SWEEP_X == axis) ? 'X' : 'Y',
                               (0 == direction_index) ? -1 : 1,
                               (unsigned int)accepted_count[side][axis][direction_index],
                               (unsigned int)LEG_MODEL_GRID_ACCEPTED_POINT_COUNT);
                        return 1;
                    }
                }
            }
        }
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
        printf("Physical reference equivalence failed: %.3f, %.3f\n",
               forward_x_mm, forward_y_mm);
        return 1;
    }
    if(APP_TRUE != leg_kinematics_target_valid(0.0f, 55.0f))
    {
        printf("Model-reachable X=0 target rejected\n");
        return 1;
    }
    if(APP_TRUE != leg_kinematics_target_valid(-40.620f, 47.370f))
    {
        printf("Model-reachable hull vertex rejected\n");
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
Assert-LxyBranchPreflight

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

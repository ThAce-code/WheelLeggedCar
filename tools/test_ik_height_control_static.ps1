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
    $text = Get-Content $Path -Raw
    if($text -notmatch $Pattern) {
        throw $Message
    }
}

function Assert-NotContains {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if((Test-Path $Path) -and ((Get-Content $Path -Raw) -match $Pattern)) {
        throw $Message
    }
}

Assert-Contains "project/code/leg_kinematics.h" "leg_kinematics_solve" "Missing IK solve API."
Assert-Contains "project/code/leg_kinematics.h" "leg_kinematics_forward" "Missing IK forward-check API."
Assert-Contains "project/code/leg_kinematics.h" "singularity_margin" "IK result must publish singularity margin."
Assert-Contains "project/code/leg_kinematics.h" "const leg_ik_result_struct \*previous" "IK solve API must accept a previous solution."
Assert-Contains "project/code/leg_kinematics.c" "sqrt" "IK implementation must solve five-bar geometry."
Assert-Contains "project/code/leg_kinematics.c" "LEG_IK_BRANCH_PLUS" "IK implementation must use configured branches."
Assert-Contains "project/code/leg_kinematics.c" "x_min_mm" "IK must validate x workspace."
Assert-Contains "project/code/leg_kinematics.c" "y_min_mm" "IK must validate y workspace."
Assert-Contains "project/code/leg_kinematics.c" "ik_min_margin" "IK must reject candidates below the configured singularity margin."
Assert-Contains "project/code/leg_kinematics.c" "leg_kinematics_wrapped_distance" "IK must select continuous wrapped-angle candidates."
Assert-Contains "project/code/leg_kinematics.c" "projection" "FK must calculate the circle-intersection projection."
Assert-Contains "project/code/leg_kinematics.c" "atan2f\(b, a\)" "IK candidate generation must use the stable angle phase."
Assert-Contains "project/code/leg_kinematics.c" "atan2f\(root, c\)" "IK candidate generation must avoid the degenerate half-angle denominator."

Assert-Contains "project/code/leg_config.h" "leg_kinematics_config_struct" "Missing kinematics config struct."
Assert-Contains "project/code/leg_config.h" "leg_height_profile_struct" "Missing height profile struct."
Assert-Contains "project/code/leg_config.h" "LEG_IK_BRANCH_PLUS" "Missing IK branch enum."
Assert-Contains "project/code/leg_config.h" "l1_mm" "Missing configured link length L1."
Assert-Contains "project/code/leg_config.h" "default_height_mm" "Missing default height config."
Assert-Contains "project/code/leg_config.h" "max_height_accel_mm_s2" "Missing maximum height acceleration config."
Assert-Contains "project/code/leg_config.h" "max_height_jerk_mm_s3" "Missing maximum height jerk config."
Assert-Contains "project/code/leg_config.h" "height_position_kp_s" "Missing height position gain config."
Assert-Contains "project/code/leg_config.h" "height_rate_kp_s" "Missing height rate gain config."
Assert-Contains "project/code/leg_config.h" "height_settle_error_mm" "Missing height settle error config."
Assert-Contains "project/code/leg_config.h" "height_settle_ms" "Missing height settle time config."
Assert-Contains "project/code/leg_config.h" "ik_min_margin" "Missing IK margin config."
Assert-Contains "project/code/leg_config.h" "safe_support_height_mm" "Missing safe support height config."
Assert-Contains "project/code/leg_config.c" "max_height_speed_mm_s = 5\.0f" "Height transition speed must be conservative for empirical board validation."
Assert-Contains "project/code/leg_config.c" "max_height_accel_mm_s2 = 10\.0f" "Height transition acceleration must be conservative for empirical board validation."
Assert-Contains "project/code/leg_config.c" "max_height_jerk_mm_s3 = 80\.0f" "Height transition jerk must limit height-acceleration changes."
Assert-Contains "project/code/leg_config.c" "height_position_kp_s = 1\.0f" "Height position gain must create a bounded desired rate."
Assert-Contains "project/code/leg_config.c" "height_rate_kp_s = 4\.0f" "Height rate gain must create a damped desired acceleration."
Assert-Contains "project/code/leg_config.c" "low_height_mm = 45\.0f" "Phase 1 low command must use the empirical differential-height interval."
Assert-Contains "project/code/leg_config.c" "high_height_mm = 65\.0f" "Phase 1 high command must use the empirical differential-height interval."
Assert-Contains "project/code/leg_config.c" "default_height_mm = 55\.0f" "Phase 1 default command must start at the measured safe center height."
Assert-Contains "project/code/leg_config.c" "safe_support_height_mm = 55\.0f" "LOCK-to-height transition must restart from the measured safe center height."
Assert-Contains "project/code/app_config.h" "APP_SERVO_MAX_SPEED_DPS\s+\(90\.0f\)" "Servo PWM output slew limit must be conservative for 7.4V empirical height validation."

Assert-Contains "project/code/app_types.h" "leg_diag_struct" "Missing leg diagnostics."
Assert-Contains "project/code/app_types.h" "LEG_MOTION_LOCKED" "Missing leg locked motion state."
Assert-Contains "project/code/app_types.h" "LEG_MOTION_STABLE" "Missing leg stable motion state."
Assert-Contains "project/code/app_types.h" "LEG_MOTION_TRANSITION" "Missing leg transition motion state."
Assert-Contains "project/code/app_types.h" "LEG_MOTION_FAULT" "Missing leg fault motion state."
Assert-Contains "project/code/app_types.h" "LEG_FAULT_NONE" "Missing no-fault reason."
Assert-Contains "project/code/app_types.h" "LEG_FAULT_IK_INVALID" "Missing invalid IK fault reason."
Assert-Contains "project/code/app_types.h" "LEG_FAULT_IK_MARGIN" "Missing IK margin fault reason."
Assert-Contains "project/code/app_types.h" "LEG_FAULT_SERVO_LIMIT" "Missing servo limit fault reason."
Assert-Contains "project/code/app_types.h" "target_height_mm" "Leg diagnostics must expose target height."
Assert-Contains "project/code/app_types.h" "actual_height_mm" "Leg diagnostics must expose actual height."
Assert-Contains "project/code/app_types.h" "height_ref_mm" "Leg diagnostics must expose height reference."
Assert-Contains "project/code/app_types.h" "height_rate_mm_s" "Leg diagnostics must expose height rate."
Assert-Contains "project/code/app_types.h" "height_norm" "Leg diagnostics must expose normalized height."
Assert-Contains "project/code/app_types.h" "ik_margin" "Leg diagnostics must expose IK margin."
Assert-Contains "project/code/app_types.h" "drive_forward_limit_rpm" "Leg diagnostics must expose drive forward limit."
Assert-Contains "project/code/app_types.h" "ik_valid" "Leg diagnostics must expose IK validity."
Assert-Contains "project/code/app_types.h" "motion_state" "Leg diagnostics must expose motion state."
Assert-Contains "project/code/app_types.h" "fault_reason" "Leg diagnostics must expose fault reason."
Assert-Contains "project/code/app_types.h" "drive_allowed" "Leg diagnostics must expose drive permission."

Assert-Contains "project/code/control_leg.h" "control_leg_set_height" "Missing height command API."
Assert-Contains "project/code/control_leg.h" "control_leg_set_calib_angles" "Missing direct calibration API."
Assert-Contains "project/code/control_leg.h" "control_leg_get_diag" "Missing leg diagnostics getter."
Assert-Contains "project/code/control_leg.c" "LEG_MODE_HEIGHT" "Control leg must implement height mode."
Assert-Contains "project/code/control_leg.c" "LEG_MODE_IK_CALIB" "Control leg must implement IK calibration mode."
Assert-NotContains "project/code/control_leg.c" "static float control_leg_ramp_toward" "Empirical LH mode must not retain the unused trapezoidal-rate helper."
Assert-NotContains "project/code/control_leg.c" "static uint8 control_leg_apply_calib" "Empirical LH mode must not retain the unused IK calibration helper."
Assert-NotContains "project/code/control_leg.c" "control_leg_left_ik" "Empirical LH mode must not retain unused left IK state."
Assert-NotContains "project/code/control_leg.c" "control_leg_right_ik" "Empirical LH mode must not retain unused right IK state."
Assert-NotContains "project/code/control_leg.c" "height \+ \(pitch \* servo_cfg->mount_x\)" "Height mode must not use old direct servo mixer."
Assert-Contains "project/code/control_leg.c" "control_leg_height_ref_mm" "Height supervisor must retain a bounded height reference."
Assert-Contains "project/code/control_leg.c" "control_leg_height_rate_mm_s" "Height supervisor must retain a bounded height rate."
Assert-Contains "project/code/control_leg.c" "control_leg_height_accel_mm_s2" "Height supervisor must retain bounded acceleration state."
Assert-Contains "project/code/control_leg.c" "control_leg_motion_state" "Height supervisor must publish motion state."
Assert-Contains "project/code/control_leg.c" "control_leg_fault_reason" "Height supervisor must publish a fault reason."
Assert-Contains "project/code/control_leg.c" "control_leg_settle_start_ms" "Height supervisor must track stable settle timing."
Assert-Contains "project/code/control_leg.c" "max_height_jerk_mm_s3" "Height supervisor must use the configured jerk limit."
Assert-Contains "project/code/control_leg.c" "desired_rate_mm_s" "Height supervisor must derive a bounded rate from position error."
Assert-Contains "project/code/control_leg.c" "height_position_kp_s" "Height supervisor must use the configured position gain."
Assert-Contains "project/code/control_leg.c" "height_rate_kp_s" "Height supervisor must use the configured rate gain."
Assert-Contains "project/code/control_leg.c" "accel_delta_mm_s2" "Height supervisor must ramp acceleration by configured jerk."
Assert-Contains "project/code/control_leg.c" "control_leg_enter_fault" "Height command validation failures must enter the soft fault state."
Assert-Contains "project/code/control_leg.c" "LEG_MOTION_FAULT" "Soft fault must be externally observable."
Assert-Contains "project/code/app_types.h" "LEG_FAULT_IK_MARGIN" "Insufficient IK margin fault reason must remain available for geometry IK validation."
Assert-Contains "project/code/control_leg.c" "control_leg_write_safe_angles\(config\)" "Soft fault must command verified safe servo angles."
Assert-Contains "project/code/control_leg.c" "control_leg_height_ref_mm = profile->safe_support_height_mm;[\s\S]*control_leg_height_rate_mm_s = 0\.0f;[\s\S]*control_leg_write_safe_angles\(config\)" "LOCK mode diagnostics must return the open-loop height reference to the measured safe support height."
Assert-Contains "project/code/control_leg.c" "drive_allowed = APP_FALSE" "Soft fault must deny drive."
Assert-Contains "project/code/control_leg.c" "app_safety_is_fault" "Global app safety must remain higher priority than the soft fault."
Assert-Contains "project/code/control_leg.c" "open-loop command estimate" "Height diagnostics must document that actual height is an open-loop estimate."
Assert-Contains "project/code/actuator_servo.h" "actuator_servo_get_current_angle" "Servo diagnostics must read the limited PWM output command."
Assert-Contains "project/code/actuator_servo.c" "actuator_servo_get_current_angle" "Servo module must expose the limited PWM output command."
Assert-Contains "project/code/control_leg.c" "actuator_servo_get_current_angle" "Leg diagnostics must use the servo output command."

Assert-Contains "project/code/host_command.c" "'L' == line\[0\].*'H' == line\[1\]" "Host command must parse LH."
Assert-Contains "project/code/host_command.c" "'L' == line\[0\].*'I' == line\[1\].*'K' == line\[2\]" "Host command must parse LIK."
Assert-Contains "project/code/app_scheduler.c" "control_leg_update\(now_ms\)[\s\S]*control_chassis_update\(now_ms\)[\s\S]*control_balance_update\(now_ms\)" "Scheduler must update leg before chassis and balance."

Assert-Contains "project/code/control_balance.c" "control_leg_get_diag" "Balance must read leg height diagnostics."
Assert-Contains "project/code/control_balance.c" "balance_pitch_kp_low" "Balance must use height profile low gain."
Assert-Contains "project/code/control_balance.c" "balance_pitch_kp_high" "Balance must use height profile high gain."
Assert-Contains "project/code/control_chassis.c" "control_leg_get_diag" "Chassis must read leg height diagnostics."
Assert-Contains "project/code/control_chassis.c" "chassis_forward_limit_low_rpm" "Chassis must use height profile forward limits."
Assert-Contains "project/code/control_chassis.c" "LEG_MOTION_TRANSITION" "Chassis must use leg transition state instead of height-error inference."
Assert-Contains "project/code/control_chassis.c" "LEG_MOTION_FAULT" "Chassis must stop forward motion on a leg fault."
Assert-Contains "project/code/control_chassis.c" "drive_allowed" "Chassis must honor leg drive permission."
Assert-Contains "project/code/control_chassis.c" "effective_fast_enable" "Chassis must interlock fast blend without clearing the operator request."
Assert-NotContains "project/code/control_chassis.c" "target_height_mm - leg->actual_height_mm" "Chassis must not infer transitions from PWM-only height error."
Assert-Contains "project/code/control_balance.c" "LEG_MOTION_FAULT[\s\S]*control_balance_stop_output" "Balance must stop wheel output on leg fault."
Assert-Contains "project/code/control_balance.c" "LEG_MOTION_TRANSITION" "Balance must schedule gains during leg transition."
Assert-Contains "project/code/control_balance.c" "LEG_MOTION_STABLE" "Balance must schedule gains during stable height control."
Assert-Contains "project/code/control_balance.c" "height_ref_mm" "Balance scheduling must use the open-loop height reference."

Assert-Contains "project/code/telemetry.c" "float vofa_data\[65\]" "Telemetry must emit 65 floats."
Assert-Contains "project/code/telemetry.c" "vofa_data\[58\] = leg->height_ref_mm;[\s\S]*vofa_data\[64\] = \(float\)leg->drive_allowed;" "Telemetry must append leg transition state without reordering the existing frame."
Assert-Contains "tools/collect_balance_data.ps1" "\$FloatCount = 65" "Collector must parse 65 floats."
Assert-Contains "tools/calib_ik_servo.ps1" "\$FloatCount = 65" "Calibration tool must parse 65 floats."
Assert-Contains "tools/collect_balance_data.ps1" "leg_height_ref_mm" "Collector must write leg height reference."
Assert-Contains "tools/collect_balance_data.ps1" "leg_height_cmd_est_mm" "Collector must label PWM-only height as a command estimate."
Assert-Contains "tools/collect_balance_data.ps1" "servo0_output_deg" "Collector must label servo output command, not encoder angle."
Assert-Contains "tools/collect_balance_data.ps1" "leg_motion_state" "Collector must write leg motion state."
Assert-Contains "tools/collect_balance_data.ps1" "leg_fault_reason" "Collector must write leg fault reason."
Assert-Contains "tools/collect_balance_data.ps1" "leg_drive_allowed" "Collector must write leg drive permission."
Assert-NotContains "tools/collect_balance_data.ps1" "leg_actual_height_mm" "Collector CSV must not imply measured leg height."
Assert-Contains "tools/calib_ik_servo.ps1" "servo0_output_deg" "Calibration CSV must record servo output command."
Assert-Contains "tools/calib_ik_servo.ps1" "leg_height_ref_mm" "Calibration CSV must record height reference."
Assert-Contains "tools/collect_balance_data.ps1" "balance_pitch_kp_eff" "Collector must write effective balance gain."
Assert-Contains "tools/collect_balance_data.ps1" "chassis_forward_limit_eff_rpm" "Collector must write effective chassis limit."

Assert-Contains "project/code/control_leg.c" "CONTROL_LEG_EMPIRICAL_MM_PER_DELTA_DEG[\s\S]*0\.595f" "Height mode must use the measured differential servo height slope."
Assert-Contains "project/code/control_leg.c" "delta_deg = \(height_mm - CONTROL_LEG_EMPIRICAL_CENTER_HEIGHT_MM\) / CONTROL_LEG_EMPIRICAL_MM_PER_DELTA_DEG" "Height mode must map measured height to differential servo angle."
Assert-Contains "project/code/control_leg.c" "servo_fl_deg = CONTROL_LEG_EMPIRICAL_CENTER_SERVO_DEG \+ half_delta_deg[\s\S]*servo_fr_deg = CONTROL_LEG_EMPIRICAL_CENTER_SERVO_DEG - half_delta_deg[\s\S]*servo_rl_deg = CONTROL_LEG_EMPIRICAL_CENTER_SERVO_DEG - half_delta_deg[\s\S]*servo_rr_deg = CONTROL_LEG_EMPIRICAL_CENTER_SERVO_DEG \+ half_delta_deg" "Height mode must command the measured extend/retract differential pattern."
Assert-Contains "project/code/control_leg.c" "APP_FALSE == control_leg_servo_angle_valid\(0U, servo0_deg\)" "set_calib_angles must reject invalid LIK input instead of silently clamping."
Assert-Contains "project/code/control_leg.c" "return APP_FALSE;[\s\S]*control_leg_set_manual_angle\(0U, servo0_deg\)" "set_calib_angles must validate all LIK inputs before writing any servo command."
Assert-Contains "project/code/control_leg.c" "control_leg_mode = LEG_MODE_LOCK;[\s\S]*?control_leg_write_safe_angles\(config\)" "IK failure must write safe angles immediately."

Assert-Contains "project/code/control_leg.c" "config = leg_config_get\(\);[\s\S]*?if\(LEG_MOTION_FAULT == control_leg_motion_state\)[\s\S]*?control_leg_write_safe_angles\(config\);[\s\S]*?control_leg_publish_diag\(APP_FALSE, control_leg_run_enabled\(\)\);[\s\S]*?else[\s\S]*?case LEG_MODE_MANUAL" "A latched leg fault must keep manual and calibration update paths on safe angles."
Assert-Contains "project/code/control_leg.c" "uint8 control_leg_set_calib_angles[\s\S]*?if\(LEG_MOTION_FAULT == control_leg_motion_state\)[\s\S]*?return APP_FALSE;[\s\S]*?control_leg_servo_angle_valid\(0U, servo0_deg\)" "LIK calibration must be rejected while a leg fault is latched."
Assert-Contains "project/code/control_leg.c" "control_leg_apply_empirical_height[\s\S]*?LEG_FAULT_SERVO_LIMIT" "Height control must reject empirical commands that exceed servo limits."

Assert-Contains "tools/calib_ik_servo.ps1" "function ConvertTo-LikAngle" "Calibration points must be canonicalized to transmitted LIK integers."
Assert-Contains "tools/calib_ik_servo.ps1" '\[int\]\[math\]::Round\(\$Angle, 0, \[System\.MidpointRounding\]::ToEven\)' "LIK canonicalization must use the same nearest-even integer rounding as F0 formatting."
Assert-Contains "tools/calib_ik_servo.ps1" '\$a0 = ConvertTo-LikAngle -Angle \$pt\.A0; \$a1 = ConvertTo-LikAngle -Angle \$pt\.A1; \$a2 = ConvertTo-LikAngle -Angle \$pt\.A2; \$a3 = ConvertTo-LikAngle -Angle \$pt\.A3' "Calibration must canonicalize all four command angles before confirmation and recording."
Assert-Contains "tools/calib_ik_servo.ps1" '\$cmd = "LIK,\{0\},\{1\},\{2\},\{3\}" -f \$a0, \$a1, \$a2, \$a3' "LIK must transmit canonical integer command angles."
Assert-Contains "tools/calib_ik_servo.ps1" "Telemetry did not match this LIK command; visually confirm the pose before measuring" "Calibration must pause for manual measurement even when IK_CALIB telemetry is stale."
Assert-Contains "tools/calib_ik_servo.ps1" "telemetry_match" "Calibration CSV must record whether telemetry matched the commanded LIK angles."
Assert-NotContains "tools/calib_ik_servo.ps1" "ToleranceDeg" "Telemetry confirmation must not allow a degree tolerance."
Assert-Contains "tools/calib_ik_servo.ps1" '\$sampleId, \$label,\s*\$a0, \$a1, \$a2, \$a3,' "CSV command fields must record canonical transmitted command values."
Assert-Contains "tools/fit_leg_ik_calibration.py" '\("cmd_a0_deg", "servo0_output_deg", "servo0_deg"\)' "Calibration fit must prefer commanded LIK angles over stale IK_CALIB telemetry."

Assert-Contains "docs/leg-height-phase1-hardware-test.md" "\| Gate \| Build SHA \| Height start/end \(mm\) \| Safe-pose measured height \(mm\) \| Max pitch \(deg\) \| Max wheel RPM \| IK margin min \| IK faults \| Safety trips \| Result \| Notes \|" "Hardware record must contain the required gate table."
Assert-Contains "docs/leg-height-phase1-hardware-test.md" "bench/no wheel output[\s\S]*supported stationary at low/default/high heights[\s\S]*balance-in-place transition[\s\S]*low-speed straight transition[\s\S]*low-speed turn and stop" "Hardware gates must be listed in the immutable order."
Assert-Contains "docs/leg-height-phase1-hardware-test.md" "45 / 55 / 65" "Hardware Gate 1 must use the empirical differential Phase 1 height commands."
Assert-NotContains "docs/leg-height-phase1-hardware-test.md" "35 / 80 / 120" "Hardware Gate 1 must not use the non-monotonic 120 mm command."
Assert-Contains "docs/leg-height-phase1-hardware-test.md" "A failure blocks all later gates" "Hardware record must state that failures block later gates."
Assert-Contains "docs/leg-height-phase1-hardware-test.md" "IAR Embedded Workbench 9\.40\.1" "Hardware record must name the required IAR version."
Assert-Contains "docs/leg-height-phase1-hardware-test.md" "not run" "Unexecuted build or hardware gates must be marked not run, not passed."

Write-Host "ik height control static checks passed"

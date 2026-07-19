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
Assert-NotContains "project/code/leg_kinematics.c" "x_min_mm|x_max_mm|y_min_mm|y_max_mm" "IK must not retain stale rectangular workspace gates."
Assert-Contains "project/code/leg_kinematics.c" "leg_kinematics_target_valid" "IK must validate targets through the active model reachability path."
Assert-Contains "project/code/leg_kinematics.c" "leg_kinematics_map_candidate" "IK must map model-valid candidates to servo commands."
Assert-Contains "project/code/leg_kinematics.c" "ik_min_margin" "IK must reject candidates below the configured singularity margin."
Assert-Contains "project/code/leg_kinematics.c" "leg_kinematics_wrapped_distance" "IK must select continuous wrapped-angle candidates."
Assert-Contains "project/code/leg_kinematics.c" "projection" "FK must calculate the circle-intersection projection."
Assert-Contains "project/code/leg_kinematics.c" "atan2f\(b, a\)" "IK candidate generation must use the stable angle phase."
Assert-Contains "project/code/leg_kinematics.c" "atan2f\(root, c\)" "IK candidate generation must avoid the degenerate half-angle denominator."

Assert-Contains "project/code/leg_config.h" "leg_kinematics_config_struct" "Missing kinematics config struct."
Assert-Contains "project/code/leg_config.h" "leg_stance_profile_struct" "Missing legacy stance profile struct."
Assert-Contains "project/code/leg_config.h" "LEG_IK_BRANCH_PLUS" "Missing IK branch enum."
Assert-Contains "project/code/leg_config.h" "l1_mm" "Missing configured link length L1."
Assert-Contains "project/code/leg_config.c" "l1_mm = 60\.0f[\s\S]*l2_mm = 90\.0f[\s\S]*l3_mm = 90\.0f[\s\S]*l4_mm = 60\.0f[\s\S]*l5_mm = 37\.0f" "Five-bar geometry must match the measured 60 mm driven links, 90 mm passive links, and 37 mm servo-axis spacing."
Assert-Contains "tools/fit_leg_ik_calibration.py" "l1_mm: float = 60\.0[\s\S]*l2_mm: float = 90\.0[\s\S]*l3_mm: float = 90\.0[\s\S]*l4_mm: float = 60\.0[\s\S]*l5_mm: float = 37\.0" "Calibration fitter defaults must match the firmware five-bar geometry."
Assert-Contains "project/code/leg_config.h" "legacy_default_units" "Missing default legacy stance config."
Assert-Contains "project/code/leg_config.h" "legacy_max_accel_units_s2" "Missing maximum legacy stance acceleration config."
Assert-Contains "project/code/leg_config.h" "legacy_max_jerk_units_s3" "Missing maximum legacy stance jerk config."
Assert-Contains "project/code/leg_config.h" "legacy_position_kp_s" "Missing legacy stance position gain config."
Assert-Contains "project/code/leg_config.h" "legacy_rate_kp_s" "Missing legacy stance rate gain config."
Assert-Contains "project/code/leg_config.h" "legacy_settle_error_units" "Missing legacy stance settle error config."
Assert-Contains "project/code/leg_config.h" "legacy_settle_ms" "Missing legacy stance settle time config."
Assert-Contains "project/code/leg_config.h" "fast_stance_transition_ms" "Missing fixed-duration fast legacy stance config."
Assert-Contains "project/code/leg_config.h" "ik_min_margin" "Missing IK margin config."
Assert-Contains "project/code/leg_config.h" "legacy_safe_support_units" "Missing safe support legacy stance config."
Assert-Contains "project/code/leg_config.c" "legacy_max_rate_units_s = 20\.0f" "Fast-response bench profile must preserve the 20 legacy-units/s rate."
Assert-Contains "project/code/leg_config.c" "legacy_max_accel_units_s2 = 20\.0f" "Fast-response bench profile must preserve the 20 legacy-units/s² acceleration."
Assert-Contains "project/code/leg_config.c" "legacy_max_jerk_units_s3 = 80\.0f" "Legacy stance transition must preserve the jerk limit."
Assert-Contains "project/code/leg_config.c" "legacy_position_kp_s = 2\.0f" "Legacy stance profile must preserve its position gain."
Assert-Contains "project/code/leg_config.c" "legacy_rate_kp_s = 4\.0f" "Legacy stance profile must preserve its rate gain."
Assert-Contains "project/code/leg_config.c" "legacy_settle_error_units = 1\.0f" "Legacy stance profile must preserve its settle error."
Assert-Contains "project/code/leg_config.c" "fast_stance_transition_ms = 500U" "Fast legacy stance profile must complete in 500 ms."
Assert-Contains "project/code/leg_config.c" "legacy_low_units = 30\.0f" "Extended low legacy stance command must remain available."
Assert-Contains "project/code/leg_config.c" "legacy_high_units = 80\.0f" "Extended high legacy stance command must remain available."
Assert-Contains "project/code/leg_config.c" "legacy_default_units = 55\.0f" "Phase 1 default command must remain at legacy center 55."
Assert-Contains "project/code/leg_config.c" "legacy_safe_support_units = 55\.0f" "LOCK transition must restart from legacy safe support 55."
Assert-Contains "project/code/app_config.h" "APP_SERVO_MAX_SPEED_DPS\s+\(90\.0f\)" "Servo PWM output slew limit must be conservative for 7.4V empirical legacy stance validation."
Assert-Contains "project/code/app_config.h" "APP_LEG_FAST_SERVO_MAX_SPEED_DPS\s+\(180\.0f\)" "Fast legacy stance profile must use a scoped 180 dps servo slew cap."
Assert-Contains "project/code/app_config.h" "APP_LEG_DIRECT_STEP_TEST_ENABLE\s+\(0U\)" "Direct-step bench mode must default off."

Assert-Contains "project/code/app_types.h" "leg_diag_struct" "Missing leg diagnostics."
Assert-Contains "project/code/app_types.h" "LEG_MOTION_LOCKED" "Missing leg locked motion state."
Assert-Contains "project/code/app_types.h" "LEG_MOTION_STABLE" "Missing leg stable motion state."
Assert-Contains "project/code/app_types.h" "LEG_MOTION_TRANSITION" "Missing leg transition motion state."
Assert-Contains "project/code/app_types.h" "LEG_MOTION_FAULT" "Missing leg fault motion state."
Assert-Contains "project/code/app_types.h" "LEG_FAULT_NONE" "Missing no-fault reason."
Assert-Contains "project/code/app_types.h" "LEG_FAULT_IK_INVALID" "Missing invalid IK fault reason."
Assert-Contains "project/code/app_types.h" "LEG_FAULT_IK_MARGIN" "Missing IK margin fault reason."
Assert-Contains "project/code/app_types.h" "LEG_FAULT_SERVO_LIMIT" "Missing servo limit fault reason."
Assert-Contains "project/code/app_types.h" "legacy_stance_target_units" "Leg diagnostics must expose the legacy stance target."
Assert-NotContains "project/code/app_types.h" "actual_height_mm" "PWM-only diagnostics must not claim an actual height."
Assert-Contains "project/code/app_types.h" "legacy_stance_ref_units" "Leg diagnostics must expose the legacy stance reference."
Assert-Contains "project/code/app_types.h" "legacy_stance_rate_units_s" "Leg diagnostics must expose the legacy stance rate."
Assert-Contains "project/code/app_types.h" "legacy_stance_norm" "Leg diagnostics must expose normalized legacy stance."
Assert-Contains "project/code/app_types.h" "left_command_pose_body_mm" "Leg diagnostics must expose the left physical command pose."
Assert-Contains "project/code/app_types.h" "right_command_pose_body_mm" "Leg diagnostics must expose the right physical command pose."
Assert-Contains "project/code/app_types.h" "ik_margin" "Leg diagnostics must expose IK margin."
Assert-Contains "project/code/app_types.h" "drive_forward_limit_rpm" "Leg diagnostics must expose drive forward limit."
Assert-Contains "project/code/app_types.h" "ik_valid" "Leg diagnostics must expose IK validity."
Assert-Contains "project/code/app_types.h" "motion_state" "Leg diagnostics must expose motion state."
Assert-Contains "project/code/app_types.h" "fault_reason" "Leg diagnostics must expose fault reason."
Assert-Contains "project/code/app_types.h" "drive_allowed" "Leg diagnostics must expose drive permission."

Assert-Contains "project/code/control_leg.h" "control_leg_set_legacy_stance" "Missing legacy stance command API."
Assert-Contains "project/code/control_leg.h" "LEG_MODE_FAST_LEGACY_STANCE" "Missing fixed-duration fast legacy stance mode."
Assert-Contains "project/code/control_leg.h" "control_leg_set_fast_legacy_stance" "Missing fast legacy stance command API."
Assert-Contains "project/code/control_leg.h" "LEG_MODE_DIRECT_LEGACY_STANCE" "Missing direct-step legacy stance mode."
Assert-Contains "project/code/control_leg.h" "control_leg_set_direct_legacy_stance" "Missing direct legacy stance API."
Assert-Contains "project/code/control_leg.h" "control_leg_set_calib_angles" "Missing direct calibration API."
Assert-Contains "project/code/control_leg.h" "control_leg_get_diag" "Missing leg diagnostics getter."
Assert-Contains "project/code/control_leg.c" "LEG_MODE_LEGACY_STANCE" "Control leg must implement legacy stance mode."
Assert-Contains "project/code/control_leg.c" "LEG_MODE_FAST_LEGACY_STANCE" "Control leg must implement fixed-duration fast legacy stance mode."
Assert-Contains "project/code/control_leg.c" "LEG_MODE_IK_CALIB" "Control leg must implement IK calibration mode."
Assert-NotContains "project/code/control_leg.c" "static float control_leg_ramp_toward" "Empirical LH mode must not retain the unused trapezoidal-rate helper."
Assert-NotContains "project/code/control_leg.c" "static uint8 control_leg_apply_calib" "Empirical LH mode must not retain the unused IK calibration helper."
Assert-NotContains "project/code/control_leg.c" "control_leg_left_ik" "Empirical LH mode must not retain unused left IK state."
Assert-NotContains "project/code/control_leg.c" "control_leg_right_ik" "Empirical LH mode must not retain unused right IK state."
Assert-NotContains "project/code/control_leg.c" "height \+ \(pitch \* servo_cfg->mount_x\)" "Legacy stance mode must not use the old direct servo mixer."
Assert-Contains "project/code/control_leg.c" "control_leg_legacy_stance_ref_units" "Legacy stance supervisor must retain a bounded reference."
Assert-Contains "project/code/control_leg.c" "control_leg_legacy_stance_rate_units_s" "Legacy stance supervisor must retain a bounded rate."
Assert-Contains "project/code/control_leg.c" "control_leg_legacy_stance_accel_units_s2" "Legacy stance supervisor must retain bounded acceleration state."
Assert-Contains "project/code/control_leg.c" "control_leg_motion_state" "Legacy stance supervisor must publish motion state."
Assert-Contains "project/code/control_leg.c" "control_leg_fault_reason" "Legacy stance supervisor must publish a fault reason."
Assert-Contains "project/code/control_leg.c" "control_leg_settle_start_ms" "Legacy stance supervisor must track stable settle timing."
Assert-Contains "project/code/control_leg.c" "legacy_max_jerk_units_s3" "Legacy stance supervisor must use the configured jerk limit."
Assert-Contains "project/code/control_leg.c" "desired_rate_units_s" "Legacy stance supervisor must derive a bounded rate from position error."
Assert-Contains "project/code/control_leg.c" "legacy_position_kp_s" "Legacy stance supervisor must use the configured position gain."
Assert-Contains "project/code/control_leg.c" "legacy_rate_kp_s" "Legacy stance supervisor must use the configured rate gain."
Assert-Contains "project/code/control_leg.c" "accel_delta_units_s2" "Legacy stance supervisor must ramp acceleration by configured jerk."
Assert-Contains "project/code/control_leg.c" "fast_stance_transition_ms" "Fast legacy stance mode must use the configured 500 ms duration."
Assert-Contains "project/code/control_leg.c" "35\.0f - \(84\.0f \* p\)" "Fast legacy stance mode must use the seven-degree S7 blend."
Assert-Contains "project/code/control_leg.c" "control_leg_pose_start_deg" "S7 pose trajectory must store four-element start array."
Assert-Contains "project/code/control_leg.c" "control_leg_pose_target_deg" "S7 pose trajectory must store four-element target array."
Assert-Contains "project/code/control_leg.c" "2\.1875f" "S7 pose trajectory must use the normalized peak-speed factor."
Assert-Contains "project/code/control_leg.c" "actuator_servo_publish_cmd" "Leg controller must publish one atomic actuator frame."
Assert-Contains "project/code/control_leg.c" "LEG_MODE_FAST_LEGACY_STANCE[\s\S]*APP_LEG_FAST_SERVO_MAX_SPEED_DPS" "Only fast legacy stance mode may select the 180 dps servo slew cap."
Assert-Contains "project/code/control_leg.c" "APP_SERVO_MAX_SPEED_DPS" "Non-fast leg modes must restore the 90 dps servo slew cap."
Assert-Contains "project/code/control_leg.c" "control_leg_enter_fault" "Legacy stance command validation failures must enter the soft fault state."
Assert-Contains "project/code/control_leg.c" "LEG_MOTION_FAULT" "Soft fault must be externally observable."
Assert-Contains "project/code/app_types.h" "LEG_FAULT_IK_MARGIN" "Insufficient IK margin fault reason must remain available for geometry IK validation."
Assert-Contains "project/code/control_leg.c" "control_leg_write_safe_angles\(config\)" "Soft fault must command verified safe servo angles."
Assert-Contains "project/code/control_leg.c" "control_leg_legacy_stance_ref_units = profile->legacy_safe_support_units;[\s\S]*control_leg_legacy_stance_rate_units_s = 0\.0f;[\s\S]*control_leg_write_safe_angles\(config\)" "LOCK mode diagnostics must return the open-loop legacy stance reference to safe support."
Assert-Contains "project/code/control_leg.c" "drive_allowed = APP_FALSE" "Soft fault must deny drive."
Assert-Contains "project/code/control_leg.c" "app_safety_is_fault" "Global app safety must remain higher priority than the soft fault."
Assert-Contains "project/code/control_leg.c" "open-loop command estimate" "Physical command-pose diagnostics must document their open-loop provenance."
Assert-Contains "project/code/actuator_servo.h" "actuator_servo_get_current_angle" "Servo diagnostics must read the limited PWM output command."
Assert-Contains "project/code/actuator_servo.h" "actuator_servo_publish_cmd" "Servo module must expose atomic frame publication."
Assert-Contains "project/code/actuator_servo.h" "direct_bypass" "Servo module must accept a direct-step bypass flag."
Assert-Contains "project/code/actuator_servo.c" "actuator_servo_get_current_angle" "Servo module must expose the limited PWM output command."
Assert-Contains "project/code/actuator_servo.c" "speed_limit_dps" "Servo module must retain its active slew limit per frame."

Assert-Contains "project/code/control_leg.c" "actuator_servo_get_diag" "Control leg must snapshot actuator diagnostics."
Assert-Contains "project/code/control_leg.c" "control_leg_motion_can_stabilize" "Control leg must gate stable state on actuator settling."

Assert-Contains "project/code/control_leg.c" "control_leg_actuator_diag" "Leg diagnostics must use the actuator diagnostic snapshot."

Assert-Contains "project/code/host_command.c" "'L' == line\[0\].*'H' == line\[1\]" "Host command must parse LH."
Assert-Contains "project/code/host_command.c" "'L' == line\[0\].*'H' == line\[1\].*'F' == line\[2\]" "Host command must parse LHF."
Assert-Contains "project/code/host_command.c" "APP_LEG_DIRECT_STEP_TEST_ENABLE[\s\S]*'L' == line\[0\].*'J' == line\[1\]" "LJ parser must be compile guarded."
Assert-Contains "project/code/host_command.c" "'L' == line\[0\].*'I' == line\[1\].*'K' == line\[2\]" "Host command must parse LIK."
Assert-Contains "project/code/app_scheduler.c" "control_leg_update\(now_ms\)[\s\S]*control_chassis_update\(now_ms\)[\s\S]*control_balance_update\(now_ms\)" "Scheduler must update leg before chassis and balance."

Assert-Contains "project/code/control_balance.c" "control_leg_get_diag" "Balance must read leg stance diagnostics."
Assert-Contains "project/code/control_balance.c" "balance_pitch_kp_low" "Balance must use stance profile low gain."
Assert-Contains "project/code/control_balance.c" "balance_pitch_kp_high" "Balance must use stance profile high gain."
Assert-Contains "project/code/control_chassis.c" "control_leg_get_diag" "Chassis must read leg stance diagnostics."
Assert-Contains "project/code/control_chassis.c" "chassis_forward_limit_low_rpm" "Chassis must use stance profile forward limits."
Assert-Contains "project/code/control_chassis.c" "LEG_MOTION_TRANSITION" "Chassis must use leg transition state instead of legacy-stance-error inference."
Assert-Contains "project/code/control_chassis.c" "LEG_MOTION_FAULT" "Chassis must stop forward motion on a leg fault."
Assert-Contains "project/code/control_chassis.c" "drive_allowed" "Chassis must honor leg drive permission."
Assert-Contains "project/code/control_chassis.c" "effective_fast_enable" "Chassis must interlock fast blend without clearing the operator request."
Assert-NotContains "project/code/control_chassis.c" "legacy_stance_target_units - leg->actual_height_mm" "Chassis must not infer transitions from PWM-only stance error."
Assert-Contains "project/code/control_balance.c" "LEG_MOTION_FAULT[\s\S]*control_balance_stop_output" "Balance must stop wheel output on leg fault."
Assert-Contains "project/code/control_balance.c" "LEG_MOTION_TRANSITION" "Balance must schedule gains during leg transition."
Assert-Contains "project/code/control_balance.c" "LEG_MOTION_STABLE" "Balance must schedule gains during stable legacy stance control."
Assert-Contains "project/code/control_balance.c" "legacy_stance_ref_units" "Balance scheduling must use the open-loop legacy stance reference."

Assert-Contains "project/code/telemetry.c" "float vofa_data\[55\]" "Telemetry must emit 55 floats."
Assert-Contains "tools/collect_balance_data.ps1" "\$FloatCount = 55" "Collector must parse 55 floats."
Assert-Contains "tools/calib_ik_servo.ps1" "\$FloatCount = 55" "Calibration tool must parse 55 floats."
Assert-Contains "tools/collect_balance_data.ps1" "leg_legacy_stance_ref_units" "Collector must write the legacy stance reference."
Assert-Contains "tools/collect_balance_data.ps1" "leg_pose_status_flags" "Collector must retain packed pose status."
Assert-Contains "tools/collect_balance_data.ps1" "leg_left_pose_source" "Collector must decode left pose provenance."
Assert-Contains "tools/collect_balance_data.ps1" "leg_right_pose_source" "Collector must decode right pose provenance."
Assert-Contains "tools/collect_balance_data.ps1" "servo0_output_deg" "Collector must label servo output command, not encoder angle."
Assert-Contains "tools/collect_balance_data.ps1" "leg_motion_state" "Collector must write leg motion state."
Assert-Contains "tools/collect_balance_data.ps1" "leg_fault_reason" "Collector must write leg fault reason."
Assert-Contains "tools/collect_balance_data.ps1" "servo0_target_deg" "Collector must write servo target angles."
Assert-Contains "tools/collect_balance_data.ps1" "servo0_filtered_deg" "Collector must write servo filtered angles."
Assert-Contains "tools/collect_balance_data.ps1" "servo_max_error_deg" "Collector must write max servo error."
Assert-Contains "tools/collect_balance_data.ps1" "servo_settled" "Collector must write servo settled flag."
Assert-Contains "tools/collect_balance_data.ps1" "servo_s7_progress" "Collector must write S7 progress."
Assert-Contains "tools/collect_balance_data.ps1" "leg_drive_allowed" "Collector must write drive permission."
Assert-Contains "tools/collect_balance_data.ps1" "leg_drive_forward_limit_rpm" "Collector must write the forward drive limit."
Assert-Contains "tools/collect_balance_data.ps1" "servo_fast_mode" "Collector must write the active speed profile."
Assert-Contains "tools/collect_balance_data.ps1" "servo_direct_bypass" "Collector must write direct bypass."
Assert-Contains "tools/collect_balance_data.ps1" "servo_trajectory_mode" "Collector must write trajectory mode."
Assert-Contains "tools/collect_balance_data.ps1" "servo_s7_remaining_ms" "Collector must write S7 remaining time."
Assert-NotContains "tools/collect_balance_data.ps1" "leg_actual_height_mm" "Collector CSV must not imply measured leg height."
Assert-Contains "tools/calib_ik_servo.ps1" "servo0_output_deg" "Calibration CSV must record servo output command."
Assert-Contains "tools/calib_ik_servo.ps1" "legacy_stance_ref_units" "Calibration CSV must record the legacy stance reference."
Assert-Contains "tools/calib_ik_servo.ps1" "left_command_x_mm" "Calibration CSV must record physical command coordinates."
Assert-Contains "tools/calib_ik_servo.ps1" "right_pose_source" "Calibration CSV must record pose provenance."
Assert-Contains "tools/calib_ik_servo.ps1" 'drive_forward_limit_rpm\s*=\s*\$values\[40\]' "Calibration parser must read the forward drive limit."
Assert-Contains "tools/calib_ik_servo.ps1" 'drive_allowed\s*=\s*\$values\[41\]' "Calibration parser must read drive permission."
Assert-Contains "tools/calib_ik_servo.ps1" 'Frame\.servo_settled\) -lt 0\.5\) \{ return \$false \}' "Calibration matching must wait for the actuator settle window."
Assert-Contains "tools/collect_balance_data.ps1" "leg_ik_margin" "Collector must write IK margin."

Assert-Contains "project/code/control_leg.c" "CONTROL_LEG_LEGACY_UNITS_PER_DELTA_DEG[\s\S]*0\.595f" "Legacy stance mode must preserve the empirical differential servo slope."
Assert-Contains "project/code/control_leg.c" "delta_deg = \(stance_units - CONTROL_LEG_LEGACY_CENTER_UNITS\) / CONTROL_LEG_LEGACY_UNITS_PER_DELTA_DEG" "Legacy stance mode must map stance units to differential servo angle."
Assert-Contains "project/code/control_leg.c" "servo_fl_deg = CONTROL_LEG_EMPIRICAL_CENTER_SERVO_DEG \+ half_delta_deg[\s\S]*servo_fr_deg = CONTROL_LEG_EMPIRICAL_CENTER_SERVO_DEG - half_delta_deg[\s\S]*servo_rl_deg = CONTROL_LEG_EMPIRICAL_CENTER_SERVO_DEG - half_delta_deg[\s\S]*servo_rr_deg = CONTROL_LEG_EMPIRICAL_CENTER_SERVO_DEG \+ half_delta_deg" "Legacy stance mode must preserve the empirical extend/retract differential pattern."
Assert-Contains "project/code/control_leg.c" "APP_FALSE == control_leg_servo_angle_valid\(0U, servo0_deg\)" "set_calib_angles must reject invalid LIK input instead of silently clamping."
Assert-Contains "project/code/control_leg.c" "return APP_FALSE;[\s\S]*control_leg_set_manual_angle\(0U, servo0_deg\)" "set_calib_angles must validate all LIK inputs before writing any servo command."
Assert-Contains "project/code/control_leg.c" "control_leg_mode = LEG_MODE_LOCK;[\s\S]*?control_leg_write_safe_angles\(config\)" "IK failure must write safe angles immediately."

Assert-Contains "project/code/control_leg.c" "config = leg_config_get\(\);[\s\S]*?if\(LEG_MOTION_FAULT == control_leg_motion_state\)[\s\S]*?control_leg_write_safe_angles\(config\);[\s\S]*?control_leg_publish_diag\(APP_FALSE, control_leg_run_enabled\(\)\);[\s\S]*?else[\s\S]*?case LEG_MODE_MANUAL" "A latched leg fault must keep manual and calibration update paths on safe angles."
Assert-Contains "project/code/control_leg.c" "uint8 control_leg_set_calib_angles[\s\S]*?if\(LEG_MOTION_FAULT == control_leg_motion_state\)[\s\S]*?return APP_FALSE;[\s\S]*?control_leg_servo_angle_valid\(0U, servo0_deg\)" "LIK calibration must be rejected while a leg fault is latched."
Assert-Contains "project/code/control_leg.c" "control_leg_apply_legacy_stance[\s\S]*?LEG_FAULT_SERVO_LIMIT" "Legacy stance control must reject empirical commands that exceed servo limits."
Assert-Contains "project/code/control_leg.c" "LEG_MODE_DIRECT_LEGACY_STANCE[\s\S]*drive_allowed = APP_FALSE" "Direct legacy stance mode must deny chassis drive."
Assert-Contains "project/code/control_leg.c" "LEG_MODE_DIRECT_LEGACY_STANCE[\s\S]*APP_TRUE" "Direct legacy stance mode must set the bypass flag in the actuator frame."

Assert-Contains "tools/calib_ik_servo.ps1" "function ConvertTo-LikAngle" "Calibration points must be canonicalized to transmitted LIK integers."
Assert-Contains "tools/calib_ik_servo.ps1" '\[int\]\[math\]::Round\(\$Angle, 0, \[System\.MidpointRounding\]::ToEven\)' "LIK canonicalization must use the same nearest-even integer rounding as F0 formatting."
Assert-Contains "tools/calib_ik_servo.ps1" '\$a0 = ConvertTo-LikAngle -Angle \$pt\.A0; \$a1 = ConvertTo-LikAngle -Angle \$pt\.A1; \$a2 = ConvertTo-LikAngle -Angle \$pt\.A2; \$a3 = ConvertTo-LikAngle -Angle \$pt\.A3' "Calibration must canonicalize all four command angles before confirmation and recording."
Assert-Contains "tools/calib_ik_servo.ps1" '\$cmd = "LIK,\{0\},\{1\},\{2\},\{3\}" -f \$a0, \$a1, \$a2, \$a3' "LIK must transmit canonical integer command angles."
Assert-Contains "tools/calib_ik_servo.ps1" "Telemetry did not match this LIK command; visually confirm the pose before measuring" "Calibration must pause for manual measurement even when IK_CALIB telemetry is stale."
Assert-Contains "tools/calib_ik_servo.ps1" "telemetry_match" "Calibration CSV must record whether telemetry matched the commanded LIK angles."
Assert-NotContains "tools/calib_ik_servo.ps1" "ToleranceDeg" "Telemetry confirmation must not allow a degree tolerance."
Assert-Contains "tools/calib_ik_servo.ps1" '\$sampleId, \$label,\s*\$a0, \$a1, \$a2, \$a3,' "CSV command fields must record canonical transmitted command values."
Assert-Contains "tools/fit_leg_ik_calibration.py" '\("cmd_a0_deg", "servo0_output_deg", "servo0_deg"\)' "Calibration fit must prefer commanded LIK angles over stale IK_CALIB telemetry."

Assert-Contains "docs/leg-height-phase1-hardware-test.md" "\| Gate \| Build SHA \| Legacy stance start/end \(units\) \| Safe-pose BODY_WHEEL measurement \| Max pitch \(deg\) \| Max wheel RPM \| IK margin min \| IK faults \| Safety trips \| Result \| Notes \|" "Hardware record must contain the legacy-stance gate table."
Assert-Contains "docs/leg-height-phase1-hardware-test.md" "bench/no wheel output[\s\S]*supported stationary at low/default/high legacy stance[\s\S]*balance-in-place transition[\s\S]*low-speed straight transition[\s\S]*low-speed turn and stop" "Hardware gates must be listed in the immutable order."
Assert-Contains "docs/leg-height-phase1-hardware-test.md" "30 / 55 / 80" "Hardware Gate 1 must include the extended legacy stance endpoints."
Assert-NotContains "docs/leg-height-phase1-hardware-test.md" "35 / 80 / 120" "Hardware Gate 1 must not use the non-monotonic 120 mm command."
Assert-Contains "docs/leg-height-phase1-hardware-test.md" "A failure blocks all later gates" "Hardware record must state that failures block later gates."
Assert-Contains "docs/leg-height-phase1-hardware-test.md" "IAR Embedded Workbench 9\.40\.1" "Hardware record must name the required IAR version."
Assert-Contains "docs/leg-height-phase1-hardware-test.md" "not run" "Unexecuted build or hardware gates must be marked not run, not passed."

$runtimeFiles = @(
    "project/code/leg_config.h", "project/code/leg_config.c", "project/code/control_leg.h",
    "project/code/control_leg.c", "project/code/host_command.c", "project/code/app_config.h",
    "project/code/app_types.h", "project/code/control_balance.c", "project/code/control_chassis.c",
    "project/code/telemetry.c", "project/code/leg_kinematics.c", "tools/collect_balance_data.ps1",
    "tools/calib_ik_servo.ps1"
)
$staleLegacyNames = 'leg_height_profile_struct|leg_config_get_height_profile|low_height_mm|high_height_mm|default_height_mm|max_height_(speed|accel|jerk)_mm|height_(position|rate)_kp_s|height_settle_(error_mm|ms)|fast_height_transition_ms|safe_support_height_mm|LEG_MODE_(HEIGHT|FAST_HEIGHT|DIRECT_STEP)|control_leg_set_(fast_)?height|control_leg_set_direct_step_height|target_height_mm|height_ref_mm|height_rate_mm_s|\bheight_norm\b|leg_height_norm|CONTROL_LEG_EMPIRICAL_CENTER_HEIGHT_MM|CONTROL_LEG_EMPIRICAL_MM_PER_DELTA_DEG|Y_real\s*~='
foreach($runtimeFile in $runtimeFiles) {
    Assert-NotContains $runtimeFile $staleLegacyNames ("Stale legacy-stance runtime name remains in {0}." -f $runtimeFile)
}

$staleNumericTestNames = 'Step-HeightSupervisor|ReferenceMm|RateMmS|AccelMmS2|TargetMm|MaxSpeedMmS|MaxAccelMmS2|MaxJerkMmS3|Assert-JerkLimitedHeightTrajectory|Get-FastHeightReference|Assert-FastHeightTrajectory|StartMm|LowMm|HighMm|errorMm|desiredRateMmS|desiredAccelMmS2|accelDeltaMmS2|nextReferenceMm|previousAccelMmS2|referenceMm|rateMmS|accelMmS2|targetMm|previousMm|Height trajectory|Fast height'
Assert-NotContains "tools/test_leg_transition_numeric.ps1" $staleNumericTestNames "Numeric legacy-stance tests must not restore height/mm terminology for the 30/55/80 scalar."

Write-Host "ik legacy stance control static checks passed"

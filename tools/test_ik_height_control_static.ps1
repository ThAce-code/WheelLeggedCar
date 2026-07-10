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

Assert-Contains "project/code/leg_config.h" "leg_kinematics_config_struct" "Missing kinematics config struct."
Assert-Contains "project/code/leg_config.h" "leg_height_profile_struct" "Missing height profile struct."
Assert-Contains "project/code/leg_config.h" "LEG_IK_BRANCH_PLUS" "Missing IK branch enum."
Assert-Contains "project/code/leg_config.h" "l1_mm" "Missing configured link length L1."
Assert-Contains "project/code/leg_config.h" "default_height_mm" "Missing default height config."
Assert-Contains "project/code/leg_config.h" "max_height_accel_mm_s2" "Missing maximum height acceleration config."
Assert-Contains "project/code/leg_config.h" "height_settle_error_mm" "Missing height settle error config."
Assert-Contains "project/code/leg_config.h" "height_settle_ms" "Missing height settle time config."
Assert-Contains "project/code/leg_config.h" "ik_min_margin" "Missing IK margin config."
Assert-Contains "project/code/leg_config.h" "safe_support_height_mm" "Missing safe support height config."

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
Assert-Contains "project/code/control_leg.c" "leg_kinematics_solve" "Control leg must call IK solver."
Assert-NotContains "project/code/control_leg.c" "height \+ \(pitch \* servo_cfg->mount_x\)" "Height mode must not use old direct servo mixer."

Assert-Contains "project/code/host_command.c" "'L' == line\[0\].*'H' == line\[1\]" "Host command must parse LH."
Assert-Contains "project/code/host_command.c" "'L' == line\[0\].*'I' == line\[1\].*'K' == line\[2\]" "Host command must parse LIK."
Assert-Contains "project/code/app_scheduler.c" "control_leg_update\(now_ms\)[\s\S]*control_chassis_update\(now_ms\)[\s\S]*control_balance_update\(now_ms\)" "Scheduler must update leg before chassis and balance."

Assert-Contains "project/code/control_balance.c" "control_leg_get_diag" "Balance must read leg height diagnostics."
Assert-Contains "project/code/control_balance.c" "balance_pitch_kp_low" "Balance must use height profile low gain."
Assert-Contains "project/code/control_balance.c" "balance_pitch_kp_high" "Balance must use height profile high gain."
Assert-Contains "project/code/control_chassis.c" "control_leg_get_diag" "Chassis must read leg height diagnostics."
Assert-Contains "project/code/control_chassis.c" "chassis_forward_limit_low_rpm" "Chassis must use height profile forward limits."

Assert-Contains "project/code/telemetry.c" "float vofa_data\[58\]" "Telemetry must emit 58 floats."
Assert-Contains "tools/collect_balance_data.ps1" "\$FloatCount = 58" "Collector must parse 58 floats."
Assert-Contains "tools/collect_balance_data.ps1" "leg_actual_height_mm" "Collector must write leg height."
Assert-Contains "tools/collect_balance_data.ps1" "balance_pitch_kp_eff" "Collector must write effective balance gain."
Assert-Contains "tools/collect_balance_data.ps1" "chassis_forward_limit_eff_rpm" "Collector must write effective chassis limit."

Assert-Contains "project/code/control_leg.c" "static uint8 control_leg_apply_calib\(uint8 servo_index,[\s\S]*float \*calibrated_deg\)" "apply_calib must report failure instead of silently clamping calibrated angle."
Assert-Contains "project/code/control_leg.c" "APP_FALSE == control_leg_servo_angle_valid\(servo_index, calibrated\)" "apply_calib must reject calibrated angles outside per-servo limits."
Assert-Contains "project/code/control_leg.c" "control_leg_apply_calib\(LEG_SERVO_FL,[\s\S]*control_leg_apply_calib\(LEG_SERVO_RR" "Height mode must require all calibrated IK angles to validate."
Assert-Contains "project/code/control_leg.c" "APP_FALSE == control_leg_servo_angle_valid\(0U, servo0_deg\)" "set_calib_angles must reject invalid LIK input instead of silently clamping."
Assert-Contains "project/code/control_leg.c" "return APP_FALSE;[\s\S]*control_leg_set_manual_angle\(0U, servo0_deg\)" "set_calib_angles must validate all LIK inputs before writing any servo command."
Assert-Contains "project/code/control_leg.c" "control_leg_mode = LEG_MODE_LOCK;[\s\S]*?control_leg_write_safe_angles\(config\)" "IK failure must write safe angles immediately."

Assert-Contains "tools/calib_ik_servo.ps1" "function ConvertTo-LikAngle" "Calibration points must be canonicalized to transmitted LIK integers."
Assert-Contains "tools/calib_ik_servo.ps1" '\[int\]\[math\]::Round\(\$Angle, 0, \[System\.MidpointRounding\]::ToEven\)' "LIK canonicalization must use the same nearest-even integer rounding as F0 formatting."
Assert-Contains "tools/calib_ik_servo.ps1" '\$a0 = ConvertTo-LikAngle -Angle \$pt\.A0; \$a1 = ConvertTo-LikAngle -Angle \$pt\.A1; \$a2 = ConvertTo-LikAngle -Angle \$pt\.A2; \$a3 = ConvertTo-LikAngle -Angle \$pt\.A3' "Calibration must canonicalize all four command angles before confirmation and recording."
Assert-Contains "tools/calib_ik_servo.ps1" '\$cmd = "LIK,\{0\},\{1\},\{2\},\{3\}" -f \$a0, \$a1, \$a2, \$a3' "LIK must transmit canonical integer command angles."
Assert-Contains "tools/calib_ik_servo.ps1" '\[double\]\$Frame\.servo0_target_deg\) -ne \$A0' "Telemetry confirmation must compare canonical command targets exactly."
Assert-Contains "tools/calib_ik_servo.ps1" '\[double\]\$Frame\.servo1_target_deg\) -ne \$A1' "Telemetry confirmation must compare servo 1 canonical command target exactly."
Assert-Contains "tools/calib_ik_servo.ps1" '\[double\]\$Frame\.servo2_target_deg\) -ne \$A2' "Telemetry confirmation must compare servo 2 canonical command target exactly."
Assert-Contains "tools/calib_ik_servo.ps1" '\[double\]\$Frame\.servo3_target_deg\) -ne \$A3' "Telemetry confirmation must compare servo 3 canonical command target exactly."
Assert-NotContains "tools/calib_ik_servo.ps1" "ToleranceDeg" "Telemetry confirmation must not allow a degree tolerance."
Assert-Contains "tools/calib_ik_servo.ps1" '\$sampleId, \$label,\s*\$a0, \$a1, \$a2, \$a3,' "CSV command fields must record canonical transmitted command values."

Write-Host "ik height control static checks passed"

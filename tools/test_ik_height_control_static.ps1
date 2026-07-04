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
Assert-Contains "project/code/leg_kinematics.c" "sqrt" "IK implementation must solve five-bar geometry."
Assert-Contains "project/code/leg_kinematics.c" "LEG_IK_BRANCH_PLUS" "IK implementation must use configured branches."
Assert-Contains "project/code/leg_kinematics.c" "x_min_mm" "IK must validate x workspace."
Assert-Contains "project/code/leg_kinematics.c" "y_min_mm" "IK must validate y workspace."

Assert-Contains "project/code/leg_config.h" "leg_kinematics_config_struct" "Missing kinematics config struct."
Assert-Contains "project/code/leg_config.h" "leg_height_profile_struct" "Missing height profile struct."
Assert-Contains "project/code/leg_config.h" "LEG_IK_BRANCH_PLUS" "Missing IK branch enum."
Assert-Contains "project/code/leg_config.h" "l1_mm" "Missing configured link length L1."
Assert-Contains "project/code/leg_config.h" "default_height_mm" "Missing default height config."

Assert-Contains "project/code/app_types.h" "leg_diag_struct" "Missing leg diagnostics."
Assert-Contains "project/code/app_types.h" "target_height_mm" "Leg diagnostics must expose target height."
Assert-Contains "project/code/app_types.h" "actual_height_mm" "Leg diagnostics must expose actual height."
Assert-Contains "project/code/app_types.h" "height_norm" "Leg diagnostics must expose normalized height."
Assert-Contains "project/code/app_types.h" "ik_valid" "Leg diagnostics must expose IK validity."

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

Write-Host "ik height control static checks passed"

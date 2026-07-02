$ErrorActionPreference = "Stop"

function Assert-Contains {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if(-not (Select-String -Path $Path -Pattern $Pattern -Quiet)) {
        throw $Message
    }
}

function Assert-NotContains {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if(Select-String -Path $Path -Pattern $Pattern -Quiet) {
        throw $Message
    }
}

Assert-Contains "project/code/app_config.h" "APP_CHASSIS_DRIVE_RPM_LIMIT" "Missing drive RPM limit."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_TURN_RPM_LIMIT" "Missing turn RPM limit."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_FORWARD_RAMP_RPM_S" "Missing forward ramp limit."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_TURN_RAMP_RPM_S" "Missing turn ramp limit."
Assert-NotContains "project/code/app_config.h" "APP_CHASSIS_CMD_TIMEOUT_MS" "Chassis command timeout must stay disabled."

Assert-Contains "project/code/app_types.h" "target_forward_rpm" "Missing target forward field."
Assert-Contains "project/code/app_types.h" "target_turn_rpm" "Missing target turn field."
Assert-Contains "project/code/app_types.h" "actual_forward_rpm" "Missing actual forward field."
Assert-Contains "project/code/app_types.h" "actual_turn_rpm" "Missing actual turn field."
Assert-Contains "project/code/app_types.h" "last_update_ms" "Missing chassis last_update_ms field."

Assert-Contains "project/code/control_chassis.c" "control_chassis_ramp_toward" "Missing chassis ramp helper."
Assert-NotContains "project/code/control_chassis.c" "APP_CHASSIS_CMD_TIMEOUT_MS" "control_chassis must not timeout C commands."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_FORWARD_RAMP_RPM_S" "Missing forward ramp use."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_TURN_RAMP_RPM_S" "Missing turn ramp use."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_DRIVE_RPM_LIMIT" "Missing drive target clamp."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_TURN_RPM_LIMIT" "Missing turn target clamp."

Assert-Contains "project/code/host_command.c" "control_chassis_stop\(now_ms\)" "STOP/B,0 must still clear chassis immediately."

Assert-NotContains "project/code/control_chassis.c" "sensor_imu" "control_chassis must not depend on IMU."
Assert-NotContains "project/code/control_chassis.c" "actuator_motor" "control_chassis must not call motor actuator."
Assert-NotContains "project/code/control_chassis.c" "bldc_foc_uart" "control_chassis must not call BLDC UART."
Assert-NotContains "project/code/control_balance.c" "bldc_foc_uart" "control_balance must not call BLDC UART."

Write-Host "balance drive v1 static checks passed"

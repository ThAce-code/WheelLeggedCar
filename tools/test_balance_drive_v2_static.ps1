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

Assert-Contains "project/code/app_config.h" "APP_CHASSIS_FORWARD_RPM_LIMIT" "Missing forward speed limit."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_TURN_RATE_LIMIT_DPS" "Missing turn-rate limit."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_TURN_RATE_RAMP_DPS_S" "Missing turn-rate ramp."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_SPEED_KP" "Missing speed loop Kp."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_SPEED_KI" "Missing speed loop Ki."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_SPEED_INTEGRAL_LIMIT" "Missing speed integral limit."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_SPEED_PITCH_LIMIT_DEG" "Missing speed pitch limit."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_TURN_KP" "Missing turn loop Kp."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_TURN_RPM_LIMIT" "Missing turn output limit."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_DRIVE_GAIN_ABS_LIMIT" "Missing drive gain limit."
Assert-NotContains "project/code/app_config.h" "APP_CHASSIS_CMD_TIMEOUT_MS" "C command timeout must not be reintroduced."

Assert-Contains "project/code/app_types.h" "target_turn_dps" "chassis_cmd_struct must store target yaw-rate."
Assert-Contains "project/code/app_types.h" "actual_turn_dps" "chassis_cmd_struct must store ramped yaw-rate."
Assert-Contains "project/code/app_types.h" "speed_pitch_offset_deg" "chassis_cmd_struct must store speed pitch offset."
Assert-Contains "project/code/app_types.h" "turn_rpm" "chassis structs must store turn output RPM."
Assert-Contains "project/code/app_types.h" "speed_integral" "chassis_cmd_struct must store speed integral."
Assert-Contains "project/code/app_types.h" "forward_actual_rpm" "chassis_output_struct must expose measured forward speed."
Assert-Contains "project/code/app_types.h" "gyro_z_dps" "chassis_output_struct must expose yaw rate."
Assert-Contains "project/code/app_types.h" "pitch_setpoint_deg" "balance_diag_struct must expose effective pitch setpoint."

Assert-Contains "project/code/control_chassis.h" "control_chassis_set_drive_gain" "Missing runtime drive gain API."
Assert-Contains "project/code/control_chassis.c" "actuator_motor_get_motor_rpm_loop_diag" "control_chassis must read wheel speed feedback."
Assert-Contains "project/code/control_chassis.c" "sensor_imu_get_state" "control_chassis must read gyro_z yaw-rate."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_SPEED_KP" "control_chassis must use speed Kp."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_SPEED_KI" "control_chassis must use speed Ki."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_TURN_KP" "control_chassis must use turn Kp."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_SPEED_PITCH_LIMIT_DEG" "control_chassis must clamp pitch offset."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_TURN_RPM_LIMIT" "control_chassis must clamp turn output."
Assert-NotContains "project/code/control_chassis.c" "APP_CHASSIS_CMD_TIMEOUT_MS" "control_chassis must not timeout C commands."
Assert-NotContains "project/code/control_chassis.c" "actuator_motor_set_" "control_chassis must not command motor output."
Assert-NotContains "project/code/control_chassis.c" "bldc_foc_uart" "control_chassis must not call BLDC UART."
Assert-NotContains "project/code/control_chassis.c" "debug_read" "control_chassis must not parse host commands."

Assert-Contains "project/code/control_balance.c" "pitch_offset_deg" "control_balance must apply chassis pitch offset."
Assert-Contains "project/code/control_balance.c" "chassis->turn_rpm" "control_balance must apply chassis turn output."
Assert-Contains "project/code/control_balance.c" "pitch_setpoint_deg" "control_balance must publish effective pitch setpoint."
Assert-NotContains "project/code/control_balance.c" "bldc_foc_uart" "control_balance must not call BLDC UART."
Assert-NotContains "project/code/control_balance.c" "debug_read" "control_balance must not parse host commands."

Assert-Contains "project/code/host_command.c" "'D' == line\[1\]" "host_command must parse BD command."
Assert-Contains "project/code/host_command.c" "control_chassis_set_drive_gain" "BD must call chassis drive gain API."

Assert-Contains "project/code/telemetry.c" 'float vofa_data\[21\]' "Telemetry must emit 21-channel V2 frame."
Assert-Contains "tools/collect_balance_data.ps1" '\$FloatCount = 21' "Collector must parse 21 floats."
Assert-Contains "tools/collect_balance_data.ps1" "forward_target_rpm" "Collector must write forward target."
Assert-Contains "tools/collect_balance_data.ps1" "speed_pitch_offset_deg" "Collector must write speed pitch offset."
Assert-Contains "tools/collect_balance_data.ps1" "turn_target_dps" "Collector must write turn target."
Assert-Contains "tools/collect_balance_data.ps1" "gyro_z_dps" "Collector must write gyro_z."
Assert-Contains "tools/collect_balance_data.ps1" "turn_rpm" "Collector must write turn output."

Write-Host "balance drive v2 static checks passed"

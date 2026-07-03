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
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_TURN_GYRO_LPF_ALPHA" "Missing turn gyro low-pass alpha."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_TURN_GYRO_DEADBAND_DPS" "Missing turn gyro deadband."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_TURN_GYRO_STEP_LIMIT_DPS" "Missing turn gyro spike step limit."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_IMU_MAX_AGE_MS" "Missing chassis IMU age gate."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_WHEEL_MAX_AGE_MS" "Missing chassis wheel feedback age gate."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_TURN_ZERO_TARGET_DPS" "Missing turn zero-target threshold."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_FORWARD_ZERO_TARGET_RPM" "Missing forward zero-target threshold."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_TURN_INTEGRAL_DECAY" "Missing turn integral decay."
Assert-NotContains "project/code/app_config.h" "APP_CHASSIS_CMD_TIMEOUT_MS" "C command timeout must not be reintroduced."

Assert-Contains "project/code/app_types.h" "target_turn_dps" "chassis_cmd_struct must store target yaw-rate."
Assert-Contains "project/code/app_types.h" "actual_turn_dps" "chassis_cmd_struct must store ramped yaw-rate."
Assert-Contains "project/code/app_types.h" "speed_pitch_offset_deg" "chassis_cmd_struct must store speed pitch offset."
Assert-Contains "project/code/app_types.h" "turn_rpm" "chassis structs must store turn output RPM."
Assert-Contains "project/code/app_types.h" "speed_integral" "chassis_cmd_struct must store speed integral."
Assert-Contains "project/code/app_types.h" "forward_actual_rpm" "chassis_output_struct must expose measured forward speed."
Assert-Contains "project/code/app_types.h" "gyro_z_dps" "chassis_output_struct must expose yaw rate."
Assert-Contains "project/code/app_types.h" "pitch_setpoint_deg" "balance_diag_struct must expose effective pitch setpoint."
Assert-Contains "project/code/app_types.h" "gyro_z_raw_dps" "Chassis output must expose raw gyro_z."
Assert-Contains "project/code/app_types.h" "gyro_z_filtered_dps" "Chassis output must expose filtered gyro_z."
Assert-Contains "project/code/app_types.h" "turn_error_dps" "Chassis output must expose turn error."
Assert-Contains "project/code/app_types.h" "turn_integral" "Chassis output must expose turn integral."
Assert-Contains "project/code/app_types.h" "turn_ki" "Chassis command must store turn ki."
Assert-Contains "project/code/app_types.h" "imu_age_ms" "Chassis output must expose IMU age."
Assert-Contains "project/code/app_types.h" "wheel_age_ms" "Chassis output must expose wheel feedback age."
Assert-Contains "project/code/control_balance.c" "drive_gyro_z_raw_dps" "Balance diag must copy raw gyro_z."
Assert-Contains "project/code/control_balance.c" "drive_turn_integral" "Balance diag must copy turn integral."

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
Assert-Contains "project/code/control_chassis.c" "control_chassis_gyro_z_filtered_dps" "Turn loop must keep filtered gyro state."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_TURN_GYRO_LPF_ALPHA" "Turn loop must apply gyro low-pass."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_TURN_GYRO_STEP_LIMIT_DPS" "Turn loop must clamp gyro spikes."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_IMU_MAX_AGE_MS" "Turn loop must gate stale IMU samples."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_WHEEL_MAX_AGE_MS" "Turn loop must gate stale wheel feedback."
Assert-Contains "project/code/control_chassis.c" "APP_FALSE == wheel_feedback->online" "Turn loop must gate offline wheel feedback."
Assert-Contains "project/code/control_chassis.c" "control_chassis_reset_turn_filter" "Turn loop must reset filter state explicitly."
Assert-Contains "project/code/control_chassis.c" "turn_unsat_rpm" "Turn loop must compute unsaturated output."
Assert-Contains "project/code/control_chassis.c" "turn_saturated" "Turn loop must track output saturation."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_TURN_ZERO_TARGET_DPS" "Turn loop must detect zero-turn target."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_TURN_INTEGRAL_DECAY" "Turn loop must decay integral at zero target."
Assert-Contains "project/code/control_chassis.c" "control_chassis_cmd.turn_integral \*= APP_CHASSIS_TURN_INTEGRAL_DECAY" "Turn integral must decay when zero target is quiet."

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

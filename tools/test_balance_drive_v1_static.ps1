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

# V2 config: forward and turn-rate limits replace V1 drive/turn limits
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_FORWARD_RPM_LIMIT" "Missing forward RPM limit."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_TURN_RATE_LIMIT_DPS" "Missing turn-rate limit."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_FORWARD_RAMP_RPM_S" "Missing forward ramp."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_TURN_RATE_RAMP_DPS_S" "Missing turn-rate ramp."
Assert-Contains "project/code/app_config.h" "APP_CHASSIS_CMD_TIMEOUT_MS\s+\(500U\)" "Chassis command timeout must stop stale host motion."

# V2 struct fields
Assert-Contains "project/code/app_types.h" "target_forward_rpm" "Missing target forward field."
Assert-Contains "project/code/app_types.h" "target_turn_dps" "Missing target turn field."
Assert-Contains "project/code/app_types.h" "actual_forward_rpm" "Missing actual forward field."
Assert-Contains "project/code/app_types.h" "actual_turn_dps" "Missing actual turn field."
Assert-Contains "project/code/app_types.h" "last_update_ms" "Missing chassis last_update_ms field."

# V2: control_chassis now reads IMU and wheel feedback for outer loops
Assert-Contains "project/code/control_chassis.c" "sensor_imu_get_state" "control_chassis must read IMU for yaw-rate loop."
Assert-Contains "project/code/control_chassis.c" "actuator_motor_get_motor_rpm_loop_diag" "control_chassis must read wheel feedback."

# V2: control_chassis must NOT call motor actuator output or BLDC UART
Assert-NotContains "project/code/control_chassis.c" "actuator_motor_set_" "control_chassis must not command motor output."
Assert-NotContains "project/code/control_chassis.c" "bldc_foc_uart" "control_chassis must not call BLDC UART."
Assert-NotContains "project/code/control_balance.c" "bldc_foc_uart" "control_balance must not call BLDC UART."

# V2: ramping and clamping still present
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_FORWARD_RAMP_RPM_S" "Missing forward ramp use."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_TURN_RATE_RAMP_DPS_S" "Missing turn-rate ramp use."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_FORWARD_RPM_LIMIT" "Missing forward target clamp."
Assert-Contains "project/code/control_chassis.c" "APP_CHASSIS_TURN_RPM_LIMIT" "Missing turn output clamp."

# STOP / B,0 must still clear chassis
Assert-Contains "project/code/host_command.c" "control_chassis_stop\(now_ms\)" "STOP/B,0 must still clear chassis immediately."

Write-Host "balance drive v1 static checks passed (updated for V2 semantics)"

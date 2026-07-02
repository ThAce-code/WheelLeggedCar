$ErrorActionPreference = "Stop"

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

Assert-Contains "project/code/bldc_foc_uart.c" "#if APP_BLDC_USE_ASCII_COMMANDS" "ASCII helpers must be hidden when ASCII commands are disabled."
Assert-NotContains "project/code/actuator_motor.c" "static float actuator_motor_limit\(" "Unused actuator_motor_limit must be removed."
Assert-NotContains "project/code/actuator_motor.c" "static void actuator_motor_send_current\(void\)" "Unused actuator_motor_send_current must be removed."
Assert-Contains "project/code/actuator_motor.c" "#if !APP_MOTOR_RPM_LOOP_ENABLE" "Open duty fallback must be compiled only when RPM loop is disabled."
Assert-Contains "project/code/app_scheduler.c" "#if \(APP_IMU_USE_INT1 != 1U\)" "imu_last_ms must only exist when periodic IMU polling uses it."
Assert-Contains "project/code/app_scheduler.c" "static uint32 imu_last_ms" "imu_last_ms declaration must remain for periodic IMU polling."
Assert-NotContains "project/code/control_leg.c" "mode < LEG_MODE_LOCK" "Unsigned enum mode must not be compared below zero."

Write-Host "IAR warning cleanup static checks passed"

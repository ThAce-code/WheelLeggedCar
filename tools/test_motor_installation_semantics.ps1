$ErrorActionPreference = "Stop"

function Require-Pattern {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )

    if($Text -notmatch $Pattern) {
        throw $Message
    }
}

$config = Get-Content "project/code/app_config.h" -Raw
$motor = Get-Content "project/code/actuator_motor.c" -Raw
$telemetry = Get-Content "project/code/telemetry.c" -Raw
$collector = Get-Content "tools/collect_balance_data.ps1" -Raw
$diagnostics = Get-Content "tools/collect_bldc_diagnostics.ps1" -Raw
$projectMemory = Get-Content "PROJECT_MEMORY.md" -Raw

# The BLDC packet's channel order is opposite to the current vehicle installation.
# Normalize that once at the actuator boundary so every controller-side left/right
# value means physical vehicle left/right.
Require-Pattern $config 'APP_MOTOR_DRIVER_CHANNELS_SWAPPED\s+\(1U\)' 'Current vehicle installation must declare swapped BLDC channels.'
Require-Pattern $motor 'actuator_motor_feedback\.left_motor_rpm\s*=\s*raw->right_motor_rpm' 'Physical left feedback must come from BLDC channel 2.'
Require-Pattern $motor 'actuator_motor_feedback\.right_motor_rpm\s*=\s*raw->left_motor_rpm' 'Physical right feedback must come from BLDC channel 1.'
Require-Pattern $motor 'bldc_foc_uart_set_duty\(right_duty, left_duty\)' 'Physical left/right commands must be swapped back into BLDC packet order.'

# Preserve the operator-visible wire contract confirmed on the installed car:
# I9/I11 are physical left, I8/I10 are physical right.
Require-Pattern $telemetry 'vofa_data\[8\]\s*=\s*rpm_diag->right_motor_rpm' 'I8 must report physical right wheel RPM.'
Require-Pattern $telemetry 'vofa_data\[9\]\s*=\s*rpm_diag->left_motor_rpm' 'I9 must report physical left wheel RPM.'
Require-Pattern $telemetry 'vofa_data\[10\]\s*=\s*rpm_diag->right_duty' 'I10 must report physical right duty.'
Require-Pattern $telemetry 'vofa_data\[11\]\s*=\s*rpm_diag->left_duty' 'I11 must report physical left duty.'
Require-Pattern $collector 'right_motor_rpm\s*=\s*\$values\[8\][\s\S]*left_motor_rpm\s*=\s*\$values\[9\]' 'Balance collector must decode I8/I9 using physical installation semantics.'
Require-Pattern $diagnostics 'right_motor_rpm\s*=\s*\$values\[8\][\s\S]*left_motor_rpm\s*=\s*\$values\[9\]' 'BLDC collector must decode I8/I9 using physical installation semantics.'
Require-Pattern $projectMemory 'I8\s*=\s*right_motor_rpm[\s\S]*I9\s*=\s*left_motor_rpm[\s\S]*I10\s*=\s*right_duty[\s\S]*I11\s*=\s*left_duty' 'Project source of truth must document I9 as the physical left wheel.'

Write-Host "motor installation semantics checks passed"

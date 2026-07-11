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

function Reject-Pattern {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )
    if($Text -match $Pattern) {
        throw $Message
    }
}

$isr = Get-Content "project/user/cm7_0_isr.c" -Raw
$scheduler = Get-Content "project/code/app_scheduler.c" -Raw
$servo = Get-Content "project/code/actuator_servo.c" -Raw
$servoH = Get-Content "project/code/actuator_servo.h" -Raw
$telemetry = Get-Content "project/code/telemetry.c" -Raw
$telemetryH = Get-Content "project/code/telemetry.h" -Raw
$app = Get-Content "project/code/app.c" -Raw
$config = Get-Content "project/code/app_config.h" -Raw
$heightDoc = Get-Content "docs/leg-height-phase1-hardware-test.md" -Raw
$ikDoc = Get-Content "docs/leg-ik-zero-calibration-hardware-test.md" -Raw

Require-Pattern $isr 'pit0_ch1_isr[\s\S]*actuator_servo_tick_300hz\(\)' 'PIT_CH1 must call the servo tick.'
Reject-Pattern $scheduler 'actuator_servo_update\(' 'The cooperative scheduler must not run the actuator.'
Require-Pattern $servo 'actuator_servo_frame\s*\[\s*2\s*\]' 'Servo commands must use two frames.'
Require-Pattern $servo 'interrupt_global_disable\(\)' 'Frame flip and direct bypass need critical sections.'
Require-Pattern $servo 'servo_motion_step\(' 'The 300 Hz tick must use the production LPF unit.'
Require-Pattern $servo 'servo_motion_apply_immediate\(' 'Direct-step must synchronize all motion states.'

Require-Pattern $config 'APP_TELEMETRY_PERIOD_MS\s+\(10U\)' 'Telemetry generation must run at 10 ms.'
Require-Pattern $telemetry 'float vofa_data\[46\]' 'Servo validation telemetry must emit 46 floats.'
Require-Pattern $telemetry 'vofa_data\[40\]\s*=\s*leg->drive_forward_limit_rpm' 'Telemetry must retain the forward drive limit.'
Require-Pattern $telemetry 'vofa_data\[41\]\s*=\s*\(float\)leg->drive_allowed' 'Telemetry must retain drive permission.'
Require-Pattern $telemetry 'vofa_data\[42\]\s*=\s*\(float\)leg->servo_fast_mode' 'Telemetry must expose the active speed profile.'
Require-Pattern $telemetry 'vofa_data\[43\]\s*=\s*\(float\)leg->servo_direct_bypass' 'Telemetry must expose direct bypass.'
Require-Pattern $telemetry 'vofa_data\[44\]\s*=\s*\(float\)leg->servo_trajectory_mode' 'Telemetry must expose trajectory mode.'
Require-Pattern $telemetry 'vofa_data\[45\]\s*=\s*\(float\)leg->servo_s7_remaining_ms' 'Telemetry must expose S7 remaining time.'
Require-Pattern $telemetryH 'void telemetry_service\(void\)' 'Telemetry must expose a nonblocking TX service.'
Require-Pattern $app 'app_scheduler_run_pending\(\);[\s\S]*telemetry_service\(\);' 'The main loop must service telemetry outside the scheduler tick.'
Require-Pattern $telemetry 'Cy_SCB_WriteArray\(' 'Telemetry TX must use the nonblocking SCB FIFO writer.'
Reject-Pattern $telemetry 'debug_send_buffer\(' 'Telemetry must not use the blocking debug sender.'
Reject-Pattern $telemetry 'uart_write_buffer\(' 'Telemetry must not use the blocking UART writer.'

$frameBytes = (46 * 4) + 4
$txMs = $frameBytes * 10.0 * 1000.0 / 460800.0
if(($txMs / 10.0) -ge 0.5) {
    throw 'Telemetry line utilization must remain below 50 percent.'
}

Require-Pattern $heightDoc 'fresh 300 Hz validation' 'Height test must define a fresh 300 Hz validation.'
Require-Pattern $heightDoc '3\.333 ms' 'Height test must require a scope period check.'
Require-Pattern $heightDoc '1\.5 ms' 'Height test must require the 90-degree pulse check.'
Require-Pattern $heightDoc 'temperature' 'Height test must include a temperature gate.'
Require-Pattern $ikDoc 'LIKREF' 'IK test must retain the safe reference prerequisite.'
Require-Pattern $ikDoc 'return to the 50 Hz build' 'IK test must define the rollback action.'

Write-Host "servo 300 Hz integration static checks passed"

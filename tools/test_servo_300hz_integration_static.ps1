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

Require-Pattern $isr 'pit0_ch1_isr[\s\S]*actuator_servo_tick_300hz\(\)' 'PIT_CH1 must call the servo tick.'
Reject-Pattern $scheduler 'actuator_servo_update\(' 'The cooperative scheduler must not run the actuator.'
Require-Pattern $servo 'actuator_servo_frame\s*\[\s*2\s*\]' 'Servo commands must use two frames.'
Require-Pattern $servo 'interrupt_global_disable\(\)' 'Frame flip and direct bypass need critical sections.'
Require-Pattern $servo 'servo_motion_step\(' 'The 300 Hz tick must use the production LPF unit.'
Require-Pattern $servo 'servo_motion_apply_immediate\(' 'Direct-step must synchronize all motion states.'

Write-Host "servo 300 Hz integration static checks passed"

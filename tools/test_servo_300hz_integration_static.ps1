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
$heightDoc = Get-Content "docs/leg-height-phase1-hardware-test.md" -Raw
$ikDoc = Get-Content "docs/leg-ik-zero-calibration-hardware-test.md" -Raw

Require-Pattern $isr 'pit0_ch1_isr[\s\S]*actuator_servo_tick_300hz\(\)' 'PIT_CH1 must call the servo tick.'
Reject-Pattern $scheduler 'actuator_servo_update\(' 'The cooperative scheduler must not run the actuator.'
Require-Pattern $servo 'actuator_servo_frame\s*\[\s*2\s*\]' 'Servo commands must use two frames.'
Require-Pattern $servo 'interrupt_global_disable\(\)' 'Frame flip and direct bypass need critical sections.'
Require-Pattern $servo 'servo_motion_step\(' 'The 300 Hz tick must use the production LPF unit.'
Require-Pattern $servo 'servo_motion_apply_immediate\(' 'Direct-step must synchronize all motion states.'

Require-Pattern $heightDoc 'fresh 300 Hz validation' 'Height test must define a fresh 300 Hz validation.'
Require-Pattern $heightDoc '3\.333 ms' 'Height test must require a scope period check.'
Require-Pattern $heightDoc '1\.5 ms' 'Height test must require the 90-degree pulse check.'
Require-Pattern $heightDoc 'temperature' 'Height test must include a temperature gate.'
Require-Pattern $ikDoc 'LIKREF' 'IK test must retain the safe reference prerequisite.'
Require-Pattern $ikDoc 'return to the 50 Hz build' 'IK test must define the rollback action.'

Write-Host "servo 300 Hz integration static checks passed"

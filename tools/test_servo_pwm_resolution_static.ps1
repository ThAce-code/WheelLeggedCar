$ErrorActionPreference = "Stop"

$pwmHeader = Get-Content "libraries/zf_driver/zf_driver_pwm.h" -Raw
$servoSource = Get-Content "project/code/actuator_servo.c" -Raw

if($pwmHeader -notmatch "#define\s+PWM_DUTY_MAX\s+20000")
{
    throw "Servo PWM duty range must use 20000 steps for a 1 us grid at a 20 ms period."
}

if($servoSource -notmatch "pulse_us \* \(float\)PWM_DUTY_MAX / \(float\)APP_SERVO_PWM_PERIOD_US")
{
    throw "Servo angle mapping must scale with the configured PWM duty range."
}

Write-Host "servo PWM 1 us resolution static check passed"

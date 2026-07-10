$ErrorActionPreference = "Stop"

$config = Get-Content "project/code/app_config.h" -Raw
$servo = Get-Content "project/code/actuator_servo.c" -Raw

if($config -notmatch "APP_SERVO_PWM_FREQ_HZ\s+\(50U\)")
{
    throw "Production PWM frequency must revert to the validated 50 Hz frame rate."
}
if($config -notmatch "APP_SERVO_PWM_PERIOD_US\s+\(1000000U / APP_SERVO_PWM_FREQ_HZ\)")
{
    throw "Servo pulse period must derive from the selected PWM frequency."
}
if($servo -notmatch "pwm_init\(actuator_servo_pwm_ch\[i\], APP_SERVO_PWM_FREQ_HZ, 0\)")
{
    throw "Servo hardware PWM must use the configured frequency."
}

Write-Host "servo PWM 50 Hz production static check passed"

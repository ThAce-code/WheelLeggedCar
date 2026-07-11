$ErrorActionPreference = "Stop"

$config = Get-Content "project/code/app_config.h" -Raw
$main = Get-Content "project/user/main_cm7_0.c" -Raw

if($config -notmatch "APP_SERVO_PWM_FREQ_HZ\s+\(300U\)") {
    throw "Servo PWM frequency must be 300 Hz."
}
if($config -notmatch "APP_SERVO_CONTROL_PERIOD_US\s+\(1000000U / APP_SERVO_PWM_FREQ_HZ\)") {
    throw "Servo control period must derive from the PWM frequency."
}
if($main -notmatch "pit_us_init\(PIT_CH1, APP_SERVO_CONTROL_PERIOD_US\)") {
    throw "PIT_CH1 must run the 300 Hz servo actuator tick."
}

Write-Host "servo PWM 300 Hz production static check passed"

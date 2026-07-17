$ErrorActionPreference = "Stop"

$servoSource = Get-Content "project/code/actuator_servo.c" -Raw

if($servoSource -notmatch "pulse_us \* \(float\)PWM_DUTY_MAX \*[\s\S]*\(float\)APP_SERVO_PWM_FREQ_HZ / 1000000\.0f") {
    throw "Servo angle mapping must use the frequency-based duty formula."
}

function Convert-PulseToDuty {
    param([double]$PulseUs)
    $pwmDutyMax = 20000.0
    $freqHz = 300.0
    return [int]($PulseUs * $pwmDutyMax * $freqHz / 1000000.0)
}

function Convert-AngleToDuty {
    param(
        [double]$AngleDeg,
        [double]$PulseTrimUs = 0.0
    )
    $pulseUs = 500.0 + $AngleDeg * (2500.0 - 500.0) / 180.0 + $PulseTrimUs
    return Convert-PulseToDuty $pulseUs
}

function Assert-Equal {
    param(
        [int]$Actual,
        [int]$Expected,
        [string]$Message
    )
    if($Actual -ne $Expected) {
        throw ("{0}: expected {1}, got {2}" -f $Message, $Expected, $Actual)
    }
}

Assert-Equal (Convert-PulseToDuty 500) 3000 "500 us duty"
Assert-Equal (Convert-PulseToDuty 1500) 9000 "1500 us duty"
Assert-Equal (Convert-PulseToDuty 2500) 15000 "2500 us duty"
Assert-Equal (Convert-AngleToDuty 90.0 -PulseTrimUs (-11.111111)) 8933 "servo0 logical 90 deg trimmed duty"
Assert-Equal (Convert-AngleToDuty 90.0) 9000 "untrimmed logical 90 deg duty"

$legConfig = Get-Content "project/code/leg_config.c" -Raw
if($legConfig -notmatch '\{0,\s*90\.0f,\s*90\.0f,') {
    throw "Servo0 logical neutral angle must remain 90 degrees."
}

$appConfig = Get-Content "project/code/app_config.h" -Raw
if($appConfig -notmatch 'APP_SERVO0_PULSE_TRIM_US\s+\(-11\.111111f\)') {
    throw "Servo0 pulse trim must map logical 90 degrees to duty 8933."
}
if($servoSource -notmatch 'actuator_servo_pulse_trim_us\[index\]') {
    throw "Servo write path must apply the per-channel pulse trim."
}

Write-Host "servo PWM 300 Hz resolution static check passed"

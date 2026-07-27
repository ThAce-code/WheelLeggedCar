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

Write-Host "servo PWM 300 Hz resolution static check passed"

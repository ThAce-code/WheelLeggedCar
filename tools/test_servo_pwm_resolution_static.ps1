$ErrorActionPreference = "Stop"

$servoSource = Get-Content "project/code/actuator_servo.c" -Raw
$configSource = Get-Content "project/code/app_config.h" -Raw

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

Require-Pattern $servoSource "pulse_us \* \(float\)PWM_DUTY_MAX \*[\s\S]*\(float\)APP_SERVO_PWM_FREQ_HZ / 1000000\.0f" "Servo angle mapping must use the frequency-based duty formula."
Require-Pattern $configSource "APP_SERVO0_PULSE_TRIM_US\s+\(-11\.111111f\)" "Servo 0 trim must match calibration."
Require-Pattern $configSource "APP_SERVO1_PULSE_TRIM_US\s+\(0\.0f\)" "Servo 1 trim must remain zero."
Require-Pattern $configSource "APP_SERVO2_PULSE_TRIM_US\s+\(0\.0f\)" "Servo 2 trim must remain zero."
Require-Pattern $configSource "APP_SERVO3_PULSE_TRIM_US\s+\(0\.0f\)" "Servo 3 trim must remain zero."
Require-Pattern $servoSource "actuator_servo_pulse_trim_us\s*\[\s*APP_SERVO_COUNT\s*\][\s\S]*APP_SERVO0_PULSE_TRIM_US" "Servo trim table must use the calibrated channel constants."
Require-Pattern $servoSource "static uint32 actuator_servo_angle_to_duty_with_trim\(uint8 index, float angle_deg\)" "Trimmed mapping must remain private."
Require-Pattern $servoSource "static void actuator_servo_write\(uint8 index, float angle_deg\)[\s\S]*actuator_servo_angle_to_duty_with_trim\(index, angle_deg\)" "Hardware writes must use the trimmed mapping."
Require-Pattern $servoSource "pulse_us \+= actuator_servo_pulse_trim_us\[index\];[\s\S]*APP_SERVO_MIN_PULSE_US > pulse_us[\s\S]*APP_SERVO_MAX_PULSE_US < pulse_us" "Trimmed pulse must clamp to the configured endpoints."

function Convert-PulseToDuty {
    param([double]$PulseUs)
    $pwmDutyMax = 20000.0
    $freqHz = 300.0
    return [int]($PulseUs * $pwmDutyMax * $freqHz / 1000000.0)
}

function Convert-TrimmedPulseToDuty {
    param(
        [double]$PulseUs,
        [double]$TrimUs
    )
    $trimmedPulseUs = $PulseUs + $TrimUs
    if($trimmedPulseUs -lt 500.0) {
        $trimmedPulseUs = 500.0
    }
    if($trimmedPulseUs -gt 2500.0) {
        $trimmedPulseUs = 2500.0
    }
    return Convert-PulseToDuty $trimmedPulseUs
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
Assert-Equal (Convert-TrimmedPulseToDuty 1500 -11.111111) 8933 "Servo 0 logical 90 degree hardware duty"
Assert-Equal (Convert-TrimmedPulseToDuty 500 -11.111111) 3000 "Servo 0 lower trim clamp"
Assert-Equal (Convert-TrimmedPulseToDuty 2500 11.111111) 15000 "Servo upper trim clamp"

Write-Host "servo PWM 300 Hz resolution static check passed"

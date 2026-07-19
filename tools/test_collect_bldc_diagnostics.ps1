$ErrorActionPreference = "Stop"

$scriptPath = Join-Path $PSScriptRoot "collect_bldc_diagnostics.ps1"
. $scriptPath -LoadOnly

function Assert-True {
    param([bool]$Condition, [string]$Message)
    if(-not $Condition) {
        throw $Message
    }
}

function Assert-Near {
    param([double]$Actual, [double]$Expected, [double]$Tolerance, [string]$Message)
    if([Math]::Abs($Actual - $Expected) -gt $Tolerance) {
        throw ("{0}: actual={1}, expected={2}" -f $Message, $Actual, $Expected)
    }
}

$schedule = @(Parse-CommandSchedule -Text "0:STOP;1:M,120;4:D,300")
Assert-True ($schedule.Count -eq 3) "expected three scheduled commands"
Assert-Near $schedule[1].AtSeconds 1.0 0.0001 "M command time"

$blocked = $false
try {
    Assert-CommandScheduleSafe -Schedule $schedule -AllowMotion:$false
} catch {
    $blocked = $_.Exception.Message -match "AllowMotion"
}
Assert-True $blocked "motion schedule must require -AllowMotion"
Assert-CommandScheduleSafe -Schedule @((Parse-CommandSchedule -Text "0:STOP;1:B,1")) -AllowMotion:$false
Assert-CommandScheduleSafe -Schedule $schedule -AllowMotion:$true

$mExpectation = Get-CommandExpectation -Command "M,-150"
Assert-True ($mExpectation.Mode -eq "rpm") "M command mode"
Assert-Near $mExpectation.TargetRpm -150.0 0.0001 "M target"
$dExpectation = Get-CommandExpectation -Command "D,500"
Assert-True ($dExpectation.Mode -eq "duty") "D command mode"
Assert-Near $dExpectation.TargetDuty 500.0 0.0001 "D target"

function Add-JustFloatFrame {
    param(
        [System.Collections.Generic.List[byte]]$Buffer,
        [single[]]$FrameValues
    )

    foreach($value in $FrameValues) {
        foreach($byte in [BitConverter]::GetBytes($value)) {
            $Buffer.Add($byte)
        }
    }
    foreach($byte in [byte[]](0x00, 0x00, 0x80, 0x7F)) {
        $Buffer.Add($byte)
    }
}

function Add-Bytes {
    param(
        [System.Collections.Generic.List[byte]]$Buffer,
        [int]$Count,
        [byte]$Value = 0x55
    )

    for($i = 0; $i -lt $Count; $i++) {
        $Buffer.Add($Value)
    }
}

$values = New-Object 'single[]' 72
Assert-True ($FloatCount -eq 72) "BLDC diagnostic collector must require the 72-float UART0 contract"
$values[0] = 1234.0
$values[1] = 2.0
$values[2] = -2.25
$values[3] = -1.50
$values[6] = 9.75
$values[7] = 1.0
$values[8] = 48.0
$values[9] = 47.0
$values[10] = -120.0
$values[11] = -118.0
$values[46] = 42.0
$values[47] = 3.0
$values[48] = 7.0
$values[49] = 4.0
$values[51] = 14543.0
$values[52] = 2.0
$values[53] = 6.0
$values[55] = 1.0
$values[62] = 50.0
$values[63] = 0.30
$values[64] = 33.0

$firstValues = [single[]]$values.Clone()
$firstValues[0] = 1111.0
$buffer = New-Object System.Collections.Generic.List[byte]
Reset-BldcTailLock
Add-Bytes -Buffer $buffer -Count 68
Add-JustFloatFrame -Buffer $buffer -FrameValues $firstValues
Add-JustFloatFrame -Buffer $buffer -FrameValues $values

$frames = @(Pop-BldcFrames -Buffer $buffer)
Assert-True ($frames.Count -eq 1) "prefix plus two 72-float frames must decode only the second frame"
Assert-Near $frames[0].time_ms 1234.0 0.001 "time_ms"
Assert-Near $frames[0].roll_deg -2.25 0.001 "roll"
Assert-Near $frames[0].pitch_deg -1.50 0.001 "pitch"
Assert-Near $frames[0].feedback_online 1.0 0.001 "feedback online"
Assert-Near $frames[0].left_motor_rpm 48.0 0.001 "left rpm"
Assert-Near $frames[0].right_motor_rpm 47.0 0.001 "right rpm"
Assert-Near $frames[0].left_duty -120.0 0.001 "left duty"
Assert-Near $frames[0].firmware_frame_sequence 42.0 0.001 "frame sequence"
Assert-Near $frames[0].telemetry_drop_count 3.0 0.001 "telemetry drops"
Assert-Near $frames[0].imu_int_count 14543.0 0.001 "IMU interrupt count"
Assert-Near $frames[0].imu_invalid_count 2.0 0.001 "IMU invalid count"
Assert-True ($buffer.Count -eq 0) "frame buffer should be consumed"

# The old 55-float UART0 payload must be rejected by default instead of being
# shifted into the current 72-float parser.
$legacyValues = New-Object 'single[]' 55
$legacyValues[0] = 999.0
$legacyPrefixBuffer = New-Object System.Collections.Generic.List[byte]
Reset-BldcTailLock
Add-Bytes -Buffer $legacyPrefixBuffer -Count 68
Add-JustFloatFrame -Buffer $legacyPrefixBuffer -FrameValues $legacyValues
Add-JustFloatFrame -Buffer $legacyPrefixBuffer -FrameValues $legacyValues
Assert-True (@(Pop-BldcFrames -Buffer $legacyPrefixBuffer).Count -eq 0) "68-byte prefix plus two legacy 55-float frames must not decode"

$continuousLegacyBuffer = New-Object System.Collections.Generic.List[byte]
Reset-BldcTailLock
Add-JustFloatFrame -Buffer $continuousLegacyBuffer -FrameValues $legacyValues
Add-JustFloatFrame -Buffer $continuousLegacyBuffer -FrameValues $legacyValues
Assert-True (@(Pop-BldcFrames -Buffer $continuousLegacyBuffer).Count -eq 0) "continuous legacy 55-float frames must not decode"

$corruptRecoveryBuffer = New-Object System.Collections.Generic.List[byte]
$corruptFirstValues = [single[]]$values.Clone()
$corruptSecondValues = [single[]]$values.Clone()
$corruptThirdValues = [single[]]$values.Clone()
$corruptFirstValues[0] = 3000.0
$corruptSecondValues[0] = 4000.0
$corruptThirdValues[0] = 5000.0
Reset-BldcTailLock
Add-JustFloatFrame -Buffer $corruptRecoveryBuffer -FrameValues $corruptFirstValues
Add-Bytes -Buffer $corruptRecoveryBuffer -Count 12 -Value 0xA5
Add-JustFloatFrame -Buffer $corruptRecoveryBuffer -FrameValues $corruptSecondValues
Add-JustFloatFrame -Buffer $corruptRecoveryBuffer -FrameValues $corruptThirdValues
$recoveredFrames = @(Pop-BldcFrames -Buffer $corruptRecoveryBuffer)
Assert-True ($recoveredFrames.Count -eq 1) "corrupt interval plus two 72-float frames must decode only the re-synchronized frame"
Assert-Near $recoveredFrames[0].time_ms 5000.0 0.001 "re-synchronized frame time_ms"

function New-DiagnosticRows {
    param(
        [int]$Count = 20,
        [string]$Mode = "rpm",
        [double]$Target = 100.0,
        [double]$Online = 1.0,
        [double]$LeftRpm = 98.0,
        [double]$RightRpm = 101.0,
        [double]$LeftDuty = 300.0,
        [double]$RightDuty = 310.0
    )

    $rows = New-Object System.Collections.Generic.List[object]
    for($i = 0; $i -lt $Count; $i++) {
        $rows.Add([pscustomobject]@{
            elapsed_s = 0.01 * $i
            command_elapsed_s = 1.0
            expected_mode = $Mode
            expected_motor_rpm = $Target
            feedback_online = $Online
            left_motor_rpm = $LeftRpm
            right_motor_rpm = $RightRpm
            left_duty = $LeftDuty
            right_duty = $RightDuty
            firmware_frame_sequence = $i
            telemetry_drop_count = 0
            scheduler_missed_tick_count = 0
            scheduler_max_gap_ms = 1
            imu_int_count = $i
            imu_invalid_count = 0
            imu_age_ms = 5
        })
    }
    return $rows
}

$healthySummary = Get-BldcDiagnosticSummary -Rows (New-DiagnosticRows) -RpmDutyLimit 2000.0
Assert-True ($healthySummary.status -eq "healthy_capture") "healthy synthetic capture"
Assert-Near $healthySummary.feedback_online_ratio 1.0 0.0001 "healthy online ratio"

$offlineSummary = Get-BldcDiagnosticSummary -Rows (New-DiagnosticRows -Online 0 -LeftRpm 0 -RightRpm 0 -LeftDuty 0 -RightDuty 0) -RpmDutyLimit 2000.0
Assert-True ($offlineSummary.status -eq "feedback_offline") "offline capture classification"
Assert-True (($offlineSummary.findings -join " ") -match "UART1") "offline finding should identify UART1 feedback layer"

$saturatedSummary = Get-BldcDiagnosticSummary -Rows (New-DiagnosticRows -Target 300 -LeftRpm 40 -RightRpm 35 -LeftDuty 1995 -RightDuty 2000) -RpmDutyLimit 2000.0
Assert-True ($saturatedSummary.status -eq "output_saturated") "saturated capture classification"

$wrongSignSummary = Get-BldcDiagnosticSummary -Rows (New-DiagnosticRows -Target 120 -LeftRpm -90 -RightRpm -95 -LeftDuty 500 -RightDuty 520) -RpmDutyLimit 2000.0
Assert-True ($wrongSignSummary.status -eq "rpm_sign_mismatch") "sign mismatch classification"

$imuBlockedRows = New-DiagnosticRows -Target 100 -LeftRpm 0 -RightRpm 0 -LeftDuty 0 -RightDuty 0
foreach($row in $imuBlockedRows) {
    $row.imu_int_count = 14543
    $row.imu_age_ms = 105000
}
$imuBlockedSummary = Get-BldcDiagnosticSummary -Rows $imuBlockedRows -RpmDutyLimit 2000.0
Assert-True ($imuBlockedSummary.status -eq "imu_stale_safety_block") "stale IMU must identify the global motor safety block"
Assert-True (($imuBlockedSummary.findings -join " ") -match "IMU") "stale IMU finding"

$inconclusiveSummary = Get-BldcDiagnosticSummary -Rows (New-DiagnosticRows -Mode "unknown") -RpmDutyLimit 2000.0
Assert-True ($inconclusiveSummary.status -eq "inconclusive_no_rpm_step") "listen-only capture must not overclaim closed-loop health"

Write-Host "collect_bldc_diagnostics tests passed"

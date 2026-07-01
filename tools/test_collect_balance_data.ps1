$ErrorActionPreference = "Stop"

$scriptPath = Join-Path $PSScriptRoot "collect_balance_data.ps1"
. $scriptPath -LoadOnly

function Assert-True {
    param(
        [bool]$Condition,
        [string]$Message
    )

    if(-not $Condition) {
        throw $Message
    }
}

function Assert-Near {
    param(
        [double]$Actual,
        [double]$Expected,
        [double]$Tolerance,
        [string]$Message
    )

    if([Math]::Abs($Actual - $Expected) -gt $Tolerance) {
        throw ("{0}: actual={1}, expected={2}" -f $Message, $Actual, $Expected)
    }
}

$schedule = Parse-CommandSchedule -Text "0:STOP;1.5:B,2;2:C,0,0"
Assert-True ($schedule.Count -eq 3) "expected three scheduled commands"
Assert-Near $schedule[1].AtSeconds 1.5 0.0001 "second command time"
Assert-True ($schedule[2].Command -eq "C,0,0") "third command text"
Assert-True ((Convert-CsvField "C,0,0") -eq '"C,0,0"') "CSV fields with commas must be quoted"
Assert-True ((Convert-CsvField 'note "quoted"') -eq '"note ""quoted"""') "CSV quotes must be escaped"

$values = [single[]](1234.0, 2.0, 4.5, -12.25, 50.0, 50.0, 9.75, 1.0)
$buffer = New-Object System.Collections.Generic.List[byte]
$buffer.Add(0x55)
foreach($value in $values) {
    foreach($byte in [BitConverter]::GetBytes($value)) {
        $buffer.Add($byte)
    }
}
foreach($byte in [byte[]](0x00, 0x00, 0x80, 0x7F)) {
    $buffer.Add($byte)
}

$frames = @(Pop-BalanceFrames -Buffer $buffer)
Assert-True ($frames.Count -eq 1) "expected one parsed frame"
Assert-Near $frames[0].time_ms 1234.0 0.001 "time_ms"
Assert-Near $frames[0].balance_mode 2.0 0.001 "balance_mode"
Assert-Near $frames[0].pitch_deg 4.5 0.001 "pitch_deg"
Assert-Near $frames[0].pitch_rate_dps -12.25 0.001 "pitch_rate_dps"
Assert-Near $frames[0].chassis_left_rpm 50.0 0.001 "chassis_left_rpm"
Assert-Near $frames[0].chassis_right_rpm 50.0 0.001 "chassis_right_rpm"
Assert-Near $frames[0].balance_rpm 9.75 0.001 "balance_rpm"
Assert-Near $frames[0].feedback_online 1.0 0.001 "feedback_online"
Assert-True ($buffer.Count -eq 0) "buffer should be consumed after frame"

Write-Host "collect_balance_data tests passed"

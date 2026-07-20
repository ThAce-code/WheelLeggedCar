$ErrorActionPreference = "Stop"

function Assert-True {
    param([bool]$Condition, [string]$Message)
    if(-not $Condition) {
        throw $Message
    }
}

$common = @(
    "-LeftMotorKp", "2", "-LeftMotorKi", "40", "-LeftMotorKd", "0",
    "-RightMotorKp", "2", "-RightMotorKi", "40", "-RightMotorKd", "0",
    "-PitchKp", "18", "-PitchRateKd", "8", "-WheelSpeedKs", "-3",
    "-WheelPositionKp", "0", "-PitchSetpointDeg", "0", "-Preview"
)
$expected = @{
    short = "BI,6,300"
    medium = "BI,8,600"
    long = "BI,10,1000"
}

foreach($profile in $expected.Keys) {
    $preview = & powershell -ExecutionPolicy Bypass -File tools/collect_balance_lqr_dataset.ps1 -Profile $profile @common 2>&1
    Assert-True ($LASTEXITCODE -eq 0) "$profile preview failed."
    $text = $preview -join "`n"
    Assert-True ($text -match [regex]::Escape($expected[$profile])) "$profile BI mapping is wrong."
    Assert-True ($text -match 'active balance window: 20000 ms') "Active window must be 20 seconds."
    Assert-True ($text -match '2:B,2;4:BI,' -and $text -match '24:BI,0,0;24\.5:STOP') "Balance settle or shutdown schedule is missing."
}

$savedErrorPreference = $ErrorActionPreference
$ErrorActionPreference = "Continue"
$blocked = & powershell -ExecutionPolicy Bypass -File tools/collect_balance_lqr_dataset.ps1 -Profile short @($common | Where-Object { $_ -ne "-Preview" }) 2>&1
$blockedExitCode = $LASTEXITCODE
$ErrorActionPreference = $savedErrorPreference
Assert-True ($blockedExitCode -ne 0) "Real collection must require AllowMotion."
Assert-True (($blocked -join "`n") -match 'AllowMotion') "The motion gate must explain authorization."

$source = Get-Content tools/collect_balance_lqr_dataset.ps1 -Raw
Assert-True ($source -match 'collect_balance_data\.ps1') "Use the existing balance collector."
Assert-True ($source -notmatch '-NoStopOnExit') "Do not suppress final STOP."
Assert-True (($source -match 'balance_lqr_id_') -and ($source -match '\$Profile') -and ($source -match '\$stamp')) "Use the new capture prefix."

$savedErrorPreference = $ErrorActionPreference
$ErrorActionPreference = "Continue"
$invalid = & powershell -ExecutionPolicy Bypass -File tools/collect_balance_lqr_dataset.ps1 -Profile invalid @common 2>&1
$invalidExitCode = $LASTEXITCODE
$ErrorActionPreference = $savedErrorPreference
Assert-True ($invalidExitCode -ne 0) "Invalid profiles must be rejected."

Write-Host "balance LQR dataset collector checks passed"

$ErrorActionPreference = "Stop"

function Assert-True {
    param([bool]$Condition, [string]$Message)
    if(-not $Condition) {
        throw $Message
    }
}

$scriptPath = "tools/collect_motor_identification.ps1"
$preview = & powershell -ExecutionPolicy Bypass -File $scriptPath -Preview -Out "data/motor_preview.csv" 2>&1
Assert-True ($LASTEXITCODE -eq 0) "Preview must not open COM."
$text = $preview -join "`n"
Assert-True ($text -match 'duration: 14(\.0+)? s') "Preview must print 14 seconds."
Assert-True ($text -match '0:STOP;0\.5:D,300;2:STOP;2\.75:D,600;4\.25:STOP;5:D,900;6\.5:STOP;7\.25:D,-300;8\.75:STOP;9\.5:D,-600;11:STOP;11\.75:D,-900;13\.25:STOP') "Preview schedule differs from the design."
Assert-True ($text -match 'data[\\/]motor_preview\.csv') "Preview must print the requested output."

$savedErrorPreference = $ErrorActionPreference
$ErrorActionPreference = "Continue"
$blocked = & powershell -ExecutionPolicy Bypass -File $scriptPath -Out "data/blocked.csv" 2>&1
$blockedExitCode = $LASTEXITCODE
$ErrorActionPreference = $savedErrorPreference
Assert-True ($blockedExitCode -ne 0) "Real collection must require AllowMotion."
Assert-True (($blocked -join "`n") -match 'AllowMotion') "The motion gate must explain authorization."

$source = Get-Content $scriptPath -Raw
Assert-True ($source -match 'collect_bldc_diagnostics\.ps1') "Use the 72-float BLDC collector."
Assert-True ($source -notmatch '-NoStopOnExit') "Do not suppress final STOP."
Assert-True ($source -match 'motor_ident_\$stamp\.csv') "Use the new capture prefix."

Write-Host "motor identification collector checks passed"

$ErrorActionPreference = "Stop"

function Assert-True {
    param([bool]$Condition, [string]$Message)
    if(-not $Condition) {
        throw $Message
    }
}

$scriptPath = "tools/collect_balance_pd_isolation.ps1"
Assert-True (Test-Path $scriptPath) "Missing balance PD isolation collector."

$preview = & powershell -ExecutionPolicy Bypass -File $scriptPath -Preview -Out "data/preview.csv" 2>&1
Assert-True ($LASTEXITCODE -eq 0) "Preview mode must succeed without opening a serial port."
$previewText = $preview -join "`n"
Assert-True ($previewText -match '0:STOP;0\.25:BL,18,8,0,0;0\.50:BS,0;0\.75:BD,0,0,0;1\.00:B,1;1\.25:C,0,0;1\.50:B,2;3\.00:STOP') "Preview must show the fixed command schedule."
Assert-True ($previewText -match 'active balance window: 1500 ms') "Preview must show the bounded active window."
Assert-True ($previewText -match 'data[\\/]preview\.csv') "Preview must show the requested output path."

$savedErrorPreference = $ErrorActionPreference
$ErrorActionPreference = "Continue"
$guardOutput = & powershell -ExecutionPolicy Bypass -File $scriptPath -Out "data/blocked.csv" 2>&1
$guardExitCode = $LASTEXITCODE
$ErrorActionPreference = $savedErrorPreference
Assert-True ($guardExitCode -ne 0) "A real capture must require -AllowMotion."
Assert-True (($guardOutput -join "`n") -match 'AllowMotion') "Motion gate failure must explain how to authorize the capture."

$source = Get-Content $scriptPath -Raw
Assert-True ($source -match 'collect_balance_data\.ps1') "Wrapper must reuse the existing 72-channel collector."
Assert-True ($source -notmatch '-NoStopOnExit') "Wrapper must preserve the collector final STOP."
Assert-True ($source -match 'balance_pd_isolation_\$stamp\.csv') "Default output must use the diagnostic prefix."
Assert-True ($source -match 'balance_pd_isolation_bd0') "Default note must identify the BD=0 probe."

Write-Host "balance PD isolation collector checks passed"

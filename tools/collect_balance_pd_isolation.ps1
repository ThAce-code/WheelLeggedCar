param(
    [string]$Port = "COM6",
    [int]$Baud = 460800,
    [string]$Out = "",
    [string]$Note = "balance_pd_isolation_bd0",
    [switch]$AllowMotion,
    [switch]$Preview
)

$ErrorActionPreference = "Stop"

$durationSeconds = 3.70
$activeWindowMs = 1500
$commands = "0:STOP;0.25:BL,18,8,0,0;0.50:BS,0;0.75:BD,0,0,0;1.00:B,1;1.25:C,0,0;1.50:B,2;3.00:STOP"

if([string]::IsNullOrWhiteSpace($Out)) {
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $Out = "data/balance_pd_isolation_$stamp.csv"
}

Write-Host ("commands: {0}" -f $commands)
Write-Host ("active balance window: {0} ms" -f $activeWindowMs)
Write-Host ("output: {0}" -f $Out)

if($Preview) {
    return
}

if(-not $AllowMotion) {
    throw "Motion is blocked. Suspend or firmly support the vehicle, keep a physical cutoff ready, then add -AllowMotion."
}

$collector = Join-Path $PSScriptRoot "collect_balance_data.ps1"
if(-not (Test-Path -LiteralPath $collector)) {
    throw "Missing collector: $collector"
}

Write-Warning "Active balance will run for 1500 ms. Keep the vehicle supported and be ready to cut power."

& $collector `
    -Port $Port `
    -Baud $Baud `
    -Duration $durationSeconds `
    -Commands $commands `
    -Out $Out `
    -Note $Note

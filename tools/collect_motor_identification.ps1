param(
    [string]$Port = "COM6",
    [int]$Baud = 460800,
    [string]$Out = "",
    [string]$Note = "motor_signed_duty_ident",
    [switch]$AllowMotion,
    [switch]$Preview
)

$ErrorActionPreference = "Stop"
$durationSeconds = 14.0
$commands = "0:STOP;0.5:D,300;2:STOP;2.75:D,600;4.25:STOP;5:D,900;6.5:STOP;7.25:D,-300;8.75:STOP;9.5:D,-600;11:STOP;11.75:D,-900;13.25:STOP"

if([string]::IsNullOrWhiteSpace($Out)) {
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $Out = "data/motor_ident_$stamp.csv"
}

Write-Host ("commands: {0}" -f $commands)
Write-Host ("duration: {0:F1} s" -f $durationSeconds)
Write-Host ("output: {0}" -f $Out)

if($Preview) {
    return
}

if(-not $AllowMotion) {
    throw "Motion is blocked. Suspend both wheels, keep a physical cutoff ready, then add -AllowMotion."
}

$collector = Join-Path $PSScriptRoot "collect_bldc_diagnostics.ps1"
if(-not (Test-Path -LiteralPath $collector)) {
    throw "Missing collector: $collector"
}

Write-Warning "Open-duty steps are enabled. Suspend both wheels and keep a physical cutoff ready."
& $collector `
    -Port $Port `
    -Baud $Baud `
    -Duration $durationSeconds `
    -Commands $commands `
    -Out $Out `
    -Note $Note `
    -AllowMotion
if($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

param(
    [string]$Port = "COM6",
    [int]$Baud = 460800,
    [Parameter(Mandatory=$true)]
    [ValidateSet("short", "medium", "long")]
    [string]$Profile,
    [Parameter(Mandatory=$true)][double]$LeftMotorKp,
    [Parameter(Mandatory=$true)][double]$LeftMotorKi,
    [Parameter(Mandatory=$true)][double]$LeftMotorKd,
    [Parameter(Mandatory=$true)][double]$RightMotorKp,
    [Parameter(Mandatory=$true)][double]$RightMotorKi,
    [Parameter(Mandatory=$true)][double]$RightMotorKd,
    [Parameter(Mandatory=$true)][double]$PitchKp,
    [Parameter(Mandatory=$true)][double]$PitchRateKd,
    [Parameter(Mandatory=$true)][double]$WheelSpeedKs,
    [Parameter(Mandatory=$true)][double]$WheelPositionKp,
    [Parameter(Mandatory=$true)][double]$PitchSetpointDeg,
    [string]$Out = "",
    [string]$Note = "",
    [switch]$AllowMotion,
    [switch]$Preview
)

$ErrorActionPreference = "Stop"
$durationSeconds = 25.5
$activeWindowMs = 22000

function Format-Number {
    param([double]$Value)
    return $Value.ToString("0.######", [System.Globalization.CultureInfo]::InvariantCulture)
}

$profiles = @{
    short = @{ Amplitude = 6; HalfPeriodMs = 300 }
    medium = @{ Amplitude = 8; HalfPeriodMs = 600 }
    long = @{ Amplitude = 10; HalfPeriodMs = 1000 }
}
$selected = $profiles[$Profile]

$leftKp = Format-Number $LeftMotorKp
$leftKi = Format-Number $LeftMotorKi
$leftKd = Format-Number $LeftMotorKd
$rightKp = Format-Number $RightMotorKp
$rightKi = Format-Number $RightMotorKi
$rightKd = Format-Number $RightMotorKd
$pitchKp = Format-Number $PitchKp
$pitchRateKd = Format-Number $PitchRateKd
$wheelSpeedKs = Format-Number $WheelSpeedKs
$wheelPositionKp = Format-Number $WheelPositionKp
$pitchSetpoint = Format-Number $PitchSetpointDeg
$amplitude = Format-Number ([double]$selected.Amplitude)
$halfPeriodMs = [int]$selected.HalfPeriodMs

$commands = @(
    "0:STOP",
    "0.25:PL,$leftKp,$leftKi,$leftKd",
    "0.5:PR,$rightKp,$rightKi,$rightKd",
    "0.75:BL,$pitchKp,$pitchRateKd,$wheelSpeedKs,$wheelPositionKp",
    "1:BS,$pitchSetpoint",
    "1.25:BD,0,0,0",
    "1.5:B,1",
    "1.75:C,0,0",
    "2:BI,$amplitude,$halfPeriodMs",
    "2.25:B,2",
    "24.25:BI,0,0",
    "24.75:STOP"
) -join ";"

if([string]::IsNullOrWhiteSpace($Out)) {
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $Out = "data/balance_lqr_id_${Profile}_$stamp.csv"
}
if([string]::IsNullOrWhiteSpace($Note)) {
    $Note = "balance_lqr_id_$Profile"
}

Write-Host ("profile: {0}" -f $Profile)
Write-Host ("commands: {0}" -f $commands)
Write-Host ("duration: {0:F1} s" -f $durationSeconds)
Write-Host ("active balance window: {0} ms" -f $activeWindowMs)
Write-Host ("output: {0}" -f $Out)

if($Preview) {
    return
}

if(-not $AllowMotion) {
    throw "Motion is blocked. Support the vehicle, keep a physical cutoff ready, then add -AllowMotion."
}

$collector = Join-Path $PSScriptRoot "collect_balance_data.ps1"
if(-not (Test-Path -LiteralPath $collector)) {
    throw "Missing collector: $collector"
}

Write-Warning "Active balance excitation will run for 22 seconds. Keep the vehicle supported and ready to cut power."
& $collector `
    -Port $Port `
    -Baud $Baud `
    -Duration $durationSeconds `
    -Commands $commands `
    -Out $Out `
    -Note $Note
if($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

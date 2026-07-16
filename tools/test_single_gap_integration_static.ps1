param(
    [string]$Root = (Split-Path -Parent $PSScriptRoot)
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Require-Pattern
{
    param(
        [string]$RelativePath,
        [string]$Pattern
    )

    $path = Join-Path $Root $RelativePath
    if(-not (Test-Path -LiteralPath $path))
    {
        throw "Missing $RelativePath"
    }
    if(-not (Select-String -Path $path -Pattern $Pattern -Quiet))
    {
        throw "Missing '$Pattern' in $RelativePath"
    }
}

Require-Pattern 'project/code/single_gap_config.h' 'SINGLE_GAP_CONTROL_PERIOD_MS\s+\(40U\)'
Require-Pattern 'project/code/single_gap_config.h' 'SINGLE_GAP_TOF_STOP_MM\s+\(350U\)'
Require-Pattern 'project/code/single_gap_config.h' 'SINGLE_GAP_SENSOR_STALE_MS\s+\(100U\)'
Require-Pattern 'project/code/single_gap_config.h' 'SINGLE_GAP_MOTION_ENABLE\s+\(0U\)'
Require-Pattern 'project/code/single_gap_config.h' 'SINGLE_GAP_WHEEL_CIRCUMFERENCE_MM\s+\(0U\)'
Require-Pattern 'project/code/single_gap_types.h' 'SINGLE_GAP_STATE_PASS_CANDIDATE'
Require-Pattern 'project/code/single_gap_types.h' 'SINGLE_GAP_STOP_TOF_NEAR'
Require-Pattern 'project/code/single_gap_types.h' 'single_gap_output_struct'

Write-Output 'single-gap static contracts: PASS'

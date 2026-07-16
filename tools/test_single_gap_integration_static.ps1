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

function Require-TextPattern
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
    $text = [System.IO.File]::ReadAllText($path)
    if(-not [Regex]::IsMatch($text, $Pattern))
    {
        throw "Missing text pattern '$Pattern' in $RelativePath"
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
Require-Pattern 'project/code/dl1b_safety.h' 'uint8\s+dl1b_safety_init\s*\(uint32\s+now_ms\)'
Require-Pattern 'project/code/dl1b_safety.h' 'void\s+dl1b_safety_update\s*\(uint32\s+now_ms\)'
Require-Pattern 'project/code/dl1b_safety_port.h' 'uint8\s+dl1b_safety_port_read\s*\(uint16\s*\*distance_mm\)'
Require-Pattern 'libraries/zf_device/zf_device_dl1b.h' 'DL1B_SCL_PIN\s+\(P19_0\)'
Require-Pattern 'libraries/zf_device/zf_device_dl1b.h' 'DL1B_SDA_PIN\s+\(P19_1\)'
Require-Pattern 'libraries/zf_device/zf_device_dl1b.h' 'DL1B_XS_PIN\s+\(\s*P07_2\s*\)'
Require-Pattern 'project/user/main_cm0plus.c' '#include\s+"single_gap_config\.h"'
Require-Pattern 'project/user/main_cm7_0.c' '#include\s+"single_gap_config\.h"'
Require-TextPattern 'project/user/main_cm0plus.c' '(?s)#if\s*\(SINGLE_GAP_ENABLE\s*==\s*0U\).*?gpio_init\s*\(P19_0.*?#endif'
Require-TextPattern 'project/user/main_cm7_0.c' '(?s)#if\s*\(SINGLE_GAP_ENABLE\s*==\s*0U\).*?gpio_(?:low|high|init|toggle_level)\s*\(P19_0.*?#endif'

Write-Output 'single-gap static contracts: PASS'

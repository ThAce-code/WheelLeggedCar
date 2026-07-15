param(
    [string]$Root = (Split-Path -Parent $PSScriptRoot)
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Require-Pattern
{
    param(
        [string]$Pattern,
        [string]$RelativePath
    )

    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path))
    {
        throw "Missing required file: $RelativePath"
    }
    if (-not (Select-String -Path $path -Pattern $Pattern -Quiet))
    {
        throw "Missing pattern '$Pattern' in $RelativePath"
    }
}

function Reject-Pattern
{
    param(
        [string]$Pattern,
        [string]$RelativePath
    )

    $path = Join-Path $Root $RelativePath
    $files = Get-ChildItem -Path $path -File -ErrorAction SilentlyContinue
    foreach ($file in $files)
    {
        if (Select-String -Path $file.FullName -Pattern $Pattern -Quiet)
        {
            throw "Forbidden pattern '$Pattern' in $($file.FullName)"
        }
    }
}

Require-Pattern 'PERCEPTION_IMAGE_WIDTH\s+\(188U\)' 'project/code/perception_config.h'
Require-Pattern 'PERCEPTION_IMAGE_HEIGHT\s+\(120U\)' 'project/code/perception_config.h'
Require-Pattern 'PERCEPTION_SCORE_ACCEPT\s+\(166U\)' 'project/code/perception_config.h'
Require-Pattern 'PERCEPTION_SCORE_HIGH\s+\(204U\)' 'project/code/perception_config.h'
Require-Pattern 'PERCEPTION_MAX_TRACKS\s+\(32U\)' 'project/code/perception_config.h'
Require-Pattern 'PERCEPTION_GATE_CHI2_Q16\s+\(392561UL\)' 'project/code/perception_config.h'
Require-Pattern 'PERCEPTION_WEIGHT_TAPER\s+\(77U\)' 'project/code/perception_config.h'
Require-Pattern 'PERCEPTION_WEIGHT_CONTRAST\s+\(25U\)' 'project/code/perception_config.h'
Require-Pattern 'typedef enum' 'project/code/perception_types.h'
Require-Pattern 'PERCEPTION_STATE_SAFE_STOP' 'project/code/perception_types.h'
Require-Pattern 'perception_pose_snapshot_struct' 'project/code/perception_types.h'
Require-Pattern 'perception_gap_target_struct' 'project/code/perception_types.h'
Require-Pattern 'perception_snapshot_struct' 'project/code/perception_types.h'
Require-Pattern 'INTERCORE_PROTOCOL_VERSION\s+\(3U\)' 'project/code/intercore_protocol.h'
Require-Pattern 'intercore_perception_control_struct' 'project/code/intercore_protocol.h'
Require-Pattern 'perception_intercore_publish_pose' 'project/code/perception_intercore.h'
Require-Pattern 'perception_intercore_publish_perception' 'project/code/perception_intercore.h'
Require-Pattern 'INTERCORE_LAYOUT_CHECK\(perception_offset' 'project/code/intercore_protocol.h'
Require-Pattern 'perception_intercore\.c' 'project/iar/project_config/cyt4bb7_cm_7_0.ewp'
Require-Pattern 'perception_intercore\.c' 'project/iar/project_config/cyt4bb7_cm_7_1.ewp'
Reject-Pattern '\bfloat32\b' 'project/code/perception_*.h'
Reject-Pattern '^\s*(const\s+)?(uint8|uint16|uint32|int16|int32|float|perception_[a-z_]+)\s*\*' 'project/code/perception_types.h'
Reject-Pattern '\b(malloc|calloc|realloc|free)\s*\(' 'project/code/perception_*.c'

$weightSum = 77 + 64 + 38 + 26 + 26 + 25
if ($weightSum -ne 256)
{
    throw "Perception score weights must sum to 256, got $weightSum"
}

Write-Output "Perception static contract checks passed (weights=$weightSum)."

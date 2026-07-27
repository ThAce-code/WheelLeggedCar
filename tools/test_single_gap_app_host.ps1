$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$binary = Join-Path $env:TEMP 'single_gap_app_test.exe'
$limitBinary = Join-Path $env:TEMP 'single_gap_app_limit_test.exe'
$disabledBinary = Join-Path $env:TEMP 'single_gap_app_disabled_test.exe'
$motionBinary = Join-Path $env:TEMP 'single_gap_app_motion_test.exe'
$mingwBin = 'C:\msys64\ucrt64\bin'
$sources = @(
    (Join-Path $repoRoot 'project\tests\single_gap_app_test.c'),
    (Join-Path $repoRoot 'project\code\single_gap_app.c'),
    (Join-Path $repoRoot 'project\code\single_gap_detector.c'),
    (Join-Path $repoRoot 'project\code\single_gap_controller.c')
)

function Build-And-Run
{
    param(
        [string]$Output,
        [string[]]$Defines
    )

    $arguments = @('-std=c11', '-Wall', '-Wextra', '-Werror') + $Defines + @(
        '-I', (Join-Path $repoRoot 'libraries\zf_common'),
        '-I', (Join-Path $repoRoot 'project\code')
    ) + $sources + @('-o', $Output)
    & gcc @arguments
    if($LASTEXITCODE -ne 0)
    {
        throw 'single-gap app host compile failed'
    }
    & $Output
    if($LASTEXITCODE -ne 0)
    {
        throw 'single-gap app host test failed'
    }
}

try
{
    $env:PATH = "$mingwBin;$env:PATH"
    Build-And-Run $binary @(
        '-DSINGLE_GAP_ENABLE=1U',
        '-DSINGLE_GAP_MOTION_ENABLE=0U',
        '-DSINGLE_GAP_WHEEL_CIRCUMFERENCE_MM=200U'
    )
    Build-And-Run $limitBinary @(
        '-DTEST_RPM_LIMIT=1',
        '-DSINGLE_GAP_ENABLE=1U',
        '-DSINGLE_GAP_MOTION_ENABLE=0U',
        '-DSINGLE_GAP_WHEEL_CIRCUMFERENCE_MM=100U'
    )
    Build-And-Run $disabledBinary @('-DTEST_FEATURE_DISABLED=1')
    Build-And-Run $motionBinary @(
        '-DTEST_MOTION_ENABLED=1',
        '-DSINGLE_GAP_ENABLE=1U',
        '-DSINGLE_GAP_MOTION_ENABLE=1U',
        '-DSINGLE_GAP_WHEEL_CIRCUMFERENCE_MM=200U'
    )
}
finally
{
    Remove-Item -LiteralPath $binary -Force -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $limitBinary -Force -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $disabledBinary -Force -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $motionBinary -Force -ErrorAction SilentlyContinue
}

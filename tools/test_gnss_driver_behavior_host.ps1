$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$binary = Join-Path $env:TEMP 'gnss_driver_behavior_host_test.exe'

try
{
    $env:PATH = "C:\msys64\ucrt64\bin;$env:PATH"
    & gcc -std=c11 -Wall -Wextra -Werror -Wno-pointer-to-int-cast `
        -ffunction-sections -fdata-sections `
        -I (Join-Path $repoRoot 'libraries\zf_common') `
        -I (Join-Path $repoRoot 'libraries\zf_driver') `
        -I (Join-Path $repoRoot 'libraries\zf_device') `
        (Join-Path $repoRoot 'project\tests\gnss_driver_behavior_test.c') `
        '-Wl,--gc-sections' -lm -o $binary
    if($LASTEXITCODE -ne 0)
    {
        throw 'GNSS driver behavior host compile failed'
    }

    & $binary
    if($LASTEXITCODE -ne 0)
    {
        throw 'GNSS driver behavior host test failed'
    }
}
finally
{
    Remove-Item -LiteralPath $binary -Force -ErrorAction SilentlyContinue
}

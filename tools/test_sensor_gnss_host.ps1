$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$binary = Join-Path $env:TEMP 'sensor_gnss_host_test.exe'

try
{
    $env:PATH = "C:\msys64\ucrt64\bin;$env:PATH"
    & gcc -std=c11 -Wall -Wextra -Werror `
        -I (Join-Path $repoRoot 'project\tests\mocks') `
        -I (Join-Path $repoRoot 'project\code') `
        -I (Join-Path $repoRoot 'libraries\zf_common') `
        (Join-Path $repoRoot 'project\tests\sensor_gnss_test.c') `
        (Join-Path $repoRoot 'project\code\sensor_gnss.c') `
        (Join-Path $repoRoot 'project\code\local_position.c') `
        -lm -o $binary
    if($LASTEXITCODE -ne 0)
    {
        throw 'GNSS sensor host compile failed'
    }

    & $binary
    if($LASTEXITCODE -ne 0)
    {
        throw 'GNSS sensor host test failed'
    }
}
finally
{
    Remove-Item -LiteralPath $binary -Force -ErrorAction SilentlyContinue
}

$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$binary = Join-Path $env:TEMP 'gnss_local_position_test.exe'

try
{
    $env:PATH = "C:\msys64\ucrt64\bin;$env:PATH"
    & gcc -std=c11 -Wall -Wextra -Werror `
        -I (Join-Path $repoRoot 'project\code') `
        -I (Join-Path $repoRoot 'libraries\zf_common') `
        (Join-Path $repoRoot 'project\tests\gnss_local_position_test.c') `
        (Join-Path $repoRoot 'project\code\local_position.c') `
        -lm -o $binary
    if($LASTEXITCODE -ne 0)
    {
        throw 'host compile failed'
    }

    & $binary
    if($LASTEXITCODE -ne 0)
    {
        throw 'host test failed'
    }
}
finally
{
    Remove-Item -LiteralPath $binary -Force -ErrorAction SilentlyContinue
}

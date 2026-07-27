$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$binary = Join-Path $env:TEMP 'single_gap_controller_test.exe'
$mingwBin = 'C:\msys64\ucrt64\bin'

try
{
    $env:PATH = "$mingwBin;$env:PATH"
    & gcc -std=c11 -Wall -Wextra -Werror `
        -I (Join-Path $repoRoot 'libraries\zf_common') `
        -I (Join-Path $repoRoot 'project\code') `
        (Join-Path $repoRoot 'project\tests\single_gap_controller_test.c') `
        (Join-Path $repoRoot 'project\code\single_gap_controller.c') `
        -o $binary
    if($LASTEXITCODE -ne 0)
    {
        throw 'single-gap controller host compile failed'
    }

    & $binary
    if($LASTEXITCODE -ne 0)
    {
        throw 'single-gap controller host test failed'
    }
}
finally
{
    Remove-Item -LiteralPath $binary -Force -ErrorAction SilentlyContinue
}

$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$binary = Join-Path $env:TEMP 'single_gap_dl1b_test.exe'
$mingwBin = 'C:\msys64\ucrt64\bin'

try
{
    $env:PATH = "$mingwBin;$env:PATH"
    & gcc -std=c11 -Wall -Wextra -Werror `
        -I (Join-Path $repoRoot 'libraries\zf_common') `
        -I (Join-Path $repoRoot 'project\code') `
        (Join-Path $repoRoot 'project\tests\single_gap_dl1b_test.c') `
        (Join-Path $repoRoot 'project\code\dl1b_safety.c') `
        -o $binary
    if($LASTEXITCODE -ne 0)
    {
        throw 'single-gap DL1B host compile failed'
    }

    & $binary
    if($LASTEXITCODE -ne 0)
    {
        throw 'single-gap DL1B host test failed'
    }
}
finally
{
    Remove-Item -LiteralPath $binary -Force -ErrorAction SilentlyContinue
}

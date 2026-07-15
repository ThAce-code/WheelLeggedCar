$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$binary = Join-Path $env:TEMP 'perception_intercore_host_test.exe'
$mingwBin = 'C:\msys64\ucrt64\bin'

try
{
    $env:PATH = "$mingwBin;$env:PATH"
    & gcc -std=c11 -Wall -Wextra -Werror `
        -I (Join-Path $repoRoot 'project\code') `
        -I (Join-Path $repoRoot 'libraries\zf_common') `
        (Join-Path $repoRoot 'project\tests\perception_intercore_test.c') `
        (Join-Path $repoRoot 'project\code\intercore_protocol.c') `
        (Join-Path $repoRoot 'project\code\perception_intercore.c') `
        -o $binary
    if($LASTEXITCODE -ne 0)
    {
        throw "host test compile failed with exit code $LASTEXITCODE"
    }
    & $binary
    if($LASTEXITCODE -ne 0)
    {
        throw "host test failed with exit code $LASTEXITCODE"
    }
}
finally
{
    Remove-Item -LiteralPath $binary -Force -ErrorAction SilentlyContinue
}

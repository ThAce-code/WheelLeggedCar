$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$source = Join-Path $PSScriptRoot 'test_camera_seekfree_transport_host.c'
$binary = Join-Path $env:TEMP 'camera_seekfree_transport_host_test.exe'
$mingwBin = 'C:\msys64\ucrt64\bin'

try
{
    $env:PATH = "$mingwBin;$env:PATH"
    & gcc -std=c99 -Wall -Wextra -Werror -I (Join-Path $repoRoot 'project\code') $source -o $binary
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

$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$binary = Join-Path $env:TEMP 'intercore_control_foundation_test.exe'

try
{
    $env:PATH = "C:\msys64\ucrt64\bin;$env:PATH"
    & gcc -std=c11 -Wall -Wextra -Werror -DINTERCORE_HOST_TEST `
        -I (Join-Path $repoRoot 'libraries\zf_common') `
        -I (Join-Path $repoRoot 'project\code') `
        (Join-Path $repoRoot 'project\tests\intercore_control_foundation_test.c') `
        (Join-Path $repoRoot 'project\code\intercore_protocol.c') `
        (Join-Path $repoRoot 'project\code\intercore_transport.c') `
        (Join-Path $repoRoot 'project\code\intercore_notify.c') `
        (Join-Path $repoRoot 'project\code\motion_command_router.c') `
        (Join-Path $repoRoot 'project\code\intercore_control.c') `
        -o $binary
    if($LASTEXITCODE -ne 0)
    {
        throw 'control-foundation host compile failed'
    }

    & $binary
    if($LASTEXITCODE -ne 0)
    {
        throw 'control-foundation host test failed'
    }
}
finally
{
    Remove-Item -LiteralPath $binary -Force -ErrorAction SilentlyContinue
}

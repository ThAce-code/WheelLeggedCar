$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$binary = Join-Path $env:TEMP 'gnss_intercore_host_test.exe'

try
{
    $env:PATH = "C:\msys64\ucrt64\bin;$env:PATH"
    & gcc -std=c11 -Wall -Wextra -Werror -DINTERCORE_HOST_TEST `
        -I (Join-Path $repoRoot 'project\code') `
        -I (Join-Path $repoRoot 'libraries\zf_common') `
        (Join-Path $repoRoot 'project\tests\gnss_intercore_test.c') `
        (Join-Path $repoRoot 'project\code\intercore_protocol.c') `
        (Join-Path $repoRoot 'project\code\intercore_transport.c') `
        -o $binary
    if($LASTEXITCODE -ne 0)
    {
        throw 'GNSS intercore host compile failed'
    }

    & $binary
    if($LASTEXITCODE -ne 0)
    {
        throw 'GNSS intercore host test failed'
    }
}
finally
{
    Remove-Item -LiteralPath $binary -Force -ErrorAction SilentlyContinue
}

$ErrorActionPreference = "Stop"

$tempPath = Join-Path ([System.IO.Path]::GetTempPath()) ("imu-rate-filter-" + [Guid]::NewGuid().ToString())
New-Item -ItemType Directory -Path $tempPath | Out-Null
$originalPath = $null

try {
    @'
#ifndef _app_types_h_
#define _app_types_h_
typedef unsigned char uint8;
typedef enum { APP_FALSE = 0, APP_TRUE = 1 } app_bool_enum;
#endif
'@ | Set-Content (Join-Path $tempPath "app_types.h") -NoNewline

    @'
#include "imu_rate_filter.h"
#include <math.h>
#include <stdio.h>

static int nearf(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

int main(void)
{
    imu_rate_filter_state_struct state;
    float output;

    imu_rate_filter_init(&state);
    output = imu_rate_filter_step(&state, 2.0f, 0.5f);
    if(!nearf(output, 2.0f, 0.0001f))
    {
        puts("first sample initialization failed");
        return 1;
    }

    output = imu_rate_filter_step(&state, 6.0f, 0.5f);
    if(!nearf(output, 4.0f, 0.0001f))
    {
        puts("LPF step failed");
        return 1;
    }

    output = imu_rate_filter_step(&state, 10.0f, 2.0f);
    if(!nearf(output, 10.0f, 0.0001f))
    {
        puts("alpha upper clamp failed");
        return 1;
    }

    imu_rate_filter_reset(&state);
    output = imu_rate_filter_step(&state, -3.0f, 0.5f);
    if(!nearf(output, -3.0f, 0.0001f))
    {
        puts("reset failed");
        return 1;
    }

    puts("IMU rate filter numeric checks passed");
    return 0;
}
'@ | Set-Content (Join-Path $tempPath "test_imu_rate_filter.c") -NoNewline

    if(-not (Test-Path "project/code/imu_rate_filter.c")) {
        throw "imu_rate_filter.c not found - implement it first."
    }
    if(-not (Test-Path "project/code/imu_rate_filter.h")) {
        throw "imu_rate_filter.h not found - implement it first."
    }

    Copy-Item "project/code/imu_rate_filter.c" (Join-Path $tempPath "imu_rate_filter.c")
    Copy-Item "project/code/imu_rate_filter.h" (Join-Path $tempPath "imu_rate_filter.h")

    $compiler = (Get-Command gcc -ErrorAction Stop).Source
    $compilerDirectory = Split-Path $compiler
    $originalPath = $env:PATH
    $env:PATH = $compilerDirectory + [System.IO.Path]::PathSeparator + $env:PATH
    $binary = Join-Path $tempPath "test_imu_rate_filter.exe"
    $compileOutput = & $compiler -std=c99 -Wall -Werror -I $tempPath `
        (Join-Path $tempPath "imu_rate_filter.c") `
        (Join-Path $tempPath "test_imu_rate_filter.c") -o $binary 2>&1
    $env:PATH = $originalPath
    if(0 -ne $LASTEXITCODE) {
        $compileOutput | Write-Host
        throw "IMU rate filter compile failed."
    }

    & $binary
    if(0 -ne $LASTEXITCODE) {
        throw "IMU rate filter numeric test failed."
    }
}
finally {
    if($null -ne $originalPath) {
        $env:PATH = $originalPath
    }
    if(Test-Path $tempPath) {
        Remove-Item -Recurse -Force $tempPath
    }
}

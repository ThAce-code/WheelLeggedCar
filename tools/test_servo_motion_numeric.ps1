$ErrorActionPreference = "Stop"

$tempPath = Join-Path ([System.IO.Path]::GetTempPath()) ("servo-motion-" + [Guid]::NewGuid().ToString())
New-Item -ItemType Directory -Path $tempPath | Out-Null
$originalPath = $null

try {
    # Write minimal host app_types.h
    @'
#ifndef _app_types_h_
#define _app_types_h_
#include <stddef.h>
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef enum { APP_FALSE = 0, APP_TRUE = 1 } app_bool_enum;
#endif
'@ | Set-Content (Join-Path $tempPath "app_types.h") -NoNewline

    # Write test body
    $testSource = Join-Path $tempPath "test_servo_motion.c"
    @'
#include "servo_motion.h"
#include <math.h>
#include <stdio.h>

static int nearf(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

int main(void)
{
    servo_motion_state_struct state;
    uint16 tick;

    servo_motion_init(&state, 90.0f);
    servo_motion_step(&state, 100.0f, 0.05f, 100.0f, 0.2f, 30U);
    if(!nearf(state.filtered_deg, 90.5f, 0.0001f) ||
       !nearf(state.output_deg, 90.5f, 0.0001f) ||
       (APP_TRUE == state.settled))
    {
        puts("LPF first step failed");
        return 1;
    }

    servo_motion_init(&state, 90.0f);
    servo_motion_step(&state, 120.0f, 0.05f, 0.3f, 0.2f, 30U);
    if(!nearf(state.filtered_deg, 91.5f, 0.0001f) ||
       !nearf(state.output_deg, 90.3f, 0.0001f))
    {
        puts("normal rate guard failed");
        return 1;
    }

    servo_motion_init(&state, 90.0f);
    servo_motion_step(&state, 120.0f, 0.05f, 0.6f, 0.2f, 30U);
    if(!nearf(state.output_deg, 90.6f, 0.0001f))
    {
        puts("fast rate guard failed");
        return 1;
    }

    servo_motion_init(&state, 90.0f);
    for(tick = 0U; tick < 29U; tick++)
    {
        servo_motion_step(&state, 90.0f, 0.05f, 0.3f, 0.2f, 30U);
    }
    if(APP_TRUE == state.settled)
    {
        puts("settled asserted early");
        return 1;
    }
    servo_motion_step(&state, 90.0f, 0.05f, 0.3f, 0.2f, 30U);
    if(APP_TRUE != state.settled)
    {
        puts("settled did not assert at 30 ticks");
        return 1;
    }

    servo_motion_apply_immediate(&state, 66.0f);
    if(!nearf(state.target_deg, 66.0f, 0.0001f) ||
       !nearf(state.filtered_deg, 66.0f, 0.0001f) ||
       !nearf(state.output_deg, 66.0f, 0.0001f))
    {
        puts("immediate state synchronization failed");
        return 1;
    }

    puts("servo motion numeric checks passed");
    return 0;
}
'@ | Set-Content $testSource -NoNewline

    # Copy production files
    $servoMotionC = "project/code/servo_motion.c"
    $servoMotionH = "project/code/servo_motion.h"

    if(-not (Test-Path $servoMotionC)) {
        throw "servo_motion.c not found - implement it first."
    }
    if(-not (Test-Path $servoMotionH)) {
        throw "servo_motion.h not found - implement it first."
    }

    Copy-Item $servoMotionC (Join-Path $tempPath "servo_motion.c")
    Copy-Item $servoMotionH (Join-Path $tempPath "servo_motion.h")

    $compiler = (Get-Command gcc -ErrorAction Stop).Source
    $compilerDirectory = Split-Path $compiler
    $originalPath = $env:PATH
    $env:PATH = $compilerDirectory + [System.IO.Path]::PathSeparator + $env:PATH
    $binary = Join-Path $tempPath "test_servo_motion.exe"
    $compileOutput = & $compiler -std=c99 -Wall -Werror -I $tempPath `
        (Join-Path $tempPath 'servo_motion.c') $testSource -lm -o $binary 2>&1
    $env:PATH = $originalPath
    if(0 -ne $LASTEXITCODE) {
        $compileOutput | Write-Host
        throw ("servo motion compile failed (exit {0})." -f $LASTEXITCODE)
    }
    & $binary
    if(0 -ne $LASTEXITCODE) {
        throw ("servo motion numeric test failed (exit {0})." -f $LASTEXITCODE)
    }
    Write-Host "servo motion numeric checks passed"
}
finally {
    if($null -ne $originalPath) {
        $env:PATH = $originalPath
    }
    if(Test-Path $tempPath) {
        Remove-Item -Recurse -Force $tempPath
    }
}

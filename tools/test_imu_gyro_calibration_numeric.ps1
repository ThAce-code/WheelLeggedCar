$ErrorActionPreference = "Stop"

$tempPath = Join-Path ([System.IO.Path]::GetTempPath()) ("imu-gyro-calibration-" + [Guid]::NewGuid().ToString())
New-Item -ItemType Directory -Path $tempPath | Out-Null
$originalPath = $null

try {
    @'
#ifndef _app_types_h_
#define _app_types_h_
typedef unsigned char uint8;
typedef unsigned int uint32;
typedef enum { APP_FALSE = 0, APP_TRUE = 1 } app_bool_enum;
#endif
'@ | Set-Content (Join-Path $tempPath "app_types.h") -NoNewline

    @'
#include "imu_gyro_calibration.h"
#include <math.h>
#include <stdio.h>

static int nearf(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static void add_alternating_samples(imu_gyro_calibration_state_struct *state,
                                    float mean_x,
                                    float mean_y,
                                    float mean_z,
                                    float noise_dps,
                                    uint32 count)
{
    uint32 i;

    for(i = 0U; i < count; i++)
    {
        float noise = (0U == (i & 1U)) ? noise_dps : -noise_dps;
        imu_gyro_calibration_add_sample(state,
                                        mean_x + noise,
                                        mean_y - noise,
                                        mean_z + noise);
    }
}

int main(void)
{
    imu_gyro_calibration_state_struct state;
    float offset_x;
    float offset_y;
    float offset_z;
    uint8 result;

    imu_gyro_calibration_init(&state);
    add_alternating_samples(&state, 0.40f, -0.30f, 0.20f, 0.79f, 200U);
    result = imu_gyro_calibration_finish(&state,
                                         200U,
                                         5.0f,
                                         4.0f,
                                         &offset_x,
                                         &offset_y,
                                         &offset_z);
    if(IMU_GYRO_CALIBRATION_OK != result)
    {
        puts("measured stationary noise was rejected");
        return 1;
    }
    if((!nearf(offset_x, 0.40f, 0.001f)) ||
       (!nearf(offset_y, -0.30f, 0.001f)) ||
       (!nearf(offset_z, 0.20f, 0.001f)))
    {
        puts("stationary offset estimate is incorrect");
        return 1;
    }

    imu_gyro_calibration_init(&state);
    add_alternating_samples(&state, 8.0f, 0.0f, 0.0f, 0.20f, 200U);
    result = imu_gyro_calibration_finish(&state,
                                         200U,
                                         5.0f,
                                         4.0f,
                                         &offset_x,
                                         &offset_y,
                                         &offset_z);
    if(IMU_GYRO_CALIBRATION_MOVING != result)
    {
        puts("constant angular motion was accepted as zero bias");
        return 1;
    }

    imu_gyro_calibration_init(&state);
    add_alternating_samples(&state, 0.0f, 0.0f, 0.0f, 3.0f, 200U);
    result = imu_gyro_calibration_finish(&state,
                                         200U,
                                         5.0f,
                                         4.0f,
                                         &offset_x,
                                         &offset_y,
                                         &offset_z);
    if(IMU_GYRO_CALIBRATION_MOVING != result)
    {
        puts("high vibration was accepted as stationary");
        return 1;
    }

    imu_gyro_calibration_init(&state);
    add_alternating_samples(&state, 0.0f, 0.0f, 0.0f, 0.10f, 199U);
    result = imu_gyro_calibration_finish(&state,
                                         200U,
                                         5.0f,
                                         4.0f,
                                         &offset_x,
                                         &offset_y,
                                         &offset_z);
    if(IMU_GYRO_CALIBRATION_NOT_READY != result)
    {
        puts("incomplete calibration window was accepted");
        return 1;
    }

    puts("IMU gyro calibration numeric checks passed");
    return 0;
}
'@ | Set-Content (Join-Path $tempPath "test_imu_gyro_calibration.c") -NoNewline

    if(-not (Test-Path "project/code/imu_gyro_calibration.c")) {
        throw "imu_gyro_calibration.c not found - implement it first."
    }
    if(-not (Test-Path "project/code/imu_gyro_calibration.h")) {
        throw "imu_gyro_calibration.h not found - implement it first."
    }

    Copy-Item "project/code/imu_gyro_calibration.c" (Join-Path $tempPath "imu_gyro_calibration.c")
    Copy-Item "project/code/imu_gyro_calibration.h" (Join-Path $tempPath "imu_gyro_calibration.h")

    $compiler = (Get-Command gcc -ErrorAction Stop).Source
    $compilerDirectory = Split-Path $compiler
    $originalPath = $env:PATH
    $env:PATH = $compilerDirectory + [System.IO.Path]::PathSeparator + $env:PATH
    $binary = Join-Path $tempPath "test_imu_gyro_calibration.exe"
    $compileOutput = & $compiler -std=c99 -Wall -Werror -I $tempPath `
        (Join-Path $tempPath "imu_gyro_calibration.c") `
        (Join-Path $tempPath "test_imu_gyro_calibration.c") -o $binary 2>&1
    $env:PATH = $originalPath
    if(0 -ne $LASTEXITCODE) {
        $compileOutput | Write-Host
        throw "IMU gyro calibration compile failed."
    }

    & $binary
    if(0 -ne $LASTEXITCODE) {
        throw "IMU gyro calibration numeric test failed."
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

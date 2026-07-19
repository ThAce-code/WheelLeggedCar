$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$tempRoot = Join-Path ([System.IO.Path]::GetTempPath()) ("race_assist_numeric_" + [guid]::NewGuid().ToString("N"))
$gcc = Get-Command gcc -ErrorAction Stop
$gccBin = Split-Path -Parent $gcc.Source
$savedPath = $env:PATH

try {
    $env:PATH = "$gccBin$([System.IO.Path]::PathSeparator)$savedPath"
    New-Item -ItemType Directory -Path $tempRoot -Force | Out-Null

    Copy-Item (Join-Path $repoRoot "project/code/control_race_assist.c") $tempRoot
    Copy-Item (Join-Path $repoRoot "project/code/control_race_assist.h") $tempRoot
    Copy-Item (Join-Path $repoRoot "project/code/app_types.h") $tempRoot
    Copy-Item (Join-Path $repoRoot "project/code/app_config.h") $tempRoot

    @'
#ifndef _zf_common_headfile_h_
#define _zf_common_headfile_h_

#include <stddef.h>
#include <stdint.h>

typedef uint8_t uint8;
typedef int8_t int8;
typedef uint16_t uint16;
typedef int16_t int16;
typedef uint32_t uint32;
typedef int32_t int32;

#endif
'@ | Set-Content -LiteralPath (Join-Path $tempRoot "zf_common_headfile.h") -NoNewline

    @'
#include <math.h>
#include <stdio.h>

#include "control_race_assist.h"

static int expect_near(float actual, float expected, float tolerance)
{
    return (fabsf(actual - expected) <= tolerance) ? 0 : 1;
}

static int expect_state(race_assist_state_enum expected)
{
    const race_assist_output_struct *output = control_race_assist_get_output();

    if(output->state != expected)
    {
        fprintf(stderr, "expected state %d, got %d\n", (int)expected, (int)output->state);
        return 1;
    }
    return 0;
}

static int expect_safe_u_request(float u_request)
{
    if((0 == isfinite(u_request)) || (1.0f < u_request) || (-1.0f > u_request))
    {
        fprintf(stderr, "unsafe u_request %.3f\n", (double)u_request);
        return 1;
    }
    return 0;
}

static race_assist_input_struct healthy_input(void)
{
    race_assist_input_struct input = {0};

    input.target_rpm = 220.0f;
    input.ramped_rpm = 220.0f;
    input.measured_rpm = 220.0f;
    input.pitch_deg = 0.0f;
    input.pitch_rate_dps = 0.0f;
    input.leg_u_actual = 0.0f;
    input.dt_s = 0.1f;
    input.fast_enable = APP_TRUE;
    input.feedback_healthy = APP_TRUE;
    input.low_pose_ready = APP_TRUE;
    input.leg_path_fault = APP_FALSE;
    return input;
}

static int check_level_profiles(void)
{
    static const float target_limit[4] = {250.0f, 300.0f, 350.0f, 400.0f};
    static const float output_limit[4] = {300.0f, 350.0f, 410.0f, 460.0f};
    uint8 level;

    for(level = 1U; level <= 4U; level++)
    {
        race_assist_level_profile_struct profile;
        if((APP_TRUE != control_race_assist_get_level_profile(level, &profile)) ||
           expect_near(profile.forward_limit_rpm, target_limit[level - 1U], 0.001f) ||
           expect_near(profile.balance_limit_rpm, output_limit[level - 1U], 0.001f) ||
           expect_near(profile.dx_mm, 2.0f, 0.001f) ||
           expect_near(profile.dy_mm, 2.0f, 0.001f))
        {
            return 1;
        }
    }
    return 0;
}

static int check_fail_closed_transitions(void)
{
    const race_assist_output_struct *output;
    race_assist_input_struct input;

    control_race_assist_init();
    if((APP_TRUE != control_race_assist_set_gains(0.005f, 0.005f, 0.15f)) ||
       (APP_TRUE != control_race_assist_set_level(0U)))
    {
        return 1;
    }

    input = healthy_input();
    control_race_assist_update(&input);
    output = control_race_assist_get_output();
    if(expect_state(RACE_ASSIST_DISABLED) ||
       expect_near(output->u_request, 0.0f, 0.001f) ||
       expect_near(output->balance_limit_rpm, 300.0f, 0.001f))
    {
        return 1;
    }

    if((APP_FALSE != control_race_assist_set_level(2U)) ||
       (APP_TRUE != control_race_assist_set_level(1U)))
    {
        return 1;
    }

    input = healthy_input();
    input.measured_rpm = 220.0f;
    control_race_assist_update(&input);
    if(expect_state(RACE_ASSIST_LOW_RACE))
    {
        return 1;
    }

    input.target_rpm = 240.0f;
    input.ramped_rpm = 240.0f;
    input.measured_rpm = 240.0f;
    control_race_assist_update(&input);
    if(expect_state(RACE_ASSIST_ARMED))
    {
        return 1;
    }

    input.target_rpm = 250.0f;
    input.ramped_rpm = 250.0f;
    input.measured_rpm = 250.0f;
    control_race_assist_update(&input);
    output = control_race_assist_get_output();
    if(expect_state(RACE_ASSIST_BOOST) || (0.0f >= output->u_request))
    {
        return 1;
    }

    control_race_assist_update(&input);
    output = control_race_assist_get_output();
    if(expect_state(RACE_ASSIST_CRUISE_HOLD) || (0.0f > output->u_request))
    {
        return 1;
    }

    input.target_rpm = 240.0f;
    input.ramped_rpm = 240.0f;
    input.measured_rpm = 250.0f;
    control_race_assist_update(&input);
    output = control_race_assist_get_output();
    if(expect_state(RACE_ASSIST_BRAKE) || (0.0f <= output->u_request))
    {
        return 1;
    }

    input.fast_enable = APP_FALSE;
    input.measured_rpm = 210.0f;
    input.leg_u_actual = 0.50f;
    control_race_assist_update(&input);
    if(expect_state(RACE_ASSIST_RECENTER))
    {
        return 1;
    }

    input.measured_rpm = 190.0f;
    input.leg_u_actual = 0.40f;
    control_race_assist_update(&input);
    output = control_race_assist_get_output();
    if(expect_state(RACE_ASSIST_RECENTER) ||
       expect_near(output->u_request, 0.0f, 0.001f))
    {
        return 1;
    }

    input.leg_u_actual = 0.0f;
    control_race_assist_update(&input);
    if(expect_state(RACE_ASSIST_DISABLED))
    {
        return 1;
    }

    control_race_assist_init();
    if((APP_TRUE != control_race_assist_set_gains(0.005f, 0.005f, 0.15f)) ||
       (APP_TRUE != control_race_assist_set_level(1U)))
    {
        return 1;
    }
    input = healthy_input();
    input.ramped_rpm = 250.0f;
    input.measured_rpm = 250.0f;
    input.pitch_deg = 16.0f;
    input.leg_u_actual = 0.37f;
    control_race_assist_update(&input);
    output = control_race_assist_get_output();
    if(expect_state(RACE_ASSIST_FAULT_HOLD) ||
       (RACE_ASSIST_FAULT_PITCH_LIMIT != output->fault_reason) ||
       expect_near(output->u_request, 0.37f, 0.001f) ||
       (0.0f > output->u_request))
    {
        return 1;
    }

    control_race_assist_init();
    input = healthy_input();
    input.target_rpm = NAN;
    input.measured_rpm = INFINITY;
    control_race_assist_update(&input);
    output = control_race_assist_get_output();
    if(expect_state(RACE_ASSIST_FAULT_HOLD) ||
       (RACE_ASSIST_FAULT_INPUT_INVALID != output->fault_reason) ||
       (0 == isfinite(output->u_request)) ||
       expect_near(output->u_request, 0.0f, 0.001f))
    {
        return 1;
    }

    control_race_assist_init();
    if((APP_TRUE != control_race_assist_set_gains(0.005f, 0.005f, 0.15f)) ||
       (APP_TRUE != control_race_assist_set_level(1U)))
    {
        return 1;
    }
    input = healthy_input();
    input.target_rpm = 300.0f;
    input.ramped_rpm = 300.0f;
    input.measured_rpm = 300.0f;
    control_race_assist_update(&input);
    if(expect_near(control_race_assist_get_output()->turn_scale, 1.0f, 0.001f))
    {
        return 1;
    }

    input.target_rpm = 350.0f;
    input.ramped_rpm = 350.0f;
    input.measured_rpm = 350.0f;
    control_race_assist_update(&input);
    if(expect_near(control_race_assist_get_output()->turn_scale, 0.5f, 0.001f))
    {
        return 1;
    }

    input.target_rpm = 400.0f;
    input.ramped_rpm = 400.0f;
    input.measured_rpm = 400.0f;
    control_race_assist_update(&input);
    return expect_near(control_race_assist_get_output()->turn_scale, 0.0f, 0.001f);
}

static int check_leg_u_actual_bounds(void)
{
    const race_assist_output_struct *output;
    race_assist_input_struct input;

    control_race_assist_init();
    if(APP_TRUE != control_race_assist_set_level(1U))
    {
        return 1;
    }
    input = healthy_input();
    input.leg_u_actual = 2.0f;
    control_race_assist_update(&input);
    output = control_race_assist_get_output();
    if(expect_state(RACE_ASSIST_FAULT_HOLD) ||
       (RACE_ASSIST_FAULT_INPUT_INVALID != output->fault_reason) ||
       expect_near(output->u_request, 0.0f, 0.001f) ||
       expect_safe_u_request(output->u_request))
    {
        return 1;
    }

    control_race_assist_init();
    if(APP_TRUE != control_race_assist_set_level(1U))
    {
        return 1;
    }
    input = healthy_input();
    input.ramped_rpm = 250.0f;
    input.measured_rpm = 250.0f;
    input.leg_u_actual = 0.75f;
    control_race_assist_update(&input);

    input.fast_enable = APP_FALSE;
    input.measured_rpm = 210.0f;
    control_race_assist_update(&input);
    output = control_race_assist_get_output();
    if(expect_state(RACE_ASSIST_RECENTER) ||
       expect_near(output->u_request, 0.75f, 0.001f) ||
       expect_safe_u_request(output->u_request))
    {
        return 1;
    }

    input.leg_u_actual = 50.0f;
    control_race_assist_update(&input);
    output = control_race_assist_get_output();
    if(expect_state(RACE_ASSIST_FAULT_HOLD) ||
       (RACE_ASSIST_FAULT_INPUT_INVALID != output->fault_reason) ||
       expect_near(output->u_request, 0.0f, 0.001f) ||
       expect_safe_u_request(output->u_request))
    {
        return 1;
    }

    return 0;
}

static int check_reported_leg_path_fault(void)
{
    const race_assist_output_struct *output;
    race_assist_input_struct input;

    control_race_assist_init();
    if(APP_TRUE != control_race_assist_set_level(1U))
    {
        return 1;
    }
    input = healthy_input();
    input.leg_u_actual = 0.42f;
    control_race_assist_report_leg_path_fault();
    control_race_assist_update(&input);
    output = control_race_assist_get_output();
    if(expect_state(RACE_ASSIST_FAULT_HOLD) ||
       (RACE_ASSIST_FAULT_LEG_PATH != output->fault_reason) ||
       expect_near(output->u_request, 0.42f, 0.001f) ||
       expect_safe_u_request(output->u_request))
    {
        return 1;
    }
    return 0;
}

static int check_low_pose_is_entry_only(void)
{
    const race_assist_output_struct *output;
    race_assist_input_struct input;

    control_race_assist_init();
    if((APP_TRUE != control_race_assist_set_gains(0.005f, 0.005f, 0.15f)) ||
       (APP_TRUE != control_race_assist_set_level(1U)))
    {
        return 1;
    }

    input = healthy_input();
    control_race_assist_update(&input);
    input.target_rpm = 240.0f;
    input.ramped_rpm = 240.0f;
    input.measured_rpm = 240.0f;
    control_race_assist_update(&input);
    if(expect_state(RACE_ASSIST_ARMED))
    {
        return 1;
    }

    input.target_rpm = 250.0f;
    input.ramped_rpm = 250.0f;
    input.measured_rpm = 250.0f;
    control_race_assist_update(&input);
    output = control_race_assist_get_output();
    if(expect_state(RACE_ASSIST_BOOST) || (0.0f >= output->u_request))
    {
        return 1;
    }

    /* Moving away from neutral makes the command estimate non-neutral and the
       open-loop servo planner unsettled.  Those are entry gates, not runtime
       faults once BOOST/CRUISE/BRAKE has begun. */
    input.low_pose_ready = APP_FALSE;
    input.leg_u_actual = 0.35f;
    control_race_assist_update(&input);
    output = control_race_assist_get_output();
    if(expect_state(RACE_ASSIST_CRUISE_HOLD) ||
       (RACE_ASSIST_FAULT_NONE != output->fault_reason) ||
       (APP_TRUE != output->enable))
    {
        fprintf(stderr, "operational assist incorrectly rechecked the neutral-pose entry gate\n");
        return 1;
    }

    input.target_rpm = 230.0f;
    input.ramped_rpm = 230.0f;
    input.measured_rpm = 250.0f;
    control_race_assist_update(&input);
    output = control_race_assist_get_output();
    if(expect_state(RACE_ASSIST_BRAKE) ||
       (RACE_ASSIST_FAULT_NONE != output->fault_reason) ||
       (0.0f <= output->u_request))
    {
        fprintf(stderr, "runtime brake was blocked after leaving the neutral pose\n");
        return 1;
    }
    return 0;
}

int main(void)
{
    if(check_level_profiles() ||
       check_fail_closed_transitions() ||
       check_leg_u_actual_bounds() ||
       check_reported_leg_path_fault() ||
       check_low_pose_is_entry_only())
    {
        fprintf(stderr, "race assist numeric check failed\n");
        return 1;
    }

    puts("race assist numeric checks passed");
    return 0;
}
'@ | Set-Content -LiteralPath (Join-Path $tempRoot "race_assist_numeric.c") -NoNewline

    $outputExe = Join-Path $tempRoot "race_assist_numeric.exe"
    $compileOutput = & $gcc.Source -std=c99 -Wall -Wextra -Werror -I $tempRoot `
        (Join-Path $tempRoot "control_race_assist.c") `
        (Join-Path $tempRoot "race_assist_numeric.c") -lm -o $outputExe 2>&1
    $compileExit = $LASTEXITCODE
    if($compileExit -ne 0) {
        throw "race assist numeric harness compilation failed.`n$compileOutput"
    }

    & $outputExe
    if($LASTEXITCODE -ne 0) {
        throw "race assist numeric checks failed."
    }
}
finally {
    if(Test-Path -LiteralPath $tempRoot) {
        Remove-Item -LiteralPath $tempRoot -Recurse -Force
    }
    $env:PATH = $savedPath
}

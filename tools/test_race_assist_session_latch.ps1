$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$tempRoot = Join-Path ([System.IO.Path]::GetTempPath()) ("race_assist_session_latch_" + [guid]::NewGuid().ToString("N"))
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

static race_assist_input_struct healthy_input(void)
{
    race_assist_input_struct input = {0};

    input.target_rpm = 220.0f;
    input.ramped_rpm = 220.0f;
    input.measured_rpm = 220.0f;
    input.dt_s = 0.1f;
    input.fast_enable = APP_TRUE;
    input.feedback_healthy = APP_TRUE;
    input.low_pose_ready = APP_TRUE;
    return input;
}

static int expect_state(race_assist_state_enum expected, const char *step)
{
    const race_assist_output_struct *output = control_race_assist_get_output();

    if(expected != output->state)
    {
        fprintf(stderr,
                "%s: expected state %d, got %d fault %d u %.3f\n",
                step,
                (int)expected,
                (int)output->state,
                (int)output->fault_reason,
                (double)output->u_request);
        return 1;
    }
    return 0;
}

static int expect_no_fault_zero_request(const char *step)
{
    const race_assist_output_struct *output = control_race_assist_get_output();

    if((RACE_ASSIST_FAULT_NONE != output->fault_reason) ||
       (0.0001f < fabsf(output->u_request)))
    {
        fprintf(stderr,
                "%s: fault %d or nonzero u_request %.3f\n",
                step,
                (int)output->fault_reason,
                (double)output->u_request);
        return 1;
    }
    return 0;
}

int main(void)
{
    const race_assist_output_struct *output;
    race_assist_input_struct input;
    unsigned int cycle;

    /* 1. A new session cannot enter from a non-neutral pose. */
    control_race_assist_init();
    if(APP_TRUE != control_race_assist_set_level(1U))
    {
        return 1;
    }
    input = healthy_input();
    input.low_pose_ready = APP_FALSE;
    input.leg_u_actual = 0.40f;
    control_race_assist_update(&input);
    output = control_race_assist_get_output();
    if(expect_state(RACE_ASSIST_FAULT_HOLD, "new non-neutral session") ||
       (RACE_ASSIST_FAULT_LEG_NOT_READY != output->fault_reason))
    {
        return 2;
    }

    /* A session that never reached operational cannot bypass the entry gate
       by cancelling its first RECENTER before DISABLED completes. */
    control_race_assist_init();
    if((APP_TRUE != control_race_assist_set_gains(0.02f, 0.0f, 0.0f)) ||
       (APP_TRUE != control_race_assist_set_level(1U)))
    {
        return 3;
    }
    input = healthy_input();
    control_race_assist_update(&input);

    input.target_rpm = 240.0f;
    input.ramped_rpm = 240.0f;
    input.measured_rpm = 240.0f;
    control_race_assist_update(&input);
    if(expect_state(RACE_ASSIST_ARMED, "pre-operational arm"))
    {
        return 4;
    }
    if(APP_TRUE != control_race_assist_set_level(0U))
    {
        return 5;
    }
    control_race_assist_update(&input);
    if(expect_state(RACE_ASSIST_RECENTER, "pre-operational recenter"))
    {
        return 6;
    }
    if(APP_TRUE != control_race_assist_set_level(1U))
    {
        return 7;
    }
    input.low_pose_ready = APP_FALSE;
    input.leg_u_actual = 0.25f;
    control_race_assist_update(&input);
    output = control_race_assist_get_output();
    if(expect_state(RACE_ASSIST_FAULT_HOLD, "pre-operational recenter re-entry") ||
       (RACE_ASSIST_FAULT_LEG_NOT_READY != output->fault_reason))
    {
        fprintf(stderr, "pre-operational RECENTER bypassed the entry gate\n");
        return 8;
    }

    /* Fault reset starts the validated operational session. */
    control_race_assist_init();
    if((APP_TRUE != control_race_assist_set_gains(0.02f, 0.0f, 0.0f)) ||
       (APP_TRUE != control_race_assist_set_level(1U)))
    {
        return 9;
    }
    input = healthy_input();
    control_race_assist_update(&input);

    /* 2. Neutral entry arms and reaches a nonzero operational request. */
    input.target_rpm = 240.0f;
    input.ramped_rpm = 240.0f;
    input.measured_rpm = 240.0f;
    control_race_assist_update(&input);
    if(expect_state(RACE_ASSIST_ARMED, "neutral arm"))
    {
        return 10;
    }
    input.target_rpm = 250.0f;
    input.ramped_rpm = 250.0f;
    input.measured_rpm = 250.0f;
    control_race_assist_update(&input);
    output = control_race_assist_get_output();
    if(expect_state(RACE_ASSIST_BOOST, "operational boost") ||
       (0.0f >= output->u_request))
    {
        return 11;
    }

    /* 3. BRA1 remains selected while speed falls below the arm threshold. */
    input.low_pose_ready = APP_FALSE;
    input.leg_u_actual = 0.50f;
    input.target_rpm = 0.0f;
    input.ramped_rpm = 220.0f;
    input.measured_rpm = 220.0f;
    control_race_assist_update(&input);
    if(expect_state(RACE_ASSIST_LOW_RACE, "BRA1 speed drop") ||
       expect_no_fault_zero_request("BRA1 speed drop"))
    {
        return 12;
    }

    /* 4. Multiple operational-session cycles keep returning u to zero
       without re-running the neutral-pose entry gate. */
    for(cycle = 0U; cycle < 20U; cycle++)
    {
        input.leg_u_actual -= 0.025f;
        control_race_assist_update(&input);
        if(expect_state(RACE_ASSIST_LOW_RACE, "operational recenter cycle") ||
           expect_no_fault_zero_request("operational recenter cycle"))
        {
            return 13;
        }
    }

    /* 5. A settled neutral leg remains in the armed level's LOW_RACE state. */
    input.leg_u_actual = 0.0f;
    input.low_pose_ready = APP_TRUE;
    control_race_assist_update(&input);
    if(expect_state(RACE_ASSIST_LOW_RACE, "settled neutral"))
    {
        return 14;
    }

    /* 6. BRA0 completes RECENTER -> DISABLED and clears the session latch. */
    if(APP_TRUE != control_race_assist_set_level(0U))
    {
        return 15;
    }
    control_race_assist_update(&input);
    if(expect_state(RACE_ASSIST_RECENTER, "BRA0 recenter"))
    {
        return 16;
    }
    input.measured_rpm = 190.0f;
    control_race_assist_update(&input);
    if(expect_state(RACE_ASSIST_DISABLED, "BRA0 disabled"))
    {
        return 17;
    }

    if(APP_TRUE != control_race_assist_set_level(1U))
    {
        return 18;
    }
    input.low_pose_ready = APP_FALSE;
    input.leg_u_actual = 0.25f;
    control_race_assist_update(&input);
    output = control_race_assist_get_output();
    if(expect_state(RACE_ASSIST_FAULT_HOLD, "next non-neutral session") ||
       (RACE_ASSIST_FAULT_LEG_NOT_READY != output->fault_reason))
    {
        fprintf(stderr, "completed DISABLED did not clear the operational-session latch\n");
        return 19;
    }

    puts("race assist operational session latch checks passed");
    return 0;
}
'@ | Set-Content -LiteralPath (Join-Path $tempRoot "race_assist_session_latch.c") -NoNewline

    $outputExe = Join-Path $tempRoot "race_assist_session_latch.exe"
    $compileOutput = & $gcc.Source -std=c99 -Wall -Wextra -Werror -I $tempRoot `
        (Join-Path $tempRoot "control_race_assist.c") `
        (Join-Path $tempRoot "race_assist_session_latch.c") -lm -o $outputExe 2>&1
    if($LASTEXITCODE -ne 0) {
        throw "race assist operational session latch harness compilation failed.`n$compileOutput"
    }

    & $outputExe
    if($LASTEXITCODE -ne 0) {
        throw "race assist operational session latch checks failed."
    }
}
finally {
    $env:PATH = $savedPath
    if(Test-Path -LiteralPath $tempRoot) {
        Remove-Item -LiteralPath $tempRoot -Recurse -Force
    }
}

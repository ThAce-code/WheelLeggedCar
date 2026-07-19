$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$tempRoot = Join-Path ([System.IO.Path]::GetTempPath()) ("control_chassis_race_" + [guid]::NewGuid().ToString("N"))
$gcc = Get-Command gcc -ErrorAction Stop
$savedPath = $env:PATH

try {
    New-Item -ItemType Directory -Path $tempRoot -Force | Out-Null
    $env:PATH = (Split-Path $gcc.Source) + [System.IO.Path]::PathSeparator + $savedPath

    @(
        "project/code/control_chassis.c",
        "project/code/control_chassis.h",
        "project/code/control_race_assist.c",
        "project/code/control_race_assist.h",
        "project/code/control_leg.h",
        "project/code/leg_config.h",
        "project/code/actuator_motor.h",
        "project/code/sensor_imu.h",
        "project/code/app_types.h",
        "project/code/app_config.h"
    ) | ForEach-Object {
        Copy-Item (Join-Path $repoRoot $_) $tempRoot
    }

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
#include "actuator_motor.h"
#include "app_config.h"
#include "control_leg.h"
#include "sensor_imu.h"

static imu_state_struct host_imu;
static wheel_feedback_struct host_wheel_feedback;
static motor_rpm_loop_diag_struct host_rpm_diag;
static leg_diag_struct host_leg_diag;
static leg_stance_profile_struct host_stance_profile;

uint8 host_race_request_accept = APP_TRUE;
uint32 host_race_request_count = 0U;

void host_chassis_reset(uint32 now_ms, leg_motion_state_enum motion_state)
{
    host_imu = (imu_state_struct){0};
    host_imu.timestamp_ms = now_ms;
    host_imu.healthy = APP_TRUE;

    host_wheel_feedback = (wheel_feedback_struct){0};
    host_wheel_feedback.online = APP_TRUE;
    host_wheel_feedback.left_online = APP_TRUE;
    host_wheel_feedback.right_online = APP_TRUE;

    host_rpm_diag = (motor_rpm_loop_diag_struct){0};

    host_leg_diag = (leg_diag_struct){0};
    host_leg_diag.motion_state = motion_state;
    host_leg_diag.mode = (uint8)LEG_MODE_IK_VALIDATE;
    host_leg_diag.ik_valid = APP_TRUE;
    host_leg_diag.output_enable = APP_TRUE;
    host_leg_diag.drive_allowed = APP_TRUE;
    host_leg_diag.servo_settled = APP_TRUE;
    host_leg_diag.left_command_pose_body_mm.x_mm = APP_RACE_ASSIST_ZERO_X_MM;
    host_leg_diag.left_command_pose_body_mm.y_mm = APP_RACE_ASSIST_ZERO_Y_MM;
    host_leg_diag.left_command_pose_body_mm.valid = APP_TRUE;
    host_leg_diag.right_command_pose_body_mm.x_mm = APP_RACE_ASSIST_ZERO_X_MM;
    host_leg_diag.right_command_pose_body_mm.y_mm = APP_RACE_ASSIST_ZERO_Y_MM;
    host_leg_diag.right_command_pose_body_mm.valid = APP_TRUE;

    host_stance_profile = (leg_stance_profile_struct){0};
    host_stance_profile.transition_forward_limit_rpm = 30.0f;
    host_stance_profile.chassis_forward_limit_low_rpm = APP_CHASSIS_FORWARD_RPM_LIMIT;
    host_stance_profile.chassis_forward_limit_high_rpm = APP_CHASSIS_FORWARD_RPM_LIMIT;
    host_stance_profile.chassis_fast_forward_limit_low_rpm = APP_CHASSIS_FAST_FORWARD_RPM_LIMIT;
    host_stance_profile.chassis_fast_forward_limit_high_rpm = APP_CHASSIS_FAST_FORWARD_RPM_LIMIT;
    host_race_request_accept = APP_TRUE;
    host_race_request_count = 0U;
}

void host_set_feedback_healthy(uint8 healthy, uint32 now_ms)
{
    host_imu.timestamp_ms = now_ms;
    host_imu.healthy = healthy;
    host_wheel_feedback.online = healthy;
    host_wheel_feedback.left_online = healthy;
    host_wheel_feedback.right_online = healthy;
}

const motor_rpm_loop_diag_struct *actuator_motor_get_motor_rpm_loop_diag(void)
{
    return &host_rpm_diag;
}

const wheel_feedback_struct *actuator_motor_get_feedback(void)
{
    return &host_wheel_feedback;
}

const imu_state_struct *sensor_imu_get_state(void)
{
    return &host_imu;
}

const leg_diag_struct *control_leg_get_diag(void)
{
    return &host_leg_diag;
}

const leg_stance_profile_struct *leg_config_get_stance_profile(void)
{
    return &host_stance_profile;
}

uint8 control_leg_set_race_assist_request(float u_request,
                                          float dx_mm,
                                          float dy_mm,
                                          uint32 now_ms)
{
    (void)u_request;
    (void)dx_mm;
    (void)dy_mm;
    (void)now_ms;
    host_race_request_count++;
    return host_race_request_accept;
}

void control_leg_disable_race_assist(uint32 now_ms)
{
    (void)now_ms;
}
'@ | Set-Content -LiteralPath (Join-Path $tempRoot "chassis_mocks.c") -NoNewline

    @'
#include <math.h>
#include <stdio.h>

#include "app_config.h"
#include "control_chassis.h"
#include "control_race_assist.h"

void host_chassis_reset(uint32 now_ms, leg_motion_state_enum motion_state);
void host_set_feedback_healthy(uint8 healthy, uint32 now_ms);
extern uint8 host_race_request_accept;
extern uint32 host_race_request_count;

static int expect_near(float actual, float expected, float tolerance)
{
    return (fabsf(actual - expected) <= tolerance) ? 0 : 1;
}

static int expect_zero_forward_output(const char *name)
{
    const chassis_cmd_struct *cmd = control_chassis_get_cmd();
    const chassis_output_struct *output = control_chassis_get_output();

    if(expect_near(cmd->target_forward_rpm, 0.0f, 0.001f) ||
       expect_near(cmd->actual_forward_rpm, 0.0f, 0.001f) ||
       expect_near(output->forward_target_rpm, 0.0f, 0.001f) ||
       expect_near(output->forward_actual_rpm, 0.0f, 0.001f) ||
       expect_near(output->forward_limit_eff_rpm, 0.0f, 0.001f) ||
       expect_near(output->fast_forward_limit_eff_rpm, 0.0f, 0.001f))
    {
        fprintf(stderr, "%s did not publish zero forward target/actual/caps\n", name);
        return 1;
    }
    return 0;
}

static int check_requested_fast_reaches_supervisor(void)
{
    const chassis_output_struct *output;

    host_chassis_reset(5U, LEG_MOTION_TRANSITION);
    control_chassis_init();
    if(APP_TRUE != control_chassis_set_race_assist_level(1U, 5U))
    {
        return 1;
    }
    control_chassis_set_fast_enable(APP_TRUE);
    control_chassis_set_cmd(200.0f, 0.0f, APP_TRUE, 5U);
    control_chassis_update(5U);
    output = control_chassis_get_output();

    if((APP_TRUE != output->race_assist_enable) ||
       expect_near(output->forward_limit_eff_rpm, 30.0f, 0.001f) ||
       expect_near(output->fast_forward_limit_eff_rpm, 30.0f, 0.001f))
    {
        fprintf(stderr, "requested fast_enable did not reach supervisor while transition remained capped\n");
        return 1;
    }
    return 0;
}

static int check_rejected_leg_request_stops_same_cycle(void)
{
    host_chassis_reset(10U, LEG_MOTION_STABLE);
    host_race_request_accept = APP_FALSE;
    control_chassis_init();
    if(APP_TRUE != control_chassis_set_race_assist_level(1U, 10U))
    {
        return 1;
    }
    control_chassis_set_fast_enable(APP_TRUE);
    control_chassis_set_cmd(200.0f, 0.0f, APP_TRUE, 10U);
    control_chassis_update(10U);
    if(0U == host_race_request_count)
    {
        fprintf(stderr, "race supervisor did not issue the Task 2 leg request\n");
        return 1;
    }
    return expect_zero_forward_output("rejected race leg request");
}

static int check_rejected_leg_request_decelerates_bounded(void)
{
    const chassis_cmd_struct *cmd;
    const chassis_output_struct *output;
    float forward_before_fault;
    float fault_ramp_delta;

    host_chassis_reset(100U, LEG_MOTION_STABLE);
    control_chassis_init();
    if(APP_TRUE != control_chassis_set_race_assist_level(1U, 100U))
    {
        return 1;
    }
    control_chassis_set_fast_enable(APP_TRUE);
    control_chassis_set_cmd(200.0f, 0.0f, APP_TRUE, 100U);
    for(uint32 now_ms = 100U; now_ms <= 600U; now_ms += 5U)
    {
        host_set_feedback_healthy(APP_TRUE, now_ms);
        control_chassis_set_cmd(200.0f, 0.0f, APP_TRUE, now_ms);
        control_chassis_update(now_ms);
    }
    cmd = control_chassis_get_cmd();
    forward_before_fault = cmd->actual_forward_rpm;
    fault_ramp_delta = APP_CHASSIS_FORWARD_RAMP_RPM_S * 0.005f;
    if(1.0f >= forward_before_fault)
    {
        fprintf(stderr,
                "race setup did not create a nonzero ramped forward target: actual %.3f target %.3f\n",
                forward_before_fault,
                cmd->target_forward_rpm);
        return 1;
    }

    host_race_request_accept = APP_FALSE;
    host_set_feedback_healthy(APP_TRUE, 605U);
    control_chassis_set_cmd(200.0f, 0.0f, APP_TRUE, 605U);
    control_chassis_update(605U);
    cmd = control_chassis_get_cmd();
    output = control_chassis_get_output();
    if(expect_near(cmd->target_forward_rpm, 0.0f, 0.001f) ||
       expect_near(output->forward_limit_eff_rpm, 0.0f, 0.001f) ||
       expect_near(output->fast_forward_limit_eff_rpm, 0.0f, 0.001f) ||
       expect_near(cmd->actual_forward_rpm,
                   forward_before_fault - fault_ramp_delta,
                   0.001f) ||
       expect_near(output->forward_target_rpm, cmd->actual_forward_rpm, 0.001f) ||
       (0.0f >= cmd->actual_forward_rpm) ||
       expect_near(output->speed_pitch_limit_deg,
                   APP_RACE_ASSIST_PITCH_OFFSET_LIMIT_DEG,
                   0.001f))
    {
        fprintf(stderr,
                "race fault policy mismatch: before %.3f target %.3f actual %.3f caps %.3f %.3f output %.3f pitchcap %.3f\n",
                forward_before_fault,
                cmd->target_forward_rpm,
                cmd->actual_forward_rpm,
                output->forward_limit_eff_rpm,
                output->fast_forward_limit_eff_rpm,
                output->forward_target_rpm,
                output->speed_pitch_limit_deg);
        return 1;
    }

    host_set_feedback_healthy(APP_TRUE, 610U);
    control_chassis_update(610U);
    cmd = control_chassis_get_cmd();
    if((0.0f >= cmd->actual_forward_rpm) ||
       expect_near(cmd->actual_forward_rpm,
                   forward_before_fault - (2.0f * fault_ramp_delta),
                   0.001f))
    {
        fprintf(stderr, "race fault forward target did not decelerate within the configured ramp bound\n");
        return 1;
    }
    return 0;
}

static int check_unhealthy_feedback_publishes_zero_caps(void)
{
    host_chassis_reset(15U, LEG_MOTION_STABLE);
    host_set_feedback_healthy(APP_FALSE, 15U);
    control_chassis_init();
    if(APP_TRUE != control_chassis_set_race_assist_level(1U, 15U))
    {
        return 1;
    }
    control_chassis_set_fast_enable(APP_TRUE);
    control_chassis_set_cmd(200.0f, 0.0f, APP_TRUE, 15U);
    control_chassis_update(15U);
    return expect_zero_forward_output("unhealthy feedback fault");
}

static int check_transition_remains_30_rpm(void)
{
    const chassis_output_struct *output;

    host_chassis_reset(20U, LEG_MOTION_TRANSITION);
    control_chassis_init();
    control_chassis_set_fast_enable(APP_TRUE);
    control_chassis_set_cmd(200.0f, 0.0f, APP_TRUE, 20U);
    control_chassis_update(20U);
    output = control_chassis_get_output();

    if((APP_TRUE != output->enable) ||
       expect_near(output->forward_limit_eff_rpm, 30.0f, 0.001f) ||
       expect_near(output->fast_forward_limit_eff_rpm, 30.0f, 0.001f))
    {
        fprintf(stderr, "ordinary leg transition no longer publishes the 30 RPM cap\n");
        return 1;
    }
    return 0;
}

int main(void)
{
    if(check_requested_fast_reaches_supervisor() ||
       check_rejected_leg_request_stops_same_cycle() ||
       check_rejected_leg_request_decelerates_bounded() ||
       check_unhealthy_feedback_publishes_zero_caps() ||
       check_transition_remains_30_rpm())
    {
        return 1;
    }

    puts("control chassis race assist checks passed");
    return 0;
}
'@ | Set-Content -LiteralPath (Join-Path $tempRoot "control_chassis_race_assist.c") -NoNewline

    $outputExe = Join-Path $tempRoot "control_chassis_race_assist.exe"
    $compileErrorAction = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    $compileOutput = & $gcc.Source -std=c99 -Wall -Wextra -Werror -I $tempRoot `
        (Join-Path $tempRoot "control_chassis.c") `
        (Join-Path $tempRoot "control_race_assist.c") `
        (Join-Path $tempRoot "chassis_mocks.c") `
        (Join-Path $tempRoot "control_chassis_race_assist.c") -lm -o $outputExe 2>&1
    $compileExit = $LASTEXITCODE
    $ErrorActionPreference = $compileErrorAction
    if($compileExit -ne 0) {
        throw "control chassis race assist harness compilation failed.`n$compileOutput"
    }

    & $outputExe
    if($LASTEXITCODE -ne 0) {
        throw "control chassis race assist checks failed."
    }
}
finally {
    if(Test-Path -LiteralPath $tempRoot) {
        Remove-Item -LiteralPath $tempRoot -Recurse -Force
    }
    $env:PATH = $savedPath
}

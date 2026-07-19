$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$tempRoot = Join-Path ([System.IO.Path]::GetTempPath()) ("race_assist_scheduler_" + [guid]::NewGuid().ToString("N"))
$gcc = Get-Command gcc -ErrorAction Stop
$savedPath = $env:PATH

try {
    New-Item -ItemType Directory -Path $tempRoot -Force | Out-Null
    $env:PATH = (Split-Path $gcc.Source) + [System.IO.Path]::PathSeparator + $savedPath

    @(
        "app_config.h", "app_types.h", "app_scheduler.c", "app_scheduler.h",
        "app_state.h", "app_safety.h", "sensor_imu.h", "actuator_servo.h",
        "actuator_motor.h", "host_command.h", "telemetry.h", "leg_config.c",
        "leg_config.h", "leg_kinematics.c", "leg_kinematics.h", "control_leg.c",
        "control_leg.h", "control_race_assist.c", "control_race_assist.h",
        "control_chassis.c", "control_chassis.h", "control_balance.c",
        "control_balance.h"
    ) | ForEach-Object {
        Copy-Item (Join-Path $repoRoot ("project/code/" + $_)) $tempRoot
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
#include <math.h>

#include "actuator_motor.h"
#include "actuator_servo.h"
#include "app_scheduler.h"
#include "app_safety.h"
#include "app_state.h"
#include "host_command.h"
#include "leg_kinematics.h"
#include "sensor_imu.h"
#include "telemetry.h"

static app_run_state_enum host_app_state = APP_STATE_STANDBY;
static uint8 host_safety_fault = APP_FALSE;
static imu_state_struct host_imu;
static wheel_feedback_struct host_wheel;
static motor_rpm_loop_diag_struct host_rpm_diag;
static actuator_servo_diag_struct host_servo_diag;
static uint8 host_servo_initialized = APP_FALSE;
static uint8 host_servo_settle_countdown = 0U;

uint32 host_motor_stop_count = 0U;
uint32 host_motor_rpm_command_count = 0U;
uint8 host_fail_next_ik_solve = APP_FALSE;

uint8 __real_leg_kinematics_solve(uint8 right_side,
                                  float x_mm,
                                  float y_mm,
                                  const leg_ik_result_struct *previous,
                                  leg_ik_result_struct *result);

uint8 __wrap_leg_kinematics_solve(uint8 right_side,
                                  float x_mm,
                                  float y_mm,
                                  const leg_ik_result_struct *previous,
                                  leg_ik_result_struct *result)
{
    if(APP_TRUE == host_fail_next_ik_solve)
    {
        host_fail_next_ik_solve = APP_FALSE;
        return APP_FALSE;
    }
    return __real_leg_kinematics_solve(right_side, x_mm, y_mm, previous, result);
}

void host_platform_init(void)
{
    uint8 i;

    host_app_state = APP_STATE_STANDBY;
    host_safety_fault = APP_FALSE;
    host_imu = (imu_state_struct){0};
    host_imu.healthy = APP_TRUE;
    host_wheel = (wheel_feedback_struct){0};
    host_wheel.online = APP_TRUE;
    host_wheel.left_online = APP_TRUE;
    host_wheel.right_online = APP_TRUE;
    host_rpm_diag = (motor_rpm_loop_diag_struct){0};
    host_servo_diag = (actuator_servo_diag_struct){0};
    for(i = 0U; i < 4U; i++)
    {
        host_servo_diag.target_deg[i] = 90.0f;
        host_servo_diag.filtered_deg[i] = 90.0f;
        host_servo_diag.output_deg[i] = 90.0f;
    }
    host_servo_diag.settled = APP_TRUE;
    host_servo_initialized = APP_TRUE;
    host_servo_settle_countdown = 0U;
    host_motor_stop_count = 0U;
    host_motor_rpm_command_count = 0U;
    host_fail_next_ik_solve = APP_FALSE;
}

void host_set_measured_rpm(float rpm)
{
    host_rpm_diag.left_motor_rpm = rpm;
    host_rpm_diag.right_motor_rpm = rpm;
}

void host_set_imu_healthy(uint8 healthy)
{
    host_imu.healthy = healthy;
}

void app_state_init(void) { host_app_state = APP_STATE_STANDBY; }
void app_state_set(app_run_state_enum state) { host_app_state = state; }
app_run_state_enum app_state_get(void) { return host_app_state; }
uint8 app_state_is_run_enabled(void)
{
    return ((APP_STATE_STANDBY == host_app_state) || (APP_STATE_RUN == host_app_state)) ?
           APP_TRUE : APP_FALSE;
}

void app_safety_init(void) { host_safety_fault = APP_FALSE; }
void app_safety_update(uint32 now_ms) { (void)now_ms; }
uint8 app_safety_is_fault(void) { return host_safety_fault; }
void app_safety_force_fault(void) { host_safety_fault = APP_TRUE; }

uint8 sensor_imu_init(void) { return SENSOR_IMU_OK; }
void sensor_imu_update(uint32 now_ms)
{
    host_imu.timestamp_ms = now_ms;
    host_imu.healthy = APP_TRUE;
}
const imu_state_struct *sensor_imu_get_state(void) { return &host_imu; }
void sensor_imu_int1_isr(void) {}
uint8 sensor_imu_take_data_ready(uint32 *source_ms)
{
    if(NULL != source_ms)
    {
        *source_ms = app_scheduler_get_ms();
    }
    return APP_TRUE;
}
uint32 sensor_imu_get_last_update_ms(void) { return host_imu.timestamp_ms; }
uint32 sensor_imu_get_int_count(void) { return 0U; }
uint32 sensor_imu_get_stale_count(void) { return 0U; }
uint32 sensor_imu_get_invalid_sample_count(void) { return 0U; }
void sensor_imu_mark_stale(void) { host_imu.healthy = APP_FALSE; }

void actuator_servo_init(void) {}
void actuator_servo_enable(void) {}
void actuator_servo_disable(void) {}
uint32 actuator_servo_angle_to_duty(float angle_deg) { (void)angle_deg; return 0U; }
float actuator_servo_get_current_angle(uint8 index) { return host_servo_diag.output_deg[index]; }
void actuator_servo_publish_cmd(const servo_cmd_struct *cmd,
                                float speed_limit_dps,
                                uint8 direct_bypass)
{
    uint8 i;
    uint8 changed = APP_FALSE;
    (void)speed_limit_dps;
    (void)direct_bypass;

    for(i = 0U; i < 4U; i++)
    {
        if(0.0001f < fabsf(cmd->angle_deg[i] - host_servo_diag.target_deg[i]))
        {
            changed = APP_TRUE;
        }
        host_servo_diag.target_deg[i] = cmd->angle_deg[i];
        host_servo_diag.filtered_deg[i] = cmd->angle_deg[i];
        host_servo_diag.output_deg[i] = cmd->angle_deg[i];
    }
    if(APP_TRUE == changed)
    {
        host_servo_settle_countdown = 2U;
    }
    else if(0U < host_servo_settle_countdown)
    {
        host_servo_settle_countdown--;
    }
    host_servo_diag.settled = (0U == host_servo_settle_countdown) ? APP_TRUE : APP_FALSE;
    host_servo_initialized = APP_TRUE;
}
void actuator_servo_tick_300hz(void) {}
void actuator_servo_get_diag(actuator_servo_diag_struct *diag)
{
    if((NULL != diag) && (APP_TRUE == host_servo_initialized))
    {
        *diag = host_servo_diag;
    }
}
uint8 actuator_servo_is_settled(void) { return host_servo_diag.settled; }
uint32 actuator_servo_get_tick_count(void) { return 0U; }

void actuator_motor_init(void) {}
void actuator_motor_set_cmd(const motor_cmd_struct *cmd) { (void)cmd; }
void actuator_motor_set_actuator_cmd(const motor_actuator_cmd_struct *cmd) { (void)cmd; }
void actuator_motor_set_mode_stop(void) { host_motor_stop_count++; }
void actuator_motor_set_mode_open_duty(float left_duty, float right_duty)
{
    (void)left_duty;
    (void)right_duty;
}
void actuator_motor_set_mode_motor_rpm(float left_motor_rpm, float right_motor_rpm)
{
    (void)left_motor_rpm;
    (void)right_motor_rpm;
    host_motor_rpm_command_count++;
}
void actuator_motor_record_host_motion(uint32 now_ms) { (void)now_ms; }
void actuator_motor_set_motor_rpm_target(float left_motor_rpm, float right_motor_rpm, uint8 enable)
{
    (void)left_motor_rpm; (void)right_motor_rpm; (void)enable;
}
void actuator_motor_set_open_loop_duty(float left_duty, float right_duty, uint8 enable)
{
    (void)left_duty; (void)right_duty; (void)enable;
}
void actuator_motor_set_rpm_pid_gain(uint8 left_enable, uint8 right_enable, float kp, float ki, float kd)
{
    (void)left_enable; (void)right_enable; (void)kp; (void)ki; (void)kd;
}
void actuator_motor_record_command_error(uint8 is_error) { (void)is_error; }
void actuator_motor_update(uint32 now_ms) { (void)now_ms; }
void actuator_motor_stop(void) { host_motor_stop_count++; }
const motor_cmd_struct *actuator_motor_get_cmd(void) { return NULL; }
const wheel_feedback_struct *actuator_motor_get_feedback(void) { return &host_wheel; }
const motor_diag_struct *actuator_motor_get_diag(void) { return NULL; }
const motor_rpm_loop_diag_struct *actuator_motor_get_motor_rpm_loop_diag(void) { return &host_rpm_diag; }

void host_command_init(void) {}
void host_command_update(uint32 now_ms) { (void)now_ms; }
void telemetry_init(void) {}
void telemetry_update(uint32 now_ms) { (void)now_ms; }
void telemetry_service(void) {}
'@ | Set-Content -LiteralPath (Join-Path $tempRoot "host_platform.c") -NoNewline

    @'
#include <math.h>
#include <stdio.h>

#include "app_config.h"
#include "app_scheduler.h"
#include "app_state.h"
#include "control_balance.h"
#include "control_chassis.h"
#include "control_leg.h"
#include "control_race_assist.h"

void host_platform_init(void);
void host_set_measured_rpm(float rpm);
void host_set_imu_healthy(uint8 healthy);
extern uint32 host_motor_stop_count;
extern uint32 host_motor_rpm_command_count;
extern uint8 host_fail_next_ik_solve;

static int expect_near(float actual, float expected, float tolerance)
{
    return (fabsf(actual - expected) <= tolerance) ? 0 : 1;
}

static void run_one_ms(void)
{
    app_scheduler_tick_1ms();
    app_scheduler_run_pending();
}

static void run_ms_with_command(uint32 duration_ms, float forward_rpm, float turn_dps)
{
    uint32 elapsed;

    for(elapsed = 0U; elapsed < duration_ms; elapsed++)
    {
        const uint32 next_ms = app_scheduler_get_ms() + 1U;
        if(0U == (next_ms % 100U))
        {
            control_chassis_set_cmd(forward_rpm, turn_dps, APP_TRUE, next_ms);
        }
        run_one_ms();
    }
}

static int enter_low_race_neutral(void)
{
    const leg_diag_struct *leg;

    if(APP_TRUE != control_leg_set_ik_reference(app_scheduler_get_ms()))
    {
        fprintf(stderr, "LIKREF rejected\n");
        return 1;
    }
    run_ms_with_command(1200U, 0.0f, 0.0f);
    if(APP_TRUE != control_leg_set_xy(APP_RACE_ASSIST_ZERO_X_MM,
                                      APP_RACE_ASSIST_ZERO_Y_MM,
                                      app_scheduler_get_ms()))
    {
        fprintf(stderr, "neutral LXY rejected\n");
        return 1;
    }
    run_ms_with_command(1500U, 0.0f, 0.0f);
    leg = control_leg_get_diag();
    if((LEG_MODE_IK_VALIDATE != (leg_mode_enum)leg->mode) ||
       (APP_TRUE != leg->ik_valid) ||
       (APP_TRUE != leg->servo_settled))
    {
        fprintf(stderr, "neutral LXY did not settle\n");
        return 1;
    }
    return 0;
}

int main(void)
{
    const chassis_cmd_struct *chassis_cmd;
    const chassis_output_struct *chassis_output;
    const leg_diag_struct *leg;
    const servo_cmd_struct *servo;
    float held_servo_deg[APP_SERVO_COUNT];
    float forward_before_fault;
    float forward_after_fault;
    uint32 stop_count_before_fault;
    uint32 rpm_count_before_fault;
    uint32 stop_count_before_safety_gate;
    uint32 reversal_elapsed_ms;
    uint32 boost_elapsed_ms;
    uint8 i;
    uint8 saw_zero_endpoint = APP_FALSE;
    uint8 saw_negative_endpoint = APP_FALSE;
    uint8 saw_unsettled_off_neutral = APP_FALSE;

    host_platform_init();
    app_scheduler_init();
    control_leg_init();
    control_chassis_init();
    control_balance_init();
    control_balance_set_mode(BALANCE_MODE_BALANCE_FAST);
    host_set_measured_rpm(0.0f);

    if(enter_low_race_neutral() ||
       (APP_TRUE != control_chassis_set_race_assist_gains(0.02f, 0.0f, 0.0f)) ||
       (APP_TRUE != control_chassis_set_race_assist_level(1U, app_scheduler_get_ms())))
    {
        return 1;
    }
    control_chassis_set_fast_enable(APP_TRUE);
    host_set_measured_rpm(240.0f);
    control_chassis_set_cmd(250.0f, 0.0f, APP_TRUE, app_scheduler_get_ms());
    for(boost_elapsed_ms = 0U; boost_elapsed_ms < 1200U; boost_elapsed_ms++)
    {
        const uint32 next_ms = app_scheduler_get_ms() + 1U;

        if(0U == (next_ms % 100U))
        {
            control_chassis_set_cmd(250.0f, 0.0f, APP_TRUE, next_ms);
        }
        run_one_ms();
        leg = control_leg_get_diag();
        chassis_output = control_chassis_get_output();
        if((0.05f < leg->race_assist_actual) &&
           (APP_TRUE != leg->servo_settled) &&
           (APP_RACE_ASSIST_POSE_TOLERANCE_MM <
            fabsf(leg->race_target_x_mm - APP_RACE_ASSIST_ZERO_X_MM)))
        {
            saw_unsettled_off_neutral = APP_TRUE;
        }
        if(RACE_ASSIST_FAULT_NONE !=
           (race_assist_fault_reason_enum)chassis_output->race_assist_fault_reason)
        {
            fprintf(stderr,
                    "operational low-pose/settled self-fault at u %.3f settled %u x %.3f fault %u\n",
                    leg->race_assist_actual,
                    leg->servo_settled,
                    leg->race_target_x_mm,
                    chassis_output->race_assist_fault_reason);
            return 2;
        }
    }

    leg = control_leg_get_diag();
    chassis_output = control_chassis_get_output();
    if((LEG_MOTION_RACE_ASSIST != leg->motion_state) ||
       (0.90f > leg->race_assist_actual) ||
       (APP_TRUE != saw_unsettled_off_neutral) ||
       (RACE_ASSIST_FAULT_NONE != (race_assist_fault_reason_enum)chassis_output->race_assist_fault_reason))
    {
        fprintf(stderr,
                "neutral->armed->boost combination did not reach positive endpoint: motion %d u %.3f fault %u\n",
                (int)leg->motion_state,
                leg->race_assist_actual,
                chassis_output->race_assist_fault_reason);
        return 2;
    }

    control_chassis_set_cmd(0.0f, 0.0f, APP_TRUE, app_scheduler_get_ms());
    for(reversal_elapsed_ms = 0U; reversal_elapsed_ms < 1800U; reversal_elapsed_ms++)
    {
        const uint32 next_ms = app_scheduler_get_ms() + 1U;

        if(0U == (next_ms % 100U))
        {
            control_chassis_set_cmd(0.0f, 0.0f, APP_TRUE, next_ms);
        }
        run_one_ms();
        leg = control_leg_get_diag();
        if(0.001f >= fabsf(leg->race_assist_actual))
        {
            saw_zero_endpoint = APP_TRUE;
        }
        if((APP_TRUE == saw_zero_endpoint) && (-0.90f >= leg->race_assist_actual))
        {
            saw_negative_endpoint = APP_TRUE;
        }
    }
    leg = control_leg_get_diag();
    if((LEG_MOTION_RACE_ASSIST != leg->motion_state) ||
       (APP_TRUE != saw_zero_endpoint) ||
       (APP_TRUE != saw_negative_endpoint) ||
       (RACE_ASSIST_FAULT_NONE !=
        (race_assist_fault_reason_enum)control_chassis_get_output()->race_assist_fault_reason))
    {
        fprintf(stderr,
                "positive/zero/negative combination failed: u %.3f zero %u negative %u fault %u\n",
                leg->race_assist_actual,
                saw_zero_endpoint,
                saw_negative_endpoint,
                control_chassis_get_output()->race_assist_fault_reason);
        return 4;
    }

    /* Restore a nonzero forward ramp before injecting the runtime IK fault. */
    control_chassis_set_cmd(250.0f, 0.0f, APP_TRUE, app_scheduler_get_ms());
    run_ms_with_command(500U, 250.0f, 0.0f);

    while(9U != (app_scheduler_get_ms() % 10U))
    {
        run_one_ms();
    }
    servo = control_leg_get_servo_cmd();
    for(i = 0U; i < APP_SERVO_COUNT; i++)
    {
        held_servo_deg[i] = servo->angle_deg[i];
    }
    chassis_cmd = control_chassis_get_cmd();
    forward_before_fault = chassis_cmd->actual_forward_rpm;
    stop_count_before_fault = host_motor_stop_count;
    rpm_count_before_fault = host_motor_rpm_command_count;
    host_fail_next_ik_solve = APP_TRUE;
    run_one_ms();

    leg = control_leg_get_diag();
    chassis_cmd = control_chassis_get_cmd();
    forward_after_fault = chassis_cmd->actual_forward_rpm;
    if((LEG_MOTION_RACE_FAULT_HOLD != leg->motion_state) ||
       (APP_TRUE != leg->held_command_valid) ||
       (APP_TRUE == leg->ik_valid) ||
       expect_near(forward_after_fault,
                   forward_before_fault - (APP_CHASSIS_FORWARD_RAMP_RPM_S * 0.005f),
                   0.001f) ||
       (stop_count_before_fault != host_motor_stop_count) ||
       (rpm_count_before_fault >= host_motor_rpm_command_count))
    {
        fprintf(stderr,
                "runtime IK fault did not preserve held command/ramped balance: motion %d held %u ik %u rpm %.3f->%.3f stop %lu->%lu\n",
                (int)leg->motion_state,
                leg->held_command_valid,
                leg->ik_valid,
                forward_before_fault,
                forward_after_fault,
                (unsigned long)stop_count_before_fault,
                (unsigned long)host_motor_stop_count);
        return 5;
    }
    servo = control_leg_get_servo_cmd();
    for(i = 0U; i < APP_SERVO_COUNT; i++)
    {
        if(expect_near(servo->angle_deg[i], held_servo_deg[i], 0.0001f))
        {
            fprintf(stderr, "servo %u changed after runtime IK fault\n", i);
            return 6;
        }
    }

    run_ms_with_command(5U, 250.0f, 0.0f);
    chassis_cmd = control_chassis_get_cmd();
    if(expect_near(chassis_cmd->actual_forward_rpm,
                   forward_after_fault - (APP_CHASSIS_FORWARD_RAMP_RPM_S * 0.005f),
                   0.001f) ||
       (stop_count_before_fault != host_motor_stop_count))
    {
        fprintf(stderr, "race fault did not retain one 0.3 RPM step per 5 ms\n");
        return 7;
    }

    /* Inject an invalid current held-command snapshot directly at the balance
       boundary.  The scheduler fault test above proves the production leg
       publishes true only for the frozen last-valid command. */
    stop_count_before_safety_gate = host_motor_stop_count;
    ((leg_diag_struct *)(void *)control_leg_get_diag())->held_command_valid = APP_FALSE;
    control_balance_update(app_scheduler_get_ms());
    if(stop_count_before_safety_gate >= host_motor_stop_count)
    {
        fprintf(stderr, "held-command-invalid snapshot did not call motor stop\n");
        return 8;
    }
    ((leg_diag_struct *)(void *)control_leg_get_diag())->held_command_valid = APP_TRUE;

    stop_count_before_safety_gate = host_motor_stop_count;
    host_set_imu_healthy(APP_FALSE);
    control_balance_update(app_scheduler_get_ms());
    if(stop_count_before_safety_gate >= host_motor_stop_count)
    {
        fprintf(stderr, "IMU fault did not call motor stop\n");
        return 9;
    }
    host_set_imu_healthy(APP_TRUE);

    stop_count_before_safety_gate = host_motor_stop_count;
    app_state_set(APP_STATE_FAULT);
    run_ms_with_command(5U, 250.0f, 0.0f);
    if(stop_count_before_safety_gate >= host_motor_stop_count)
    {
        fprintf(stderr, "application emergency fault did not call motor stop\n");
        return 10;
    }

    puts("race assist scheduler integration checks passed");
    return 0;
}
'@ | Set-Content -LiteralPath (Join-Path $tempRoot "race_assist_scheduler_integration.c") -NoNewline

    $outputExe = Join-Path $tempRoot "race_assist_scheduler_integration.exe"
    $sources = @(
        "leg_config.c", "leg_kinematics.c", "control_leg.c", "control_race_assist.c",
        "control_chassis.c", "control_balance.c", "app_scheduler.c", "host_platform.c",
        "race_assist_scheduler_integration.c"
    ) | ForEach-Object { Join-Path $tempRoot $_ }
    $savedErrorAction = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    $compileOutput = & $gcc.Source -std=c99 -Wall -Wextra -Werror -I $tempRoot `
        $sources '-Wl,--wrap=leg_kinematics_solve' -lm -o $outputExe 2>&1
    $compileExit = $LASTEXITCODE
    $ErrorActionPreference = $savedErrorAction
    if($compileExit -ne 0) {
        throw "race assist scheduler integration compilation failed.`n$compileOutput"
    }

    & $outputExe
    if($LASTEXITCODE -ne 0) {
        throw "race assist scheduler integration checks failed."
    }
}
finally {
    if(Test-Path -LiteralPath $tempRoot) {
        Remove-Item -LiteralPath $tempRoot -Recurse -Force
    }
    $env:PATH = $savedPath
}

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$tempRoot = Join-Path ([System.IO.Path]::GetTempPath()) ("host-command-race-" + [guid]::NewGuid().ToString("N"))
$gcc = Get-Command gcc -ErrorAction Stop
$savedPath = $env:PATH

try {
    New-Item -ItemType Directory -Path $tempRoot -Force | Out-Null
    $env:PATH = (Split-Path $gcc.Source) + [System.IO.Path]::PathSeparator + $savedPath

    @(
        "project/code/host_command.c",
        "project/code/host_command.h",
        "project/code/actuator_motor.h",
        "project/code/control_chassis.h",
        "project/code/control_balance.h",
        "project/code/control_leg.h",
        "project/code/leg_config.h",
        "project/code/lsm6dsv16x_driver.h",
        "project/code/app_types.h",
        "project/code/app_config.h"
    ) | ForEach-Object {
        Copy-Item (Join-Path $repoRoot $_) $tempRoot
    }

    @'
#ifndef _zf_common_headfile_h_
#define _zf_common_headfile_h_
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
#ifndef _zf_common_debug_h_
#define _zf_common_debug_h_
#include "zf_common_headfile.h"
uint32 debug_read_ring_buffer(uint8 *buffer, uint32 length);
#endif
'@ | Set-Content -LiteralPath (Join-Path $tempRoot "zf_common_debug.h") -NoNewline

    @'
#ifndef _zf_common_interrupt_h_
#define _zf_common_interrupt_h_
#include "zf_common_headfile.h"
uint32 interrupt_global_disable(void);
void interrupt_global_enable(uint32 primask);
#endif
'@ | Set-Content -LiteralPath (Join-Path $tempRoot "zf_common_interrupt.h") -NoNewline

    @'
#include "actuator_motor.h"
#include "app_config.h"
#include "control_balance.h"
#include "control_chassis.h"
#include "control_leg.h"
#include "lsm6dsv16x_driver.h"
#include "zf_common_debug.h"

const char *host_rx_text = 0;
uint32 host_rx_index = 0U;
uint8 host_last_command_error = APP_FALSE;
uint32 host_command_error_count = 0U;
uint8 host_race_level = 0U;
float host_accel_gain = 0.0f;
float host_error_gain = 0.0f;
float host_hold_bias = 0.0f;
uint8 host_fast_enable = APP_FALSE;
uint8 host_motor_stopped = APP_FALSE;
uint8 host_chassis_stopped = APP_FALSE;
balance_mode_enum host_balance_mode = BALANCE_MODE_OFF;
uint8 host_balance_level_at_set = 0xffU;
uint8 host_stop_level_at_stop = 0xffU;

uint32 debug_read_ring_buffer(uint8 *buffer, uint32 length)
{
    uint32 count = 0U;

    while((0 != host_rx_text) && ('\0' != host_rx_text[host_rx_index]) && (count < length))
    {
        buffer[count] = (uint8)host_rx_text[host_rx_index];
        count++;
        host_rx_index++;
    }
    return count;
}

uint32 interrupt_global_disable(void) { return 0U; }
void interrupt_global_enable(uint32 primask) { (void)primask; }

void actuator_motor_set_mode_stop(void) { host_motor_stopped = APP_TRUE; }
void actuator_motor_set_mode_open_duty(float left_duty, float right_duty)
{
    (void)left_duty;
    (void)right_duty;
}
void actuator_motor_set_mode_motor_rpm(float left_motor_rpm, float right_motor_rpm)
{
    (void)left_motor_rpm;
    (void)right_motor_rpm;
}
void actuator_motor_record_host_motion(uint32 now_ms) { (void)now_ms; }
void actuator_motor_set_rpm_pid_gain(uint8 left_enable, uint8 right_enable,
                                     float kp, float ki, float kd)
{
    (void)left_enable;
    (void)right_enable;
    (void)kp;
    (void)ki;
    (void)kd;
}
void actuator_motor_record_command_error(uint8 is_error)
{
    host_last_command_error = is_error;
    if(APP_TRUE == is_error)
    {
        host_command_error_count++;
    }
}

void control_chassis_set_fast_enable(uint8 enable)
{
    host_fast_enable = (APP_TRUE == enable) ? APP_TRUE : APP_FALSE;
}
void control_chassis_stop(uint32 now_ms)
{
    (void)now_ms;
    host_chassis_stopped = APP_TRUE;
    host_stop_level_at_stop = host_race_level;
}
uint8 control_chassis_set_race_assist_level(uint8 level, uint32 now_ms)
{
    (void)now_ms;
    if(APP_RACE_ASSIST_MAX_VALIDATED_LEVEL < level)
    {
        return APP_FALSE;
    }
    host_race_level = level;
    return APP_TRUE;
}
uint8 control_chassis_set_race_assist_gains(float accel_gain,
                                            float error_gain,
                                            float hold_bias)
{
    if((accel_gain != accel_gain) ||
       (error_gain != error_gain) ||
       (hold_bias != hold_bias) ||
       (0.0f > accel_gain) || (APP_RACE_ASSIST_GAIN_A_MAX < accel_gain) ||
       (0.0f > error_gain) || (APP_RACE_ASSIST_GAIN_E_MAX < error_gain) ||
       (0.0f > hold_bias) || (APP_RACE_ASSIST_HOLD_MAX < hold_bias))
    {
        return APP_FALSE;
    }
    host_accel_gain = accel_gain;
    host_error_gain = error_gain;
    host_hold_bias = hold_bias;
    return APP_TRUE;
}
uint8 control_chassis_set_drive_gain(float speed_kp, float speed_ki, float turn_kp)
{
    (void)speed_kp;
    (void)speed_ki;
    (void)turn_kp;
    return APP_TRUE;
}
uint8 control_chassis_set_turn_gain(float turn_kp, float turn_ki)
{
    (void)turn_kp;
    (void)turn_ki;
    return APP_TRUE;
}
void control_chassis_set_cmd(float forward_rpm, float turn_rpm, uint8 enable, uint32 now_ms)
{
    (void)forward_rpm;
    (void)turn_rpm;
    (void)enable;
    (void)now_ms;
}

void control_balance_set_mode(balance_mode_enum mode)
{
    host_balance_level_at_set = host_race_level;
    host_balance_mode = mode;
}
uint8 control_balance_set_gain(float pitch_kp, float pitch_rate_kd)
{
    (void)pitch_kp;
    (void)pitch_rate_kd;
    return APP_TRUE;
}
uint8 control_balance_set_full_gain(float pitch_kp, float pitch_rate_kd,
                                    float wheel_speed_ks, float wheel_pos_kp)
{
    (void)pitch_kp;
    (void)pitch_rate_kd;
    (void)wheel_speed_ks;
    (void)wheel_pos_kp;
    return APP_TRUE;
}
void control_balance_reset_motion_state_public(void) {}
uint8 control_balance_set_ident_excitation(float amp_rpm, uint32 period_ms, uint32 now_ms)
{
    (void)amp_rpm;
    (void)period_ms;
    (void)now_ms;
    return APP_TRUE;
}
void control_balance_set_pitch_setpoint(float offset_deg) { (void)offset_deg; }

void control_leg_set_mode(leg_mode_enum mode) { (void)mode; }
uint8 control_leg_set_fast_legacy_stance(float stance_units, uint32 now_ms)
{
    (void)stance_units;
    (void)now_ms;
    return APP_TRUE;
}
uint8 control_leg_set_legacy_stance(float stance_units, uint32 now_ms)
{
    (void)stance_units;
    (void)now_ms;
    return APP_TRUE;
}
uint8 control_leg_set_ik_reference(uint32 now_ms) { (void)now_ms; return APP_TRUE; }
uint8 control_leg_set_xy(float x_mm, float y_mm, uint32 now_ms)
{
    (void)x_mm;
    (void)y_mm;
    (void)now_ms;
    return APP_TRUE;
}
uint8 control_leg_set_calib_angles(float servo0_deg, float servo1_deg,
                                   float servo2_deg, float servo3_deg)
{
    (void)servo0_deg;
    (void)servo1_deg;
    (void)servo2_deg;
    (void)servo3_deg;
    return APP_TRUE;
}
uint8 lsm6dsv16x_gyro_offset_init(void) { return 0U; }
'@ | Set-Content -LiteralPath (Join-Path $tempRoot "host_command_stubs.c") -NoNewline

    @'
#include <math.h>
#include <stdio.h>

#include "host_command.h"
#include "control_balance.h"

extern const char *host_rx_text;
extern uint32 host_rx_index;
extern uint8 host_last_command_error;
extern uint8 host_race_level;
extern float host_accel_gain;
extern float host_error_gain;
extern float host_hold_bias;
extern uint8 host_fast_enable;
extern uint8 host_motor_stopped;
extern uint8 host_chassis_stopped;
extern balance_mode_enum host_balance_mode;
extern uint8 host_balance_level_at_set;
extern uint8 host_stop_level_at_stop;

static int expect_near(float actual, float expected)
{
    return (fabsf(actual - expected) <= 0.0001f) ? 0 : 1;
}

static void send_command(const char *command, uint32 now_ms)
{
    host_rx_text = command;
    host_rx_index = 0U;
    host_command_update(now_ms);
}

static int check_bra_valid_and_rejected(void)
{
    host_command_init();
    send_command("BRA,1\n", 10U);
    if((1U != host_race_level) || (APP_FALSE != host_last_command_error))
    {
        fprintf(stderr, "BRA,1 was not accepted\n");
        return 1;
    }

    send_command("BRA,2\n", 11U);
    if((1U != host_race_level) || (APP_TRUE != host_last_command_error))
    {
        fprintf(stderr, "BRA,2 bypassed the compiled level gate\n");
        return 1;
    }

    send_command("BRA,1.5\n", 12U);
    if((1U != host_race_level) || (APP_TRUE != host_last_command_error))
    {
        fprintf(stderr, "fractional BRA partially changed the level\n");
        return 1;
    }

    send_command("BRA,-1\n", 13U);
    if((1U != host_race_level) || (APP_TRUE != host_last_command_error))
    {
        fprintf(stderr, "negative BRA partially changed the level\n");
        return 1;
    }
    return 0;
}

static int check_brg_is_bounded_and_atomic(void)
{
    send_command("BRG,0.02,0.05,0.50\n", 20U);
    if((APP_FALSE != host_last_command_error) ||
       expect_near(host_accel_gain, 0.02f) ||
       expect_near(host_error_gain, 0.05f) ||
       expect_near(host_hold_bias, 0.50f))
    {
        fprintf(stderr, "valid BRG was not applied\n");
        return 1;
    }

    send_command("BRG,0.01,0.02,0.51\n", 21U);
    if((APP_TRUE != host_last_command_error) ||
       expect_near(host_accel_gain, 0.02f) ||
       expect_near(host_error_gain, 0.05f) ||
       expect_near(host_hold_bias, 0.50f))
    {
        fprintf(stderr, "out-of-range BRG was not fail-closed and atomic\n");
        return 1;
    }
    return 0;
}

static int check_bra_requires_lexical_integer(void)
{
    const char *invalid_commands[] =
    {
        "BRA,0.99999999\n",
        "BRA,1.00000001\n",
        "BRA,+1\n",
        "BRA,-0\n",
        "BRA,1.0\n",
        "BRA,01\n",
        "BRA,NaN\n",
        "BRA,Inf\n",
        "BRA,1,0\n",
        "BRA,\n"
    };
    uint8 expected_level = host_race_level;
    float expected_accel_gain = host_accel_gain;
    float expected_error_gain = host_error_gain;
    float expected_hold_bias = host_hold_bias;
    uint32 index;

    send_command("BRA, 1\n", 22U);
    if((APP_FALSE != host_last_command_error) || (1U != host_race_level))
    {
        fprintf(stderr, "existing whitespace normalization changed for BRA\n");
        return 1;
    }

    for(index = 0U; index < (sizeof(invalid_commands) / sizeof(invalid_commands[0])); index++)
    {
        send_command(invalid_commands[index], 23U + index);
        if((APP_TRUE != host_last_command_error) ||
           (expected_level != host_race_level) ||
           expect_near(expected_accel_gain, host_accel_gain) ||
           expect_near(expected_error_gain, host_error_gain) ||
           expect_near(expected_hold_bias, host_hold_bias))
        {
            fprintf(stderr, "non-lexical BRA changed runtime state: %s", invalid_commands[index]);
            return 1;
        }
    }
    return 0;
}

static int check_brg_rejects_nonfinite_or_extra_fields(void)
{
    const char *invalid_commands[] =
    {
        "BRG,NaN,0,0\n",
        "BRG,Inf,0,0\n",
        "BRG,0.01,0.02,0.03,0.04\n"
    };
    float expected_accel_gain = host_accel_gain;
    float expected_error_gain = host_error_gain;
    float expected_hold_bias = host_hold_bias;
    uint32 index;

    for(index = 0U; index < (sizeof(invalid_commands) / sizeof(invalid_commands[0])); index++)
    {
        send_command(invalid_commands[index], 40U + index);
        if((APP_TRUE != host_last_command_error) ||
           expect_near(expected_accel_gain, host_accel_gain) ||
           expect_near(expected_error_gain, host_error_gain) ||
           expect_near(expected_hold_bias, host_hold_bias))
        {
            fprintf(stderr, "non-finite or extra-field BRG changed gains: %s", invalid_commands[index]);
            return 1;
        }
    }
    return 0;
}

static int check_partial_command_times_out_without_update(void)
{
    uint8 level_before_timeout;

    send_command("BRA,1\n", 24U);
    level_before_timeout = host_race_level;
    send_command("BRA,1", 25U);
    if((level_before_timeout != host_race_level) || (APP_TRUE == host_last_command_error))
    {
        fprintf(stderr, "partial BRA changed state before its line timeout\n");
        return 1;
    }

    host_command_update(126U);
    if((level_before_timeout != host_race_level) || (APP_TRUE != host_last_command_error))
    {
        fprintf(stderr, "partial BRA did not preserve the existing timeout contract\n");
        return 1;
    }
    return 0;
}

static int check_stop_and_balance_commands_disarm(void)
{
    send_command("BRA,0\n", 29U);
    send_command("B,3\n", 30U);
    if((0U != host_race_level) || (BALANCE_MODE_BALANCE_FAST != host_balance_mode))
    {
        fprintf(stderr, "B,3 silently enabled race assist\n");
        return 1;
    }

    send_command("BRA,1\n", 31U);
    send_command("B,1\n", 32U);
    if((0U != host_race_level) || (0U != host_balance_level_at_set) ||
       (BALANCE_MODE_STANDBY != host_balance_mode))
    {
        fprintf(stderr, "B,1 did not disarm before balance standby\n");
        return 1;
    }

    send_command("BRA,1\n", 33U);
    send_command("B,2\n", 34U);
    if((0U != host_race_level) || (0U != host_balance_level_at_set) ||
       (BALANCE_MODE_BALANCE_TEST != host_balance_mode))
    {
        fprintf(stderr, "B,2 did not disarm before balance test\n");
        return 1;
    }

    send_command("BRA,1\n", 35U);
    send_command("B,0\n", 36U);
    if((0U != host_race_level) || (0U != host_stop_level_at_stop) ||
       (APP_TRUE != host_chassis_stopped) || (APP_FALSE != host_fast_enable))
    {
        fprintf(stderr, "B,0 did not disarm before the chassis stop\n");
        return 1;
    }

    send_command("BRA,1\n", 37U);
    send_command("STOP\n", 38U);
    if((0U != host_race_level) || (0U != host_stop_level_at_stop) ||
       (APP_TRUE != host_motor_stopped) || (BALANCE_MODE_OFF != host_balance_mode))
    {
        fprintf(stderr, "STOP did not disarm before its existing stop actions\n");
        return 1;
    }
    return 0;
}

int main(void)
{
    if(check_bra_valid_and_rejected() ||
       check_brg_is_bounded_and_atomic() ||
       check_bra_requires_lexical_integer() ||
       check_brg_rejects_nonfinite_or_extra_fields() ||
       check_partial_command_times_out_without_update() ||
       check_stop_and_balance_commands_disarm())
    {
        return 1;
    }

    puts("host command race assist checks passed");
    return 0;
}
'@ | Set-Content -LiteralPath (Join-Path $tempRoot "host_command_race_assist.c") -NoNewline

    $outputExe = Join-Path $tempRoot "host_command_race_assist.exe"
    $compileErrorAction = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    $compileOutput = & $gcc.Source -std=c99 -Wall -Wextra -Werror -I $tempRoot `
        (Join-Path $tempRoot "host_command.c") `
        (Join-Path $tempRoot "host_command_stubs.c") `
        (Join-Path $tempRoot "host_command_race_assist.c") -lm -o $outputExe 2>&1
    $compileExit = $LASTEXITCODE
    $ErrorActionPreference = $compileErrorAction
    if($compileExit -ne 0) {
        throw "host command race assist harness compilation failed.`n$compileOutput"
    }

    & $outputExe
    if($LASTEXITCODE -ne 0) {
        throw "host command race assist checks failed."
    }
}
finally {
    if(Test-Path -LiteralPath $tempRoot) {
        Remove-Item -LiteralPath $tempRoot -Recurse -Force
    }
    $env:PATH = $savedPath
}

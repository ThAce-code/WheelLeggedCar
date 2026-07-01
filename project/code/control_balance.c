/*********************************************************************************************************************
* File: control_balance.c
* Description: Low-power balance mode and pitch feedback controller.
*********************************************************************************************************************/

#include "control_balance.h"
#include "app_config.h"
#include "app_state.h"
#include "sensor_imu.h"
#include "control_chassis.h"
#include "actuator_motor.h"

static balance_mode_enum control_balance_mode;
static balance_diag_struct control_balance_diag;
static uint32 control_balance_last_update_ms;
static float control_balance_pitch_kp;
static float control_balance_pitch_rate_kd;
static float control_balance_wheel_speed_ks;
static float control_balance_wheel_pos_kp;
static float control_balance_wheel_pos_rev;
static float control_balance_ident_amp_rpm;
static uint32 control_balance_ident_period_ms;
static uint32 control_balance_ident_start_ms;

static float control_balance_absf(float value)
{
    return (0.0f > value) ? -value : value;
}

static float control_balance_limit_abs(float value, float limit)
{
    float abs_limit;

    abs_limit = control_balance_absf(limit);
    if(abs_limit < value)
    {
        return abs_limit;
    }
    if((-abs_limit) > value)
    {
        return -abs_limit;
    }
    return value;
}

static uint8 control_balance_is_finite(float value)
{
    if(value != value)
    {
        return APP_FALSE;
    }
    if(APP_BALANCE_FINITE_ABS_LIMIT < value)
    {
        return APP_FALSE;
    }
    if((-APP_BALANCE_FINITE_ABS_LIMIT) > value)
    {
        return APP_FALSE;
    }
    return APP_TRUE;
}

static void control_balance_reset_motion_state(void)
{
    control_balance_wheel_pos_rev = 0.0f;
    control_balance_last_update_ms = 0;
    control_balance_diag.wheel_speed_rpm = 0.0f;
    control_balance_diag.wheel_pos_rev = 0.0f;
}

static float control_balance_get_ident_rpm(uint32 now_ms)
{
    uint32 elapsed_ms;
    uint32 phase;

    if((0.0f == control_balance_ident_amp_rpm) ||
       (0U == control_balance_ident_period_ms))
    {
        return 0.0f;
    }

    elapsed_ms = now_ms - control_balance_ident_start_ms;
    phase = (elapsed_ms / control_balance_ident_period_ms) & 0x01U;
    return (0U == phase) ? control_balance_ident_amp_rpm : -control_balance_ident_amp_rpm;
}

static void control_balance_stop_output(void)
{
    control_balance_diag.balance_rpm = 0.0f;
    control_balance_diag.output_left_rpm = 0.0f;
    control_balance_diag.output_right_rpm = 0.0f;
    control_balance_diag.output_enable = APP_FALSE;
    actuator_motor_set_mode_stop();
}

void control_balance_init(void)
{
    control_balance_mode = BALANCE_MODE_STANDBY;
    control_balance_diag.mode = BALANCE_MODE_STANDBY;
    control_balance_diag.pitch_deg = 0.0f;
    control_balance_diag.pitch_rate_dps = 0.0f;
    control_balance_diag.chassis_left_rpm = 0.0f;
    control_balance_diag.chassis_right_rpm = 0.0f;
    control_balance_diag.balance_rpm = 0.0f;
    control_balance_diag.output_left_rpm = 0.0f;
    control_balance_diag.output_right_rpm = 0.0f;
    control_balance_pitch_kp = APP_BALANCE_PITCH_KP;
    control_balance_pitch_rate_kd = APP_BALANCE_PITCH_RATE_KD;
    control_balance_diag.pitch_kp = control_balance_pitch_kp;
    control_balance_diag.pitch_rate_kd = control_balance_pitch_rate_kd;
    control_balance_wheel_speed_ks = APP_BALANCE_WHEEL_SPEED_KS;
    control_balance_wheel_pos_kp = APP_BALANCE_WHEEL_POS_KP;
    control_balance_wheel_pos_rev = 0.0f;
    control_balance_diag.wheel_speed_rpm = 0.0f;
    control_balance_diag.wheel_pos_rev = 0.0f;
    control_balance_diag.wheel_speed_ks = control_balance_wheel_speed_ks;
    control_balance_diag.wheel_pos_kp = control_balance_wheel_pos_kp;
    control_balance_diag.output_enable = APP_FALSE;
    control_balance_diag.safety_blocked = APP_TRUE;
    control_balance_ident_amp_rpm = 0.0f;
    control_balance_ident_period_ms = 0U;
    control_balance_ident_start_ms = 0U;
    control_balance_last_update_ms = 0;
}

void control_balance_update(uint32 now_ms)
{
    const imu_state_struct *imu;
    const wheel_feedback_struct *wheel;
    const chassis_output_struct *chassis;
    float dt_s;
    float pitch_rate_dps;
    float balance_rpm;
    float ident_rpm;
    float output_left_rpm;
    float output_right_rpm;
    const motor_rpm_loop_diag_struct *rpm_diag;
    float wheel_speed_rpm;
    float wheel_pos_delta_rev;
    uint32 imu_age_ms;

    imu = sensor_imu_get_state();
    wheel = actuator_motor_get_feedback();
    chassis = control_chassis_get_output();
    rpm_diag = actuator_motor_get_motor_rpm_loop_diag();

    control_balance_diag.mode = control_balance_mode;
    control_balance_diag.pitch_deg = imu->pitch;
    control_balance_diag.chassis_left_rpm = chassis->left_base_rpm;
    control_balance_diag.chassis_right_rpm = chassis->right_base_rpm;

    pitch_rate_dps = imu->pitch_rate_dps;
    control_balance_diag.pitch_rate_dps = pitch_rate_dps;

    /* Wrap-safe dt_s for wheel position integration */
    if(0U == control_balance_last_update_ms)
    {
        dt_s = (float)APP_BALANCE_PERIOD_MS / 1000.0f;
    }
    else
    {
        dt_s = (float)(now_ms - control_balance_last_update_ms) / 1000.0f;
    }
    control_balance_last_update_ms = now_ms;

    imu_age_ms = now_ms - imu->timestamp_ms;

    wheel_speed_rpm = 0.5f * (rpm_diag->left_motor_rpm + rpm_diag->right_motor_rpm);
    control_balance_diag.wheel_speed_rpm = wheel_speed_rpm;

    /* OFF or STANDBY: update telemetry only, do not touch motor output.
       M / D direct actuator debug commands must not be overridden. */
    if(BALANCE_MODE_BALANCE_TEST != control_balance_mode)
    {
        control_balance_diag.safety_blocked = APP_TRUE;
        control_balance_diag.balance_rpm = 0.0f;
        control_balance_diag.output_left_rpm = 0.0f;
        control_balance_diag.output_right_rpm = 0.0f;
        control_balance_diag.output_enable = APP_FALSE;
        return;
    }

    /* BALANCE_TEST only from here: safety gates + motor output */
    control_balance_diag.safety_blocked = APP_FALSE;
    if((APP_STATE_FAULT == app_state_get()) ||
       (APP_FALSE == imu->healthy) ||
       (APP_FALSE == wheel->online) ||
       (APP_FALSE == wheel->left_online) ||
       (APP_FALSE == wheel->right_online) ||
       (APP_FALSE == chassis->enable) ||
       (APP_BALANCE_IMU_MAX_AGE_MS < imu_age_ms) ||
       (APP_FALSE == control_balance_is_finite(imu->pitch)) ||
       (APP_FALSE == control_balance_is_finite(pitch_rate_dps)) ||
       (APP_FALSE == control_balance_is_finite(wheel_speed_rpm)) ||
       (APP_BALANCE_TEST_PITCH_LIMIT_DEG < control_balance_absf(imu->pitch)))
    {
        control_balance_diag.safety_blocked = APP_TRUE;
        control_balance_reset_motion_state();
        control_balance_stop_output();
        return;
    }

    wheel_pos_delta_rev = wheel_speed_rpm * dt_s / 60.0f;
    control_balance_wheel_pos_rev += wheel_pos_delta_rev;
    control_balance_wheel_pos_rev *= APP_BALANCE_WHEEL_POS_DECAY;
    control_balance_wheel_pos_rev = control_balance_limit_abs(control_balance_wheel_pos_rev,
                                                              APP_BALANCE_WHEEL_POS_LIMIT_REV);
    control_balance_diag.wheel_pos_rev = control_balance_wheel_pos_rev;

    balance_rpm = (control_balance_pitch_kp * imu->pitch) +
                  (control_balance_pitch_rate_kd * pitch_rate_dps) +
                  (control_balance_wheel_speed_ks * wheel_speed_rpm) +
                  (control_balance_wheel_pos_kp * control_balance_wheel_pos_rev);
    ident_rpm = control_balance_get_ident_rpm(now_ms);
    balance_rpm += ident_rpm;
    balance_rpm = control_balance_limit_abs(balance_rpm, APP_BALANCE_RPM_LIMIT);

    output_left_rpm = chassis->left_base_rpm + balance_rpm;
    output_right_rpm = chassis->right_base_rpm + balance_rpm;

    if((APP_FALSE == control_balance_is_finite(control_balance_wheel_pos_rev)) ||
       (APP_FALSE == control_balance_is_finite(balance_rpm)) ||
       (APP_FALSE == control_balance_is_finite(output_left_rpm)) ||
       (APP_FALSE == control_balance_is_finite(output_right_rpm)))
    {
        control_balance_diag.safety_blocked = APP_TRUE;
        control_balance_reset_motion_state();
        control_balance_stop_output();
        return;
    }

    control_balance_diag.balance_rpm = balance_rpm;
    control_balance_diag.output_left_rpm = output_left_rpm;
    control_balance_diag.output_right_rpm = output_right_rpm;
    control_balance_diag.output_enable = APP_TRUE;

    actuator_motor_set_mode_motor_rpm(output_left_rpm, output_right_rpm);
}

void control_balance_set_mode(balance_mode_enum mode)
{
    if(mode > BALANCE_MODE_BALANCE_TEST)
    {
        mode = BALANCE_MODE_OFF;
    }

    if(mode != control_balance_mode)
    {
        control_balance_reset_motion_state();
    }

    control_balance_mode = mode;
    control_balance_diag.mode = mode;

    if(BALANCE_MODE_BALANCE_TEST != mode)
    {
        control_balance_stop_output();
    }
}

uint8 control_balance_set_gain(float pitch_kp, float pitch_rate_kd)
{
    return control_balance_set_full_gain(pitch_kp, pitch_rate_kd, 0.0f, 0.0f);
}

uint8 control_balance_set_full_gain(float pitch_kp, float pitch_rate_kd, float wheel_speed_ks, float wheel_pos_kp)
{
    if((APP_FALSE == control_balance_is_finite(pitch_kp)) ||
       (APP_FALSE == control_balance_is_finite(pitch_rate_kd)) ||
       (APP_FALSE == control_balance_is_finite(wheel_speed_ks)) ||
       (APP_FALSE == control_balance_is_finite(wheel_pos_kp)) ||
       (APP_BALANCE_GAIN_ABS_LIMIT < control_balance_absf(pitch_kp)) ||
       (APP_BALANCE_GAIN_ABS_LIMIT < control_balance_absf(pitch_rate_kd)) ||
       (APP_BALANCE_GAIN_ABS_LIMIT < control_balance_absf(wheel_speed_ks)) ||
       (APP_BALANCE_GAIN_ABS_LIMIT < control_balance_absf(wheel_pos_kp)))
    {
        return APP_FALSE;
    }

    control_balance_pitch_kp = pitch_kp;
    control_balance_pitch_rate_kd = pitch_rate_kd;
    control_balance_wheel_speed_ks = wheel_speed_ks;
    control_balance_wheel_pos_kp = wheel_pos_kp;
    control_balance_diag.pitch_kp = control_balance_pitch_kp;
    control_balance_diag.pitch_rate_kd = control_balance_pitch_rate_kd;
    control_balance_diag.wheel_speed_ks = control_balance_wheel_speed_ks;
    control_balance_diag.wheel_pos_kp = control_balance_wheel_pos_kp;
    control_balance_reset_motion_state();
    return APP_TRUE;
}

void control_balance_reset_motion_state_public(void)
{
    control_balance_reset_motion_state();
}

uint8 control_balance_set_ident_excitation(float amp_rpm, uint32 period_ms, uint32 now_ms)
{
    if((APP_FALSE == control_balance_is_finite(amp_rpm)) ||
       (APP_BALANCE_IDENT_RPM_LIMIT < control_balance_absf(amp_rpm)))
    {
        return APP_FALSE;
    }

    if(0.0f == amp_rpm)
    {
        control_balance_ident_amp_rpm = 0.0f;
        control_balance_ident_period_ms = 0U;
        control_balance_ident_start_ms = now_ms;
        return APP_TRUE;
    }

    if((APP_BALANCE_IDENT_MIN_PERIOD_MS > period_ms) ||
       (APP_BALANCE_IDENT_MAX_PERIOD_MS < period_ms))
    {
        return APP_FALSE;
    }

    control_balance_ident_amp_rpm = amp_rpm;
    control_balance_ident_period_ms = period_ms;
    control_balance_ident_start_ms = now_ms;
    return APP_TRUE;
}

balance_mode_enum control_balance_get_mode(void)
{
    return control_balance_mode;
}

const balance_diag_struct *control_balance_get_diag(void)
{
    return &control_balance_diag;
}

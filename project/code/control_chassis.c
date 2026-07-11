/*********************************************************************************************************************
* File: control_chassis.c
* Description: Chassis forward/turn command shaper and mixer.
********************************************************************************************************************/

#include "control_chassis.h"
#include "app_config.h"
#include "control_leg.h"
#include "actuator_motor.h"
#include "sensor_imu.h"

static chassis_cmd_struct control_chassis_cmd;
static chassis_output_struct control_chassis_output;
static float control_chassis_gyro_z_filtered_dps;
static uint8 control_chassis_gyro_z_filter_valid;

static void control_chassis_reset_turn_filter(void)
{
    control_chassis_gyro_z_filtered_dps = 0.0f;
    control_chassis_gyro_z_filter_valid = APP_FALSE;
}

static float control_chassis_absf(float value)
{
    return (0.0f > value) ? -value : value;
}

static float control_chassis_limit_abs(float value, float limit)
{
    float abs_limit;

    abs_limit = control_chassis_absf(limit);
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

static uint8 control_chassis_is_finite(float value)
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

static float control_chassis_clamp01(float value)
{
    if(0.0f > value)
    {
        return 0.0f;
    }
    if(1.0f < value)
    {
        return 1.0f;
    }
    return value;
}

static float control_chassis_lerp(float low, float high, float blend)
{
    blend = control_chassis_clamp01(blend);
    return low + ((high - low) * blend);
}

static float control_chassis_smoothstep(float edge0, float edge1, float value)
{
    float t;

    if(edge1 <= edge0)
    {
        return 0.0f;
    }

    t = control_chassis_clamp01((value - edge0) / (edge1 - edge0));
    return t * t * (3.0f - (2.0f * t));
}

static float control_chassis_ramp_toward(float current, float target, float max_delta)
{
    float delta;

    delta = target - current;
    if(max_delta >= control_chassis_absf(delta))
    {
        return target;
    }
    if(0.0f < delta)
    {
        return current + max_delta;
    }
    return current - max_delta;
}

static void control_chassis_resolve_leg_motion_policy(const leg_diag_struct *leg,
                                                      const leg_height_profile_struct *height_profile,
                                                      float configured_forward_limit_rpm,
                                                      float configured_fast_forward_limit_rpm,
                                                      uint8 fast_requested,
                                                      float *forward_limit_rpm,
                                                      float *fast_forward_limit_rpm,
                                                      uint8 *effective_fast_enable)
{
    if((NULL == leg) ||
       (NULL == height_profile) ||
       (NULL == forward_limit_rpm) ||
       (NULL == fast_forward_limit_rpm) ||
       (NULL == effective_fast_enable))
    {
        return;
    }

    if((LEG_MOTION_FAULT == leg->motion_state) || (APP_FALSE == leg->drive_allowed))
    {
        *forward_limit_rpm = 0.0f;
        *fast_forward_limit_rpm = 0.0f;
        *effective_fast_enable = APP_FALSE;
    }
    else if(LEG_MOTION_TRANSITION == leg->motion_state)
    {
        *forward_limit_rpm = height_profile->transition_forward_limit_rpm;
        *fast_forward_limit_rpm = height_profile->transition_forward_limit_rpm;
        *effective_fast_enable = APP_FALSE;
    }
    else
    {
        *forward_limit_rpm = configured_forward_limit_rpm;
        *fast_forward_limit_rpm = configured_fast_forward_limit_rpm;
        *effective_fast_enable = (APP_TRUE == fast_requested) ? APP_TRUE : APP_FALSE;
    }
}

static void control_chassis_clear_output(void)
{
    control_chassis_output.pitch_offset_deg = 0.0f;
    control_chassis_output.turn_rpm = 0.0f;
    control_chassis_output.forward_target_rpm = 0.0f;
    control_chassis_output.forward_actual_rpm = 0.0f;
    control_chassis_output.turn_target_dps = 0.0f;
    control_chassis_output.gyro_z_dps = 0.0f;
    control_chassis_output.gyro_z_raw_dps = 0.0f;
    control_chassis_output.gyro_z_filtered_dps = 0.0f;
    control_chassis_output.turn_error_dps = 0.0f;
    control_chassis_output.turn_integral = 0.0f;
    control_chassis_output.turn_kp = control_chassis_cmd.turn_kp;
    control_chassis_output.turn_ki = control_chassis_cmd.turn_ki;
    control_chassis_output.fast_blend = 0.0f;
    control_chassis_output.speed_integral = 0.0f;
    control_chassis_output.speed_pitch_limit_deg = APP_CHASSIS_SPEED_PITCH_LIMIT_DEG;
    control_chassis_output.speed_ff_rpm = 0.0f;
    control_chassis_output.forward_limit_eff_rpm = APP_CHASSIS_FORWARD_RPM_LIMIT;
    control_chassis_output.fast_forward_limit_eff_rpm = APP_CHASSIS_FAST_FORWARD_RPM_LIMIT;
    control_chassis_output.imu_age_ms = 0U;
    control_chassis_output.wheel_age_ms = 0U;
    control_chassis_output.enable = APP_FALSE;
}

void control_chassis_init(void)
{
    control_chassis_cmd.target_forward_rpm = 0.0f;
    control_chassis_cmd.target_turn_dps = 0.0f;
    control_chassis_cmd.actual_forward_rpm = 0.0f;
    control_chassis_cmd.actual_turn_dps = 0.0f;
    control_chassis_cmd.speed_pitch_offset_deg = 0.0f;
    control_chassis_cmd.turn_rpm = 0.0f;
    control_chassis_cmd.speed_kp = APP_CHASSIS_SPEED_KP;
    control_chassis_cmd.speed_ki = APP_CHASSIS_SPEED_KI;
    control_chassis_cmd.turn_kp = APP_CHASSIS_TURN_KP;
    control_chassis_cmd.turn_ki = APP_CHASSIS_TURN_KI;
    control_chassis_cmd.speed_integral = 0.0f;
    control_chassis_cmd.turn_integral = 0.0f;
    control_chassis_cmd.fast_blend = 0.0f;
    control_chassis_cmd.speed_pitch_limit_deg = APP_CHASSIS_SPEED_PITCH_LIMIT_DEG;
    control_chassis_cmd.speed_ff_rpm = 0.0f;
    control_chassis_cmd.fast_enable = APP_FALSE;
    control_chassis_cmd.enable = APP_FALSE;
    control_chassis_cmd.last_cmd_ms = 0;
    control_chassis_cmd.last_update_ms = 0;

    control_chassis_reset_turn_filter();
    control_chassis_clear_output();
}

void control_chassis_update(uint32 now_ms)
{
    const motor_rpm_loop_diag_struct *rpm_diag;
    const imu_state_struct *imu;
    float target_forward_rpm;
    float target_turn_dps;
    float forward_max_delta;
    float turn_max_delta;
    float speed_error_rpm;
    float speed_pitch_offset_deg;
    float avg_wheel_speed_rpm;
    float turn_error_dps;
    float turn_rpm;
    float gyro_z_raw_dps;
    float gyro_z_delta_dps;
    float gyro_z_filtered_dps;
    float turn_unsat_rpm;
    uint8 turn_saturated;
    float forward_limit_rpm;
    float height_forward_limit_rpm;
    float height_fast_forward_limit_rpm;
    float raw_fast_blend;
    float speed_pitch_limit_deg;
    uint8 effective_fast_enable = APP_FALSE;
    uint32 imu_age_ms;
    uint32 wheel_age_ms;
    const wheel_feedback_struct *wheel_feedback;
    float dt_s;

    if((APP_TRUE == control_chassis_cmd.enable) &&
       (0U != control_chassis_cmd.last_cmd_ms) &&
       (APP_CHASSIS_CMD_TIMEOUT_MS < (now_ms - control_chassis_cmd.last_cmd_ms)))
    {
        control_chassis_stop(now_ms);
    }

    if(0U == control_chassis_cmd.last_update_ms)
    {
        dt_s = (float)APP_CHASSIS_PERIOD_MS / 1000.0f;
    }
    else
    {
        dt_s = (float)(now_ms - control_chassis_cmd.last_update_ms) / 1000.0f;
    }
    control_chassis_cmd.last_update_ms = now_ms;

    if((0.0f >= dt_s) ||
       (1.0f < dt_s) ||
       (APP_FALSE == control_chassis_is_finite(dt_s)))
    {
        dt_s = (float)APP_CHASSIS_PERIOD_MS / 1000.0f;
    }

    rpm_diag = actuator_motor_get_motor_rpm_loop_diag();
    imu = sensor_imu_get_state();
    wheel_feedback = actuator_motor_get_feedback();
    imu_age_ms = now_ms - imu->timestamp_ms;
    wheel_age_ms = wheel_feedback->age_ms;

    if(APP_FALSE == control_chassis_cmd.enable)
    {
        control_chassis_cmd.target_forward_rpm = 0.0f;
        control_chassis_cmd.target_turn_dps = 0.0f;
    }

    {
        const leg_diag_struct *leg;
        const leg_height_profile_struct *height_profile;
        float height_norm;

        leg = control_leg_get_diag();
        height_profile = leg_config_get_height_profile();
        height_norm = control_chassis_limit_abs(leg->height_norm, 1.0f);
        height_forward_limit_rpm =
            control_chassis_lerp(height_profile->chassis_forward_limit_low_rpm,
                                 height_profile->chassis_forward_limit_high_rpm,
                                 height_norm);
        height_fast_forward_limit_rpm =
            control_chassis_lerp(height_profile->chassis_fast_forward_limit_low_rpm,
                                 height_profile->chassis_fast_forward_limit_high_rpm,
                                 height_norm);

        control_chassis_resolve_leg_motion_policy(leg,
                                                  height_profile,
                                                  height_forward_limit_rpm,
                                                  height_fast_forward_limit_rpm,
                                                  control_chassis_cmd.fast_enable,
                                                  &height_forward_limit_rpm,
                                                  &height_fast_forward_limit_rpm,
                                                  &effective_fast_enable);

        forward_limit_rpm = (APP_TRUE == effective_fast_enable) ?
                            height_fast_forward_limit_rpm :
                            height_forward_limit_rpm;
    }

    target_forward_rpm = control_chassis_limit_abs(control_chassis_cmd.target_forward_rpm,
                                                   forward_limit_rpm);
    target_turn_dps = control_chassis_limit_abs(control_chassis_cmd.target_turn_dps,
                                                APP_CHASSIS_TURN_RATE_LIMIT_DPS);

    forward_max_delta = APP_CHASSIS_FORWARD_RAMP_RPM_S * dt_s;
    turn_max_delta = APP_CHASSIS_TURN_RATE_RAMP_DPS_S * dt_s;

    control_chassis_cmd.actual_forward_rpm =
        control_chassis_ramp_toward(control_chassis_cmd.actual_forward_rpm,
                                    target_forward_rpm,
                                    forward_max_delta);
    control_chassis_cmd.actual_turn_dps =
        control_chassis_ramp_toward(control_chassis_cmd.actual_turn_dps,
                                    target_turn_dps,
                                    turn_max_delta);

    raw_fast_blend = control_chassis_smoothstep(APP_CHASSIS_FAST_BLEND_START_RPM,
                                                 APP_CHASSIS_FAST_BLEND_FULL_RPM,
                                                 control_chassis_absf(control_chassis_cmd.actual_forward_rpm));
    if(APP_FALSE == effective_fast_enable)
    {
        raw_fast_blend = 0.0f;
    }

    control_chassis_cmd.fast_blend =
        control_chassis_ramp_toward(control_chassis_cmd.fast_blend,
                                    raw_fast_blend,
                                    APP_CHASSIS_FAST_BLEND_RAMP_S * dt_s);

    speed_pitch_limit_deg =
        control_chassis_lerp(APP_CHASSIS_SPEED_PITCH_LIMIT_DEG,
                             APP_CHASSIS_FAST_SPEED_PITCH_LIMIT_DEG,
                             control_chassis_cmd.fast_blend);
    control_chassis_cmd.speed_pitch_limit_deg = speed_pitch_limit_deg;

    control_chassis_cmd.speed_ff_rpm =
        APP_BALANCE_FAST_SPEED_FF_GAIN *
        control_chassis_cmd.actual_forward_rpm *
        control_chassis_cmd.fast_blend;

    if((APP_FALSE == imu->healthy) ||
       (APP_FALSE == wheel_feedback->online) ||
       (APP_FALSE == wheel_feedback->left_online) ||
       (APP_FALSE == wheel_feedback->right_online) ||
       (APP_CHASSIS_IMU_MAX_AGE_MS < imu_age_ms) ||
       (APP_CHASSIS_WHEEL_MAX_AGE_MS < wheel_age_ms))
    {
        control_chassis_cmd.speed_integral = 0.0f;
        control_chassis_cmd.turn_integral = 0.0f;
        control_chassis_cmd.fast_blend = 0.0f;
        control_chassis_cmd.speed_ff_rpm = 0.0f;
        control_chassis_cmd.speed_pitch_limit_deg = APP_CHASSIS_SPEED_PITCH_LIMIT_DEG;
        control_chassis_reset_turn_filter();
        control_chassis_clear_output();
        /* Preserve age diagnostics so telemetry can record the root cause
           of this output block rather than reporting 0 ms. */
        control_chassis_output.imu_age_ms = imu_age_ms;
        control_chassis_output.wheel_age_ms = wheel_age_ms;
        return;
    }

    avg_wheel_speed_rpm = 0.5f * (rpm_diag->left_motor_rpm + rpm_diag->right_motor_rpm);
    speed_error_rpm = control_chassis_cmd.actual_forward_rpm - avg_wheel_speed_rpm;
    control_chassis_cmd.speed_integral += speed_error_rpm * dt_s;
    control_chassis_cmd.speed_integral =
        control_chassis_limit_abs(control_chassis_cmd.speed_integral,
                                  APP_CHASSIS_SPEED_INTEGRAL_LIMIT);

    speed_pitch_offset_deg =
        (control_chassis_cmd.speed_kp * speed_error_rpm) +
        (control_chassis_cmd.speed_ki * control_chassis_cmd.speed_integral);
    speed_pitch_offset_deg =
        control_chassis_limit_abs(speed_pitch_offset_deg,
                                  speed_pitch_limit_deg);

    gyro_z_raw_dps = imu->gyro_z_dps;
    if(APP_FALSE == control_chassis_gyro_z_filter_valid)
    {
        control_chassis_gyro_z_filtered_dps = gyro_z_raw_dps;
        control_chassis_gyro_z_filter_valid = APP_TRUE;
    }

    gyro_z_delta_dps = gyro_z_raw_dps - control_chassis_gyro_z_filtered_dps;
    gyro_z_delta_dps = control_chassis_limit_abs(gyro_z_delta_dps,
                                                 APP_CHASSIS_TURN_GYRO_STEP_LIMIT_DPS);
    control_chassis_gyro_z_filtered_dps += APP_CHASSIS_TURN_GYRO_LPF_ALPHA * gyro_z_delta_dps;
    gyro_z_filtered_dps = control_chassis_gyro_z_filtered_dps;

    turn_error_dps = control_chassis_cmd.actual_turn_dps - gyro_z_filtered_dps;
    if(APP_CHASSIS_TURN_GYRO_DEADBAND_DPS > control_chassis_absf(turn_error_dps))
    {
        turn_error_dps = 0.0f;
    }

    turn_unsat_rpm = (control_chassis_cmd.turn_kp * turn_error_dps) +
                    (control_chassis_cmd.turn_ki * control_chassis_cmd.turn_integral);
    turn_rpm = control_chassis_limit_abs(turn_unsat_rpm, APP_CHASSIS_TURN_RPM_LIMIT);
    turn_saturated = (turn_rpm != turn_unsat_rpm) ? APP_TRUE : APP_FALSE;

    if((APP_CHASSIS_TURN_ZERO_TARGET_DPS > control_chassis_absf(control_chassis_cmd.actual_turn_dps)) &&
       (0.0f == turn_error_dps))
    {
        control_chassis_cmd.turn_integral *= APP_CHASSIS_TURN_INTEGRAL_DECAY;
    }
    else if((APP_FALSE == turn_saturated) ||
            ((0.0f < turn_rpm) && (0.0f > turn_error_dps)) ||
            ((0.0f > turn_rpm) && (0.0f < turn_error_dps)))
    {
        control_chassis_cmd.turn_integral += turn_error_dps * dt_s;
        control_chassis_cmd.turn_integral =
            control_chassis_limit_abs(control_chassis_cmd.turn_integral,
                                      APP_CHASSIS_TURN_INTEGRAL_LIMIT);
    }

    turn_unsat_rpm = (control_chassis_cmd.turn_kp * turn_error_dps) +
                    (control_chassis_cmd.turn_ki * control_chassis_cmd.turn_integral);
    turn_rpm = control_chassis_limit_abs(turn_unsat_rpm, APP_CHASSIS_TURN_RPM_LIMIT);

    if((APP_FALSE == control_chassis_is_finite(avg_wheel_speed_rpm)) ||
       (APP_FALSE == control_chassis_is_finite(gyro_z_raw_dps)) ||
       (APP_FALSE == control_chassis_is_finite(gyro_z_filtered_dps)) ||
       (APP_FALSE == control_chassis_is_finite(speed_pitch_offset_deg)) ||
       (APP_FALSE == control_chassis_is_finite(turn_rpm)))
    {
        control_chassis_stop(now_ms);
        return;
    }

    control_chassis_cmd.speed_pitch_offset_deg = speed_pitch_offset_deg;
    control_chassis_cmd.turn_rpm = turn_rpm;

    control_chassis_output.pitch_offset_deg = speed_pitch_offset_deg;
    control_chassis_output.turn_rpm = turn_rpm;
    control_chassis_output.forward_target_rpm = control_chassis_cmd.actual_forward_rpm;
    control_chassis_output.forward_actual_rpm = avg_wheel_speed_rpm;
    control_chassis_output.turn_target_dps = control_chassis_cmd.actual_turn_dps;
    control_chassis_output.gyro_z_dps = gyro_z_filtered_dps;
    control_chassis_output.gyro_z_raw_dps = gyro_z_raw_dps;
    control_chassis_output.gyro_z_filtered_dps = gyro_z_filtered_dps;
    control_chassis_output.turn_error_dps = turn_error_dps;
    control_chassis_output.turn_integral = control_chassis_cmd.turn_integral;
    control_chassis_output.turn_kp = control_chassis_cmd.turn_kp;
    control_chassis_output.turn_ki = control_chassis_cmd.turn_ki;
    control_chassis_output.fast_blend = control_chassis_cmd.fast_blend;
    control_chassis_output.speed_integral = control_chassis_cmd.speed_integral;
    control_chassis_output.speed_pitch_limit_deg = control_chassis_cmd.speed_pitch_limit_deg;
    control_chassis_output.speed_ff_rpm = control_chassis_cmd.speed_ff_rpm;
    control_chassis_output.forward_limit_eff_rpm = height_forward_limit_rpm;
    control_chassis_output.fast_forward_limit_eff_rpm = height_fast_forward_limit_rpm;
    control_chassis_output.imu_age_ms = imu_age_ms;
    control_chassis_output.wheel_age_ms = wheel_age_ms;
    control_chassis_output.enable = APP_TRUE;

    if((APP_FALSE == control_chassis_cmd.enable) &&
       (0.0f == control_chassis_cmd.actual_forward_rpm) &&
       (0.0f == control_chassis_cmd.actual_turn_dps) &&
       (0.0f == control_chassis_cmd.target_forward_rpm) &&
       (0.0f == control_chassis_cmd.target_turn_dps))
    {
        control_chassis_cmd.speed_pitch_offset_deg = 0.0f;
        control_chassis_cmd.turn_rpm = 0.0f;
        control_chassis_cmd.speed_integral = 0.0f;
        control_chassis_cmd.turn_integral = 0.0f;
        control_chassis_clear_output();
    }
}

void control_chassis_set_cmd(float forward_rpm, float turn_rpm, uint8 enable, uint32 now_ms)
{
    float forward_limit_rpm;

    if((APP_FALSE == control_chassis_is_finite(forward_rpm)) ||
       (APP_FALSE == control_chassis_is_finite(turn_rpm)))
    {
        control_chassis_stop(now_ms);
        return;
    }

    {
        const leg_diag_struct *leg;
        const leg_height_profile_struct *height_profile;
        float height_forward_limit_rpm;
        float height_fast_forward_limit_rpm;
        float height_norm;
        uint8 effective_fast_enable = APP_FALSE;

        leg = control_leg_get_diag();
        height_profile = leg_config_get_height_profile();
        height_norm = control_chassis_limit_abs(leg->height_norm, 1.0f);
        height_forward_limit_rpm =
            control_chassis_lerp(height_profile->chassis_forward_limit_low_rpm,
                                 height_profile->chassis_forward_limit_high_rpm,
                                 height_norm);
        height_fast_forward_limit_rpm =
            control_chassis_lerp(height_profile->chassis_fast_forward_limit_low_rpm,
                                 height_profile->chassis_fast_forward_limit_high_rpm,
                                 height_norm);

        control_chassis_resolve_leg_motion_policy(leg,
                                                  height_profile,
                                                  height_forward_limit_rpm,
                                                  height_fast_forward_limit_rpm,
                                                  control_chassis_cmd.fast_enable,
                                                  &height_forward_limit_rpm,
                                                  &height_fast_forward_limit_rpm,
                                                  &effective_fast_enable);

        forward_limit_rpm = (APP_TRUE == effective_fast_enable) ?
                            height_fast_forward_limit_rpm :
                            height_forward_limit_rpm;
    }

    control_chassis_cmd.target_forward_rpm = control_chassis_limit_abs(forward_rpm,
                                                                       forward_limit_rpm);
    control_chassis_cmd.target_turn_dps = control_chassis_limit_abs(turn_rpm,
                                                                    APP_CHASSIS_TURN_RATE_LIMIT_DPS);
    control_chassis_cmd.enable = (APP_TRUE == enable) ? APP_TRUE : APP_FALSE;
    control_chassis_cmd.last_cmd_ms = now_ms;

    if((APP_CHASSIS_TURN_ZERO_TARGET_DPS > control_chassis_absf(control_chassis_cmd.target_turn_dps)) &&
       (APP_CHASSIS_FORWARD_ZERO_TARGET_RPM > control_chassis_absf(control_chassis_cmd.target_forward_rpm)))
    {
        control_chassis_cmd.speed_integral = 0.0f;
        control_chassis_cmd.turn_integral = 0.0f;
    }
}

uint8 control_chassis_set_drive_gain(float speed_kp, float speed_ki, float turn_kp)
{
    if((APP_FALSE == control_chassis_is_finite(speed_kp)) ||
       (APP_FALSE == control_chassis_is_finite(speed_ki)) ||
       (APP_FALSE == control_chassis_is_finite(turn_kp)) ||
       (APP_CHASSIS_DRIVE_GAIN_ABS_LIMIT < control_chassis_absf(speed_kp)) ||
       (APP_CHASSIS_DRIVE_GAIN_ABS_LIMIT < control_chassis_absf(speed_ki)) ||
       (APP_CHASSIS_DRIVE_GAIN_ABS_LIMIT < control_chassis_absf(turn_kp)))
    {
        return APP_FALSE;
    }

    control_chassis_cmd.speed_kp = speed_kp;
    control_chassis_cmd.speed_ki = speed_ki;
    control_chassis_cmd.speed_integral = 0.0f;

    if(turn_kp != control_chassis_cmd.turn_kp)
    {
        control_chassis_cmd.turn_kp = turn_kp;
        control_chassis_cmd.turn_integral = 0.0f;
    }
    return APP_TRUE;
}

uint8 control_chassis_set_turn_gain(float turn_kp, float turn_ki)
{
    if((APP_FALSE == control_chassis_is_finite(turn_kp)) ||
       (APP_FALSE == control_chassis_is_finite(turn_ki)) ||
       (APP_CHASSIS_DRIVE_GAIN_ABS_LIMIT < control_chassis_absf(turn_kp)) ||
       (APP_CHASSIS_DRIVE_GAIN_ABS_LIMIT < control_chassis_absf(turn_ki)))
    {
        return APP_FALSE;
    }

    control_chassis_cmd.turn_kp = turn_kp;
    control_chassis_cmd.turn_ki = turn_ki;
    control_chassis_cmd.turn_integral = 0.0f;
    return APP_TRUE;
}

void control_chassis_stop(uint32 now_ms)
{
    control_chassis_cmd.target_forward_rpm = 0.0f;
    control_chassis_cmd.target_turn_dps = 0.0f;
    control_chassis_cmd.actual_forward_rpm = 0.0f;
    control_chassis_cmd.actual_turn_dps = 0.0f;
    control_chassis_cmd.speed_pitch_offset_deg = 0.0f;
    control_chassis_cmd.turn_rpm = 0.0f;
    control_chassis_cmd.speed_integral = 0.0f;
    control_chassis_cmd.turn_integral = 0.0f;
    control_chassis_cmd.fast_blend = 0.0f;
    control_chassis_cmd.speed_pitch_limit_deg = APP_CHASSIS_SPEED_PITCH_LIMIT_DEG;
    control_chassis_cmd.speed_ff_rpm = 0.0f;
    control_chassis_cmd.fast_enable = APP_FALSE;
    control_chassis_cmd.enable = APP_FALSE;
    control_chassis_cmd.last_cmd_ms = now_ms;
    control_chassis_cmd.last_update_ms = now_ms;
    control_chassis_clear_output();
}

void control_chassis_set_fast_enable(uint8 enable)
{
    control_chassis_cmd.fast_enable = (APP_TRUE == enable) ? APP_TRUE : APP_FALSE;
    /* When disabling fast mode, only disarm the flag.  The update loop
       forces raw_fast_blend to 0 when fast_enable is false, so fast_blend,
       speed_ff_rpm, and speed_pitch_limit_deg ramp down smoothly via
       control_chassis_ramp_toward instead of a hard gain jump. */
}

const chassis_cmd_struct *control_chassis_get_cmd(void)
{
    return &control_chassis_cmd;
}

const chassis_output_struct *control_chassis_get_output(void)
{
    return &control_chassis_output;
}

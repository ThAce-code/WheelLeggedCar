/*********************************************************************************************************************
* File: control_chassis.c
* Description: Chassis forward/turn command shaper and mixer.
********************************************************************************************************************/

#include "control_chassis.h"
#include "app_config.h"
#include "actuator_motor.h"
#include "sensor_imu.h"

static chassis_cmd_struct control_chassis_cmd;
static chassis_output_struct control_chassis_output;

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

static void control_chassis_clear_output(void)
{
    control_chassis_output.pitch_offset_deg = 0.0f;
    control_chassis_output.turn_rpm = 0.0f;
    control_chassis_output.forward_target_rpm = 0.0f;
    control_chassis_output.forward_actual_rpm = 0.0f;
    control_chassis_output.turn_target_dps = 0.0f;
    control_chassis_output.gyro_z_dps = 0.0f;
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
    control_chassis_cmd.speed_integral = 0.0f;
    control_chassis_cmd.enable = APP_FALSE;
    control_chassis_cmd.last_cmd_ms = 0;
    control_chassis_cmd.last_update_ms = 0;

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
    float dt_s;

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

    if(APP_FALSE == control_chassis_cmd.enable)
    {
        control_chassis_cmd.target_forward_rpm = 0.0f;
        control_chassis_cmd.target_turn_dps = 0.0f;
    }

    target_forward_rpm = control_chassis_limit_abs(control_chassis_cmd.target_forward_rpm,
                                                   APP_CHASSIS_FORWARD_RPM_LIMIT);
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
                                  APP_CHASSIS_SPEED_PITCH_LIMIT_DEG);

    turn_error_dps = control_chassis_cmd.actual_turn_dps - imu->gyro_z_dps;
    turn_rpm = control_chassis_cmd.turn_kp * turn_error_dps;
    turn_rpm = control_chassis_limit_abs(turn_rpm, APP_CHASSIS_TURN_RPM_LIMIT);

    if((APP_FALSE == control_chassis_is_finite(avg_wheel_speed_rpm)) ||
       (APP_FALSE == control_chassis_is_finite(imu->gyro_z_dps)) ||
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
    control_chassis_output.gyro_z_dps = imu->gyro_z_dps;
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
        control_chassis_clear_output();
    }
}

void control_chassis_set_cmd(float forward_rpm, float turn_rpm, uint8 enable, uint32 now_ms)
{
    if((APP_FALSE == control_chassis_is_finite(forward_rpm)) ||
       (APP_FALSE == control_chassis_is_finite(turn_rpm)))
    {
        control_chassis_stop(now_ms);
        return;
    }

    control_chassis_cmd.target_forward_rpm = control_chassis_limit_abs(forward_rpm,
                                                                       APP_CHASSIS_FORWARD_RPM_LIMIT);
    control_chassis_cmd.target_turn_dps = control_chassis_limit_abs(turn_rpm,
                                                                    APP_CHASSIS_TURN_RATE_LIMIT_DPS);
    control_chassis_cmd.enable = (APP_TRUE == enable) ? APP_TRUE : APP_FALSE;
    control_chassis_cmd.last_cmd_ms = now_ms;
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
    control_chassis_cmd.turn_kp = turn_kp;
    control_chassis_cmd.speed_integral = 0.0f;
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
    control_chassis_cmd.enable = APP_FALSE;
    control_chassis_cmd.last_cmd_ms = now_ms;
    control_chassis_cmd.last_update_ms = now_ms;
    control_chassis_clear_output();
}

const chassis_cmd_struct *control_chassis_get_cmd(void)
{
    return &control_chassis_cmd;
}

const chassis_output_struct *control_chassis_get_output(void)
{
    return &control_chassis_output;
}

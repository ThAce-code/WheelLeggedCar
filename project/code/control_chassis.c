/*********************************************************************************************************************
* File: control_chassis.c
* Description: Chassis forward/turn command shaper and mixer.
********************************************************************************************************************/

#include "control_chassis.h"
#include "app_config.h"

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
    control_chassis_output.left_base_rpm = 0.0f;
    control_chassis_output.right_base_rpm = 0.0f;
    control_chassis_output.enable = APP_FALSE;
}

void control_chassis_init(void)
{
    control_chassis_cmd.target_forward_rpm = 0.0f;
    control_chassis_cmd.target_turn_rpm = 0.0f;
    control_chassis_cmd.actual_forward_rpm = 0.0f;
    control_chassis_cmd.actual_turn_rpm = 0.0f;
    control_chassis_cmd.enable = APP_FALSE;
    control_chassis_cmd.last_cmd_ms = 0;
    control_chassis_cmd.last_update_ms = 0;

    control_chassis_clear_output();
}

void control_chassis_update(uint32 now_ms)
{
    float target_forward_rpm;
    float target_turn_rpm;
    float left_rpm;
    float right_rpm;
    float dt_s;
    float forward_max_delta;
    float turn_max_delta;

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

    if(APP_FALSE == control_chassis_cmd.enable)
    {
        control_chassis_cmd.target_forward_rpm = 0.0f;
        control_chassis_cmd.target_turn_rpm = 0.0f;
    }

    target_forward_rpm = control_chassis_limit_abs(control_chassis_cmd.target_forward_rpm,
                                                   APP_CHASSIS_DRIVE_RPM_LIMIT);
    target_turn_rpm = control_chassis_limit_abs(control_chassis_cmd.target_turn_rpm,
                                                APP_CHASSIS_TURN_RPM_LIMIT);

    forward_max_delta = APP_CHASSIS_FORWARD_RAMP_RPM_S * dt_s;
    turn_max_delta = APP_CHASSIS_TURN_RAMP_RPM_S * dt_s;

    control_chassis_cmd.actual_forward_rpm =
        control_chassis_ramp_toward(control_chassis_cmd.actual_forward_rpm,
                                    target_forward_rpm,
                                    forward_max_delta);
    control_chassis_cmd.actual_turn_rpm =
        control_chassis_ramp_toward(control_chassis_cmd.actual_turn_rpm,
                                    target_turn_rpm,
                                    turn_max_delta);

    control_chassis_cmd.actual_forward_rpm =
        control_chassis_limit_abs(control_chassis_cmd.actual_forward_rpm,
                                  APP_CHASSIS_DRIVE_RPM_LIMIT);
    control_chassis_cmd.actual_turn_rpm =
        control_chassis_limit_abs(control_chassis_cmd.actual_turn_rpm,
                                  APP_CHASSIS_TURN_RPM_LIMIT);

    if((APP_FALSE == control_chassis_is_finite(control_chassis_cmd.actual_forward_rpm)) ||
       (APP_FALSE == control_chassis_is_finite(control_chassis_cmd.actual_turn_rpm)))
    {
        control_chassis_stop(now_ms);
        return;
    }

    left_rpm = control_chassis_cmd.actual_forward_rpm - control_chassis_cmd.actual_turn_rpm;
    right_rpm = control_chassis_cmd.actual_forward_rpm + control_chassis_cmd.actual_turn_rpm;

    control_chassis_output.left_base_rpm = control_chassis_limit_abs(left_rpm, APP_CHASSIS_RPM_LIMIT);
    control_chassis_output.right_base_rpm = control_chassis_limit_abs(right_rpm, APP_CHASSIS_RPM_LIMIT);
    control_chassis_output.enable = APP_TRUE;

    if((0.0f == control_chassis_cmd.actual_forward_rpm) &&
       (0.0f == control_chassis_cmd.actual_turn_rpm) &&
       (0.0f == control_chassis_cmd.target_forward_rpm) &&
       (0.0f == control_chassis_cmd.target_turn_rpm) &&
       (APP_FALSE == control_chassis_cmd.enable))
    {
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
                                                                       APP_CHASSIS_DRIVE_RPM_LIMIT);
    control_chassis_cmd.target_turn_rpm = control_chassis_limit_abs(turn_rpm,
                                                                    APP_CHASSIS_TURN_RPM_LIMIT);
    control_chassis_cmd.enable = (APP_TRUE == enable) ? APP_TRUE : APP_FALSE;
    control_chassis_cmd.last_cmd_ms = now_ms;
}

void control_chassis_stop(uint32 now_ms)
{
    control_chassis_cmd.target_forward_rpm = 0.0f;
    control_chassis_cmd.target_turn_rpm = 0.0f;
    control_chassis_cmd.actual_forward_rpm = 0.0f;
    control_chassis_cmd.actual_turn_rpm = 0.0f;
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

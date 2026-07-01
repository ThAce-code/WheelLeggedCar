/*********************************************************************************************************************
* File: control_chassis.c
* Description: Chassis forward/turn command mixer.
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

void control_chassis_init(void)
{
    control_chassis_cmd.forward_rpm = 0.0f;
    control_chassis_cmd.turn_rpm = 0.0f;
    control_chassis_cmd.enable = APP_FALSE;
    control_chassis_cmd.last_cmd_ms = 0;

    control_chassis_output.left_base_rpm = 0.0f;
    control_chassis_output.right_base_rpm = 0.0f;
    control_chassis_output.enable = APP_FALSE;
}

void control_chassis_update(uint32 now_ms)
{
    float forward_rpm;
    float turn_rpm;
    float left_rpm;
    float right_rpm;

    (void)now_ms;

    if(APP_FALSE == control_chassis_cmd.enable)
    {
        control_chassis_output.left_base_rpm = 0.0f;
        control_chassis_output.right_base_rpm = 0.0f;
        control_chassis_output.enable = APP_FALSE;
        return;
    }

    forward_rpm = control_chassis_limit_abs(control_chassis_cmd.forward_rpm, APP_CHASSIS_RPM_LIMIT);
    turn_rpm = control_chassis_limit_abs(control_chassis_cmd.turn_rpm, APP_CHASSIS_RPM_LIMIT);

    left_rpm = forward_rpm - turn_rpm;
    right_rpm = forward_rpm + turn_rpm;

    control_chassis_output.left_base_rpm = control_chassis_limit_abs(left_rpm, APP_CHASSIS_RPM_LIMIT);
    control_chassis_output.right_base_rpm = control_chassis_limit_abs(right_rpm, APP_CHASSIS_RPM_LIMIT);
    control_chassis_output.enable = APP_TRUE;
}

void control_chassis_set_cmd(float forward_rpm, float turn_rpm, uint8 enable, uint32 now_ms)
{
    control_chassis_cmd.forward_rpm = control_chassis_limit_abs(forward_rpm, APP_CHASSIS_RPM_LIMIT);
    control_chassis_cmd.turn_rpm = control_chassis_limit_abs(turn_rpm, APP_CHASSIS_RPM_LIMIT);
    control_chassis_cmd.enable = (APP_TRUE == enable) ? APP_TRUE : APP_FALSE;
    control_chassis_cmd.last_cmd_ms = now_ms;
}

void control_chassis_stop(uint32 now_ms)
{
    control_chassis_set_cmd(0.0f, 0.0f, APP_FALSE, now_ms);
    control_chassis_update(now_ms);
}

const chassis_cmd_struct *control_chassis_get_cmd(void)
{
    return &control_chassis_cmd;
}

const chassis_output_struct *control_chassis_get_output(void)
{
    return &control_chassis_output;
}

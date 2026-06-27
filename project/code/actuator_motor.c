/*********************************************************************************************************************
* File: actuator_motor.c
* Description: Brushless motor actuator placeholder.
********************************************************************************************************************/

#include "actuator_motor.h"
#include "app_config.h"

static motor_cmd_struct actuator_motor_cmd;

static float actuator_motor_limit(float value)
{
    if(APP_MOTOR_CMD_LIMIT < value)
    {
        return APP_MOTOR_CMD_LIMIT;
    }
    if((-APP_MOTOR_CMD_LIMIT) > value)
    {
        return -APP_MOTOR_CMD_LIMIT;
    }
    return value;
}

void actuator_motor_init(void)
{
    actuator_motor_stop();
}

void actuator_motor_set_cmd(const motor_cmd_struct *cmd)
{
    actuator_motor_cmd.left_target = actuator_motor_limit(cmd->left_target);
    actuator_motor_cmd.right_target = actuator_motor_limit(cmd->right_target);
    actuator_motor_cmd.enable = cmd->enable;
}

void actuator_motor_update(uint32 now_ms)
{
    (void)now_ms;
    if(APP_FALSE == actuator_motor_cmd.enable)
    {
        actuator_motor_stop();
    }
}

void actuator_motor_stop(void)
{
    actuator_motor_cmd.left_target = 0.0f;
    actuator_motor_cmd.right_target = 0.0f;
    actuator_motor_cmd.enable = APP_FALSE;
}

const motor_cmd_struct *actuator_motor_get_cmd(void)
{
    return &actuator_motor_cmd;
}

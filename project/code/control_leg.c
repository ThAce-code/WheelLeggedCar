/*********************************************************************************************************************
* File: control_leg.c
* Description: Leg controller placeholder.
********************************************************************************************************************/

#include "control_leg.h"
#include "app_config.h"
#include "app_state.h"
#include "actuator_servo.h"

static servo_cmd_struct control_leg_servo_cmd;

void control_leg_init(void)
{
    uint8 i;

    for(i = 0; i < APP_SERVO_COUNT; i++)
    {
        control_leg_servo_cmd.angle_deg[i] = APP_SERVO_MID_DEG;
    }
    control_leg_servo_cmd.enable = APP_FALSE;
}

void control_leg_update(uint32 now_ms)
{
    (void)now_ms;
    control_leg_servo_cmd.enable = app_state_is_run_enabled();
    actuator_servo_set_cmd(&control_leg_servo_cmd);
}

const servo_cmd_struct *control_leg_get_servo_cmd(void)
{
    return &control_leg_servo_cmd;
}

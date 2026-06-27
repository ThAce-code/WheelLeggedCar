/*********************************************************************************************************************
* File: control_chassis.c
* Description: Chassis controller placeholder.
********************************************************************************************************************/

#include "control_chassis.h"
#include "app_state.h"
#include "estimator.h"
#include "perception.h"
#include "actuator_motor.h"

static motor_cmd_struct control_chassis_motor_cmd;

void control_chassis_init(void)
{
    control_chassis_motor_cmd.left_target = 0.0f;
    control_chassis_motor_cmd.right_target = 0.0f;
    control_chassis_motor_cmd.enable = APP_FALSE;
}

void control_chassis_update(uint32 now_ms)
{
    const vehicle_state_struct *vehicle;
    const vision_state_struct *vision;

    (void)now_ms;
    vehicle = estimator_get_state();
    vision = perception_get_state();
    (void)vehicle;
    (void)vision;

    control_chassis_motor_cmd.enable = app_state_is_run_enabled();
    actuator_motor_set_cmd(&control_chassis_motor_cmd);
}

const motor_cmd_struct *control_chassis_get_motor_cmd(void)
{
    return &control_chassis_motor_cmd;
}

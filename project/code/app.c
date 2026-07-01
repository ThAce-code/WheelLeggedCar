/*********************************************************************************************************************
* File: app.c
* Description: Top-level application.
********************************************************************************************************************/

#include "app.h"
#include "app_config.h"
#include "app_state.h"
#include "app_safety.h"
#include "app_scheduler.h"
#include "sensor_imu.h"
#include "control_leg.h"
#include "actuator_servo.h"
#include "actuator_motor.h"
#include "host_command.h"
#include "telemetry.h"

uint8 app_init(void)
{
    uint8 result = 0;

    app_state_init();
    app_safety_init();
    app_scheduler_init();
    host_command_init();

    control_leg_init();
    actuator_servo_init();
    actuator_motor_init();
    telemetry_init();

    result |= sensor_imu_init();
    if(0U == result)
    {
        control_leg_set_body_cmd(0.0f, 0.0f, 0.0f);
        control_leg_set_mode(LEG_MODE_ATTITUDE);
        app_state_set(APP_STATE_STANDBY);
    }
    else
    {
        app_state_set(APP_STATE_FAULT);
    }

    return result;
}

void app_run_once(void)
{
    app_scheduler_run_pending();
}

uint32 app_get_ms(void)
{
    return app_scheduler_get_ms();
}

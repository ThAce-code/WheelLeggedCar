/*********************************************************************************************************************
* File: app.c
* Description: Top-level application — IMU-only polling test.
********************************************************************************************************************/

#include "app.h"
#include "app_config.h"
#include "app_state.h"
#include "app_scheduler.h"
#include "sensor_imu.h"
#include "actuator_servo.h"
#include "telemetry.h"

uint8 app_init(void)
{
    uint8 result = 0;

    app_state_init();
    app_scheduler_init();
    telemetry_init();
    actuator_servo_init();

    result |= sensor_imu_init();
    if(0U == result)
    {
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

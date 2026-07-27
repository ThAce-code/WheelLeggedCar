/*********************************************************************************************************************
* File: app_safety.c
* Description: Safety monitor — disables actuators on IMU fault or angle limit exceed.
********************************************************************************************************************/

#include "app_safety.h"
#include "app_config.h"
#include "app_state.h"
#include "sensor_imu.h"
#include "actuator_motor.h"
#include "actuator_servo.h"

static uint8 app_safety_fault = APP_FALSE;
static uint8 app_safety_armed = APP_FALSE;

static float app_absf(float value)
{
    return (0.0f > value) ? -value : value;
}

static uint8 app_safety_is_finite(float value)
{
    return ((value == value) &&
            (3.402823466e+38F >= value) &&
            (-3.402823466e+38F <= value)) ? APP_TRUE : APP_FALSE;
}

void app_safety_init(void)
{
    app_safety_fault = APP_FALSE;
    app_safety_armed = APP_FALSE;
}

void app_safety_update(uint32 now_ms)
{
    const imu_state_struct *imu;

    if(APP_TRUE == app_safety_fault)
    {
        return;
    }

    if(APP_FALSE == app_safety_armed)
    {
        if(APP_SAFETY_ARM_DELAY_MS > now_ms)
        {
            return;
        }
        app_safety_armed = APP_TRUE;
    }

    imu = sensor_imu_get_state();
    if((APP_FALSE == imu->healthy) ||
       (APP_FALSE == app_safety_is_finite(imu->roll)) ||
       (APP_FALSE == app_safety_is_finite(imu->pitch)) ||
       (APP_ROLL_LIMIT_DEG < app_absf(imu->roll)) ||
       (APP_PITCH_LIMIT_DEG < app_absf(imu->pitch)))
    {
        app_safety_force_fault();
    }
}

uint8 app_safety_is_fault(void)
{
    return app_safety_fault;
}

void app_safety_force_fault(void)
{
    if(APP_TRUE == app_safety_fault)
    {
        return;
    }

    app_safety_fault = APP_TRUE;
    app_state_set(APP_STATE_FAULT);
    actuator_servo_disable();
    actuator_motor_stop();
}

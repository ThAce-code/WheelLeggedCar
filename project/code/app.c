/*********************************************************************************************************************
* File: app.c
* Description: Top-level application composition.
********************************************************************************************************************/

#include "app.h"
#include "app_config.h"
#include "app_state.h"
#include "app_safety.h"
#include "app_scheduler.h"
#include "sensor_imu.h"
#include "sensor_encoder.h"
#include "estimator.h"
#include "perception.h"
#include "control_chassis.h"
#include "control_leg.h"
#include "actuator_motor.h"
#include "actuator_servo.h"
#include "telemetry.h"

uint8 app_init(void)
{
    uint8 result = 0;

    app_state_init();
    app_safety_init();
    app_scheduler_init();

    sensor_encoder_init();
    estimator_init();
    perception_init();
    control_chassis_init();
    control_leg_init();
    actuator_motor_init();
    actuator_servo_init();
    telemetry_init();

    result |= sensor_imu_init();
    if(0U == result)
    {
#if APP_SERVO_TEST_ENABLE
        actuator_servo_set_angle(APP_SERVO_TEST_INDEX, APP_SERVO_TEST_TARGET_DEG);
        actuator_servo_enable();
#endif
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

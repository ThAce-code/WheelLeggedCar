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
#if APP_SERVO_TEST_ENABLE
    uint8 i;
    servo_cmd_struct servo_test_cmd;
#endif

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
        for(i = 0; i < APP_SERVO_COUNT; i++)
        {
            servo_test_cmd.angle_deg[i] = APP_SERVO_MID_DEG;
            servo_test_cmd.enable[i] = APP_FALSE;
        }
        servo_test_cmd.angle_deg[APP_SERVO_TEST_INDEX] = APP_SERVO_TEST_MIN_DEG;
        servo_test_cmd.enable[APP_SERVO_TEST_INDEX] = APP_TRUE;
        actuator_servo_set_cmd(&servo_test_cmd);
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

/*********************************************************************************************************************
* File: app_scheduler.c
* Description: Cooperative fixed-period scheduler.
********************************************************************************************************************/

#include "app_scheduler.h"
#include "app_config.h"
#include "app_types.h"

#if !APP_SCHEDULER_IMU_ONLY
#include "app_safety.h"
#include "sensor_camera.h"
#include "sensor_encoder.h"
#include "estimator.h"
#include "perception.h"
#include "control_chassis.h"
#include "control_leg.h"
#include "actuator_motor.h"
#include "actuator_servo.h"
#endif

#include "sensor_imu.h"
#include "telemetry.h"

#if APP_SERVO_TEST_ENABLE
#include "actuator_servo.h"
#endif

static volatile uint32 app_tick_ms = 0;
static volatile uint8 app_scheduler_pending = APP_FALSE;

#if APP_SERVO_TEST_ENABLE
static void app_scheduler_servo_test_update(uint32 now_ms)
{
    static uint32 servo_test_last_ms = 0;
    static uint8 servo_test_high = APP_FALSE;

    if(APP_SERVO_TEST_PERIOD_MS <= (now_ms - servo_test_last_ms))
    {
        servo_test_last_ms = now_ms;
        servo_test_high = (APP_FALSE == servo_test_high) ? APP_TRUE : APP_FALSE;
        actuator_servo_set_angle(APP_SERVO_TEST_INDEX,
                                 (APP_TRUE == servo_test_high) ? APP_SERVO_TEST_MAX_DEG : APP_SERVO_TEST_MIN_DEG);
    }
}
#endif

static uint8 app_task_elapsed(uint32 now_ms, uint32 *last_ms, uint32 period_ms)
{
    if(period_ms <= (now_ms - *last_ms))
    {
        *last_ms = now_ms;
        return APP_TRUE;
    }
    return APP_FALSE;
}

void app_scheduler_init(void)
{
    app_tick_ms = 0;
    app_scheduler_pending = APP_FALSE;
}

void app_scheduler_tick_1ms(void)
{
    app_tick_ms++;
    app_scheduler_pending = APP_TRUE;
}

void app_scheduler_run_pending(void)
{
#if !APP_SCHEDULER_IMU_ONLY
    static uint32 safety_last_ms = 0;
    static uint32 encoder_last_ms = 0;
    static uint32 motor_last_ms = 0;
    static uint32 estimator_last_ms = 0;
    static uint32 chassis_last_ms = 0;
    static uint32 leg_last_ms = 0;
    static uint32 servo_last_ms = 0;
    static uint32 camera_last_ms = 0;
    static uint32 perception_last_ms = 0;
#endif
    static uint32 telemetry_last_ms = 0;
#if !APP_SCHEDULER_IMU_ONLY || APP_SERVO_TEST_ENABLE
    static uint32 servo_last_ms = 0;
#endif
#if !((APP_SCHEDULER_IMU_ONLY == 1U) && (APP_IMU_USE_INT1 == 1U))
    static uint32 imu_last_ms = 0;
#endif
    uint32 now_ms;

    if(APP_FALSE == app_scheduler_pending)
    {
        return;
    }
    app_scheduler_pending = APP_FALSE;
    now_ms = app_tick_ms;

#if APP_SCHEDULER_IMU_ONLY
#if APP_IMU_USE_INT1
    {
        static uint8 imu_stale_active = APP_FALSE;

        if(APP_TRUE == sensor_imu_take_data_ready())
        {
            sensor_imu_update(now_ms);
        }

        if((now_ms - sensor_imu_get_last_update_ms()) > APP_IMU_STALE_TIMEOUT_MS)
        {
            if(APP_FALSE == imu_stale_active)
            {
                sensor_imu_mark_stale();
                imu_stale_active = APP_TRUE;
            }
        }
        else
        {
            imu_stale_active = APP_FALSE;
        }
    }
#else
    if(APP_TRUE == app_task_elapsed(now_ms, &imu_last_ms, APP_IMU_PERIOD_MS))
    {
        sensor_imu_update(now_ms);
    }
#endif
#if APP_SERVO_TEST_ENABLE
    if(APP_TRUE == app_task_elapsed(now_ms, &servo_last_ms, APP_SERVO_PERIOD_MS))
    {
        app_scheduler_servo_test_update(now_ms);
        actuator_servo_update(now_ms);
    }
#endif
    if(APP_TRUE == app_task_elapsed(now_ms, &telemetry_last_ms, APP_TELEMETRY_PERIOD_MS))
    {
        telemetry_update(now_ms);
    }
#else
    if(APP_TRUE == app_task_elapsed(now_ms, &safety_last_ms, APP_SAFETY_PERIOD_MS))
    {
        app_safety_update(now_ms);
    }
    if(APP_TRUE == app_task_elapsed(now_ms, &encoder_last_ms, APP_ENCODER_PERIOD_MS))
    {
        sensor_encoder_update(now_ms);
    }
    if(APP_TRUE == app_task_elapsed(now_ms, &motor_last_ms, APP_MOTOR_PERIOD_MS))
    {
        actuator_motor_update(now_ms);
    }
    if(APP_TRUE == app_task_elapsed(now_ms, &imu_last_ms, APP_IMU_PERIOD_MS))
    {
        sensor_imu_update(now_ms);
    }
    if(APP_TRUE == app_task_elapsed(now_ms, &estimator_last_ms, APP_ESTIMATOR_PERIOD_MS))
    {
        estimator_update(now_ms);
    }
    if(APP_TRUE == app_task_elapsed(now_ms, &chassis_last_ms, APP_CHASSIS_CONTROL_PERIOD_MS))
    {
        control_chassis_update(now_ms);
    }
    if(APP_TRUE == app_task_elapsed(now_ms, &leg_last_ms, APP_LEG_CONTROL_PERIOD_MS))
    {
        control_leg_update(now_ms);
    }
    if(APP_TRUE == app_task_elapsed(now_ms, &servo_last_ms, APP_SERVO_PERIOD_MS))
    {
        actuator_servo_update(now_ms);
    }
    if(APP_TRUE == app_task_elapsed(now_ms, &camera_last_ms, APP_CAMERA_PERIOD_MS))
    {
        sensor_camera_update(now_ms);
    }
    if(APP_TRUE == app_task_elapsed(now_ms, &perception_last_ms, APP_PERCEPTION_PERIOD_MS))
    {
        perception_update(now_ms);
    }
    if(APP_TRUE == app_task_elapsed(now_ms, &telemetry_last_ms, APP_TELEMETRY_PERIOD_MS))
    {
        telemetry_update(now_ms);
    }
#endif
}

uint32 app_scheduler_get_ms(void)
{
    return app_tick_ms;
}

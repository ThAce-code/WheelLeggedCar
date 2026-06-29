/*********************************************************************************************************************
* File: app_scheduler.c
* Description: Cooperative fixed-period scheduler — IMU polling + telemetry.
********************************************************************************************************************/

#include "app_scheduler.h"
#include "app_config.h"
#include "app_types.h"
#include "sensor_imu.h"
#include "telemetry.h"

static volatile uint32 app_tick_ms = 0;
static volatile uint8 app_scheduler_pending = APP_FALSE;

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
    static uint32 imu_last_ms = 0;
    static uint32 telemetry_last_ms = 0;
    uint32 now_ms;

    if(APP_FALSE == app_scheduler_pending)
    {
        return;
    }
    app_scheduler_pending = APP_FALSE;
    now_ms = app_tick_ms;

    if(APP_TRUE == app_task_elapsed(now_ms, &imu_last_ms, APP_IMU_PERIOD_MS))
    {
        sensor_imu_update(now_ms);
    }

    if(APP_TRUE == app_task_elapsed(now_ms, &telemetry_last_ms, APP_TELEMETRY_PERIOD_MS))
    {
        telemetry_update(now_ms);
    }
}

uint32 app_scheduler_get_ms(void)
{
    return app_tick_ms;
}

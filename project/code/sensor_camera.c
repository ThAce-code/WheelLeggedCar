/*********************************************************************************************************************
* File: sensor_camera.c
* Description: MT9V03X camera adapter.
********************************************************************************************************************/

#include "sensor_camera.h"
#include "zf_device_mt9v03x.h"

static camera_state_struct sensor_camera_state;

uint8 sensor_camera_init(void)
{
    uint8 result;

    result = mt9v03x_init();
    sensor_camera_state.healthy = (0U == result) ? APP_TRUE : APP_FALSE;
    return result;
}

void sensor_camera_update(uint32 now_ms)
{
    if(0U != mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;
        sensor_camera_state.timestamp_ms = now_ms;
        sensor_camera_state.frame_ready = APP_TRUE;
        sensor_camera_state.healthy = APP_TRUE;
    }
    else
    {
        sensor_camera_state.frame_ready = APP_FALSE;
    }
}

const camera_state_struct *sensor_camera_get_state(void)
{
    return &sensor_camera_state;
}

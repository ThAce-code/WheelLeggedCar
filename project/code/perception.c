/*********************************************************************************************************************
* File: perception.c
* Description: Vision perception placeholder.
********************************************************************************************************************/

#include "perception.h"
#include "sensor_camera.h"

static vision_state_struct perception_state;

void perception_init(void)
{
    perception_state.healthy = APP_FALSE;
}

void perception_update(uint32 now_ms)
{
    const camera_state_struct *camera;

    camera = sensor_camera_get_state();
    perception_state.timestamp_ms = now_ms;
    if(APP_TRUE == camera->frame_ready)
    {
        perception_state.lateral_error = 0.0f;
        perception_state.heading_error = 0.0f;
        perception_state.confidence = 0;
        perception_state.healthy = APP_TRUE;
    }
}

const vision_state_struct *perception_get_state(void)
{
    return &perception_state;
}

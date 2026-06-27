/*********************************************************************************************************************
* File: sensor_encoder.c
* Description: Wheel encoder adapter placeholder.
********************************************************************************************************************/

#include "sensor_encoder.h"

static wheel_state_struct sensor_encoder_state;

void sensor_encoder_init(void)
{
    sensor_encoder_state.healthy = APP_TRUE;
}

void sensor_encoder_update(uint32 now_ms)
{
    sensor_encoder_state.timestamp_ms = now_ms;
    sensor_encoder_state.left_count = 0;
    sensor_encoder_state.right_count = 0;
    sensor_encoder_state.left_speed = 0.0f;
    sensor_encoder_state.right_speed = 0.0f;
}

const wheel_state_struct *sensor_encoder_get_state(void)
{
    return &sensor_encoder_state;
}

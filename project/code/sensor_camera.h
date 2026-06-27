/*********************************************************************************************************************
* File: sensor_camera.h
* Description: Camera sensor snapshot interface.
********************************************************************************************************************/

#ifndef _sensor_camera_h_
#define _sensor_camera_h_

#include "app_types.h"

uint8 sensor_camera_init(void);
void sensor_camera_update(uint32 now_ms);
const camera_state_struct *sensor_camera_get_state(void);

#endif

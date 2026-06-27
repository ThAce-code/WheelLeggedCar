/*********************************************************************************************************************
* File: sensor_encoder.h
* Description: Wheel encoder snapshot interface.
********************************************************************************************************************/

#ifndef _sensor_encoder_h_
#define _sensor_encoder_h_

#include "app_types.h"

void sensor_encoder_init(void);
void sensor_encoder_update(uint32 now_ms);
const wheel_state_struct *sensor_encoder_get_state(void);

#endif

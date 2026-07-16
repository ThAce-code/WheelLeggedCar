/*********************************************************************************************************************
* File: single_gap_app.h
* Description: CM7_1 orchestration for the bounded single-gap validation path.
*********************************************************************************************************************/

#ifndef _single_gap_app_h_
#define _single_gap_app_h_

#include "camera_frame_consumer.h"

uint8 single_gap_app_init(void);
void single_gap_app_on_frame(const camera_vision_frame_view_struct *frame);
void single_gap_app_service(uint32 now_ms);
float single_gap_speed_mps_to_rpm(float speed_mps, float circumference_m);

#endif

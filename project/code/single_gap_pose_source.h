/*********************************************************************************************************************
* File: single_gap_pose_source.h
* Description: CM7_0 short-range wheel-odometry publisher for single-gap validation.
*********************************************************************************************************************/

#ifndef _single_gap_pose_source_h_
#define _single_gap_pose_source_h_

#include "zf_common_typedef.h"

uint8 single_gap_pose_source_init(void);
void single_gap_pose_source_update(uint32 now_ms);
float single_gap_pose_integrate_distance(float previous_m,
                                         float wheel_rpm,
                                         float circumference_m,
                                         uint32 dt_ms);

#endif

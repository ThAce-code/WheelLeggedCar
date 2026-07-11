/*********************************************************************************************************************
* File: imu_rate_filter.h
* Description: First-order filter for IMU angular-rate samples.
********************************************************************************************************************/

#ifndef _imu_rate_filter_h_
#define _imu_rate_filter_h_

#include "app_types.h"

typedef struct
{
    float output_dps;
    uint8 initialized;
}imu_rate_filter_state_struct;

void imu_rate_filter_init(imu_rate_filter_state_struct *state);
void imu_rate_filter_reset(imu_rate_filter_state_struct *state);
float imu_rate_filter_step(imu_rate_filter_state_struct *state,
                           float input_dps,
                           float alpha);

#endif

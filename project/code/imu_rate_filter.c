/*********************************************************************************************************************
* File: imu_rate_filter.c
* Description: First-order filter for IMU angular-rate samples.
********************************************************************************************************************/

#include "imu_rate_filter.h"
#include <stddef.h>

void imu_rate_filter_init(imu_rate_filter_state_struct *state)
{
    if(NULL == state)
    {
        return;
    }
    state->output_dps = 0.0f;
    state->initialized = APP_FALSE;
}

void imu_rate_filter_reset(imu_rate_filter_state_struct *state)
{
    imu_rate_filter_init(state);
}

float imu_rate_filter_step(imu_rate_filter_state_struct *state,
                           float input_dps,
                           float alpha)
{
    if(NULL == state)
    {
        return input_dps;
    }

    if(0.0f > alpha)
    {
        alpha = 0.0f;
    }
    else if(1.0f < alpha)
    {
        alpha = 1.0f;
    }

    if(APP_FALSE == state->initialized)
    {
        state->output_dps = input_dps;
        state->initialized = APP_TRUE;
    }
    else
    {
        state->output_dps += alpha * (input_dps - state->output_dps);
    }

    return state->output_dps;
}

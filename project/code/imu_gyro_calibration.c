/*********************************************************************************************************************
* File: imu_gyro_calibration.c
* Description: Online statistics and stationarity gate for gyro zero calibration.
********************************************************************************************************************/

#include "imu_gyro_calibration.h"
#include <stddef.h>

static float imu_gyro_calibration_absf(float value)
{
    return (0.0f > value) ? -value : value;
}

void imu_gyro_calibration_init(imu_gyro_calibration_state_struct *state)
{
    if(NULL == state)
    {
        return;
    }

    state->sample_count = 0U;
    state->mean_x = 0.0f;
    state->mean_y = 0.0f;
    state->mean_z = 0.0f;
    state->m2_x = 0.0f;
    state->m2_y = 0.0f;
    state->m2_z = 0.0f;
}

void imu_gyro_calibration_add_sample(imu_gyro_calibration_state_struct *state,
                                     float gyro_x_dps,
                                     float gyro_y_dps,
                                     float gyro_z_dps)
{
    float count;
    float delta;

    if(NULL == state)
    {
        return;
    }

    state->sample_count++;
    count = (float)state->sample_count;

    delta = gyro_x_dps - state->mean_x;
    state->mean_x += delta / count;
    state->m2_x += delta * (gyro_x_dps - state->mean_x);

    delta = gyro_y_dps - state->mean_y;
    state->mean_y += delta / count;
    state->m2_y += delta * (gyro_y_dps - state->mean_y);

    delta = gyro_z_dps - state->mean_z;
    state->mean_z += delta / count;
    state->m2_z += delta * (gyro_z_dps - state->mean_z);
}

uint8 imu_gyro_calibration_finish(const imu_gyro_calibration_state_struct *state,
                                  uint32 required_samples,
                                  float max_abs_mean_dps,
                                  float max_variance_dps2,
                                  float *offset_x_dps,
                                  float *offset_y_dps,
                                  float *offset_z_dps)
{
    float sample_count;
    float variance_x;
    float variance_y;
    float variance_z;

    if((NULL == state) ||
       (NULL == offset_x_dps) ||
       (NULL == offset_y_dps) ||
       (NULL == offset_z_dps) ||
       (0U == required_samples) ||
       (required_samples > state->sample_count))
    {
        return IMU_GYRO_CALIBRATION_NOT_READY;
    }

    sample_count = (float)state->sample_count;
    variance_x = state->m2_x / sample_count;
    variance_y = state->m2_y / sample_count;
    variance_z = state->m2_z / sample_count;

    if((max_abs_mean_dps < imu_gyro_calibration_absf(state->mean_x)) ||
       (max_abs_mean_dps < imu_gyro_calibration_absf(state->mean_y)) ||
       (max_abs_mean_dps < imu_gyro_calibration_absf(state->mean_z)) ||
       (max_variance_dps2 < variance_x) ||
       (max_variance_dps2 < variance_y) ||
       (max_variance_dps2 < variance_z))
    {
        return IMU_GYRO_CALIBRATION_MOVING;
    }

    *offset_x_dps = state->mean_x;
    *offset_y_dps = state->mean_y;
    *offset_z_dps = state->mean_z;
    return IMU_GYRO_CALIBRATION_OK;
}

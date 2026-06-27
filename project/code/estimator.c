/*********************************************************************************************************************
* File: estimator.c
* Description: Vehicle state estimator.
********************************************************************************************************************/

#include "estimator.h"
#include "sensor_imu.h"
#include "sensor_encoder.h"

static vehicle_state_struct estimator_state;

void estimator_init(void)
{
    estimator_state.healthy = APP_FALSE;
}

void estimator_update(uint32 now_ms)
{
    const imu_state_struct *imu;
    const wheel_state_struct *wheel;

    imu = sensor_imu_get_state();
    wheel = sensor_encoder_get_state();

    estimator_state.timestamp_ms = now_ms;
    estimator_state.roll = imu->roll;
    estimator_state.pitch = imu->pitch;
    estimator_state.yaw = imu->yaw;
    estimator_state.left_speed = wheel->left_speed;
    estimator_state.right_speed = wheel->right_speed;
    estimator_state.healthy = (imu->healthy && wheel->healthy) ? APP_TRUE : APP_FALSE;
}

const vehicle_state_struct *estimator_get_state(void)
{
    return &estimator_state;
}

/*********************************************************************************************************************
* File: sensor_imu.c
* Description: LSM6DSV16X IMU adapter.
********************************************************************************************************************/

#include "sensor_imu.h"
#include "lsm6dsv16x_driver.h"

#define SENSOR_IMU_WHO_AM_I_RETRY      (30U)
#define SENSOR_IMU_WHO_AM_I_DELAY_MS   (100U)

static imu_state_struct sensor_imu_state;

uint8 sensor_imu_init(void)
{
    uint16 retry;

    sensor_imu_state.healthy = APP_FALSE;
    lsm6dsv16x_spi_init();

    for(retry = 0; retry < SENSOR_IMU_WHO_AM_I_RETRY; retry++)
    {
        if(0U == lsm6dsv16x_self_check())
        {
            break;
        }
        system_delay_ms(SENSOR_IMU_WHO_AM_I_DELAY_MS);
    }

    if(SENSOR_IMU_WHO_AM_I_RETRY <= retry)
    {
        return SENSOR_IMU_ERR_WHO_AM_I;
    }

    if(0U != lsm6dsv16x_init())
    {
        return SENSOR_IMU_ERR_INIT;
    }

    if(0U != lsm6dsv16x_sflp_init())
    {
        return SENSOR_IMU_ERR_SFLP;
    }

    sensor_imu_state.healthy = APP_TRUE;
    return SENSOR_IMU_OK;
}

void sensor_imu_update(uint32 now_ms)
{
    lsm6dsv16x_angle_data_struct *angle;

    if(0U == lsm6dsv16x_sflp_update())
    {
        angle = lsm6dsv16x_get_angle_data();
        sensor_imu_state.timestamp_ms = now_ms;
        sensor_imu_state.roll = angle->roll;
        sensor_imu_state.pitch = angle->pitch;
        sensor_imu_state.yaw = angle->yaw;
        sensor_imu_state.quat_w = angle->quat_w;
        sensor_imu_state.quat_x = angle->quat_x;
        sensor_imu_state.quat_y = angle->quat_y;
        sensor_imu_state.quat_z = angle->quat_z;
        sensor_imu_state.healthy = APP_TRUE;
    }
}

const imu_state_struct *sensor_imu_get_state(void)
{
    return &sensor_imu_state;
}

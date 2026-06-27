/*********************************************************************************************************************
* File: sensor_imu.h
* Description: IMU sensor snapshot interface.
********************************************************************************************************************/

#ifndef _sensor_imu_h_
#define _sensor_imu_h_

#include "app_types.h"

#define SENSOR_IMU_OK                  (0U)
#define SENSOR_IMU_ERR_WHO_AM_I        (1U)
#define SENSOR_IMU_ERR_INIT            (2U)
#define SENSOR_IMU_ERR_SFLP            (3U)

uint8 sensor_imu_init(void);
void sensor_imu_update(uint32 now_ms);
const imu_state_struct *sensor_imu_get_state(void);

#endif

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
#define SENSOR_IMU_ERR_GYRO_CAL        (4U)

uint8 sensor_imu_init(void);
void sensor_imu_update(uint32 now_ms);
const imu_state_struct *sensor_imu_get_state(void);

void sensor_imu_int1_isr(void);
uint8 sensor_imu_take_data_ready(uint32 *source_ms);
uint32 sensor_imu_get_last_update_ms(void);
uint32 sensor_imu_get_int_count(void);
uint32 sensor_imu_get_stale_count(void);
uint32 sensor_imu_get_invalid_sample_count(void);
void sensor_imu_mark_stale(void);

#endif

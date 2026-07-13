/*********************************************************************************************************************
* File: imu_gyro_calibration.h
* Description: Online statistics and stationarity gate for gyro zero calibration.
********************************************************************************************************************/

#ifndef _imu_gyro_calibration_h_
#define _imu_gyro_calibration_h_

#include "app_types.h"

#define IMU_GYRO_CALIBRATION_OK          (0U)
#define IMU_GYRO_CALIBRATION_NOT_READY   (1U)
#define IMU_GYRO_CALIBRATION_MOVING      (2U)

typedef struct
{
    uint32 sample_count;
    float mean_x;
    float mean_y;
    float mean_z;
    float m2_x;
    float m2_y;
    float m2_z;
}imu_gyro_calibration_state_struct;

void imu_gyro_calibration_init(imu_gyro_calibration_state_struct *state);
void imu_gyro_calibration_add_sample(imu_gyro_calibration_state_struct *state,
                                     float gyro_x_dps,
                                     float gyro_y_dps,
                                     float gyro_z_dps);
uint8 imu_gyro_calibration_finish(const imu_gyro_calibration_state_struct *state,
                                  uint32 required_samples,
                                  float max_abs_mean_dps,
                                  float max_variance_dps2,
                                  float *offset_x_dps,
                                  float *offset_y_dps,
                                  float *offset_z_dps);

#endif

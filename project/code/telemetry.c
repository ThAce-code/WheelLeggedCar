/*********************************************************************************************************************
* File: telemetry.c
* Description: VOFA+ telemetry.
********************************************************************************************************************/

#include "telemetry.h"
#include "app_config.h"
#include "sensor_imu.h"
#include "actuator_motor.h"

void telemetry_init(void)
{
}

void telemetry_update(uint32 now_ms)
{
    static const uint8 tail[4] = {0x00, 0x00, 0x80, 0x7F};
    const imu_state_struct *imu;
    const wheel_feedback_struct *wheel;
    const motor_diag_struct *motor_diag;
    float vofa_data[20];

    imu = sensor_imu_get_state();
    wheel = actuator_motor_get_feedback();
    motor_diag = actuator_motor_get_diag();

    vofa_data[0] = imu->roll;
    vofa_data[1] = imu->pitch;
    vofa_data[2] = imu->yaw;
    vofa_data[3] = imu->quat_w;
    vofa_data[4] = imu->quat_x;
    vofa_data[5] = imu->quat_y;
    vofa_data[6] = imu->quat_z;
    vofa_data[7] = (float)(now_ms - sensor_imu_get_last_update_ms());
    vofa_data[8] = (float)sensor_imu_get_int_count();
    vofa_data[9] = (float)sensor_imu_get_stale_count();
    vofa_data[10] = (float)wheel->online;
    vofa_data[11] = (float)wheel->age_ms;
    vofa_data[12] = (float)wheel->left_speed;
    vofa_data[13] = (float)wheel->right_speed;
    vofa_data[14] = (float)wheel->left_reduced_angle;
    vofa_data[15] = (float)wheel->right_reduced_angle;
    vofa_data[16] = (float)motor_diag->left_raw_angle;
    vofa_data[17] = (float)motor_diag->right_raw_angle;
    vofa_data[18] = (float)motor_diag->checksum_error_count;
    vofa_data[19] = (float)motor_diag->unknown_frame_count;

    debug_send_buffer((const uint8 *)vofa_data, sizeof(vofa_data));
    debug_send_buffer(tail, sizeof(tail));
}

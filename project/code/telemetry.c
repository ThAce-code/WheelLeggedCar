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
    float vofa_data[12];

    (void)now_ms;
    imu = sensor_imu_get_state();
    wheel = actuator_motor_get_feedback();
    motor_diag = actuator_motor_get_diag();

    vofa_data[0] = imu->roll;
    vofa_data[1] = imu->pitch;
    vofa_data[2] = imu->yaw;
    vofa_data[3] = (float)wheel->online;
    vofa_data[4] = (float)wheel->age_ms;
    vofa_data[5] = (float)wheel->left_speed;
    vofa_data[6] = (float)wheel->right_speed;
    vofa_data[7] = (float)wheel->left_reduced_angle;
    vofa_data[8] = (float)wheel->right_reduced_angle;
    vofa_data[9] = (float)motor_diag->left_raw_angle;
    vofa_data[10] = (float)motor_diag->checksum_error_count;
    vofa_data[11] = (float)motor_diag->unknown_frame_count;

    debug_send_buffer((const uint8 *)vofa_data, sizeof(vofa_data));
    debug_send_buffer(tail, sizeof(tail));
}

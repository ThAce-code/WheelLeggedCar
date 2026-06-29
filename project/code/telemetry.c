/*********************************************************************************************************************
* File: telemetry.c
* Description: VOFA+ telemetry — IMU data only.
********************************************************************************************************************/

#include "telemetry.h"
#include "app_config.h"
#include "sensor_imu.h"

void telemetry_init(void)
{
}

void telemetry_update(uint32 now_ms)
{
    static const uint8 tail[4] = {0x00, 0x00, 0x80, 0x7F};
    const imu_state_struct *imu;
    float vofa_data[15];
    uint32 last_update_ms;

    imu = sensor_imu_get_state();
    last_update_ms = sensor_imu_get_last_update_ms();

    vofa_data[0] = imu->roll;
    vofa_data[1] = imu->pitch;
    vofa_data[2] = imu->yaw;
    vofa_data[3] = imu->quat_w;
    vofa_data[4] = imu->quat_x;
    vofa_data[5] = imu->quat_y;
    vofa_data[6] = imu->quat_z;
    vofa_data[7] = (float)(now_ms - last_update_ms);
    vofa_data[8] = (float)sensor_imu_get_int_count();
    vofa_data[9] = (float)sensor_imu_get_stale_count();
    vofa_data[10] = 0.0f;
    vofa_data[11] = 0.0f;
    vofa_data[12] = 0.0f;
    vofa_data[13] = 0.0f;
    vofa_data[14] = 0.0f;

    debug_send_buffer((const uint8 *)vofa_data, sizeof(vofa_data));
    debug_send_buffer(tail, sizeof(tail));
}

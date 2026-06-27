/*********************************************************************************************************************
* File: telemetry.c
* Description: VOFA+ telemetry placeholder.
********************************************************************************************************************/

#include "telemetry.h"
#include "sensor_imu.h"

void telemetry_init(void)
{
}

void telemetry_update(uint32 now_ms)
{
    static const uint8 tail[4] = {0x00, 0x00, 0x80, 0x7F};
    const imu_state_struct *imu;
    float vofa_data[3];

    (void)now_ms;
    imu = sensor_imu_get_state();

    vofa_data[0] = imu->roll;
    vofa_data[1] = imu->pitch;
    vofa_data[2] = imu->yaw;

    debug_send_buffer((const uint8 *)vofa_data, sizeof(vofa_data));
    debug_send_buffer(tail, sizeof(tail));
}

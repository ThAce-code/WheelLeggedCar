/*********************************************************************************************************************
* File: telemetry.c
* Description: VOFA+ telemetry with servo diagnostic channels.
********************************************************************************************************************/

#include "telemetry.h"
#include "app_config.h"
#include "app_state.h"
#include "sensor_imu.h"
#include "control_leg.h"
#include "actuator_servo.h"

void telemetry_init(void)
{
}

void telemetry_update(uint32 now_ms)
{
    static const uint8 tail[4] = {0x00, 0x00, 0x80, 0x7F};
    const imu_state_struct *imu;
    const servo_cmd_struct *leg_cmd;
    float vofa_data[7];
    uint32 last_update_ms;

    (void)now_ms;
    imu = sensor_imu_get_state();
    last_update_ms = sensor_imu_get_last_update_ms();
    leg_cmd = control_leg_get_servo_cmd();

    vofa_data[0] = imu->roll;
    vofa_data[1] = imu->pitch;
    vofa_data[2] = (float)app_state_get();
    vofa_data[3] = (float)control_leg_get_safe_ready();
    vofa_data[4] = (float)leg_cmd->enable[LEG_SERVO_RR];
    vofa_data[5] = actuator_servo_get_current_angle(LEG_SERVO_RR);
    vofa_data[6] = (float)actuator_servo_angle_to_duty(actuator_servo_get_current_angle(LEG_SERVO_RR));

    debug_send_buffer((const uint8 *)vofa_data, sizeof(vofa_data));
    debug_send_buffer(tail, sizeof(tail));
}

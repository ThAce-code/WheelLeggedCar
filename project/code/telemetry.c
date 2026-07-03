/*********************************************************************************************************************
* File: telemetry.c
* Description: VOFA+ telemetry.
********************************************************************************************************************/

#include "telemetry.h"
#include "app_config.h"
#include "actuator_motor.h"
#include "control_balance.h"
#include "sensor_imu.h"

void telemetry_init(void)
{
}

void telemetry_update(uint32 now_ms)
{
#if APP_TELEMETRY_ENABLE
    static const uint8 tail[4] = {0x00, 0x00, 0x80, 0x7F};
    const wheel_feedback_struct *wheel;
    const motor_rpm_loop_diag_struct *rpm_diag;
#if APP_TELEMETRY_BALANCE_ENABLE
    const balance_diag_struct *balance;
    const imu_state_struct *imu;
    float vofa_data[28];
#else
    float vofa_data[8];
#endif

    wheel = actuator_motor_get_feedback();
    rpm_diag = actuator_motor_get_motor_rpm_loop_diag();

#if APP_TELEMETRY_BALANCE_ENABLE
    balance = control_balance_get_diag();
    imu = sensor_imu_get_state();
    vofa_data[0] = (float)now_ms;
    vofa_data[1] = (float)balance->mode;
    vofa_data[2] = imu->roll;
    vofa_data[3] = imu->pitch;
    vofa_data[4] = imu->yaw;
    vofa_data[5] = balance->pitch_rate_dps;
    vofa_data[6] = balance->balance_rpm;
    vofa_data[7] = (float)(wheel->online && wheel->left_online && wheel->right_online);
    vofa_data[8] = rpm_diag->left_motor_rpm;
    vofa_data[9] = rpm_diag->right_motor_rpm;
    vofa_data[10] = rpm_diag->left_duty;
    vofa_data[11] = rpm_diag->right_duty;
    vofa_data[12] = balance->pitch_kp;
    vofa_data[13] = balance->pitch_rate_kd;
    vofa_data[14] = balance->drive_forward_target_rpm;
    vofa_data[15] = balance->drive_forward_actual_rpm;
    vofa_data[16] = balance->drive_speed_pitch_offset_deg;
    vofa_data[17] = balance->pitch_setpoint_deg;
    vofa_data[18] = balance->drive_turn_target_dps;
    vofa_data[19] = balance->drive_gyro_z_dps;
    vofa_data[20] = balance->drive_turn_rpm;
    vofa_data[21] = balance->drive_gyro_z_raw_dps;
    vofa_data[22] = balance->drive_turn_error_dps;
    vofa_data[23] = balance->drive_turn_integral;
    vofa_data[24] = balance->drive_turn_kp;
    vofa_data[25] = balance->drive_turn_ki;
    vofa_data[26] = (float)balance->drive_imu_age_ms;
    vofa_data[27] = (float)balance->drive_wheel_age_ms;
#else
    vofa_data[0] = (float)now_ms;
    vofa_data[1] = (float)rpm_diag->mode;
    vofa_data[2] = rpm_diag->target_motor_rpm;
    vofa_data[3] = rpm_diag->left_motor_rpm;
    vofa_data[4] = rpm_diag->right_motor_rpm;
    vofa_data[5] = rpm_diag->left_duty;
    vofa_data[6] = rpm_diag->right_duty;
    vofa_data[7] = (float)(wheel->online && wheel->left_online && wheel->right_online);
#endif

    debug_send_buffer((const uint8 *)vofa_data, sizeof(vofa_data));
    debug_send_buffer(tail, sizeof(tail));
#else
    (void)now_ms;
#endif
}

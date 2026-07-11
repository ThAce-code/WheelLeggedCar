/*********************************************************************************************************************
* File: telemetry.c
* Description: VOFA+ telemetry.
********************************************************************************************************************/

#include "telemetry.h"
#include "app_config.h"
#include "actuator_motor.h"
#include "control_balance.h"
#include "control_leg.h"
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
    const leg_diag_struct *leg;
    const imu_state_struct *imu;
    float vofa_data[80];
#else
    float vofa_data[8];
#endif

    wheel = actuator_motor_get_feedback();
    rpm_diag = actuator_motor_get_motor_rpm_loop_diag();

#if APP_TELEMETRY_BALANCE_ENABLE
    balance = control_balance_get_diag();
    leg = control_leg_get_diag();
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
    vofa_data[28] = balance->drive_fast_blend;
    vofa_data[29] = balance->drive_speed_integral;
    vofa_data[30] = balance->drive_speed_pitch_limit_deg;
    vofa_data[31] = balance->drive_speed_ff_rpm;
    vofa_data[32] = balance->wheel_speed_ks;
    vofa_data[33] = balance->pitch_term_rpm;
    vofa_data[34] = balance->rate_term_rpm;
    vofa_data[35] = balance->speed_term_rpm;
    vofa_data[36] = balance->pos_term_rpm;
    vofa_data[37] = balance->ff_term_rpm;
    vofa_data[38] = (float)leg->mode;
    vofa_data[39] = leg->target_height_mm;
    vofa_data[40] = leg->actual_height_mm;
    vofa_data[41] = leg->height_norm;
    vofa_data[42] = leg->left_x_mm;
    vofa_data[43] = leg->left_y_mm;
    vofa_data[44] = leg->right_x_mm;
    vofa_data[45] = leg->right_y_mm;
    vofa_data[46] = (float)leg->ik_valid;
    vofa_data[47] = (float)leg->output_enable;
    vofa_data[48] = leg->servo_actual_deg[0];
    vofa_data[49] = leg->servo_actual_deg[1];
    vofa_data[50] = leg->servo_actual_deg[2];
    vofa_data[51] = leg->servo_actual_deg[3];
    vofa_data[52] = balance->balance_pitch_kp_eff;
    vofa_data[53] = balance->balance_pitch_rate_kd_eff;
    vofa_data[54] = balance->balance_wheel_speed_ks_eff;
    vofa_data[55] = balance->balance_pitch_setpoint_base_eff_deg;
    vofa_data[56] = balance->chassis_forward_limit_eff_rpm;
    vofa_data[57] = balance->chassis_fast_forward_limit_eff_rpm;
    vofa_data[58] = leg->height_ref_mm;
    vofa_data[59] = leg->height_rate_mm_s;
    vofa_data[60] = leg->ik_margin;
    vofa_data[61] = leg->drive_forward_limit_rpm;
    vofa_data[62] = (float)leg->motion_state;
    vofa_data[63] = (float)leg->fault_reason;
    vofa_data[64] = (float)leg->drive_allowed;
    vofa_data[65] = leg->servo_target_deg[0];
    vofa_data[66] = leg->servo_target_deg[1];
    vofa_data[67] = leg->servo_target_deg[2];
    vofa_data[68] = leg->servo_target_deg[3];
    vofa_data[69] = leg->servo_filtered_deg[0];
    vofa_data[70] = leg->servo_filtered_deg[1];
    vofa_data[71] = leg->servo_filtered_deg[2];
    vofa_data[72] = leg->servo_filtered_deg[3];
    vofa_data[73] = leg->servo_max_error_deg;
    vofa_data[74] = (float)leg->servo_settled;
    vofa_data[75] = leg->servo_s7_progress;
    vofa_data[76] = (float)leg->servo_direct_bypass;
    vofa_data[77] = (float)leg->servo_fast_mode;
    vofa_data[78] = (float)leg->servo_trajectory_mode;
    vofa_data[79] = (float)leg->servo_s7_remaining_ms;
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

/*********************************************************************************************************************
* File: telemetry.c
* Description: VOFA+ telemetry — 32-float compact frame for leg-servo validation.
*              At 460800 baud / 8N1, 132 bytes (32×4 + 4B tail) ≈ 2.87 ms.
*********************************************************************************************************************/

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
    float vofa_data[32];
#else
    float vofa_data[8];
#endif

    wheel = actuator_motor_get_feedback();
    rpm_diag = actuator_motor_get_motor_rpm_loop_diag();

#if APP_TELEMETRY_BALANCE_ENABLE
    balance = control_balance_get_diag();
    leg = control_leg_get_diag();
    imu = sensor_imu_get_state();

    /* 0-11: core motor / balance / IMU */
    vofa_data[0]  = (float)now_ms;
    vofa_data[1]  = (float)balance->mode;
    vofa_data[2]  = imu->roll;
    vofa_data[3]  = imu->pitch;
    vofa_data[4]  = imu->yaw;
    vofa_data[5]  = balance->pitch_rate_dps;
    vofa_data[6]  = balance->balance_rpm;
    vofa_data[7]  = (float)(wheel->online && wheel->left_online && wheel->right_online);
    vofa_data[8]  = rpm_diag->left_motor_rpm;
    vofa_data[9]  = rpm_diag->right_motor_rpm;
    vofa_data[10] = rpm_diag->left_duty;
    vofa_data[11] = rpm_diag->right_duty;

    /* 12-21: leg height / IK */
    vofa_data[12] = (float)leg->mode;
    vofa_data[13] = leg->target_height_mm;
    vofa_data[14] = leg->actual_height_mm;
    vofa_data[15] = leg->height_norm;
    vofa_data[16] = leg->left_x_mm;
    vofa_data[17] = leg->left_y_mm;
    vofa_data[18] = leg->right_x_mm;
    vofa_data[19] = leg->right_y_mm;
    vofa_data[20] = (float)leg->ik_valid;
    vofa_data[21] = (float)leg->output_enable;

    /* 22-25: servo output commands (open-loop PWM) */
    vofa_data[22] = leg->servo_actual_deg[0];
    vofa_data[23] = leg->servo_actual_deg[1];
    vofa_data[24] = leg->servo_actual_deg[2];
    vofa_data[25] = leg->servo_actual_deg[3];

    /* 26-31: leg motion state + settle diagnostics */
    vofa_data[26] = leg->height_ref_mm;
    vofa_data[27] = leg->height_rate_mm_s;
    vofa_data[28] = leg->ik_margin;
    vofa_data[29] = (float)leg->motion_state;
    vofa_data[30] = (float)leg->fault_reason;
    vofa_data[31] = (float)leg->drive_allowed;
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

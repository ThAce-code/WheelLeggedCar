/*********************************************************************************************************************
* File: telemetry.c
* Description: VOFA+ telemetry.
********************************************************************************************************************/

#include "telemetry.h"
#include "app_config.h"
#include "actuator_motor.h"
#include "control_balance.h"

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
    float vofa_data[14];
#else
    float vofa_data[8];
#endif

    wheel = actuator_motor_get_feedback();
    rpm_diag = actuator_motor_get_motor_rpm_loop_diag();

#if APP_TELEMETRY_BALANCE_ENABLE
    const balance_diag_struct *balance;

    balance = control_balance_get_diag();
    vofa_data[0] = (float)now_ms;
    vofa_data[1] = (float)balance->mode;
    vofa_data[2] = balance->pitch_deg;
    vofa_data[3] = balance->pitch_rate_dps;
    vofa_data[4] = balance->chassis_left_rpm;
    vofa_data[5] = balance->chassis_right_rpm;
    vofa_data[6] = balance->balance_rpm;
    vofa_data[7] = (float)wheel->online;
    vofa_data[8] = rpm_diag->left_motor_rpm;
    vofa_data[9] = rpm_diag->right_motor_rpm;
    vofa_data[10] = rpm_diag->left_duty;
    vofa_data[11] = rpm_diag->right_duty;
    vofa_data[12] = balance->pitch_kp;
    vofa_data[13] = balance->pitch_rate_kd;
#else
    vofa_data[0] = (float)now_ms;
    vofa_data[1] = (float)rpm_diag->mode;
    vofa_data[2] = rpm_diag->target_motor_rpm;
    vofa_data[3] = rpm_diag->left_motor_rpm;
    vofa_data[4] = rpm_diag->right_motor_rpm;
    vofa_data[5] = rpm_diag->left_duty;
    vofa_data[6] = rpm_diag->right_duty;
    vofa_data[7] = (float)wheel->online;
#endif

    debug_send_buffer((const uint8 *)vofa_data, sizeof(vofa_data));
    debug_send_buffer(tail, sizeof(tail));
#else
    (void)now_ms;
#endif
}

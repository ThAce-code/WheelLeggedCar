/*********************************************************************************************************************
* File: telemetry.c
* Description: VOFA+ telemetry.
********************************************************************************************************************/

#include "telemetry.h"
#include "app_config.h"
#include "actuator_motor.h"

void telemetry_init(void)
{
}

void telemetry_update(uint32 now_ms)
{
#if APP_TELEMETRY_ENABLE
    static const uint8 tail[4] = {0x00, 0x00, 0x80, 0x7F};
    const wheel_feedback_struct *wheel;
    const motor_rpm_loop_diag_struct *rpm_diag;
    float vofa_data[8];

    wheel = actuator_motor_get_feedback();
    rpm_diag = actuator_motor_get_motor_rpm_loop_diag();

    vofa_data[0] = (float)now_ms;
    vofa_data[1] = (float)rpm_diag->enable;
    vofa_data[2] = rpm_diag->target_motor_rpm;
    vofa_data[3] = rpm_diag->left_motor_rpm;
    vofa_data[4] = rpm_diag->right_motor_rpm;
    vofa_data[5] = rpm_diag->left_duty;
    vofa_data[6] = rpm_diag->right_duty;
    vofa_data[7] = (float)wheel->online;

    debug_send_buffer((const uint8 *)vofa_data, sizeof(vofa_data));
    debug_send_buffer(tail, sizeof(tail));
#else
    (void)now_ms;
#endif
}

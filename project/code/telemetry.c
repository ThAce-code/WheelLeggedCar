/*********************************************************************************************************************
* File: telemetry.c
* Description: VOFA+ telemetry with fixed GNSS, balance/race, and motor profiles.
*              At 460800 baud / 8N1, the largest 72-float frame takes about 6.34 ms every 20 ms.
*********************************************************************************************************************/

#include "telemetry.h"
#include "app_config.h"
#include "actuator_motor.h"
#include "actuator_servo.h"
#include "app_scheduler.h"
#include "control_balance.h"
#include "control_chassis.h"
#include "control_leg.h"
#include "gnss_types.h"
#include "intercore_control.h"
#include "sensor_imu.h"

static const uint8 telemetry_tail[4] = {0x00, 0x00, 0x80, 0x7F};
#if (APP_TELEMETRY_PROFILE == APP_TELEMETRY_PROFILE_GNSS)
static float vofa_data[20];
#elif APP_TELEMETRY_BALANCE_ENABLE
static float vofa_data[72];
#else
static float vofa_data[8];
#endif
static uint32 telemetry_tx_offset;
static uint8 telemetry_tx_busy;
static uint32 telemetry_frame_sequence;
static uint32 telemetry_drop_count;

void telemetry_init(void)
{
    telemetry_tx_offset = 0U;
    telemetry_tx_busy = APP_FALSE;
    telemetry_frame_sequence = 0U;
    telemetry_drop_count = 0U;
}

void telemetry_update(uint32 now_ms)
{
#if APP_TELEMETRY_ENABLE
    const wheel_feedback_struct *wheel;
    const motor_rpm_loop_diag_struct *rpm_diag;
#if (APP_TELEMETRY_PROFILE == APP_TELEMETRY_PROFILE_GNSS)
    intercore_gnss_payload_struct gps = {0};
    uint32 gps_received_ms = 0U;
    uint32 gps_age_ms = 0xFFFFFFFFUL;
    uint8 gps_available;
#elif APP_TELEMETRY_BALANCE_ENABLE
    const balance_diag_struct *balance;
    const chassis_output_struct *chassis;
    const leg_diag_struct *leg;
    const imu_state_struct *imu;
    uint32 pose_status_flags;
#endif

    if(APP_TRUE == telemetry_tx_busy)
    {
        telemetry_drop_count++;
        return;
    }

    wheel = actuator_motor_get_feedback();
    rpm_diag = actuator_motor_get_motor_rpm_loop_diag();

#if (APP_TELEMETRY_PROFILE == APP_TELEMETRY_PROFILE_GNSS)
    gps_available = intercore_control_get_latest_gnss(&gps, &gps_received_ms);
    if(0U != gps_available)
    {
        gps_age_ms = now_ms - gps_received_ms;
    }

    vofa_data[0]  = (float)now_ms;
    vofa_data[1]  = (float)telemetry_frame_sequence;
    vofa_data[2]  = (float)telemetry_drop_count;
    vofa_data[3]  = (float)((0U != gps_available) && (0U != gps.fix_valid) &&
                            (GNSS_SNAPSHOT_MAX_AGE_MS >= gps_age_ms));
    vofa_data[4]  = (float)gps.origin_valid;
    vofa_data[5]  = gps.local_x_m;
    vofa_data[6]  = gps.local_y_m;
    vofa_data[7]  = (float)gps.satellite_count;
    vofa_data[8]  = gps.hdop;
    vofa_data[9]  = (float)gps_age_ms;
    vofa_data[10] = gps.speed_mps;
    vofa_data[11] = gps.course_deg;
    vofa_data[12] = gps.position_sigma_m;
    vofa_data[13] = (float)gps.fix_quality;
    vofa_data[14] = (float)gps.checksum_error_count;
    vofa_data[15] = (float)gps.timeout_count;
    vofa_data[16] = (float)app_scheduler_get_missed_tick_count();
    vofa_data[17] = (float)app_scheduler_get_max_gap_ms();
    vofa_data[18] = (float)(wheel->online && wheel->left_online && wheel->right_online);
    vofa_data[19] = (float)rpm_diag->mode;
    telemetry_frame_sequence++;
#elif APP_TELEMETRY_BALANCE_ENABLE
    balance = control_balance_get_diag();
    chassis = control_chassis_get_output();
    leg = control_leg_get_diag();
    imu = sensor_imu_get_state();
    pose_status_flags = 0U;
    if(APP_TRUE == leg->ik_valid)
    {
        pose_status_flags |= LEG_POSE_STATUS_IK_VALID;
    }
    if(APP_TRUE == leg->left_command_pose_body_mm.valid)
    {
        pose_status_flags |= LEG_POSE_STATUS_LEFT_VALID;
    }
    if(APP_TRUE == leg->right_command_pose_body_mm.valid)
    {
        pose_status_flags |= LEG_POSE_STATUS_RIGHT_VALID;
    }
    if(LEG_POSE_SOURCE_MEASURED_CALIBRATION == leg->left_command_pose_body_mm.source)
    {
        pose_status_flags |= LEG_POSE_STATUS_LEFT_MEASURED;
    }
    if(LEG_POSE_SOURCE_MIRROR_ASSUMPTION == leg->right_command_pose_body_mm.source)
    {
        pose_status_flags |= LEG_POSE_STATUS_RIGHT_MIRROR;
    }

    /* 0-11: core motor / balance / IMU */
    vofa_data[0]  = (float)now_ms;
    vofa_data[1]  = (float)balance->mode;
    vofa_data[2]  = imu->roll;
    vofa_data[3]  = imu->pitch;
    vofa_data[4]  = imu->yaw;
    vofa_data[5]  = balance->pitch_rate_dps;
    vofa_data[6]  = balance->balance_rpm;
    vofa_data[7]  = (float)(wheel->online && wheel->left_online && wheel->right_online);
    /* Installed-car wire contract: channel 2 (I9/I11) is physical left. */
    vofa_data[8]  = rpm_diag->right_motor_rpm;
    vofa_data[9]  = rpm_diag->left_motor_rpm;
    vofa_data[10] = rpm_diag->right_duty;
    vofa_data[11] = rpm_diag->left_duty;

    /* 12-17: legacy leg stance / IK */
    vofa_data[12] = (float)leg->mode;
    vofa_data[13] = leg->legacy_stance_target_units;
    vofa_data[14] = leg->legacy_stance_ref_units;
    vofa_data[15] = leg->legacy_stance_norm;
    vofa_data[16] = (float)pose_status_flags;
    vofa_data[17] = (float)leg->output_enable;

    /* 18-21: servo output commands (open-loop PWM) */
    vofa_data[18] = leg->servo_actual_deg[0];
    vofa_data[19] = leg->servo_actual_deg[1];
    vofa_data[20] = leg->servo_actual_deg[2];
    vofa_data[21] = leg->servo_actual_deg[3];

    /* 22-25: servo planner targets */
    vofa_data[22] = leg->servo_target_deg[0];
    vofa_data[23] = leg->servo_target_deg[1];
    vofa_data[24] = leg->servo_target_deg[2];
    vofa_data[25] = leg->servo_target_deg[3];

    /* 26-29: servo LPF filtered angles */
    vofa_data[26] = leg->servo_filtered_deg[0];
    vofa_data[27] = leg->servo_filtered_deg[1];
    vofa_data[28] = leg->servo_filtered_deg[2];
    vofa_data[29] = leg->servo_filtered_deg[3];

    /* 30-32: servo settle diagnostics */
    vofa_data[30] = leg->servo_max_error_deg;
    vofa_data[31] = (float)leg->servo_settled;
    vofa_data[32] = leg->servo_s7_progress;

    /* 33-37: physical command poses and IK margin */
    vofa_data[33] = leg->left_command_pose_body_mm.x_mm;
    vofa_data[34] = leg->left_command_pose_body_mm.y_mm;
    vofa_data[35] = leg->right_command_pose_body_mm.x_mm;
    vofa_data[36] = leg->right_command_pose_body_mm.y_mm;
    vofa_data[37] = leg->ik_margin;

    /* 38-39: leg motion state */
    vofa_data[38] = (float)leg->motion_state;
    vofa_data[39] = (float)leg->fault_reason;

    /* 40-45: safety permission + actuator trajectory state */
    vofa_data[40] = leg->drive_forward_limit_rpm;
    vofa_data[41] = (float)leg->drive_allowed;
    vofa_data[42] = (float)leg->servo_fast_mode;
    vofa_data[43] = (float)leg->servo_direct_bypass;
    vofa_data[44] = (float)leg->servo_trajectory_mode;
    vofa_data[45] = (float)leg->servo_s7_remaining_ms;

    /* 46-54: timing and sample-integrity diagnostics */
    vofa_data[46] = (float)telemetry_frame_sequence;
    vofa_data[47] = (float)telemetry_drop_count;
    vofa_data[48] = (float)app_scheduler_get_missed_tick_count();
    vofa_data[49] = (float)app_scheduler_get_max_gap_ms();
    vofa_data[50] = (float)actuator_servo_get_tick_count();
    vofa_data[51] = (float)sensor_imu_get_int_count();
    vofa_data[52] = (float)sensor_imu_get_invalid_sample_count();
    vofa_data[53] = (float)(now_ms - imu->timestamp_ms);
    vofa_data[54] = imu->gyro_y_dps;

    /* 55-71: race-assist and bounded-drive diagnostics from this scheduler snapshot */
    vofa_data[55] = (float)chassis->race_assist_enable;
    vofa_data[56] = (float)chassis->race_assist_level;
    vofa_data[57] = (float)chassis->race_assist_state;
    vofa_data[58] = (float)chassis->race_assist_fault_reason;
    vofa_data[59] = chassis->race_u_request;
    vofa_data[60] = leg->race_assist_actual;
    vofa_data[61] = chassis->requested_accel_rpm_s;
    vofa_data[62] = chassis->forward_target_rpm;
    vofa_data[63] = chassis->forward_ramped_rpm;
    vofa_data[64] = chassis->wheel_speed_measured_rpm;
    vofa_data[65] = chassis->speed_error_rpm;
    vofa_data[66] = balance->pitch_setpoint_deg;
    vofa_data[67] = balance->balance_output_limit_rpm;
    vofa_data[68] = chassis->race_turn_scale;
    vofa_data[69] = leg->left_ik_margin;
    vofa_data[70] = leg->right_ik_margin;
    vofa_data[71] = (float)leg->ik_branch_flags;
    telemetry_frame_sequence++;
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

    telemetry_tx_offset = 0U;
    telemetry_tx_busy = APP_TRUE;
#else
    (void)now_ms;
#endif
}

void telemetry_service(void)
{
#if APP_TELEMETRY_ENABLE
    uint32 payload_size;
    uint32 tail_offset;
    uint32 written;

    if(APP_TRUE != telemetry_tx_busy)
    {
        return;
    }

    payload_size = (uint32)sizeof(vofa_data);
    if(telemetry_tx_offset < payload_size)
    {
        written = Cy_SCB_WriteArray(get_scb_module(DEBUG_UART_INDEX),
                                    (void *)(((uint8 *)vofa_data) + telemetry_tx_offset),
                                    payload_size - telemetry_tx_offset);
        telemetry_tx_offset += written;
        if(telemetry_tx_offset < payload_size)
        {
            return;
        }
    }

    tail_offset = telemetry_tx_offset - payload_size;
    if(tail_offset < (uint32)sizeof(telemetry_tail))
    {
        written = Cy_SCB_WriteArray(get_scb_module(DEBUG_UART_INDEX),
                                    (void *)(telemetry_tail + tail_offset),
                                    (uint32)sizeof(telemetry_tail) - tail_offset);
        telemetry_tx_offset += written;
        tail_offset += written;
    }

    if(tail_offset >= (uint32)sizeof(telemetry_tail))
    {
        telemetry_tx_busy = APP_FALSE;
    }
#endif
}

/*********************************************************************************************************************
* File: app_types.h
* Description: Shared data snapshots and command types.
********************************************************************************************************************/

#ifndef _app_types_h_
#define _app_types_h_

#include "zf_common_headfile.h"

typedef enum
{
    APP_FALSE = 0,
    APP_TRUE = 1
}app_bool_enum;

typedef enum
{
    APP_STATUS_OK = 0,
    APP_STATUS_ERROR = 1
}app_status_enum;

typedef enum
{
    APP_STATE_BOOT = 0,
    APP_STATE_CALIBRATE,
    APP_STATE_STANDBY,
    APP_STATE_RUN,
    APP_STATE_FAULT
}app_run_state_enum;

typedef enum
{
    MOTOR_MODE_STOP = 0,
    MOTOR_MODE_RPM_CLOSED_LOOP = 1,
    MOTOR_MODE_OPEN_DUTY = 2
}motor_mode_enum;

typedef struct
{
    uint32 timestamp_ms;
    float roll;
    float pitch;
    float yaw;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float pitch_rate_dps;
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;
    uint8 healthy;
}imu_state_struct;

typedef struct
{
    float left_target_motor_rpm;
    float right_target_motor_rpm;
    uint8 enable;
}motor_cmd_struct;

typedef struct
{
    motor_mode_enum mode;
    float left_motor_rpm;
    float right_motor_rpm;
    float left_open_duty;
    float right_open_duty;
    uint8 enable;
}motor_actuator_cmd_struct;

#define MOTOR_DIAG_ASCII_LINE_MAX       (64U)

typedef struct
{
    int16 left_motor_rpm;
    int16 right_motor_rpm;
    int16 left_reduced_angle;
    int16 right_reduced_angle;
    uint32 last_rx_ms;
    uint32 age_ms;
    uint8 online;
    uint8 left_online;
    uint8 right_online;
}wheel_feedback_struct;

typedef struct
{
    int16 left_raw_angle;
    int16 right_raw_angle;
    int16 last_tx_left;
    int16 last_tx_right;
    uint32 checksum_error_count;
    uint32 feedback_range_error_count;
    uint32 unknown_frame_count;
    uint32 tx_frame_count;
    uint8 last_tx_func;
    char last_unknown_ascii[MOTOR_DIAG_ASCII_LINE_MAX];
}motor_diag_struct;

typedef struct
{
    uint8 enable;
    motor_mode_enum mode;
    uint8 host_motion_active;
    uint32 host_motion_age_ms;
    float target_motor_rpm;
    float left_target_motor_rpm;
    float right_target_motor_rpm;
    float left_motor_rpm;
    float right_motor_rpm;
    float left_rpm_error;
    float right_rpm_error;
    float left_duty;
    float right_duty;
    float left_integral;
    float right_integral;
    float left_kp;
    float left_ki;
    float left_kd;
    float right_kp;
    float right_ki;
    float right_kd;
    uint32 command_error_count;
}motor_rpm_loop_diag_struct;

typedef enum
{
    BALANCE_MODE_OFF = 0,
    BALANCE_MODE_STANDBY,
    BALANCE_MODE_BALANCE_TEST,
    BALANCE_MODE_BALANCE_FAST
}balance_mode_enum;

typedef struct
{
    float target_forward_rpm;
    float target_turn_dps;
    float actual_forward_rpm;
    float actual_turn_dps;
    float speed_pitch_offset_deg;
    float turn_rpm;
    float speed_kp;
    float speed_ki;
    float turn_kp;
    float speed_integral;
    float turn_ki;
    float turn_integral;
    float fast_blend;
    float speed_pitch_limit_deg;
    float speed_ff_rpm;
    uint8 fast_enable;
    uint8 enable;
    uint32 last_cmd_ms;
    uint32 last_update_ms;
}chassis_cmd_struct;

typedef struct
{
    float pitch_offset_deg;
    float turn_rpm;
    float forward_target_rpm;
    float forward_actual_rpm;
    float turn_target_dps;
    float gyro_z_dps;
    float gyro_z_raw_dps;
    float gyro_z_filtered_dps;
    float turn_error_dps;
    float turn_integral;
    float turn_kp;
    float turn_ki;
    float fast_blend;
    float speed_integral;
    float speed_pitch_limit_deg;
    float speed_ff_rpm;
    float forward_limit_eff_rpm;
    float fast_forward_limit_eff_rpm;
    float wheel_speed_measured_rpm;
    float speed_error_rpm;
    float requested_accel_rpm_s;
    float race_u_request;
    float race_balance_limit_rpm;
    float race_turn_scale;
    uint32 imu_age_ms;
    uint32 wheel_age_ms;
    uint8 race_assist_enable;
    uint8 race_assist_level;
    uint8 race_assist_state;
    uint8 race_assist_fault_reason;
    uint8 enable;
}chassis_output_struct;

typedef struct
{
    balance_mode_enum mode;
    float pitch_deg;
    float pitch_rate_dps;
    float chassis_left_rpm;
    float chassis_right_rpm;
    float balance_rpm;
    float balance_output_limit_rpm;
    float output_left_rpm;
    float output_right_rpm;
    float pitch_kp;
    float pitch_rate_kd;
    float wheel_speed_rpm;
    float wheel_pos_rev;
    float wheel_speed_ks;
    float wheel_pos_kp;
    float pitch_setpoint_deg;
    float drive_forward_target_rpm;
    float drive_forward_actual_rpm;
    float drive_speed_pitch_offset_deg;
    float drive_turn_target_dps;
    float drive_gyro_z_dps;
    float drive_turn_rpm;
    float drive_gyro_z_raw_dps;
    float drive_gyro_z_filtered_dps;
    float drive_turn_error_dps;
    float drive_turn_integral;
    float drive_turn_kp;
    float drive_turn_ki;
    float drive_fast_blend;
    float drive_speed_integral;
    float drive_speed_pitch_limit_deg;
    float drive_speed_ff_rpm;
    float pitch_term_rpm;
    float rate_term_rpm;
    float speed_term_rpm;
    float pos_term_rpm;
    float ff_term_rpm;
    float legacy_stance_norm;
    float balance_pitch_kp_eff;
    float balance_pitch_rate_kd_eff;
    float balance_wheel_speed_ks_eff;
    float balance_pitch_setpoint_base_eff_deg;
    float chassis_forward_limit_eff_rpm;
    float chassis_fast_forward_limit_eff_rpm;
    uint32 drive_imu_age_ms;
    uint32 drive_wheel_age_ms;
    uint8 output_enable;
    uint8 safety_blocked;
}balance_diag_struct;

typedef struct
{
    float angle_deg[4];
    uint8 enable[4];
}servo_cmd_struct;

typedef struct
{
    float target_deg[4];
    float filtered_deg[4];
    float output_deg[4];
    float max_error_deg;
    uint8 settled;
    uint8 fast_mode;
    uint8 direct_bypass;
}actuator_servo_diag_struct;

typedef enum
{
    LEG_MOTION_LOCKED = 0,
    LEG_MOTION_STABLE,
    LEG_MOTION_TRANSITION,
    LEG_MOTION_FAULT,
    LEG_MOTION_RACE_ASSIST,
    LEG_MOTION_RACE_FAULT_HOLD
}leg_motion_state_enum;

typedef enum
{
    LEG_FAULT_NONE = 0,
    LEG_FAULT_IK_INVALID,
    LEG_FAULT_IK_MARGIN,
    LEG_FAULT_SERVO_LIMIT
}leg_fault_reason_enum;

typedef enum
{
    LEG_POSE_SOURCE_NONE = 0,
    LEG_POSE_SOURCE_MEASURED_CALIBRATION = 1,
    LEG_POSE_SOURCE_MIRROR_ASSUMPTION = 2
}leg_pose_source_enum;

typedef enum
{
    LEG_POSE_STATUS_IK_VALID = (1U << 0),
    LEG_POSE_STATUS_LEFT_VALID = (1U << 1),
    LEG_POSE_STATUS_RIGHT_VALID = (1U << 2),
    LEG_POSE_STATUS_LEFT_MEASURED = (1U << 3),
    LEG_POSE_STATUS_RIGHT_MIRROR = (1U << 4)
}leg_pose_status_flag_enum;

typedef struct
{
    float x_mm;
    float y_mm;
    leg_pose_source_enum source;
    uint8 valid;
}leg_pose_command_estimate_struct;

typedef struct
{
    float legacy_stance_target_units;
    float legacy_stance_ref_units;
    float legacy_stance_rate_units_s;
    float legacy_stance_norm;
    float ik_margin;
    leg_pose_command_estimate_struct left_command_pose_body_mm;
    leg_pose_command_estimate_struct right_command_pose_body_mm;
    float servo_target_deg[4];
    float servo_actual_deg[4];
    float servo_filtered_deg[4];
    float servo_max_error_deg;
    float servo_s7_progress;
    uint32 servo_s7_remaining_ms;
    uint8 servo_settled;
    uint8 servo_fast_mode;
    uint8 servo_direct_bypass;
    uint8 servo_trajectory_mode;
    float drive_forward_limit_rpm;
    uint8 mode;
    uint8 ik_valid;
    uint8 output_enable;
    uint8 drive_allowed;
    leg_motion_state_enum motion_state;
    leg_fault_reason_enum fault_reason;
    uint32 ik_error_count;
    float race_assist_request;
    float race_assist_actual;
    float race_target_x_mm;
    float race_target_y_mm;
    float left_ik_margin;
    float right_ik_margin;
    uint8 ik_branch_flags;
    uint8 race_path_valid;
}leg_diag_struct;

#endif

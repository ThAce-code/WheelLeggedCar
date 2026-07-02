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
    BALANCE_MODE_BALANCE_TEST
}balance_mode_enum;

typedef struct
{
    float target_forward_rpm;
    float target_turn_rpm;
    float actual_forward_rpm;
    float actual_turn_rpm;
    uint8 enable;
    uint32 last_cmd_ms;
    uint32 last_update_ms;
}chassis_cmd_struct;

typedef struct
{
    float left_base_rpm;
    float right_base_rpm;
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
    float output_left_rpm;
    float output_right_rpm;
    float pitch_kp;
    float pitch_rate_kd;
    float wheel_speed_rpm;
    float wheel_pos_rev;
    float wheel_speed_ks;
    float wheel_pos_kp;
    uint8 output_enable;
    uint8 safety_blocked;
}balance_diag_struct;

typedef struct
{
    float angle_deg[4];
    uint8 enable[4];
}servo_cmd_struct;

#endif

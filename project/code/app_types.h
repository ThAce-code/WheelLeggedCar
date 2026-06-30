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

typedef struct
{
    uint32 timestamp_ms;
    float roll;
    float pitch;
    float yaw;
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;
    uint8 healthy;
}imu_state_struct;

typedef struct
{
    float left_target;
    float right_target;
    uint8 enable;
}motor_cmd_struct;

#define MOTOR_DIAG_ASCII_LINE_MAX       (64U)

typedef struct
{
    int16 left_speed;
    int16 right_speed;
    int16 left_reduced_angle;
    int16 right_reduced_angle;
    uint32 last_rx_ms;
    uint32 age_ms;
    uint8 online;
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
    float angle_deg[4];
    uint8 enable[4];
}servo_cmd_struct;

#endif

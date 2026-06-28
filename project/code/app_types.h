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
    uint32 timestamp_ms;
    int16 left_count;
    int16 right_count;
    float left_speed;
    float right_speed;
    uint8 healthy;
}wheel_state_struct;

typedef struct
{
    uint32 timestamp_ms;
    uint8 frame_ready;
    uint8 healthy;
}camera_state_struct;

typedef struct
{
    uint32 timestamp_ms;
    float lateral_error;
    float heading_error;
    uint8 confidence;
    uint8 healthy;
}vision_state_struct;

typedef struct
{
    uint32 timestamp_ms;
    float roll;
    float pitch;
    float yaw;
    float left_speed;
    float right_speed;
    uint8 healthy;
}vehicle_state_struct;

typedef struct
{
    float left_target;
    float right_target;
    uint8 enable;
}motor_cmd_struct;

typedef struct
{
    float angle_deg[4];
    uint8 enable[4];
}servo_cmd_struct;

#endif

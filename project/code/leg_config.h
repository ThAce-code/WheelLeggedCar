/*********************************************************************************************************************
* File: leg_config.h
* Description: Static configuration for the four wheel-leg servos.
********************************************************************************************************************/

#ifndef _leg_config_h_
#define _leg_config_h_

#include "zf_common_headfile.h"

typedef enum
{
    LEG_SERVO_FL = 0,
    LEG_SERVO_FR = 1,
    LEG_SERVO_RL = 2,
    LEG_SERVO_RR = 3,
    LEG_SERVO_COUNT = 4
}leg_servo_id_enum;

typedef struct
{
    uint8  servo_index;
    float  safe_deg;
    float  neutral_deg;
    float  min_deg;
    float  max_deg;
    float  direction;
    float  mount_x;
    float  mount_y;
}leg_servo_config_struct;

typedef enum
{
    LEG_IK_BRANCH_PLUS = 0,
    LEG_IK_BRANCH_MINUS = 1
}leg_ik_branch_enum;

typedef struct
{
    float l1_mm;
    float l2_mm;
    float l3_mm;
    float l4_mm;
    float l5_mm;
    float x_min_mm;
    float x_max_mm;
    float y_min_mm;
    float y_max_mm;
    float x_offset_mm;
    float y_offset_mm;
    leg_ik_branch_enum left_alpha_branch;
    leg_ik_branch_enum left_beta_branch;
    leg_ik_branch_enum right_alpha_branch;
    leg_ik_branch_enum right_beta_branch;
}leg_kinematics_config_struct;

typedef struct
{
    float low_height_mm;
    float high_height_mm;
    float default_height_mm;
    float max_height_speed_mm_s;
    float max_height_accel_mm_s2;
    float max_height_jerk_mm_s3;
    float height_position_kp_s;
    float height_rate_kp_s;
    float height_settle_error_mm;
    uint32 height_settle_ms;
    uint32 fast_height_transition_ms;
    float ik_min_margin;
    float safe_support_height_mm;
    float transition_forward_limit_rpm;
    float balance_pitch_kp_low;
    float balance_pitch_kp_high;
    float balance_pitch_rate_kd_low;
    float balance_pitch_rate_kd_high;
    float balance_wheel_speed_ks_low;
    float balance_wheel_speed_ks_high;
    float balance_pitch_setpoint_low_deg;
    float balance_pitch_setpoint_high_deg;
    float chassis_forward_limit_low_rpm;
    float chassis_forward_limit_high_rpm;
    float chassis_fast_forward_limit_low_rpm;
    float chassis_fast_forward_limit_high_rpm;
}leg_height_profile_struct;

typedef struct
{
    leg_servo_config_struct servo[LEG_SERVO_COUNT];
    leg_kinematics_config_struct kinematics;
    leg_height_profile_struct height_profile;
    float  height_min;
    float  height_max;
    float  pitch_limit;
    float  roll_limit;
}leg_config_struct;

const leg_config_struct           *leg_config_get(void);
const leg_servo_config_struct     *leg_config_get_servo(uint8 leg_id);
const leg_kinematics_config_struct *leg_config_get_kinematics(void);
const leg_height_profile_struct   *leg_config_get_height_profile(void);

#endif

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
    float  ik_offset_deg;
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
    float physical_reference_x_mm;
    float physical_reference_y_mm;
    float alpha_reference_deg;
    float beta_reference_deg;
    float model_reference_x_mm;
    float model_reference_y_mm;
    float model_to_physical_scale;
    float model_to_physical_m00;
    float model_to_physical_m01;
    float model_to_physical_m10;
    float model_to_physical_m11;
    leg_ik_branch_enum left_alpha_branch;
    leg_ik_branch_enum left_beta_branch;
    leg_ik_branch_enum right_alpha_branch;
    leg_ik_branch_enum right_beta_branch;
}leg_kinematics_config_struct;

typedef struct
{
    float legacy_low_units;
    float legacy_high_units;
    float legacy_default_units;
    float legacy_max_rate_units_s;
    float legacy_max_accel_units_s2;
    float legacy_max_jerk_units_s3;
    float legacy_position_kp_s;
    float legacy_rate_kp_s;
    float legacy_settle_error_units;
    uint32 legacy_settle_ms;
    uint32 fast_stance_transition_ms;
    float ik_min_margin;
    float legacy_safe_support_units;
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
}leg_stance_profile_struct;

typedef struct
{
    leg_servo_config_struct servo[LEG_SERVO_COUNT];
    leg_kinematics_config_struct kinematics;
    leg_stance_profile_struct stance_profile;
    float  legacy_body_min_units;
    float  legacy_body_max_units;
    float  pitch_limit;
    float  roll_limit;
}leg_config_struct;

const leg_config_struct           *leg_config_get(void);
const leg_servo_config_struct     *leg_config_get_servo(uint8 leg_id);
const leg_kinematics_config_struct *leg_config_get_kinematics(void);
const leg_stance_profile_struct   *leg_config_get_stance_profile(void);

#endif

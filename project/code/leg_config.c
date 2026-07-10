/*********************************************************************************************************************
* File: leg_config.c
* Description: Default wheel-leg servo configuration.
********************************************************************************************************************/

#include "leg_config.h"

static const leg_config_struct leg_config_default =
{
    {
        {0,  90.0f,  90.0f, 10.0f, 175.0f,  1.0f,  1.0f,  1.0f},
        {1,  90.0f,  90.0f, 10.0f, 175.0f, -1.0f,  1.0f, -1.0f},
        {2,  90.0f,  90.0f, 10.0f, 175.0f,  1.0f, -1.0f,  1.0f},
        {3,  90.0f,  90.0f, 10.0f, 175.0f, -1.0f, -1.0f, -1.0f}
    },
    {
        90.0f,   /* L1 — driven link (temp for calib) */
        60.0f,   /* L2 — passive link (temp for calib) */
        60.0f,   /* L3 — passive link (temp for calib) */
        90.0f,   /* L4 — driven link (temp for calib) */
        38.0f,   /* L5 — base spacing (measured) */
        -35.0f,
        35.0f,
        35.0f,
        150.0f,
        0.0f,
        0.0f,
        LEG_IK_BRANCH_PLUS,   /* left alpha */
        LEG_IK_BRANCH_MINUS,  /* left beta  */
        LEG_IK_BRANCH_PLUS,   /* right alpha */
        LEG_IK_BRANCH_MINUS   /* right beta  */
    },
    {
        .low_height_mm = 30.0f,
        .high_height_mm = 80.0f,
        .default_height_mm = 55.0f,
        .max_height_speed_mm_s = 20.0f,
        .max_height_accel_mm_s2 = 20.0f,
        .max_height_jerk_mm_s3 = 80.0f,
        .height_position_kp_s = 2.0f,
        .height_rate_kp_s = 4.0f,
        .height_settle_error_mm = 1.0f,
        .height_settle_ms = 300U,
        .fast_height_transition_ms = 500U,
        .ik_min_margin = 0.20f,
        .safe_support_height_mm = 55.0f,
        .transition_forward_limit_rpm = 30.0f,
        .balance_pitch_kp_low = 18.0f,
        .balance_pitch_kp_high = 22.0f,
        .balance_pitch_rate_kd_low = 8.0f,
        .balance_pitch_rate_kd_high = 10.0f,
        .balance_wheel_speed_ks_low = 3.0f,
        .balance_wheel_speed_ks_high = 2.0f,
        .balance_pitch_setpoint_low_deg = -1.35f,
        .balance_pitch_setpoint_high_deg = -1.35f,
        .chassis_forward_limit_low_rpm = 80.0f,
        .chassis_forward_limit_high_rpm = 40.0f,
        .chassis_fast_forward_limit_low_rpm = 220.0f,
        .chassis_fast_forward_limit_high_rpm = 120.0f
    },
    45.0f,
    65.0f,
    30.0f,
    30.0f
};

const leg_config_struct *leg_config_get(void)
{
    return &leg_config_default;
}

const leg_servo_config_struct *leg_config_get_servo(uint8 leg_id)
{
    if(LEG_SERVO_COUNT <= leg_id)
    {
        return NULL;
    }
    return &leg_config_default.servo[leg_id];
}

const leg_kinematics_config_struct *leg_config_get_kinematics(void)
{
    return &leg_config_default.kinematics;
}

const leg_height_profile_struct *leg_config_get_height_profile(void)
{
    return &leg_config_default.height_profile;
}

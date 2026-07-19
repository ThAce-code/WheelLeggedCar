/*********************************************************************************************************************
* File: leg_config.c
* Description: Default wheel-leg servo configuration.
********************************************************************************************************************/

#include "leg_config.h"

static const leg_config_struct leg_config_default =
{
    {
        {0,  90.0f,  90.0f, 10.0f, 175.0f, -1.0f, 0.0f,  1.0f,  1.0f},
        {1,  90.0f,  90.0f, 10.0f, 175.0f,  1.0f, 0.0f,  1.0f, -1.0f},
        {2,  90.0f,  90.0f, 10.0f, 175.0f, -1.0f, 0.0f, -1.0f,  1.0f},
        {3,  90.0f,  90.0f, 10.0f, 175.0f,  1.0f, 0.0f, -1.0f, -1.0f}
    },
    {
        .l1_mm = 60.0f,   /* driven link (SolidWorks measured) */
        .l2_mm = 90.0f,   /* passive link (SolidWorks measured) */
        .l3_mm = 90.0f,   /* passive link (SolidWorks measured) */
        .l4_mm = 60.0f,   /* driven link (SolidWorks measured) */
        .l5_mm = 37.0f,   /* servo-axis spacing (SolidWorks measured) */
        /* Five-bar model-space bounds; physical LXY uses the hull below. */
        .x_min_mm = 10.0f,
        .x_max_mm = 50.0f,
        .y_min_mm = 23.0f,
        .y_max_mm = 100.0f,
        .physical_reference_x_mm = -20.766667f,
        .physical_reference_y_mm = 47.356667f,
        .alpha_reference_deg = 170.536799f,
        .beta_reference_deg = -4.081158f,
        .model_reference_x_mm = 22.830129f,
        .model_reference_y_mm = 46.929213f,
        .model_to_physical_scale = 0.955219899f,
        .model_to_physical_m00 = -0.996313812f,
        .model_to_physical_m01 = 0.085783378f,
        .model_to_physical_m10 = 0.085783378f,
        .model_to_physical_m11 = 0.996313812f,
        .physical_workspace =
        {
            {-40.620f, 47.370f},
            {-30.910f, 39.630f},
            {-20.380f, 32.170f},
            {-15.040f, 47.600f},
            {-22.030f, 88.490f},
            {-31.420f, 74.120f},
            {-37.940f, 59.340f},
            {-39.580f, 53.010f}
        },
        .physical_workspace_inset_mm = 2.0f,
        .experimental_race_enable = 1U,
        .experimental_race_x_mm = -18.831f,
        .experimental_race_y_mm = 25.076f,
        .experimental_race_tolerance_x_mm = 0.5f,
        .experimental_race_tolerance_y_mm = 0.5f,
        .experimental_race_ik_min_margin = 0.02f,
        .experimental_race_alpha_branch = LEG_IK_BRANCH_PLUS,
        .experimental_race_beta_branch = LEG_IK_BRANCH_PLUS,
        .left_alpha_branch = LEG_IK_BRANCH_PLUS,
        .left_beta_branch = LEG_IK_BRANCH_MINUS,
        .right_alpha_branch = LEG_IK_BRANCH_PLUS,
        .right_beta_branch = LEG_IK_BRANCH_MINUS
    },
    {
        .legacy_low_units = 30.0f,
        .legacy_high_units = 80.0f,
        .legacy_default_units = 55.0f,
        .legacy_max_rate_units_s = 20.0f,
        .legacy_max_accel_units_s2 = 20.0f,
        .legacy_max_jerk_units_s3 = 80.0f,
        .legacy_position_kp_s = 2.0f,
        .legacy_rate_kp_s = 4.0f,
        .legacy_settle_error_units = 1.0f,
        .legacy_settle_ms = 300U,
        .fast_stance_transition_ms = 500U,
        .ik_min_margin = 0.02f,
        .legacy_safe_support_units = 55.0f,
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

const leg_stance_profile_struct *leg_config_get_stance_profile(void)
{
    return &leg_config_default.stance_profile;
}

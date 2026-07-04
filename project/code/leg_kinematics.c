/*********************************************************************************************************************
* File: leg_kinematics.c
* Description: Five-bar wheel-leg inverse kinematics.
********************************************************************************************************************/

#include "leg_kinematics.h"
#include "app_config.h"
#include <math.h>

#define LEG_KINEMATICS_PI        (3.14159265358979323846f)
#define LEG_KINEMATICS_TWO_PI    (6.28318530717958647692f)
#define LEG_KINEMATICS_EPS       (0.000001f)

static float leg_kinematics_absf(float value)
{
    return (0.0f > value) ? -value : value;
}

static uint8 leg_kinematics_is_finite(float value)
{
    if(value != value)
    {
        return APP_FALSE;
    }
    if(APP_BALANCE_FINITE_ABS_LIMIT < value)
    {
        return APP_FALSE;
    }
    if((-APP_BALANCE_FINITE_ABS_LIMIT) > value)
    {
        return APP_FALSE;
    }
    return APP_TRUE;
}

static float leg_kinematics_wrap_positive(float angle_rad)
{
    while(0.0f > angle_rad)
    {
        angle_rad += LEG_KINEMATICS_TWO_PI;
    }
    while(LEG_KINEMATICS_TWO_PI <= angle_rad)
    {
        angle_rad -= LEG_KINEMATICS_TWO_PI;
    }
    return angle_rad;
}

static uint8 leg_kinematics_pick_branch(float plus_value,
                                        float minus_value,
                                        leg_ik_branch_enum branch,
                                        float *selected)
{
    *selected = (LEG_IK_BRANCH_PLUS == branch) ? plus_value : minus_value;
    return leg_kinematics_is_finite(*selected);
}

static uint8 leg_kinematics_solve_angle(float a,
                                        float b,
                                        float c,
                                        leg_ik_branch_enum branch,
                                        float *angle_rad)
{
    float disc;
    float root;
    float denom;
    float plus_value;
    float minus_value;
    float selected;

    disc = (a * a) + (b * b) - (c * c);
    if(0.0f > disc)
    {
        return APP_FALSE;
    }

    root = sqrtf(disc);
    denom = a + c;
    if(LEG_KINEMATICS_EPS > leg_kinematics_absf(denom))
    {
        return APP_FALSE;
    }

    plus_value = 2.0f * atanf((b + root) / denom);
    minus_value = 2.0f * atanf((b - root) / denom);

    if(APP_FALSE == leg_kinematics_pick_branch(plus_value, minus_value, branch, &selected))
    {
        return APP_FALSE;
    }

    *angle_rad = leg_kinematics_wrap_positive(selected);
    return APP_TRUE;
}

static float leg_kinematics_rad_to_deg(float angle_rad)
{
    return angle_rad * 180.0f / LEG_KINEMATICS_PI;
}

static uint8 leg_kinematics_servo_valid(uint8 servo_index, float angle_deg)
{
    const leg_servo_config_struct *servo_cfg;

    servo_cfg = leg_config_get_servo(servo_index);
    if(NULL == servo_cfg)
    {
        return APP_FALSE;
    }
    if((servo_cfg->min_deg > angle_deg) || (servo_cfg->max_deg < angle_deg))
    {
        return APP_FALSE;
    }
    return leg_kinematics_is_finite(angle_deg);
}

static uint8 leg_kinematics_workspace_valid(const leg_kinematics_config_struct *cfg,
                                            float x_mm,
                                            float y_mm)
{
    if((cfg->x_min_mm > x_mm) || (cfg->x_max_mm < x_mm))
    {
        return APP_FALSE;
    }
    if((cfg->y_min_mm > y_mm) || (cfg->y_max_mm < y_mm))
    {
        return APP_FALSE;
    }
    return APP_TRUE;
}

uint8 leg_kinematics_solve(uint8 right_side,
                           float x_mm,
                           float y_mm,
                           leg_ik_result_struct *result)
{
    const leg_kinematics_config_struct *cfg;
    uint8 servo_a;
    uint8 servo_b;
    float x;
    float y;
    float a;
    float b;
    float c;
    float d;
    float e;
    float f;
    float alpha_rad;
    float beta_rad;
    float alpha_deg;
    float beta_deg;
    leg_ik_branch_enum alpha_branch;
    leg_ik_branch_enum beta_branch;

    if(NULL == result)
    {
        return APP_FALSE;
    }

    result->servo_deg[0] = 0.0f;
    result->servo_deg[1] = 0.0f;
    result->alpha_rad = 0.0f;
    result->beta_rad = 0.0f;
    result->valid = APP_FALSE;

    cfg = leg_config_get_kinematics();
    x = x_mm + cfg->x_offset_mm;
    y = y_mm + cfg->y_offset_mm;

    if((APP_FALSE == leg_kinematics_workspace_valid(cfg, x, y)) ||
       (APP_FALSE == leg_kinematics_is_finite(x)) ||
       (APP_FALSE == leg_kinematics_is_finite(y)))
    {
        return APP_FALSE;
    }

    a = 2.0f * x * cfg->l1_mm;
    b = 2.0f * y * cfg->l1_mm;
    c = (x * x) + (y * y) + (cfg->l1_mm * cfg->l1_mm) - (cfg->l2_mm * cfg->l2_mm);
    d = 2.0f * (x - cfg->l5_mm) * cfg->l4_mm;
    e = 2.0f * y * cfg->l4_mm;
    f = ((x - cfg->l5_mm) * (x - cfg->l5_mm)) + (y * y) +
        (cfg->l4_mm * cfg->l4_mm) - (cfg->l3_mm * cfg->l3_mm);

    alpha_branch = (APP_TRUE == right_side) ? cfg->right_alpha_branch : cfg->left_alpha_branch;
    beta_branch = (APP_TRUE == right_side) ? cfg->right_beta_branch : cfg->left_beta_branch;

    if((APP_FALSE == leg_kinematics_solve_angle(a, b, c, alpha_branch, &alpha_rad)) ||
       (APP_FALSE == leg_kinematics_solve_angle(d, e, f, beta_branch, &beta_rad)))
    {
        return APP_FALSE;
    }

    alpha_deg = leg_kinematics_rad_to_deg(alpha_rad);
    beta_deg = leg_kinematics_rad_to_deg(beta_rad);

    if(APP_TRUE == right_side)
    {
        servo_a = LEG_SERVO_FR;
        servo_b = LEG_SERVO_RR;
    }
    else
    {
        servo_a = LEG_SERVO_FL;
        servo_b = LEG_SERVO_RL;
    }

    if((APP_FALSE == leg_kinematics_servo_valid(servo_a, alpha_deg)) ||
       (APP_FALSE == leg_kinematics_servo_valid(servo_b, beta_deg)))
    {
        return APP_FALSE;
    }

    result->servo_deg[0] = alpha_deg;
    result->servo_deg[1] = beta_deg;
    result->alpha_rad = alpha_rad;
    result->beta_rad = beta_rad;
    result->valid = APP_TRUE;
    return APP_TRUE;
}

uint8 leg_kinematics_forward(uint8 right_side,
                             float servo_a_deg,
                             float servo_b_deg,
                             float *x_mm,
                             float *y_mm)
{
    (void)right_side;
    (void)servo_a_deg;
    (void)servo_b_deg;
    if((NULL == x_mm) || (NULL == y_mm))
    {
        return APP_FALSE;
    }
    *x_mm = 0.0f;
    *y_mm = 0.0f;
    return APP_TRUE;
}

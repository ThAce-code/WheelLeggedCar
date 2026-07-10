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
#define LEG_KINEMATICS_WORKSPACE_EPS (0.01f)
#define LEG_KINEMATICS_FK_MATCH_EPS (0.001f)

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

static float leg_kinematics_wrapped_distance(float first_rad, float second_rad)
{
    float delta;

    delta = leg_kinematics_wrap_positive(first_rad - second_rad);
    if(LEG_KINEMATICS_PI < delta)
    {
        delta = LEG_KINEMATICS_TWO_PI - delta;
    }
    return delta;
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

static uint8 leg_kinematics_solve_angle_candidates(float a,
                                                    float b,
                                                    float c,
                                                    float *plus_rad,
                                                    float *minus_rad,
                                                    float *margin)
{
    float disc;
    float root;
    float denom;
    float magnitude;

    if((NULL == plus_rad) || (NULL == minus_rad) || (NULL == margin))
    {
        return APP_FALSE;
    }
    if((APP_FALSE == leg_kinematics_is_finite(a)) ||
       (APP_FALSE == leg_kinematics_is_finite(b)) ||
       (APP_FALSE == leg_kinematics_is_finite(c)))
    {
        return APP_FALSE;
    }

    disc = (a * a) + (b * b) - (c * c);
    if(0.0f > disc)
    {
        return APP_FALSE;
    }

    magnitude = sqrtf((a * a) + (b * b));
    if(LEG_KINEMATICS_EPS > magnitude)
    {
        return APP_FALSE;
    }

    root = sqrtf(disc);
    *margin = root / magnitude;
    if((APP_FALSE == leg_kinematics_is_finite(*margin)) ||
       (0.0f > *margin) || (1.0f < *margin))
    {
        return APP_FALSE;
    }

    denom = a + c;
    if(LEG_KINEMATICS_EPS > leg_kinematics_absf(denom))
    {
        return APP_FALSE;
    }

    *plus_rad = leg_kinematics_wrap_positive(2.0f * atanf((b + root) / denom));
    *minus_rad = leg_kinematics_wrap_positive(2.0f * atanf((b - root) / denom));
    if((APP_FALSE == leg_kinematics_is_finite(*plus_rad)) ||
       (APP_FALSE == leg_kinematics_is_finite(*minus_rad)))
    {
        return APP_FALSE;
    }
    return APP_TRUE;
}

static uint8 leg_kinematics_select_angle(float plus_rad,
                                         float minus_rad,
                                         leg_ik_branch_enum branch,
                                         uint8 servo_index,
                                         const leg_ik_result_struct *previous,
                                         uint8 joint_index,
                                         float *selected_rad)
{
    float first_rad;
    float second_rad;
    uint8 first_valid;
    uint8 second_valid;
    float first_distance;
    float second_distance;

    if(NULL == selected_rad)
    {
        return APP_FALSE;
    }

    if(LEG_IK_BRANCH_PLUS == branch)
    {
        first_rad = plus_rad;
        second_rad = minus_rad;
    }
    else
    {
        first_rad = minus_rad;
        second_rad = plus_rad;
    }

    first_valid = leg_kinematics_servo_valid(servo_index, leg_kinematics_rad_to_deg(first_rad));
    second_valid = leg_kinematics_servo_valid(servo_index, leg_kinematics_rad_to_deg(second_rad));
    if((APP_FALSE == first_valid) && (APP_FALSE == second_valid))
    {
        return APP_FALSE;
    }

    if((NULL == previous) || (APP_FALSE == previous->valid) ||
       (APP_FALSE == leg_kinematics_is_finite(previous->servo_deg[joint_index])))
    {
        *selected_rad = (APP_TRUE == first_valid) ? first_rad : second_rad;
        return APP_TRUE;
    }

    first_distance = leg_kinematics_wrapped_distance(first_rad,
                                                      previous->servo_deg[joint_index] * LEG_KINEMATICS_PI / 180.0f);
    second_distance = leg_kinematics_wrapped_distance(second_rad,
                                                       previous->servo_deg[joint_index] * LEG_KINEMATICS_PI / 180.0f);
    if((APP_TRUE == first_valid) &&
       ((APP_FALSE == second_valid) || (first_distance <= second_distance)))
    {
        *selected_rad = first_rad;
    }
    else
    {
        *selected_rad = second_rad;
    }
    return APP_TRUE;
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

static uint8 leg_kinematics_workspace_valid_fk(const leg_kinematics_config_struct *cfg,
                                                float x_mm,
                                                float y_mm)
{
    if(((cfg->x_min_mm - LEG_KINEMATICS_WORKSPACE_EPS) > x_mm) ||
       ((cfg->x_max_mm + LEG_KINEMATICS_WORKSPACE_EPS) < x_mm))
    {
        return APP_FALSE;
    }
    if(((cfg->y_min_mm - LEG_KINEMATICS_WORKSPACE_EPS) > y_mm) ||
       ((cfg->y_max_mm + LEG_KINEMATICS_WORKSPACE_EPS) < y_mm))
    {
        return APP_FALSE;
    }
    return APP_TRUE;
}

static float leg_kinematics_clamp_fk_workspace(float value, float minimum, float maximum)
{
    if((minimum > value) && ((minimum - LEG_KINEMATICS_WORKSPACE_EPS) <= value))
    {
        return minimum;
    }
    if((maximum < value) && ((maximum + LEG_KINEMATICS_WORKSPACE_EPS) >= value))
    {
        return maximum;
    }
    return value;
}

uint8 leg_kinematics_solve(uint8 right_side,
                           float x_mm,
                           float y_mm,
                           const leg_ik_result_struct *previous,
                           leg_ik_result_struct *result)
{
    const leg_kinematics_config_struct *cfg;
    const leg_height_profile_struct *profile;
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
    float alpha_plus_rad;
    float alpha_minus_rad;
    float beta_plus_rad;
    float beta_minus_rad;
    float alpha_margin;
    float beta_margin;
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
    result->singularity_margin = 0.0f;
    result->valid = APP_FALSE;

    cfg = leg_config_get_kinematics();
    profile = leg_config_get_height_profile();
    if((NULL == cfg) || (NULL == profile))
    {
        return APP_FALSE;
    }
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

    if((APP_FALSE == leg_kinematics_solve_angle_candidates(a, b, c,
                                                            &alpha_plus_rad, &alpha_minus_rad, &alpha_margin)) ||
       (APP_FALSE == leg_kinematics_solve_angle_candidates(d, e, f,
                                                            &beta_plus_rad, &beta_minus_rad, &beta_margin)))
    {
        return APP_FALSE;
    }

    result->singularity_margin = (alpha_margin < beta_margin) ? alpha_margin : beta_margin;
    if(profile->ik_min_margin > result->singularity_margin)
    {
        return APP_FALSE;
    }

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

    if((APP_FALSE == leg_kinematics_select_angle(alpha_plus_rad, alpha_minus_rad,
                                                  alpha_branch, servo_a, previous, 0U, &alpha_rad)) ||
       (APP_FALSE == leg_kinematics_select_angle(beta_plus_rad, beta_minus_rad,
                                                  beta_branch, servo_b, previous, 1U, &beta_rad)))
    {
        return APP_FALSE;
    }

    alpha_deg = leg_kinematics_rad_to_deg(alpha_rad);
    beta_deg = leg_kinematics_rad_to_deg(beta_rad);

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
    const leg_kinematics_config_struct *cfg;
    float alpha_rad;
    float beta_rad;
    float c_x;
    float c_y;
    float d_x;
    float d_y;
    float dx;
    float dy;
    float distance;
    float projection;
    float root_term;
    float height;
    float base_x;
    float base_y;
    float plus_x;
    float plus_y;
    float minus_x;
    float minus_y;
    uint8 plus_valid;
    uint8 minus_valid;
    uint8 plus_match;
    uint8 minus_match;
    leg_ik_result_struct plus_ik;
    leg_ik_result_struct minus_ik;

    if((NULL == x_mm) || (NULL == y_mm))
    {
        return APP_FALSE;
    }
    *x_mm = 0.0f;
    *y_mm = 0.0f;

    cfg = leg_config_get_kinematics();
    if((NULL == cfg) ||
       (APP_FALSE == leg_kinematics_is_finite(servo_a_deg)) ||
       (APP_FALSE == leg_kinematics_is_finite(servo_b_deg)))
    {
        return APP_FALSE;
    }

    alpha_rad = servo_a_deg * LEG_KINEMATICS_PI / 180.0f;
    beta_rad = servo_b_deg * LEG_KINEMATICS_PI / 180.0f;
    c_x = cfg->l1_mm * cosf(alpha_rad);
    c_y = cfg->l1_mm * sinf(alpha_rad);
    d_x = cfg->l5_mm + (cfg->l4_mm * cosf(beta_rad));
    d_y = cfg->l4_mm * sinf(beta_rad);
    dx = d_x - c_x;
    dy = d_y - c_y;
    distance = sqrtf((dx * dx) + (dy * dy));
    if((APP_FALSE == leg_kinematics_is_finite(distance)) ||
       (LEG_KINEMATICS_EPS > distance))
    {
        return APP_FALSE;
    }

    projection = ((cfg->l2_mm * cfg->l2_mm) - (cfg->l3_mm * cfg->l3_mm) + (distance * distance)) /
                 (2.0f * distance);
    root_term = (cfg->l2_mm * cfg->l2_mm) - (projection * projection);
    if(0.0f > root_term)
    {
        return APP_FALSE;
    }
    height = sqrtf(root_term);
    base_x = c_x + (projection * dx / distance);
    base_y = c_y + (projection * dy / distance);
    plus_x = base_x - (dy * height / distance);
    plus_y = base_y + (dx * height / distance);
    minus_x = base_x + (dy * height / distance);
    minus_y = base_y - (dx * height / distance);
    plus_x = leg_kinematics_clamp_fk_workspace(plus_x, cfg->x_min_mm, cfg->x_max_mm);
    plus_y = leg_kinematics_clamp_fk_workspace(plus_y, cfg->y_min_mm, cfg->y_max_mm);
    minus_x = leg_kinematics_clamp_fk_workspace(minus_x, cfg->x_min_mm, cfg->x_max_mm);
    minus_y = leg_kinematics_clamp_fk_workspace(minus_y, cfg->y_min_mm, cfg->y_max_mm);

    plus_valid = ((APP_TRUE == leg_kinematics_is_finite(plus_x)) &&
                  (APP_TRUE == leg_kinematics_is_finite(plus_y)) &&
                  (0.0f < plus_y) &&
                  (APP_TRUE == leg_kinematics_workspace_valid_fk(cfg, plus_x, plus_y))) ? APP_TRUE : APP_FALSE;
    minus_valid = ((APP_TRUE == leg_kinematics_is_finite(minus_x)) &&
                   (APP_TRUE == leg_kinematics_is_finite(minus_y)) &&
                   (0.0f < minus_y) &&
                   (APP_TRUE == leg_kinematics_workspace_valid_fk(cfg, minus_x, minus_y))) ? APP_TRUE : APP_FALSE;
    if((APP_FALSE == plus_valid) && (APP_FALSE == minus_valid))
    {
        return APP_FALSE;
    }

    plus_match = APP_FALSE;
    minus_match = APP_FALSE;
    if((APP_TRUE == plus_valid) &&
       (APP_TRUE == leg_kinematics_solve(right_side,
                                          plus_x - cfg->x_offset_mm,
                                          plus_y - cfg->y_offset_mm,
                                          NULL,
                                          &plus_ik)) &&
       (LEG_KINEMATICS_FK_MATCH_EPS >= leg_kinematics_wrapped_distance(alpha_rad, plus_ik.alpha_rad)) &&
       (LEG_KINEMATICS_FK_MATCH_EPS >= leg_kinematics_wrapped_distance(beta_rad, plus_ik.beta_rad)))
    {
        plus_match = APP_TRUE;
    }
    if((APP_TRUE == minus_valid) &&
       (APP_TRUE == leg_kinematics_solve(right_side,
                                          minus_x - cfg->x_offset_mm,
                                          minus_y - cfg->y_offset_mm,
                                          NULL,
                                          &minus_ik)) &&
       (LEG_KINEMATICS_FK_MATCH_EPS >= leg_kinematics_wrapped_distance(alpha_rad, minus_ik.alpha_rad)) &&
       (LEG_KINEMATICS_FK_MATCH_EPS >= leg_kinematics_wrapped_distance(beta_rad, minus_ik.beta_rad)))
    {
        minus_match = APP_TRUE;
    }
    if((APP_FALSE == plus_match) && (APP_FALSE == minus_match))
    {
        return APP_FALSE;
    }

    /* The configured wheel centerline resolves the remaining dual-root FK pose. */
    if(((APP_TRUE == minus_match) && (APP_FALSE == plus_match)) ||
       ((APP_TRUE == minus_match) && (APP_TRUE == plus_match) &&
        (leg_kinematics_absf(minus_x - cfg->x_offset_mm) <=
         leg_kinematics_absf(plus_x - cfg->x_offset_mm))))
    {
        *x_mm = minus_x - cfg->x_offset_mm;
        *y_mm = minus_y - cfg->y_offset_mm;
    }
    else
    {
        *x_mm = plus_x - cfg->x_offset_mm;
        *y_mm = plus_y - cfg->y_offset_mm;
    }
    return APP_TRUE;
}

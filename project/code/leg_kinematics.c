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

static float leg_kinematics_wrapped_delta(float target_rad, float reference_rad)
{
    float delta;

    delta = leg_kinematics_wrap_positive(target_rad - reference_rad);
    if(LEG_KINEMATICS_PI < delta)
    {
        delta -= LEG_KINEMATICS_TWO_PI;
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

static uint8 leg_kinematics_map_one(uint8 servo_index,
                                     float reference_deg,
                                     float target_deg,
                                     float *command_deg)
{
    const leg_servo_config_struct *cfg;
    float value;

    if(NULL == command_deg)
    {
        return APP_FALSE;
    }
    cfg = leg_config_get_servo(servo_index);
    if((NULL == cfg) ||
       (APP_FALSE == leg_kinematics_is_finite(reference_deg)) ||
       (APP_FALSE == leg_kinematics_is_finite(target_deg)))
    {
        return APP_FALSE;
    }

    value = cfg->neutral_deg +
            (cfg->direction * leg_kinematics_rad_to_deg(
                leg_kinematics_wrapped_delta(target_deg * LEG_KINEMATICS_PI / 180.0f,
                                             reference_deg * LEG_KINEMATICS_PI / 180.0f))) +
            cfg->ik_offset_deg;
    if(APP_FALSE == leg_kinematics_servo_valid(servo_index, value))
    {
        return APP_FALSE;
    }
    *command_deg = value;
    return APP_TRUE;
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
    float magnitude;
    float phase_rad;
    float offset_rad;

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

    /* a*cos(theta) + b*sin(theta) = c.  This form keeps both roots when a+c is zero. */
    phase_rad = atan2f(b, a);
    offset_rad = atan2f(root, c);
    *plus_rad = leg_kinematics_wrap_positive(phase_rad + offset_rad);
    *minus_rad = leg_kinematics_wrap_positive(phase_rad - offset_rad);
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

    /* Linkage angles are geometric coordinates, not servo command angles. */
    first_valid = leg_kinematics_is_finite(first_rad);
    second_valid = leg_kinematics_is_finite(second_rad);
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

static uint8 leg_kinematics_model_workspace_valid(const leg_kinematics_config_struct *cfg,
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

static uint8 leg_kinematics_model_workspace_valid_fk(const leg_kinematics_config_struct *cfg,
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

static uint8 leg_kinematics_experimental_race_target_valid(
    const leg_kinematics_config_struct *cfg,
    float x_mm,
    float y_mm)
{
    if((NULL == cfg) ||
       (0U == cfg->experimental_race_enable) ||
       (APP_FALSE == leg_kinematics_is_finite(cfg->experimental_race_x_mm)) ||
       (APP_FALSE == leg_kinematics_is_finite(cfg->experimental_race_y_mm)) ||
       (APP_FALSE == leg_kinematics_is_finite(cfg->experimental_race_tolerance_x_mm)) ||
       (APP_FALSE == leg_kinematics_is_finite(cfg->experimental_race_tolerance_y_mm)) ||
       (0.0f > cfg->experimental_race_tolerance_x_mm) ||
       (0.0f > cfg->experimental_race_tolerance_y_mm))
    {
        return APP_FALSE;
    }
    if((leg_kinematics_absf(x_mm - cfg->experimental_race_x_mm) >
        cfg->experimental_race_tolerance_x_mm) ||
       (leg_kinematics_absf(y_mm - cfg->experimental_race_y_mm) >
        cfg->experimental_race_tolerance_y_mm))
    {
        return APP_FALSE;
    }
    return APP_TRUE;
}

uint8 leg_kinematics_target_valid(float x_mm, float y_mm)
{
    const leg_kinematics_config_struct *cfg;
    uint8 i;

    cfg = leg_config_get_kinematics();
    if((NULL == cfg) ||
       (APP_FALSE == leg_kinematics_is_finite(x_mm)) ||
       (APP_FALSE == leg_kinematics_is_finite(y_mm)) ||
       (APP_FALSE == leg_kinematics_is_finite(cfg->physical_workspace_inset_mm)) ||
       (0.0f > cfg->physical_workspace_inset_mm))
    {
        return APP_FALSE;
    }

    if(APP_TRUE == leg_kinematics_experimental_race_target_valid(cfg, x_mm, y_mm))
    {
        return APP_TRUE;
    }

    /* Vertices are stored counter-clockwise.  The signed distance from the
       target to every directed edge must remain inside by the fitted margin. */
    for(i = 0U; i < LEG_PHYSICAL_WORKSPACE_VERTEX_COUNT; i++)
    {
        uint8 next;
        float first_x;
        float first_y;
        float edge_x;
        float edge_y;
        float edge_length;
        float edge_cross;

        next = (uint8)((i + 1U) % LEG_PHYSICAL_WORKSPACE_VERTEX_COUNT);
        first_x = cfg->physical_workspace[i][0];
        first_y = cfg->physical_workspace[i][1];
        edge_x = cfg->physical_workspace[next][0] - first_x;
        edge_y = cfg->physical_workspace[next][1] - first_y;
        edge_length = sqrtf((edge_x * edge_x) + (edge_y * edge_y));
        if((APP_FALSE == leg_kinematics_is_finite(edge_length)) ||
           (LEG_KINEMATICS_EPS > edge_length))
        {
            return APP_FALSE;
        }
        edge_cross = (edge_x * (y_mm - first_y)) -
                     (edge_y * (x_mm - first_x));
        if(edge_cross < (cfg->physical_workspace_inset_mm * edge_length))
        {
            return APP_FALSE;
        }
    }
    return APP_TRUE;
}

static uint8 leg_kinematics_physical_to_model(float physical_x_mm,
                                               float physical_y_mm,
                                               float *model_x_mm,
                                               float *model_y_mm)
{
    const leg_kinematics_config_struct *cfg;
    float delta_x;
    float delta_y;

    if((NULL == model_x_mm) || (NULL == model_y_mm))
    {
        return APP_FALSE;
    }
    cfg = leg_config_get_kinematics();
    if((NULL == cfg) ||
       (APP_FALSE == leg_kinematics_is_finite(physical_x_mm)) ||
       (APP_FALSE == leg_kinematics_is_finite(physical_y_mm)) ||
       (APP_FALSE == leg_kinematics_is_finite(cfg->model_to_physical_scale)) ||
       (LEG_KINEMATICS_EPS > cfg->model_to_physical_scale))
    {
        return APP_FALSE;
    }

    delta_x = (physical_x_mm - cfg->physical_reference_x_mm) /
              cfg->model_to_physical_scale;
    delta_y = (physical_y_mm - cfg->physical_reference_y_mm) /
              cfg->model_to_physical_scale;
    /* Q is orthogonal, so Q^-1 is Q transposed. */
    *model_x_mm = cfg->model_reference_x_mm +
                  (cfg->model_to_physical_m00 * delta_x) +
                  (cfg->model_to_physical_m10 * delta_y);
    *model_y_mm = cfg->model_reference_y_mm +
                  (cfg->model_to_physical_m01 * delta_x) +
                  (cfg->model_to_physical_m11 * delta_y);
    if((APP_FALSE == leg_kinematics_is_finite(*model_x_mm)) ||
       (APP_FALSE == leg_kinematics_is_finite(*model_y_mm)))
    {
        return APP_FALSE;
    }
    return APP_TRUE;
}

static uint8 leg_kinematics_model_to_physical(float model_x_mm,
                                               float model_y_mm,
                                               float *physical_x_mm,
                                               float *physical_y_mm)
{
    const leg_kinematics_config_struct *cfg;
    float delta_x;
    float delta_y;

    if((NULL == physical_x_mm) || (NULL == physical_y_mm))
    {
        return APP_FALSE;
    }
    cfg = leg_config_get_kinematics();
    if((NULL == cfg) ||
       (APP_FALSE == leg_kinematics_is_finite(model_x_mm)) ||
       (APP_FALSE == leg_kinematics_is_finite(model_y_mm)) ||
       (APP_FALSE == leg_kinematics_is_finite(cfg->model_to_physical_scale)) ||
       (LEG_KINEMATICS_EPS > cfg->model_to_physical_scale))
    {
        return APP_FALSE;
    }

    delta_x = model_x_mm - cfg->model_reference_x_mm;
    delta_y = model_y_mm - cfg->model_reference_y_mm;
    *physical_x_mm = cfg->physical_reference_x_mm +
                     cfg->model_to_physical_scale *
                     ((cfg->model_to_physical_m00 * delta_x) +
                      (cfg->model_to_physical_m01 * delta_y));
    *physical_y_mm = cfg->physical_reference_y_mm +
                     cfg->model_to_physical_scale *
                     ((cfg->model_to_physical_m10 * delta_x) +
                      (cfg->model_to_physical_m11 * delta_y));
    if((APP_FALSE == leg_kinematics_is_finite(*physical_x_mm)) ||
       (APP_FALSE == leg_kinematics_is_finite(*physical_y_mm)))
    {
        return APP_FALSE;
    }
    return APP_TRUE;
}

static uint8 leg_kinematics_solve_model(uint8 right_side,
                                        float x_mm,
                                        float y_mm,
                                        const leg_ik_result_struct *previous,
                                        leg_ik_result_struct *result)
{
    const leg_kinematics_config_struct *cfg;
    const leg_stance_profile_struct *profile;
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
    float minimum_margin;
    float physical_x_mm;
    float physical_y_mm;
    uint8 experimental_race_target;
    const leg_ik_result_struct *selection_previous;
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
    profile = leg_config_get_stance_profile();
    if((NULL == cfg) || (NULL == profile))
    {
        return APP_FALSE;
    }
    x = x_mm;
    y = y_mm;

    if((APP_FALSE == leg_kinematics_model_workspace_valid(cfg, x, y)) ||
       (APP_FALSE == leg_kinematics_is_finite(x)) ||
       (APP_FALSE == leg_kinematics_is_finite(y)))
    {
        return APP_FALSE;
    }

    experimental_race_target = APP_FALSE;
    if((APP_TRUE == leg_kinematics_model_to_physical(x, y,
                                                      &physical_x_mm,
                                                      &physical_y_mm)) &&
       (APP_TRUE == leg_kinematics_experimental_race_target_valid(cfg,
                                                                   physical_x_mm,
                                                                   physical_y_mm)))
    {
        experimental_race_target = APP_TRUE;
    }

    a = 2.0f * x * cfg->l1_mm;
    b = 2.0f * y * cfg->l1_mm;
    c = (x * x) + (y * y) + (cfg->l1_mm * cfg->l1_mm) - (cfg->l2_mm * cfg->l2_mm);
    d = 2.0f * (x - cfg->l5_mm) * cfg->l4_mm;
    e = 2.0f * y * cfg->l4_mm;
    f = ((x - cfg->l5_mm) * (x - cfg->l5_mm)) + (y * y) +
        (cfg->l4_mm * cfg->l4_mm) - (cfg->l3_mm * cfg->l3_mm);

    if(APP_TRUE == experimental_race_target)
    {
        alpha_branch = cfg->experimental_race_alpha_branch;
        beta_branch = cfg->experimental_race_beta_branch;
        if((LEG_IK_BRANCH_MINUS < alpha_branch) ||
           (LEG_IK_BRANCH_MINUS < beta_branch))
        {
            return APP_FALSE;
        }
    }
    else
    {
        alpha_branch = (APP_TRUE == right_side) ? cfg->right_alpha_branch : cfg->left_alpha_branch;
        beta_branch = (APP_TRUE == right_side) ? cfg->right_beta_branch : cfg->left_beta_branch;
    }

    if((APP_FALSE == leg_kinematics_solve_angle_candidates(a, b, c,
                                                            &alpha_plus_rad, &alpha_minus_rad, &alpha_margin)) ||
       (APP_FALSE == leg_kinematics_solve_angle_candidates(d, e, f,
                                                            &beta_plus_rad, &beta_minus_rad, &beta_margin)))
    {
        return APP_FALSE;
    }

    result->singularity_margin = (alpha_margin < beta_margin) ? alpha_margin : beta_margin;
    minimum_margin = profile->ik_min_margin;
    if(APP_TRUE == experimental_race_target)
    {
        if((APP_FALSE == leg_kinematics_is_finite(cfg->experimental_race_ik_min_margin)) ||
           (0.0f > cfg->experimental_race_ik_min_margin) ||
           (1.0f < cfg->experimental_race_ik_min_margin))
        {
            return APP_FALSE;
        }
        minimum_margin = cfg->experimental_race_ik_min_margin;
    }
    if(minimum_margin > result->singularity_margin)
    {
        return APP_FALSE;
    }

    selection_previous = (APP_TRUE == experimental_race_target) ? NULL : previous;
    if((APP_FALSE == leg_kinematics_select_angle(alpha_plus_rad, alpha_minus_rad,
                                                  alpha_branch, selection_previous, 0U, &alpha_rad)) ||
       (APP_FALSE == leg_kinematics_select_angle(beta_plus_rad, beta_minus_rad,
                                                  beta_branch, selection_previous, 1U, &beta_rad)))
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

uint8 leg_kinematics_solve(uint8 right_side,
                           float x_mm,
                           float y_mm,
                           const leg_ik_result_struct *previous,
                           leg_ik_result_struct *result)
{
    float model_x_mm;
    float model_y_mm;

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

    if((APP_FALSE == leg_kinematics_target_valid(x_mm, y_mm)) ||
       (APP_FALSE == leg_kinematics_physical_to_model(x_mm, y_mm,
                                                       &model_x_mm,
                                                       &model_y_mm)))
    {
        return APP_FALSE;
    }
    return leg_kinematics_solve_model(right_side,
                                       model_x_mm,
                                       model_y_mm,
                                       previous,
                                       result);
}

uint8 leg_kinematics_map_reference_pose(const leg_ik_result_struct *left,
                                        const leg_ik_result_struct *right,
                                        float servo_deg[LEG_SERVO_COUNT])
{
    return leg_kinematics_map_target_pose(left, right, left, right, servo_deg);
}

uint8 leg_kinematics_map_target_pose(const leg_ik_result_struct *left_reference,
                                     const leg_ik_result_struct *right_reference,
                                     const leg_ik_result_struct *left_target,
                                     const leg_ik_result_struct *right_target,
                                     float servo_deg[LEG_SERVO_COUNT])
{
    float mapped_deg[LEG_SERVO_COUNT];

    if((NULL == left_reference) || (NULL == right_reference) ||
       (NULL == left_target) || (NULL == right_target) || (NULL == servo_deg) ||
       (APP_FALSE == left_reference->valid) || (APP_FALSE == right_reference->valid) ||
       (APP_FALSE == left_target->valid) || (APP_FALSE == right_target->valid))
    {
        return APP_FALSE;
    }

    if((APP_FALSE == leg_kinematics_map_one(LEG_SERVO_FL,
                                             left_reference->servo_deg[0],
                                             left_target->servo_deg[0],
                                             &mapped_deg[LEG_SERVO_FL])) ||
       (APP_FALSE == leg_kinematics_map_one(LEG_SERVO_RL,
                                             left_reference->servo_deg[1],
                                             left_target->servo_deg[1],
                                             &mapped_deg[LEG_SERVO_RL])) ||
       (APP_FALSE == leg_kinematics_map_one(LEG_SERVO_FR,
                                             right_reference->servo_deg[0],
                                             right_target->servo_deg[0],
                                             &mapped_deg[LEG_SERVO_FR])) ||
       (APP_FALSE == leg_kinematics_map_one(LEG_SERVO_RR,
                                             right_reference->servo_deg[1],
                                             right_target->servo_deg[1],
                                             &mapped_deg[LEG_SERVO_RR])))
    {
        return APP_FALSE;
    }

    servo_deg[LEG_SERVO_FL] = mapped_deg[LEG_SERVO_FL];
    servo_deg[LEG_SERVO_FR] = mapped_deg[LEG_SERVO_FR];
    servo_deg[LEG_SERVO_RL] = mapped_deg[LEG_SERVO_RL];
    servo_deg[LEG_SERVO_RR] = mapped_deg[LEG_SERVO_RR];
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
                  (APP_TRUE == leg_kinematics_model_workspace_valid_fk(cfg, plus_x, plus_y))) ? APP_TRUE : APP_FALSE;
    minus_valid = ((APP_TRUE == leg_kinematics_is_finite(minus_x)) &&
                   (APP_TRUE == leg_kinematics_is_finite(minus_y)) &&
                   (0.0f < minus_y) &&
                   (APP_TRUE == leg_kinematics_model_workspace_valid_fk(cfg, minus_x, minus_y))) ? APP_TRUE : APP_FALSE;
    if((APP_FALSE == plus_valid) && (APP_FALSE == minus_valid))
    {
        return APP_FALSE;
    }

    plus_match = APP_FALSE;
    minus_match = APP_FALSE;
    if((APP_TRUE == plus_valid) &&
       (APP_TRUE == leg_kinematics_solve_model(right_side,
                                                plus_x,
                                                plus_y,
                                                NULL,
                                                &plus_ik)) &&
       (LEG_KINEMATICS_FK_MATCH_EPS >= leg_kinematics_wrapped_distance(alpha_rad, plus_ik.alpha_rad)) &&
       (LEG_KINEMATICS_FK_MATCH_EPS >= leg_kinematics_wrapped_distance(beta_rad, plus_ik.beta_rad)))
    {
        plus_match = APP_TRUE;
    }
    if((APP_TRUE == minus_valid) &&
       (APP_TRUE == leg_kinematics_solve_model(right_side,
                                                minus_x,
                                                minus_y,
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
        (leg_kinematics_absf(minus_x - cfg->model_reference_x_mm) <=
         leg_kinematics_absf(plus_x - cfg->model_reference_x_mm))))
    {
        return leg_kinematics_model_to_physical(minus_x, minus_y, x_mm, y_mm);
    }
    return leg_kinematics_model_to_physical(plus_x, plus_y, x_mm, y_mm);
}

uint8 leg_kinematics_forward_command(uint8 right_side,
                                     float servo_a_command_deg,
                                     float servo_b_command_deg,
                                     float *x_mm,
                                     float *y_mm)
{
    const leg_kinematics_config_struct *cfg;
    const leg_servo_config_struct *servo_a;
    const leg_servo_config_struct *servo_b;
    uint8 servo_a_index;
    uint8 servo_b_index;
    float alpha_deg;
    float beta_deg;
    float physical_x_mm;
    float physical_y_mm;

    if((NULL == x_mm) || (NULL == y_mm))
    {
        return APP_FALSE;
    }
    cfg = leg_config_get_kinematics();
    servo_a_index = (APP_TRUE == right_side) ? LEG_SERVO_FR : LEG_SERVO_FL;
    servo_b_index = (APP_TRUE == right_side) ? LEG_SERVO_RR : LEG_SERVO_RL;
    servo_a = leg_config_get_servo(servo_a_index);
    servo_b = leg_config_get_servo(servo_b_index);
    if((NULL == cfg) || (NULL == servo_a) || (NULL == servo_b) ||
       (0.0f == servo_a->direction) || (0.0f == servo_b->direction) ||
       (APP_FALSE == leg_kinematics_is_finite(servo_a_command_deg)) ||
       (APP_FALSE == leg_kinematics_is_finite(servo_b_command_deg)) ||
       (servo_a_command_deg < servo_a->min_deg) ||
       (servo_a_command_deg > servo_a->max_deg) ||
       (servo_b_command_deg < servo_b->min_deg) ||
       (servo_b_command_deg > servo_b->max_deg))
    {
        return APP_FALSE;
    }

    alpha_deg = cfg->alpha_reference_deg +
                ((servo_a_command_deg - servo_a->neutral_deg - servo_a->ik_offset_deg) /
                 servo_a->direction);
    beta_deg = cfg->beta_reference_deg +
               ((servo_b_command_deg - servo_b->neutral_deg - servo_b->ik_offset_deg) /
                servo_b->direction);
    if(APP_TRUE != leg_kinematics_forward(right_side,
                                           alpha_deg,
                                           beta_deg,
                                           &physical_x_mm,
                                           &physical_y_mm))
    {
        return APP_FALSE;
    }
    *x_mm = physical_x_mm;
    *y_mm = physical_y_mm;
    return APP_TRUE;
}

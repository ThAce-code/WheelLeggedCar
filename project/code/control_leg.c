/*********************************************************************************************************************
* File: control_leg.c
* Description: Leg controller: mix body attitude commands into servo angles.
********************************************************************************************************************/

#include "control_leg.h"
#include "app_config.h"
#include "app_safety.h"
#include "app_state.h"
#include "actuator_servo.h"
#include "leg_kinematics.h"

static servo_cmd_struct    control_leg_servo_cmd;
static leg_mode_enum       control_leg_mode;
static float               control_leg_manual_angle[APP_SERVO_COUNT];
static float               control_leg_legacy_stance_cmd;
static float               control_leg_pitch_cmd;
static float               control_leg_roll_cmd;

#if (APP_LEG_VERIFY_ENABLE == 1U)
static uint32              control_leg_verify_start_ms = 0;
static uint8               control_leg_verify_active = APP_FALSE;
#endif

static leg_diag_struct     control_leg_diag;
static float               control_leg_legacy_stance_target_units;
static float               control_leg_legacy_stance_ref_units;
static float               control_leg_legacy_stance_rate_units_s;
static float               control_leg_legacy_stance_accel_units_s2;
static leg_motion_state_enum control_leg_motion_state;
static leg_fault_reason_enum  control_leg_fault_reason;
static uint32              control_leg_settle_start_ms;
static uint32              control_leg_last_update_ms;
static float               control_leg_fast_stance_start_units;
static uint32              control_leg_fast_stance_start_ms;
static float               control_leg_ik_target_x_mm;
static float               control_leg_ik_target_y_mm;
static leg_ik_result_struct control_leg_ik_reference_left;
static leg_ik_result_struct control_leg_ik_reference_right;
static leg_ik_result_struct control_leg_ik_previous_left;
static leg_ik_result_struct control_leg_ik_previous_right;
static uint8               control_leg_ik_reference_valid;
static float               control_leg_race_u_start;
static float               control_leg_race_u_target;
static float               control_leg_race_u_pending;
static float               control_leg_race_u_actual;
static float               control_leg_race_dx_mm;
static float               control_leg_race_dy_mm;
static uint32              control_leg_race_start_ms;
static uint32              control_leg_race_duration_ms;
static uint8               control_leg_race_pending_valid;
static uint8               control_leg_race_path_valid;
static float               control_leg_race_preflight_dx_mm;
static float               control_leg_race_preflight_dy_mm;
static uint8               control_leg_race_preflight_branch_flags;
static uint8               control_leg_race_preflight_cache_valid;
static uint8               control_leg_race_held_command_valid;

#if (APP_RACE_ASSIST_PREFLIGHT_WCET_ENABLE == 1U)
#define CONTROL_LEG_DEMCR              (*((volatile uint32 *)0xE000EDFCUL))
#define CONTROL_LEG_DWT_CTRL           (*((volatile uint32 *)0xE0001000UL))
#define CONTROL_LEG_DWT_CYCCNT         (*((volatile uint32 *)0xE0001004UL))
#define CONTROL_LEG_DEMCR_TRCENA       (1UL << 24)
#define CONTROL_LEG_DWT_CYCCNTENA      (1UL << 0)

volatile uint32 control_leg_race_preflight_count = 0U;
volatile uint32 control_leg_race_preflight_last_cycles = 0U;
volatile uint32 control_leg_race_preflight_max_cycles = 0U;

static uint32 control_leg_race_preflight_wcet_begin(void)
{
    CONTROL_LEG_DEMCR |= CONTROL_LEG_DEMCR_TRCENA;
    CONTROL_LEG_DWT_CTRL |= CONTROL_LEG_DWT_CYCCNTENA;
    return CONTROL_LEG_DWT_CYCCNT;
}

static void control_leg_race_preflight_wcet_end(uint32 start_cycles)
{
    const uint32 elapsed_cycles = CONTROL_LEG_DWT_CYCCNT - start_cycles;

    control_leg_race_preflight_count++;
    control_leg_race_preflight_last_cycles = elapsed_cycles;
    if(control_leg_race_preflight_max_cycles < elapsed_cycles)
    {
        control_leg_race_preflight_max_cycles = elapsed_cycles;
    }
}
#endif

#define CONTROL_LEG_LEGACY_CENTER_UNITS          (55.0f)
#define CONTROL_LEG_EMPIRICAL_CENTER_SERVO_DEG   (90.0f)
#define CONTROL_LEG_LEGACY_UNITS_PER_DELTA_DEG   (0.595f)
#define CONTROL_LEG_EMPIRICAL_IK_MARGIN          (1.0f)

static float control_leg_clamp(float value, float min_val, float max_val)
{
    if(value < min_val)
    {
        return min_val;
    }
    if(value > max_val)
    {
        return max_val;
    }
    return value;
}

static uint8 control_leg_servo_is_active(uint8 index)
{
    return (0U != (APP_SERVO_ACTIVE_MASK & (1U << index))) ? APP_TRUE : APP_FALSE;
}

static uint8 control_leg_safe_deg_valid(float safe_deg)
{
    return ((safe_deg >= APP_SERVO_MIN_DEG) && (safe_deg <= APP_SERVO_MAX_DEG)) ? APP_TRUE : APP_FALSE;
}

static uint8 control_leg_is_finite(float value)
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

static uint8 control_leg_servo_angle_valid(uint8 servo_index, float angle_deg)
{
    const leg_servo_config_struct *servo_cfg;

    servo_cfg = leg_config_get_servo(servo_index);
    if(NULL == servo_cfg)
    {
        return APP_FALSE;
    }
    if(APP_FALSE == control_leg_is_finite(angle_deg))
    {
        return APP_FALSE;
    }
    if((servo_cfg->min_deg > angle_deg) || (servo_cfg->max_deg < angle_deg))
    {
        return APP_FALSE;
    }
    return APP_TRUE;
}

static float control_leg_absf(float value)
{
    return (0.0f > value) ? -value : value;
}

static void control_leg_race_preflight_cache_invalidate(void)
{
    control_leg_race_preflight_cache_valid = APP_FALSE;
}

static uint8 control_leg_race_branch_flags(uint8 *branch_flags)
{
    uint8 flags = 0U;

    if((NULL == branch_flags) ||
       (APP_TRUE != control_leg_ik_previous_left.valid) ||
       (APP_TRUE != control_leg_ik_previous_right.valid))
    {
        return APP_FALSE;
    }
    if(LEG_IK_BRANCH_MINUS == control_leg_ik_previous_left.alpha_branch)
    {
        flags |= (1U << 0);
    }
    if(LEG_IK_BRANCH_MINUS == control_leg_ik_previous_left.beta_branch)
    {
        flags |= (1U << 1);
    }
    if(LEG_IK_BRANCH_MINUS == control_leg_ik_previous_right.alpha_branch)
    {
        flags |= (1U << 2);
    }
    if(LEG_IK_BRANCH_MINUS == control_leg_ik_previous_right.beta_branch)
    {
        flags |= (1U << 3);
    }
    *branch_flags = flags;
    return APP_TRUE;
}

static uint8 control_leg_race_command_valid(void)
{
    uint8 i;

    for(i = 0U; i < APP_SERVO_COUNT; i++)
    {
        if(APP_TRUE != control_leg_servo_angle_valid(i,
                                                     control_leg_servo_cmd.angle_deg[i]))
        {
            return APP_FALSE;
        }
    }
    return APP_TRUE;
}

typedef enum
{
    LEG_TRAJECTORY_NONE = 0,
    LEG_TRAJECTORY_FAST_LEGACY_STANCE,
    LEG_TRAJECTORY_POSE,
    LEG_TRAJECTORY_RACE_ASSIST
}leg_trajectory_mode_enum;

static float control_leg_pose_start_deg[APP_SERVO_COUNT];
static float control_leg_pose_target_deg[APP_SERVO_COUNT];
static uint32 control_leg_pose_start_ms;
static uint32 control_leg_pose_duration_ms;
static float control_leg_s7_progress;
static uint32 control_leg_s7_remaining_ms;
static leg_trajectory_mode_enum control_leg_trajectory_mode;
static actuator_servo_diag_struct control_leg_actuator_diag;

static float control_leg_s7_blend(float progress)
{
    float p;
    float p2;
    float p4;

    p = control_leg_clamp(progress, 0.0f, 1.0f);
    p2 = p * p;
    p4 = p2 * p2;
    return p4 * (35.0f - (84.0f * p) + (70.0f * p2) - (20.0f * p2 * p));
}

static float control_leg_fast_stance_blend(float progress)
{
    return control_leg_s7_blend(progress);
}

static uint8 control_leg_pose_start_if_changed(const float desired_deg[APP_SERVO_COUNT],
                                               float speed_limit_dps,
                                               uint32 now_ms)
{
    uint8 i;
    uint8 changed;
    float max_delta_deg;
    float duration_s;

    /* Force a fresh trajectory when re-entering pose mode after a non-pose
       mode (e.g. LH → LIKREF).  The old timeline is stale even when the
       target angles happen to match. */
    if(LEG_TRAJECTORY_POSE != control_leg_trajectory_mode)
    {
        changed = APP_TRUE;
    }
    else
    {
        changed = APP_FALSE;
        for(i = 0U; i < APP_SERVO_COUNT; i++)
        {
            if(0.01f < control_leg_absf(desired_deg[i] - control_leg_pose_target_deg[i]))
            {
                changed = APP_TRUE;
                break;
            }
        }
    }
    if(APP_FALSE == changed)
    {
        return APP_FALSE;
    }

    /* Start from the current planned command, not a stale endpoint. */
    for(i = 0U; i < APP_SERVO_COUNT; i++)
    {
        control_leg_pose_start_deg[i] = control_leg_servo_cmd.angle_deg[i];
        control_leg_pose_target_deg[i] = desired_deg[i];
    }

    max_delta_deg = 0.0f;
    for(i = 0U; i < APP_SERVO_COUNT; i++)
    {
        float delta;
        delta = control_leg_absf(control_leg_pose_target_deg[i] - control_leg_pose_start_deg[i]);
        if(delta > max_delta_deg)
        {
            max_delta_deg = delta;
        }
    }

    duration_s = (2.1875f * max_delta_deg) / speed_limit_dps;
    if(duration_s < 0.10f)
    {
        duration_s = 0.10f;
    }
    control_leg_pose_duration_ms = (uint32)((duration_s * 1000.0f) + 0.5f);
    control_leg_pose_start_ms = now_ms;
    control_leg_trajectory_mode = LEG_TRAJECTORY_POSE;
    return APP_TRUE;
}

static void control_leg_pose_update(uint32 now_ms)
{
    uint8 i;
    uint32 elapsed_ms;
    float u;

    elapsed_ms = now_ms - control_leg_pose_start_ms;
    if(elapsed_ms >= control_leg_pose_duration_ms)
    {
        u = 1.0f;
        control_leg_s7_progress = 1.0f;
        control_leg_s7_remaining_ms = 0U;
    }
    else
    {
        u = (float)elapsed_ms / (float)control_leg_pose_duration_ms;
        control_leg_s7_progress = u;
        control_leg_s7_remaining_ms = control_leg_pose_duration_ms - elapsed_ms;
    }

    u = control_leg_s7_blend(u);
    for(i = 0U; i < APP_SERVO_COUNT; i++)
    {
        control_leg_servo_cmd.angle_deg[i] = control_leg_pose_start_deg[i] +
            ((control_leg_pose_target_deg[i] - control_leg_pose_start_deg[i]) * u);
    }
}

static uint8 control_leg_motion_can_stabilize(uint8 planner_complete)
{
    uint8 i;

    if((APP_TRUE != planner_complete) ||
       (APP_TRUE != control_leg_actuator_diag.settled))
    {
        return APP_FALSE;
    }
    for(i = 0U; i < APP_SERVO_COUNT; i++)
    {
        if(APP_SERVO_SETTLE_ERROR_DEG <
           control_leg_absf(control_leg_servo_cmd.angle_deg[i] -
                            control_leg_actuator_diag.output_deg[i]))
        {
            return APP_FALSE;
        }
    }
    return APP_TRUE;
}

static void control_leg_publish_command_pose(uint8 right_side,
                                             leg_pose_source_enum source,
                                             uint8 servo_a_index,
                                             uint8 servo_b_index,
                                             leg_pose_command_estimate_struct *command_pose)
{
    float x_mm;
    float y_mm;

    /* PWM-only forward kinematics is an open-loop command estimate, not feedback. */
    command_pose->source = source;
    if(APP_TRUE == leg_kinematics_forward_command(right_side,
                                                  control_leg_actuator_diag.output_deg[servo_a_index],
                                                  control_leg_actuator_diag.output_deg[servo_b_index],
                                                  &x_mm,
                                                  &y_mm))
    {
        command_pose->x_mm = x_mm;
        command_pose->y_mm = y_mm;
        command_pose->valid = APP_TRUE;
    }
    else
    {
        /* Retain the last valid coordinates while explicitly invalidating them. */
        command_pose->valid = APP_FALSE;
    }
}

static void control_leg_publish_diag(uint8 ik_valid, uint8 output_enable)
{
    uint8 i;
    uint8 race_cmd_valid;
    const leg_stance_profile_struct *profile;
    float configured_forward_limit_rpm;

    profile = leg_config_get_stance_profile();
    control_leg_diag.legacy_stance_target_units = control_leg_legacy_stance_target_units;
    control_leg_diag.legacy_stance_ref_units = control_leg_legacy_stance_ref_units;
    control_leg_diag.legacy_stance_rate_units_s = control_leg_legacy_stance_rate_units_s;
    if(profile->legacy_high_units > profile->legacy_low_units)
    {
        control_leg_diag.legacy_stance_norm =
            (control_leg_legacy_stance_ref_units - profile->legacy_low_units) /
            (profile->legacy_high_units - profile->legacy_low_units);
    }
    else
    {
        control_leg_diag.legacy_stance_norm = 0.0f;
    }
    control_leg_diag.legacy_stance_norm = control_leg_clamp(control_leg_diag.legacy_stance_norm, 0.0f, 1.0f);
    control_leg_diag.mode = (uint8)control_leg_mode;
    control_leg_diag.ik_valid = ik_valid;
    control_leg_diag.output_enable = output_enable;
    control_leg_diag.motion_state = control_leg_motion_state;
    control_leg_diag.fault_reason = control_leg_fault_reason;
    control_leg_diag.held_command_valid = control_leg_race_held_command_valid;
    control_leg_publish_command_pose(APP_FALSE,
                                     LEG_POSE_SOURCE_MEASURED_CALIBRATION,
                                     LEG_SERVO_FL,
                                     LEG_SERVO_RL,
                                     &control_leg_diag.left_command_pose_body_mm);
    control_leg_publish_command_pose(APP_TRUE,
                                     LEG_POSE_SOURCE_MIRROR_ASSUMPTION,
                                     LEG_SERVO_FR,
                                     LEG_SERVO_RR,
                                     &control_leg_diag.right_command_pose_body_mm);
    configured_forward_limit_rpm =
        profile->chassis_forward_limit_low_rpm +
        ((profile->chassis_forward_limit_high_rpm - profile->chassis_forward_limit_low_rpm) *
         control_leg_diag.legacy_stance_norm);
    race_cmd_valid = APP_TRUE;
    for(i = 0U; i < APP_SERVO_COUNT; i++)
    {
        if(APP_FALSE == control_leg_is_finite(control_leg_servo_cmd.angle_deg[i]))
        {
            race_cmd_valid = APP_FALSE;
            break;
        }
    }
    if((LEG_MOTION_FAULT == control_leg_motion_state) ||
       (LEG_MODE_DIRECT_LEGACY_STANCE == control_leg_mode))
    {
        control_leg_diag.drive_allowed = APP_FALSE;
        control_leg_diag.drive_forward_limit_rpm = 0.0f;
    }
    else if(LEG_MOTION_RACE_FAULT_HOLD == control_leg_motion_state)
    {
        /* Preserve the finite held command while making the chassis ramp down. */
        control_leg_diag.drive_allowed =
            ((APP_TRUE == output_enable) &&
             (APP_TRUE == race_cmd_valid) &&
             (APP_TRUE == control_leg_race_held_command_valid)) ? APP_TRUE : APP_FALSE;
        control_leg_diag.drive_forward_limit_rpm = 0.0f;
    }
    else
    {
        control_leg_diag.drive_allowed =
            ((APP_TRUE == ik_valid) && (APP_TRUE == output_enable)) ? APP_TRUE : APP_FALSE;
        if(APP_FALSE == control_leg_diag.drive_allowed)
        {
            control_leg_diag.drive_forward_limit_rpm = 0.0f;
        }
        else if(LEG_MOTION_TRANSITION == control_leg_motion_state)
        {
            control_leg_diag.drive_forward_limit_rpm = profile->transition_forward_limit_rpm;
        }
        else if(LEG_MOTION_RACE_ASSIST == control_leg_motion_state)
        {
            /* Task 3 supplies the race-speed cap; this leg mode only authorizes drive. */
            control_leg_diag.drive_forward_limit_rpm = 0.0f;
        }
        else
        {
            control_leg_diag.drive_forward_limit_rpm = configured_forward_limit_rpm;
        }
    }
    for(i = 0; i < APP_SERVO_COUNT; i++)
    {
        control_leg_diag.servo_target_deg[i] = control_leg_actuator_diag.target_deg[i];
        control_leg_diag.servo_actual_deg[i] = control_leg_actuator_diag.output_deg[i];
        control_leg_diag.servo_filtered_deg[i] = control_leg_actuator_diag.filtered_deg[i];
    }
    control_leg_diag.servo_max_error_deg = control_leg_actuator_diag.max_error_deg;
    control_leg_diag.servo_settled = control_leg_actuator_diag.settled;
    control_leg_diag.servo_fast_mode = control_leg_actuator_diag.fast_mode;
    control_leg_diag.servo_direct_bypass = control_leg_actuator_diag.direct_bypass;
    control_leg_diag.servo_s7_progress = control_leg_s7_progress;
    control_leg_diag.servo_s7_remaining_ms = control_leg_s7_remaining_ms;
    control_leg_diag.servo_trajectory_mode = (uint8)control_leg_trajectory_mode;
}

static uint8 control_leg_apply_legacy_stance(float stance_units,
                                             float *servo_fl_deg,
                                             float *servo_fr_deg,
                                             float *servo_rl_deg,
                                             float *servo_rr_deg)
{
    float delta_deg;
    float half_delta_deg;

    if((NULL == servo_fl_deg) ||
       (NULL == servo_fr_deg) ||
       (NULL == servo_rl_deg) ||
       (NULL == servo_rr_deg) ||
       (APP_FALSE == control_leg_is_finite(stance_units)) ||
       (0.0f == CONTROL_LEG_LEGACY_UNITS_PER_DELTA_DEG))
    {
        return APP_FALSE;
    }

    /*
     * Phase 1 preserves the measured differential legacy-stance map
     * independently from physical five-bar Y:
     *   d = a0 - a1 = a3 - a2
     */
    delta_deg = (stance_units - CONTROL_LEG_LEGACY_CENTER_UNITS) / CONTROL_LEG_LEGACY_UNITS_PER_DELTA_DEG;
    half_delta_deg = 0.5f * delta_deg;

    *servo_fl_deg = CONTROL_LEG_EMPIRICAL_CENTER_SERVO_DEG + half_delta_deg;
    *servo_fr_deg = CONTROL_LEG_EMPIRICAL_CENTER_SERVO_DEG - half_delta_deg;
    *servo_rl_deg = CONTROL_LEG_EMPIRICAL_CENTER_SERVO_DEG - half_delta_deg;
    *servo_rr_deg = CONTROL_LEG_EMPIRICAL_CENTER_SERVO_DEG + half_delta_deg;

    if((APP_FALSE == control_leg_servo_angle_valid(LEG_SERVO_FL, *servo_fl_deg)) ||
       (APP_FALSE == control_leg_servo_angle_valid(LEG_SERVO_FR, *servo_fr_deg)) ||
       (APP_FALSE == control_leg_servo_angle_valid(LEG_SERVO_RL, *servo_rl_deg)) ||
       (APP_FALSE == control_leg_servo_angle_valid(LEG_SERVO_RR, *servo_rr_deg)))
    {
        return APP_FALSE;
    }

    return APP_TRUE;
}

static uint8 control_leg_run_enabled(void)
{
    app_run_state_enum state;

    if(APP_TRUE == app_safety_is_fault())
    {
        return APP_FALSE;
    }
    state = app_state_get();
    return ((APP_STATE_STANDBY == state) || (APP_STATE_RUN == state)) ? APP_TRUE : APP_FALSE;
}

static uint8 control_leg_race_pose_within_zero_tolerance(float x_mm, float y_mm)
{
    float dx_mm;
    float dy_mm;
    float tolerance_squared_mm;

    if((APP_FALSE == control_leg_is_finite(x_mm)) ||
       (APP_FALSE == control_leg_is_finite(y_mm)))
    {
        return APP_FALSE;
    }
    dx_mm = x_mm - APP_RACE_ASSIST_ZERO_X_MM;
    dy_mm = y_mm - APP_RACE_ASSIST_ZERO_Y_MM;
    tolerance_squared_mm = APP_RACE_ASSIST_POSE_TOLERANCE_MM *
                             APP_RACE_ASSIST_POSE_TOLERANCE_MM;
    return (tolerance_squared_mm >= ((dx_mm * dx_mm) + (dy_mm * dy_mm))) ?
           APP_TRUE : APP_FALSE;
}

static uint8 control_leg_race_entry_ready(void)
{
    float left_planned_x_mm;
    float left_planned_y_mm;
    float right_planned_x_mm;
    float right_planned_y_mm;
    float left_output_x_mm;
    float left_output_y_mm;
    float right_output_x_mm;
    float right_output_y_mm;

    /* Race starts only from the already-settled low-race Cartesian neutral. */
    if(APP_TRUE != control_leg_actuator_diag.settled)
    {
        return APP_FALSE;
    }
    if((APP_TRUE != leg_kinematics_forward_command(APP_FALSE,
                                                    control_leg_servo_cmd.angle_deg[LEG_SERVO_FL],
                                                    control_leg_servo_cmd.angle_deg[LEG_SERVO_RL],
                                                    &left_planned_x_mm,
                                                    &left_planned_y_mm)) ||
       (APP_TRUE != leg_kinematics_forward_command(APP_TRUE,
                                                    control_leg_servo_cmd.angle_deg[LEG_SERVO_FR],
                                                    control_leg_servo_cmd.angle_deg[LEG_SERVO_RR],
                                                    &right_planned_x_mm,
                                                    &right_planned_y_mm)) ||
       (APP_TRUE != leg_kinematics_forward_command(APP_FALSE,
                                                    control_leg_actuator_diag.output_deg[LEG_SERVO_FL],
                                                    control_leg_actuator_diag.output_deg[LEG_SERVO_RL],
                                                    &left_output_x_mm,
                                                    &left_output_y_mm)) ||
       (APP_TRUE != leg_kinematics_forward_command(APP_TRUE,
                                                    control_leg_actuator_diag.output_deg[LEG_SERVO_FR],
                                                    control_leg_actuator_diag.output_deg[LEG_SERVO_RR],
                                                    &right_output_x_mm,
                                                    &right_output_y_mm)))
    {
        return APP_FALSE;
    }
    if((APP_TRUE != control_leg_race_pose_within_zero_tolerance(left_planned_x_mm,
                                                                 left_planned_y_mm)) ||
       (APP_TRUE != control_leg_race_pose_within_zero_tolerance(right_planned_x_mm,
                                                                 right_planned_y_mm)) ||
       (APP_TRUE != control_leg_race_pose_within_zero_tolerance(left_output_x_mm,
                                                                 left_output_y_mm)) ||
       (APP_TRUE != control_leg_race_pose_within_zero_tolerance(right_output_x_mm,
                                                                 right_output_y_mm)))
    {
        return APP_FALSE;
    }
    return APP_TRUE;
}

static void control_leg_write_safe_angles(const leg_config_struct *config)
{
    uint8 i;
    const leg_servo_config_struct *servo_cfg;

    for(i = 0; i < APP_SERVO_COUNT; i++)
    {
        servo_cfg = &config->servo[i];
        control_leg_servo_cmd.angle_deg[i] = control_leg_clamp(servo_cfg->safe_deg,
                                                               servo_cfg->min_deg,
                                                               servo_cfg->max_deg);
    }
}

static void control_leg_enter_fault(leg_fault_reason_enum reason)
{
    const leg_config_struct *config;

    config = leg_config_get();
    control_leg_motion_state = LEG_MOTION_FAULT;
    control_leg_fault_reason = reason;
    control_leg_legacy_stance_rate_units_s = 0.0f;
    control_leg_legacy_stance_accel_units_s2 = 0.0f;
    control_leg_settle_start_ms = 0U;
    control_leg_trajectory_mode = LEG_TRAJECTORY_NONE;
    control_leg_s7_progress = 0.0f;
    control_leg_s7_remaining_ms = 0U;
    control_leg_race_held_command_valid = APP_FALSE;
    control_leg_race_preflight_cache_invalidate();
    control_leg_diag.ik_error_count++;
    /* Keep PWM enabled when the application is otherwise runnable; actuator_servo limits this move. */
    control_leg_write_safe_angles(config);
}

static uint8 control_leg_race_solve_target(float u,
                                            const leg_ik_result_struct *left_previous,
                                            const leg_ik_result_struct *right_previous,
                                            leg_ik_result_struct *left_target,
                                            leg_ik_result_struct *right_target,
                                            float servo_deg[LEG_SERVO_COUNT])
{
    float x_mm;
    float y_mm;

    if((APP_FALSE == control_leg_is_finite(u)) ||
       (APP_FALSE == control_leg_is_finite(control_leg_race_dx_mm)) ||
       (APP_FALSE == control_leg_is_finite(control_leg_race_dy_mm)) ||
       (0.0f >= control_leg_race_dx_mm) ||
       (0.0f >= control_leg_race_dy_mm) ||
       (NULL == left_previous) ||
       (NULL == right_previous) ||
       (NULL == left_target) ||
       (NULL == right_target) ||
       (NULL == servo_deg) ||
       (APP_FALSE == left_previous->valid) ||
       (APP_FALSE == right_previous->valid))
    {
        return APP_FALSE;
    }

    x_mm = APP_RACE_ASSIST_ZERO_X_MM - (control_leg_race_dx_mm * u);
    y_mm = APP_RACE_ASSIST_ZERO_Y_MM +
           (control_leg_race_dy_mm * control_leg_absf(u));
    if((APP_TRUE != leg_kinematics_solve(APP_FALSE,
                                         x_mm,
                                         y_mm,
                                         left_previous,
                                         left_target)) ||
       (APP_TRUE != leg_kinematics_solve(APP_TRUE,
                                         x_mm,
                                         y_mm,
                                         right_previous,
                                         right_target)) ||
       (left_target->alpha_branch != left_previous->alpha_branch) ||
       (left_target->beta_branch != left_previous->beta_branch) ||
       (right_target->alpha_branch != right_previous->alpha_branch) ||
       (right_target->beta_branch != right_previous->beta_branch) ||
       (APP_TRUE != leg_kinematics_map_target_pose(&control_leg_ik_reference_left,
                                                    &control_leg_ik_reference_right,
                                                    left_target,
                                                    right_target,
                                                    servo_deg)))
    {
        return APP_FALSE;
    }
    return APP_TRUE;
}

static uint8 control_leg_race_preflight(float dx_mm, float dy_mm)
{
    uint8 sign_index;
    uint8 branch_flags;
#if (APP_RACE_ASSIST_PREFLIGHT_WCET_ENABLE == 1U)
    uint32 wcet_start_cycles;
#endif

    if((APP_FALSE == control_leg_ik_reference_valid) ||
       (APP_FALSE == control_leg_ik_previous_left.valid) ||
       (APP_FALSE == control_leg_ik_previous_right.valid) ||
       (APP_FALSE == control_leg_is_finite(dx_mm)) ||
       (APP_FALSE == control_leg_is_finite(dy_mm)) ||
       (0.0f >= dx_mm) ||
       (0.0f >= dy_mm))
    {
        return APP_FALSE;
    }

    if(APP_TRUE != control_leg_race_branch_flags(&branch_flags))
    {
        return APP_FALSE;
    }
    if((APP_TRUE == control_leg_race_preflight_cache_valid) &&
       (0.001f >= control_leg_absf(dx_mm - control_leg_race_preflight_dx_mm)) &&
       (0.001f >= control_leg_absf(dy_mm - control_leg_race_preflight_dy_mm)) &&
       (branch_flags == control_leg_race_preflight_branch_flags))
    {
        return APP_TRUE;
    }

#if (APP_RACE_ASSIST_PREFLIGHT_WCET_ENABLE == 1U)
    wcet_start_cycles = control_leg_race_preflight_wcet_begin();
#endif

    for(sign_index = 0U; sign_index < 2U; sign_index++)
    {
        leg_ik_result_struct left_previous;
        leg_ik_result_struct right_previous;
        int step;

        left_previous = control_leg_ik_previous_left;
        right_previous = control_leg_ik_previous_right;
        for(step = 0; step < (int)APP_RACE_ASSIST_PATH_SAMPLE_COUNT; step++)
        {
            const float sign = (0U == sign_index) ? 1.0f : -1.0f;
            const float u = sign * (float)step /
                            (float)(APP_RACE_ASSIST_PATH_SAMPLE_COUNT - 1U);
            leg_ik_result_struct left_target;
            leg_ik_result_struct right_target;
            float servo_deg[LEG_SERVO_COUNT];

            if(APP_TRUE != control_leg_race_solve_target(u,
                                                          &left_previous,
                                                          &right_previous,
                                                          &left_target,
                                                          &right_target,
                                                          servo_deg))
            {
                control_leg_race_preflight_cache_invalidate();
#if (APP_RACE_ASSIST_PREFLIGHT_WCET_ENABLE == 1U)
                control_leg_race_preflight_wcet_end(wcet_start_cycles);
#endif
                return APP_FALSE;
            }
            left_previous = left_target;
            right_previous = right_target;
        }
    }
    control_leg_race_preflight_dx_mm = dx_mm;
    control_leg_race_preflight_dy_mm = dy_mm;
    control_leg_race_preflight_branch_flags = branch_flags;
    control_leg_race_preflight_cache_valid = APP_TRUE;
#if (APP_RACE_ASSIST_PREFLIGHT_WCET_ENABLE == 1U)
    control_leg_race_preflight_wcet_end(wcet_start_cycles);
#endif
    return APP_TRUE;
}

static uint8 control_leg_race_start_segment(float u_target, uint32 now_ms)
{
    leg_ik_result_struct left_start;
    leg_ik_result_struct right_start;
    leg_ik_result_struct left_target;
    leg_ik_result_struct right_target;
    float start_deg[LEG_SERVO_COUNT];
    float target_deg[LEG_SERVO_COUNT];
    float max_delta_deg;
    float duration_s;
    uint8 i;

    if((APP_FALSE == control_leg_is_finite(u_target)) ||
       (1.0f < control_leg_absf(u_target)) ||
       (APP_TRUE != control_leg_race_solve_target(control_leg_race_u_actual,
                                                   &control_leg_ik_previous_left,
                                                   &control_leg_ik_previous_right,
                                                   &left_start,
                                                   &right_start,
                                                   start_deg)) ||
       (APP_TRUE != control_leg_race_solve_target(u_target,
                                                   &left_start,
                                                   &right_start,
                                                   &left_target,
                                                   &right_target,
                                                   target_deg)))
    {
        return APP_FALSE;
    }

    max_delta_deg = 0.0f;
    for(i = 0U; i < APP_SERVO_COUNT; i++)
    {
        const float delta_deg = control_leg_absf(target_deg[i] - start_deg[i]);

        if(delta_deg > max_delta_deg)
        {
            max_delta_deg = delta_deg;
        }
    }
    duration_s = (2.1875f * max_delta_deg) / APP_RACE_ASSIST_SERVO_SPEED_DPS;
    if(duration_s < 0.10f)
    {
        duration_s = 0.10f;
    }

    control_leg_race_u_start = control_leg_race_u_actual;
    control_leg_race_u_target = u_target;
    control_leg_race_start_ms = now_ms;
    control_leg_race_duration_ms = (uint32)((duration_s * 1000.0f) + 0.5f);
    control_leg_trajectory_mode = LEG_TRAJECTORY_RACE_ASSIST;
    control_leg_s7_progress = 0.0f;
    control_leg_s7_remaining_ms = control_leg_race_duration_ms;
    return APP_TRUE;
}

static uint8 control_leg_race_sign_change(float current, float requested)
{
    if((0.0f < current) && (0.0f > requested))
    {
        return APP_TRUE;
    }
    if((0.0f > current) && (0.0f < requested))
    {
        return APP_TRUE;
    }
    return APP_FALSE;
}

static void control_leg_enter_race_fault_hold(leg_fault_reason_enum reason)
{
    control_leg_race_held_command_valid =
        ((APP_TRUE == control_leg_race_held_command_valid) &&
         (APP_TRUE == control_leg_race_command_valid())) ? APP_TRUE : APP_FALSE;
    control_leg_motion_state = LEG_MOTION_RACE_FAULT_HOLD;
    control_leg_fault_reason = reason;
    control_leg_trajectory_mode = LEG_TRAJECTORY_NONE;
    control_leg_race_u_target = control_leg_race_u_actual;
    control_leg_race_u_pending = control_leg_race_u_actual;
    control_leg_race_pending_valid = APP_FALSE;
    control_leg_race_path_valid = APP_FALSE;
    control_leg_race_preflight_cache_invalidate();
    control_leg_diag.race_path_valid = APP_FALSE;
    control_leg_diag.ik_error_count++;
}

static uint8 control_leg_race_start_pending(uint32 now_ms)
{
    const float requested = control_leg_race_u_pending;

    control_leg_race_pending_valid = APP_FALSE;
    if(APP_TRUE == control_leg_race_sign_change(control_leg_race_u_actual, requested))
    {
        control_leg_race_u_pending = requested;
        control_leg_race_pending_valid = APP_TRUE;
        return control_leg_race_start_segment(0.0f, now_ms);
    }
    return control_leg_race_start_segment(requested, now_ms);
}

static void control_leg_race_update_diag(const leg_ik_result_struct *left_target,
                                         const leg_ik_result_struct *right_target,
                                         float x_mm,
                                         float y_mm)
{
    control_leg_diag.race_assist_request = (APP_TRUE == control_leg_race_pending_valid) ?
                                           control_leg_race_u_pending : control_leg_race_u_target;
    control_leg_diag.race_assist_actual = control_leg_race_u_actual;
    control_leg_diag.race_target_x_mm = x_mm;
    control_leg_diag.race_target_y_mm = y_mm;
    control_leg_diag.left_ik_margin = left_target->singularity_margin;
    control_leg_diag.right_ik_margin = right_target->singularity_margin;
    control_leg_diag.ik_margin = (left_target->singularity_margin < right_target->singularity_margin) ?
                                      left_target->singularity_margin : right_target->singularity_margin;
    control_leg_diag.ik_branch_flags = 0U;
    if(LEG_IK_BRANCH_MINUS == left_target->alpha_branch)
    {
        control_leg_diag.ik_branch_flags |= (1U << 0);
    }
    if(LEG_IK_BRANCH_MINUS == left_target->beta_branch)
    {
        control_leg_diag.ik_branch_flags |= (1U << 1);
    }
    if(LEG_IK_BRANCH_MINUS == right_target->alpha_branch)
    {
        control_leg_diag.ik_branch_flags |= (1U << 2);
    }
    if(LEG_IK_BRANCH_MINUS == right_target->beta_branch)
    {
        control_leg_diag.ik_branch_flags |= (1U << 3);
    }
    control_leg_diag.race_path_valid = control_leg_race_path_valid;
}

static void control_leg_race_update(uint32 now_ms)
{
    leg_ik_result_struct left_target;
    leg_ik_result_struct right_target;
    float servo_deg[LEG_SERVO_COUNT];
    float progress;
    float blend;
    float x_mm;
    float y_mm;
    uint32 elapsed_ms;
    uint8 i;
    uint8 output_enable;

    if((APP_FALSE == control_leg_race_path_valid) ||
       (0U == control_leg_race_duration_ms) ||
       (APP_FALSE == control_leg_ik_reference_valid))
    {
        control_leg_enter_race_fault_hold(LEG_FAULT_IK_INVALID);
        control_leg_publish_diag(APP_FALSE, control_leg_run_enabled());
        return;
    }

    elapsed_ms = now_ms - control_leg_race_start_ms;
    progress = control_leg_clamp((float)elapsed_ms / (float)control_leg_race_duration_ms,
                                 0.0f,
                                 1.0f);
    blend = control_leg_s7_blend(progress);
    control_leg_race_u_actual = control_leg_race_u_start +
        ((control_leg_race_u_target - control_leg_race_u_start) * blend);
    x_mm = APP_RACE_ASSIST_ZERO_X_MM -
           (control_leg_race_dx_mm * control_leg_race_u_actual);
    y_mm = APP_RACE_ASSIST_ZERO_Y_MM +
           (control_leg_race_dy_mm * control_leg_absf(control_leg_race_u_actual));

    if(APP_TRUE != control_leg_race_solve_target(control_leg_race_u_actual,
                                                  &control_leg_ik_previous_left,
                                                  &control_leg_ik_previous_right,
                                                  &left_target,
                                                  &right_target,
                                                  servo_deg))
    {
        control_leg_enter_race_fault_hold(LEG_FAULT_IK_INVALID);
        control_leg_publish_diag(APP_FALSE, control_leg_run_enabled());
        return;
    }
    for(i = 0U; i < APP_SERVO_COUNT; i++)
    {
        control_leg_servo_cmd.angle_deg[i] = servo_deg[i];
    }
    control_leg_race_held_command_valid = APP_TRUE;
    control_leg_ik_previous_left = left_target;
    control_leg_ik_previous_right = right_target;
    control_leg_s7_progress = progress;
    control_leg_s7_remaining_ms = (elapsed_ms < control_leg_race_duration_ms) ?
                                       (control_leg_race_duration_ms - elapsed_ms) : 0U;
    control_leg_motion_state = LEG_MOTION_RACE_ASSIST;
    control_leg_fault_reason = LEG_FAULT_NONE;
    control_leg_race_update_diag(&left_target, &right_target, x_mm, y_mm);

    if((1.0f <= progress) && (APP_TRUE == control_leg_race_pending_valid) &&
       (APP_TRUE != control_leg_race_start_pending(now_ms)))
    {
        control_leg_enter_race_fault_hold(LEG_FAULT_IK_INVALID);
        control_leg_publish_diag(APP_FALSE, control_leg_run_enabled());
        return;
    }

    output_enable = control_leg_run_enabled();
    control_leg_publish_diag(APP_TRUE, output_enable);
}

void control_leg_init(void)
{
    uint8 i;
    const leg_config_struct *config;

    config = leg_config_get();
    control_leg_mode = LEG_MODE_LOCK;
    control_leg_legacy_stance_cmd = 0.0f;
    control_leg_pitch_cmd = 0.0f;
    control_leg_roll_cmd = 0.0f;

    for(i = 0; i < APP_SERVO_COUNT; i++)
    {
        control_leg_manual_angle[i] = config->servo[i].safe_deg;
        control_leg_servo_cmd.angle_deg[i] = config->servo[i].safe_deg;
        control_leg_servo_cmd.enable[i] = APP_FALSE;
    }

    {
        const leg_stance_profile_struct *profile;
        profile = leg_config_get_stance_profile();
        control_leg_legacy_stance_target_units = profile->legacy_default_units;
        control_leg_legacy_stance_ref_units = profile->legacy_default_units;
        control_leg_legacy_stance_rate_units_s = 0.0f;
        control_leg_legacy_stance_accel_units_s2 = 0.0f;
        control_leg_motion_state = LEG_MOTION_LOCKED;
        control_leg_fault_reason = LEG_FAULT_NONE;
        control_leg_settle_start_ms = 0U;
        control_leg_last_update_ms = 0U;
        control_leg_fast_stance_start_units = control_leg_legacy_stance_ref_units;
        control_leg_fast_stance_start_ms = 0U;
        control_leg_ik_target_x_mm = 0.0f;
        control_leg_ik_target_y_mm = 0.0f;
        control_leg_ik_reference_left.valid = APP_FALSE;
        control_leg_ik_reference_right.valid = APP_FALSE;
        control_leg_ik_previous_left.valid = APP_FALSE;
        control_leg_ik_previous_right.valid = APP_FALSE;
        control_leg_ik_reference_valid = APP_FALSE;
        control_leg_race_u_start = 0.0f;
        control_leg_race_u_target = 0.0f;
        control_leg_race_u_pending = 0.0f;
        control_leg_race_u_actual = 0.0f;
        control_leg_race_dx_mm = 0.0f;
        control_leg_race_dy_mm = 0.0f;
        control_leg_race_start_ms = 0U;
        control_leg_race_duration_ms = 0U;
        control_leg_race_pending_valid = APP_FALSE;
        control_leg_race_path_valid = APP_FALSE;
        control_leg_race_preflight_dx_mm = 0.0f;
        control_leg_race_preflight_dy_mm = 0.0f;
        control_leg_race_preflight_branch_flags = 0U;
        control_leg_race_preflight_cache_valid = APP_FALSE;
        control_leg_race_held_command_valid = APP_FALSE;
#if (APP_RACE_ASSIST_PREFLIGHT_WCET_ENABLE == 1U)
        control_leg_race_preflight_count = 0U;
        control_leg_race_preflight_last_cycles = 0U;
        control_leg_race_preflight_max_cycles = 0U;
#endif
        control_leg_trajectory_mode = LEG_TRAJECTORY_NONE;
        control_leg_s7_progress = 0.0f;
        control_leg_s7_remaining_ms = 0U;
        control_leg_pose_start_ms = 0U;
        control_leg_pose_duration_ms = 0U;
        {
            uint8 j;
            for(j = 0; j < APP_SERVO_COUNT; j++)
            {
                control_leg_pose_start_deg[j] = config->servo[j].safe_deg;
                control_leg_pose_target_deg[j] = config->servo[j].safe_deg;
            }
        }
        control_leg_diag.ik_error_count = 0U;
        control_leg_diag.race_assist_request = 0.0f;
        control_leg_diag.race_assist_actual = 0.0f;
        control_leg_diag.race_target_x_mm = APP_RACE_ASSIST_ZERO_X_MM;
        control_leg_diag.race_target_y_mm = APP_RACE_ASSIST_ZERO_Y_MM;
        control_leg_diag.left_ik_margin = 0.0f;
        control_leg_diag.right_ik_margin = 0.0f;
        control_leg_diag.ik_branch_flags = 0U;
        control_leg_diag.race_path_valid = APP_FALSE;
        control_leg_diag.left_command_pose_body_mm.source = LEG_POSE_SOURCE_NONE;
        control_leg_diag.left_command_pose_body_mm.valid = APP_FALSE;
        control_leg_diag.right_command_pose_body_mm.source = LEG_POSE_SOURCE_NONE;
        control_leg_diag.right_command_pose_body_mm.valid = APP_FALSE;
    }

    control_leg_publish_diag(APP_FALSE, APP_FALSE);
}

void control_leg_update(uint32 now_ms)
{
    uint8 i;
    const leg_config_struct *config;
    const leg_servo_config_struct *servo_cfg;

    config = leg_config_get();

    actuator_servo_get_diag(&control_leg_actuator_diag);

#if (APP_LEG_CALIB_ENABLE == 1U)
    control_leg_mode = LEG_MODE_MANUAL;
#endif

#if (APP_LEG_VERIFY_ENABLE == 1U)
    if(APP_FALSE == control_leg_verify_active)
    {
        control_leg_verify_start_ms = now_ms;
        control_leg_verify_active = APP_TRUE;
    }
    if((now_ms - control_leg_verify_start_ms) >= APP_LEG_VERIFY_DELAY_MS)
    {
        control_leg_mode = LEG_MODE_LEGACY_STANCE;
        control_leg_legacy_stance_cmd = APP_LEG_VERIFY_LEGACY_STANCE_CMD;
        control_leg_pitch_cmd = APP_LEG_VERIFY_PITCH_CMD;
        control_leg_roll_cmd = APP_LEG_VERIFY_ROLL_CMD;
    }
#endif

    if(LEG_MOTION_FAULT == control_leg_motion_state)
    {
        control_leg_write_safe_angles(config);
        control_leg_publish_diag(APP_FALSE, control_leg_run_enabled());
    }
    else if(LEG_MOTION_RACE_FAULT_HOLD == control_leg_motion_state)
    {
        /* Race fault hold intentionally retains the last finite planned command. */
        control_leg_publish_diag(APP_FALSE, control_leg_run_enabled());
    }
    else
    {
        switch(control_leg_mode)
        {
            case LEG_MODE_MANUAL:
            {
                float desired_deg[APP_SERVO_COUNT];

#if (APP_LEG_CALIB_ENABLE == 1U)
                servo_cfg = &config->servo[APP_LEG_CALIB_SERVO_ID];
                control_leg_manual_angle[APP_LEG_CALIB_SERVO_ID] = control_leg_clamp(servo_cfg->safe_deg + APP_LEG_CALIB_OFFSET_DEG,
                                                                                      servo_cfg->min_deg,
                                                                                      servo_cfg->max_deg);
#endif
                for(i = 0; i < APP_SERVO_COUNT; i++)
                {
                    servo_cfg = &config->servo[i];
                    desired_deg[i] = control_leg_clamp(control_leg_manual_angle[i],
                                                       servo_cfg->min_deg,
                                                       servo_cfg->max_deg);
                }
                control_leg_pose_start_if_changed(desired_deg, APP_SERVO_MAX_SPEED_DPS, now_ms);
                control_leg_pose_update(now_ms);
                if(APP_TRUE == control_leg_motion_can_stabilize(1.0f <= control_leg_s7_progress ? APP_TRUE : APP_FALSE))
                {
                    control_leg_motion_state = LEG_MOTION_STABLE;
                }
                else
                {
                    control_leg_motion_state = LEG_MOTION_TRANSITION;
                }
                control_leg_fault_reason = LEG_FAULT_NONE;
                control_leg_publish_diag(APP_FALSE, control_leg_run_enabled());
                break;
            }

            case LEG_MODE_LEGACY_STANCE:
            {
                const leg_stance_profile_struct *profile;
                float servo_fl_deg;
                float servo_fr_deg;
                float servo_rl_deg;
                float servo_rr_deg;
                float error_units;
                float desired_rate_units_s;
                float desired_accel_units_s2;
                float accel_delta_units_s2;
                float dt_s;
                uint32 dt_ms;
                uint8 output_enable;

                dt_ms = now_ms - control_leg_last_update_ms;
                if(0U == dt_ms)
                {
                    dt_s = 0.0f;
                }
                else
                {
                    dt_s = (float)dt_ms * 0.001f;
                }
                control_leg_last_update_ms = now_ms;

                profile = leg_config_get_stance_profile();
                if((APP_FALSE == control_leg_is_finite(control_leg_legacy_stance_ref_units)) ||
                   (APP_FALSE == control_leg_is_finite(control_leg_legacy_stance_rate_units_s)) ||
                   (APP_FALSE == control_leg_is_finite(control_leg_legacy_stance_accel_units_s2)) ||
                   (0.0f >= profile->legacy_max_rate_units_s) ||
                   (0.0f >= profile->legacy_max_accel_units_s2) ||
                   (0.0f >= profile->legacy_max_jerk_units_s3) ||
                   (0.0f >= profile->legacy_position_kp_s) ||
                   (0.0f >= profile->legacy_rate_kp_s))
                {
                    control_leg_enter_fault(LEG_FAULT_SERVO_LIMIT);
                    control_leg_publish_diag(APP_FALSE, control_leg_run_enabled());
                    break;
                }
                error_units = control_leg_legacy_stance_target_units - control_leg_legacy_stance_ref_units;
                desired_rate_units_s = control_leg_clamp(error_units * profile->legacy_position_kp_s,
                                                         -profile->legacy_max_rate_units_s,
                                                         profile->legacy_max_rate_units_s);
                desired_accel_units_s2 = control_leg_clamp(
                    (desired_rate_units_s - control_leg_legacy_stance_rate_units_s) * profile->legacy_rate_kp_s,
                    -profile->legacy_max_accel_units_s2,
                    profile->legacy_max_accel_units_s2);
                accel_delta_units_s2 = control_leg_clamp(
                    desired_accel_units_s2 - control_leg_legacy_stance_accel_units_s2,
                    -profile->legacy_max_jerk_units_s3 * dt_s,
                    profile->legacy_max_jerk_units_s3 * dt_s);
                control_leg_legacy_stance_accel_units_s2 = control_leg_clamp(
                    control_leg_legacy_stance_accel_units_s2 + accel_delta_units_s2,
                    -profile->legacy_max_accel_units_s2,
                    profile->legacy_max_accel_units_s2);
                control_leg_legacy_stance_rate_units_s = control_leg_clamp(
                    control_leg_legacy_stance_rate_units_s +
                    (control_leg_legacy_stance_accel_units_s2 * dt_s),
                    -profile->legacy_max_rate_units_s,
                    profile->legacy_max_rate_units_s);
                control_leg_legacy_stance_ref_units += control_leg_legacy_stance_rate_units_s * dt_s;
                if((0.01f >= control_leg_absf(control_leg_legacy_stance_target_units -
                                               control_leg_legacy_stance_ref_units)) &&
                   (0.05f >= control_leg_absf(control_leg_legacy_stance_rate_units_s)) &&
                   ((profile->legacy_max_jerk_units_s3 * dt_s) >=
                    control_leg_absf(control_leg_legacy_stance_accel_units_s2)))
                {
                    control_leg_legacy_stance_ref_units = control_leg_legacy_stance_target_units;
                    control_leg_legacy_stance_rate_units_s = 0.0f;
                    control_leg_legacy_stance_accel_units_s2 = 0.0f;
                }

                if(APP_TRUE == control_leg_apply_legacy_stance(control_leg_legacy_stance_ref_units,
                                                               &servo_fl_deg,
                                                               &servo_fr_deg,
                                                               &servo_rl_deg,
                                                               &servo_rr_deg))
                {
                    control_leg_servo_cmd.angle_deg[LEG_SERVO_FL] = servo_fl_deg;
                    control_leg_servo_cmd.angle_deg[LEG_SERVO_RL] = servo_rl_deg;
                    control_leg_servo_cmd.angle_deg[LEG_SERVO_FR] = servo_fr_deg;
                    control_leg_servo_cmd.angle_deg[LEG_SERVO_RR] = servo_rr_deg;
                    control_leg_diag.ik_margin = CONTROL_LEG_EMPIRICAL_IK_MARGIN;
                    {
                        uint8 planner_complete;
                        planner_complete = APP_FALSE;
                        if((profile->legacy_settle_error_units >=
                            control_leg_absf(control_leg_legacy_stance_target_units -
                                             control_leg_legacy_stance_ref_units)) &&
                           (0.0f == control_leg_legacy_stance_rate_units_s))
                        {
                            if(LEG_MOTION_TRANSITION != control_leg_motion_state)
                            {
                                control_leg_settle_start_ms = now_ms;
                                control_leg_motion_state = LEG_MOTION_TRANSITION;
                            }
                            if((now_ms - control_leg_settle_start_ms) >= profile->legacy_settle_ms)
                            {
                                planner_complete = APP_TRUE;
                            }
                        }
                        else
                        {
                            control_leg_settle_start_ms = now_ms;
                            control_leg_motion_state = LEG_MOTION_TRANSITION;
                        }
                        if(APP_TRUE == control_leg_motion_can_stabilize(planner_complete))
                        {
                            control_leg_motion_state = LEG_MOTION_STABLE;
                        }
                    }
                    output_enable = control_leg_run_enabled();
                    control_leg_publish_diag(APP_TRUE, output_enable);
                }
                else
                {
                    control_leg_enter_fault(LEG_FAULT_SERVO_LIMIT);
                    control_leg_publish_diag(APP_FALSE, control_leg_run_enabled());
                }
                break;
            }

            case LEG_MODE_IK_CALIB:
            {
                float desired_deg[APP_SERVO_COUNT];

                for(i = 0; i < APP_SERVO_COUNT; i++)
                {
                    desired_deg[i] = control_leg_manual_angle[i];
                }
                control_leg_pose_start_if_changed(desired_deg, APP_SERVO_MAX_SPEED_DPS, now_ms);
                control_leg_pose_update(now_ms);
                if(APP_TRUE == control_leg_motion_can_stabilize(1.0f <= control_leg_s7_progress ? APP_TRUE : APP_FALSE))
                {
                    control_leg_motion_state = LEG_MOTION_STABLE;
                }
                else
                {
                    control_leg_motion_state = LEG_MOTION_TRANSITION;
                }
                control_leg_fault_reason = LEG_FAULT_NONE;
                control_leg_publish_diag(APP_FALSE, control_leg_run_enabled());
                break;
            }

            case LEG_MODE_FAST_LEGACY_STANCE:
            {
                const leg_stance_profile_struct *profile;
                float progress;
                float blend;
                float blend_rate;
                float servo_fl_deg;
                float servo_fr_deg;
                float servo_rl_deg;
                float servo_rr_deg;
                uint32 elapsed_ms;
                uint8 output_enable;

                profile = leg_config_get_stance_profile();
                if((0U == profile->fast_stance_transition_ms) ||
                   (APP_FALSE == control_leg_is_finite(control_leg_fast_stance_start_units)) ||
                   (APP_FALSE == control_leg_is_finite(control_leg_legacy_stance_target_units)))
                {
                    control_leg_enter_fault(LEG_FAULT_SERVO_LIMIT);
                    control_leg_publish_diag(APP_FALSE, control_leg_run_enabled());
                    break;
                }
                elapsed_ms = now_ms - control_leg_fast_stance_start_ms;
                progress = control_leg_clamp((float)elapsed_ms /
                                             (float)profile->fast_stance_transition_ms,
                                             0.0f,
                                             1.0f);
                blend = control_leg_fast_stance_blend(progress);
                blend_rate = (140.0f * progress * progress * progress) -
                             (420.0f * progress * progress * progress * progress) +
                             (420.0f * progress * progress * progress * progress * progress) -
                             (140.0f * progress * progress * progress * progress * progress * progress);
                control_leg_s7_progress = progress;
                control_leg_s7_remaining_ms = (elapsed_ms < profile->fast_stance_transition_ms) ?
                    (profile->fast_stance_transition_ms - elapsed_ms) : 0U;
                control_leg_trajectory_mode = LEG_TRAJECTORY_FAST_LEGACY_STANCE;
                control_leg_legacy_stance_ref_units = control_leg_fast_stance_start_units +
                    ((control_leg_legacy_stance_target_units - control_leg_fast_stance_start_units) * blend);
                control_leg_legacy_stance_rate_units_s =
                    ((control_leg_legacy_stance_target_units - control_leg_fast_stance_start_units) *
                     blend_rate * 1000.0f) / (float)profile->fast_stance_transition_ms;
                control_leg_legacy_stance_accel_units_s2 = 0.0f;
                if(APP_TRUE == control_leg_apply_legacy_stance(control_leg_legacy_stance_ref_units,
                                                               &servo_fl_deg,
                                                               &servo_fr_deg,
                                                               &servo_rl_deg,
                                                               &servo_rr_deg))
                {
                    control_leg_servo_cmd.angle_deg[LEG_SERVO_FL] = servo_fl_deg;
                    control_leg_servo_cmd.angle_deg[LEG_SERVO_RL] = servo_rl_deg;
                    control_leg_servo_cmd.angle_deg[LEG_SERVO_FR] = servo_fr_deg;
                    control_leg_servo_cmd.angle_deg[LEG_SERVO_RR] = servo_rr_deg;
                    control_leg_diag.ik_margin = CONTROL_LEG_EMPIRICAL_IK_MARGIN;
                    if(1.0f <= progress)
                    {
                        control_leg_legacy_stance_ref_units = control_leg_legacy_stance_target_units;
                        control_leg_legacy_stance_rate_units_s = 0.0f;
                        if(APP_TRUE == control_leg_motion_can_stabilize(APP_TRUE))
                        {
                            control_leg_motion_state = LEG_MOTION_STABLE;
                        }
                        else
                        {
                            control_leg_motion_state = LEG_MOTION_TRANSITION;
                        }
                    }
                    else
                    {
                        control_leg_motion_state = LEG_MOTION_TRANSITION;
                    }
                    control_leg_fault_reason = LEG_FAULT_NONE;
                    output_enable = control_leg_run_enabled();
                    control_leg_publish_diag(APP_TRUE, output_enable);
                }
                else
                {
                    control_leg_enter_fault(LEG_FAULT_SERVO_LIMIT);
                    control_leg_publish_diag(APP_FALSE, control_leg_run_enabled());
                }
                break;
            }

            case LEG_MODE_DIRECT_LEGACY_STANCE:
            {
                float servo_fl_deg;
                float servo_fr_deg;
                float servo_rl_deg;
                float servo_rr_deg;
                uint8 output_enable;

                control_leg_legacy_stance_rate_units_s = 0.0f;
                control_leg_legacy_stance_accel_units_s2 = 0.0f;
                if(APP_TRUE == control_leg_apply_legacy_stance(control_leg_legacy_stance_ref_units,
                                                               &servo_fl_deg,
                                                               &servo_fr_deg,
                                                               &servo_rl_deg,
                                                               &servo_rr_deg))
                {
                    control_leg_servo_cmd.angle_deg[LEG_SERVO_FL] = servo_fl_deg;
                    control_leg_servo_cmd.angle_deg[LEG_SERVO_RL] = servo_rl_deg;
                    control_leg_servo_cmd.angle_deg[LEG_SERVO_FR] = servo_fr_deg;
                    control_leg_servo_cmd.angle_deg[LEG_SERVO_RR] = servo_rr_deg;
                    control_leg_diag.ik_margin = CONTROL_LEG_EMPIRICAL_IK_MARGIN;
                    control_leg_motion_state = LEG_MOTION_TRANSITION;
                    control_leg_fault_reason = LEG_FAULT_NONE;
                    output_enable = control_leg_run_enabled();
                    control_leg_publish_diag(APP_TRUE, output_enable);
                }
                else
                {
                    control_leg_enter_fault(LEG_FAULT_SERVO_LIMIT);
                    control_leg_publish_diag(APP_FALSE, control_leg_run_enabled());
                }
                break;
            }

            case LEG_MODE_IK_REFERENCE:
            {
                const leg_kinematics_config_struct *kinematics;
                float servo_deg[LEG_SERVO_COUNT];
                float desired_deg[APP_SERVO_COUNT];
                uint8 output_enable;

                kinematics = leg_config_get_kinematics();
                if((NULL == kinematics) || (APP_FALSE == control_leg_ik_reference_valid) ||
                   (APP_TRUE != leg_kinematics_map_reference_pose(&control_leg_ik_reference_left,
                                                                   &control_leg_ik_reference_right,
                                                                   servo_deg)))
                {
                    control_leg_enter_fault(LEG_FAULT_IK_INVALID);
                    control_leg_publish_diag(APP_FALSE, control_leg_run_enabled());
                    break;
                }
                for(i = 0; i < APP_SERVO_COUNT; i++)
                {
                    desired_deg[i] = servo_deg[i];
                }
                control_leg_pose_start_if_changed(desired_deg, APP_SERVO_MAX_SPEED_DPS, now_ms);
                control_leg_pose_update(now_ms);
                control_leg_diag.ik_margin = (control_leg_ik_reference_left.singularity_margin <
                                               control_leg_ik_reference_right.singularity_margin) ?
                                              control_leg_ik_reference_left.singularity_margin :
                                              control_leg_ik_reference_right.singularity_margin;
                if(APP_TRUE == control_leg_motion_can_stabilize(1.0f <= control_leg_s7_progress ? APP_TRUE : APP_FALSE))
                {
                    control_leg_motion_state = LEG_MOTION_STABLE;
                }
                else
                {
                    control_leg_motion_state = LEG_MOTION_TRANSITION;
                }
                control_leg_fault_reason = LEG_FAULT_NONE;
                output_enable = control_leg_run_enabled();
                control_leg_publish_diag(APP_TRUE, output_enable);
                break;
            }

            case LEG_MODE_IK_VALIDATE:
            {
                leg_ik_result_struct left_target;
                leg_ik_result_struct right_target;
                float servo_deg[LEG_SERVO_COUNT];
                float desired_deg[APP_SERVO_COUNT];
                uint8 output_enable;

                if((APP_FALSE == control_leg_ik_reference_valid) ||
                   (APP_TRUE != leg_kinematics_solve(APP_FALSE,
                                                      control_leg_ik_target_x_mm,
                                                      control_leg_ik_target_y_mm,
                                                      &control_leg_ik_previous_left,
                                                      &left_target)) ||
                   (APP_TRUE != leg_kinematics_solve(APP_TRUE,
                                                      control_leg_ik_target_x_mm,
                                                      control_leg_ik_target_y_mm,
                                                      &control_leg_ik_previous_right,
                                                      &right_target)))
                {
                    control_leg_enter_fault(LEG_FAULT_IK_INVALID);
                    control_leg_publish_diag(APP_FALSE, control_leg_run_enabled());
                    break;
                }
                if(APP_TRUE != leg_kinematics_map_target_pose(&control_leg_ik_reference_left,
                                                               &control_leg_ik_reference_right,
                                                               &left_target,
                                                               &right_target,
                                                               servo_deg))
                {
                    control_leg_enter_fault(LEG_FAULT_SERVO_LIMIT);
                    control_leg_publish_diag(APP_FALSE, control_leg_run_enabled());
                    break;
                }
                for(i = 0; i < APP_SERVO_COUNT; i++)
                {
                    desired_deg[i] = servo_deg[i];
                }
                control_leg_pose_start_if_changed(desired_deg, APP_SERVO_MAX_SPEED_DPS, now_ms);
                control_leg_pose_update(now_ms);
                control_leg_ik_previous_left = left_target;
                control_leg_ik_previous_right = right_target;
                control_leg_diag.ik_margin = (left_target.singularity_margin < right_target.singularity_margin) ?
                                              left_target.singularity_margin : right_target.singularity_margin;
                if(APP_TRUE == control_leg_motion_can_stabilize(1.0f <= control_leg_s7_progress ? APP_TRUE : APP_FALSE))
                {
                    control_leg_motion_state = LEG_MOTION_STABLE;
                }
                else
                {
                    control_leg_motion_state = LEG_MOTION_TRANSITION;
                }
                control_leg_fault_reason = LEG_FAULT_NONE;
                output_enable = control_leg_run_enabled();
                control_leg_publish_diag(APP_TRUE, output_enable);
                break;
            }

            case LEG_MODE_RACE_ASSIST:
            {
                control_leg_race_update(now_ms);
                break;
            }

            case LEG_MODE_LOCK:
            default:
            {
                const leg_stance_profile_struct *profile;

                profile = leg_config_get_stance_profile();
                control_leg_legacy_stance_target_units = profile->legacy_safe_support_units;
                control_leg_legacy_stance_ref_units = profile->legacy_safe_support_units;
                control_leg_legacy_stance_rate_units_s = 0.0f;
                control_leg_legacy_stance_accel_units_s2 = 0.0f;
                control_leg_write_safe_angles(config);
                control_leg_motion_state = LEG_MOTION_LOCKED;
                control_leg_fault_reason = LEG_FAULT_NONE;
                control_leg_trajectory_mode = LEG_TRAJECTORY_NONE;
                control_leg_s7_progress = 0.0f;
                control_leg_s7_remaining_ms = 0U;
                control_leg_publish_diag(APP_FALSE, control_leg_run_enabled());
                break;
            }
        }
    }

    for(i = 0; i < APP_SERVO_COUNT; i++)
    {
        servo_cfg = &config->servo[i];
#if (APP_LEG_CALIB_ENABLE == 1U)
        if((i == APP_LEG_CALIB_SERVO_ID) &&
           (APP_TRUE == control_leg_safe_deg_valid(servo_cfg->safe_deg)) &&
           (APP_TRUE == control_leg_run_enabled()))
        {
            control_leg_servo_cmd.enable[i] = APP_TRUE;
        }
        else
        {
            control_leg_servo_cmd.enable[i] = APP_FALSE;
        }
#else
        if((APP_TRUE == control_leg_servo_is_active(i)) &&
           (APP_TRUE == control_leg_safe_deg_valid(servo_cfg->safe_deg)) &&
           (APP_TRUE == control_leg_run_enabled()))
        {
            control_leg_servo_cmd.enable[i] = APP_TRUE;
        }
        else
        {
            control_leg_servo_cmd.enable[i] = APP_FALSE;
        }
#endif
    }

    {
        float speed_limit_dps;
        uint8 direct_bypass;

        speed_limit_dps = (LEG_MODE_FAST_LEGACY_STANCE == control_leg_mode) ?
                          APP_LEG_FAST_SERVO_MAX_SPEED_DPS :
                          ((LEG_MODE_RACE_ASSIST == control_leg_mode) ?
                           APP_RACE_ASSIST_SERVO_SPEED_DPS : APP_SERVO_MAX_SPEED_DPS);
        direct_bypass = (LEG_MODE_DIRECT_LEGACY_STANCE == control_leg_mode) ? APP_TRUE : APP_FALSE;
        actuator_servo_publish_cmd(&control_leg_servo_cmd,
                                   speed_limit_dps,
                                   direct_bypass);
    }
}

void control_leg_set_mode(leg_mode_enum mode)
{
    if(mode > LEG_MODE_RACE_ASSIST)
    {
        control_leg_mode = LEG_MODE_LOCK;
        return;
    }
    control_leg_mode = mode;
    if((LEG_MODE_LOCK == mode) && (LEG_MOTION_FAULT != control_leg_motion_state))
    {
        control_leg_motion_state = LEG_MOTION_LOCKED;
        control_leg_fault_reason = LEG_FAULT_NONE;
        control_leg_ik_previous_left.valid = APP_FALSE;
        control_leg_ik_previous_right.valid = APP_FALSE;
    }
}

void control_leg_set_manual_angle(uint8 leg_id, float angle_deg)
{
    const leg_servo_config_struct *servo_cfg;

    if(LEG_MOTION_FAULT == control_leg_motion_state)
    {
        return;
    }
    if(APP_SERVO_COUNT <= leg_id)
    {
        return;
    }
    servo_cfg = leg_config_get_servo(leg_id);
    if(NULL == servo_cfg)
    {
        return;
    }
    control_leg_manual_angle[leg_id] = control_leg_clamp(angle_deg, servo_cfg->min_deg, servo_cfg->max_deg);
}

void control_leg_set_body_cmd(float legacy_stance_cmd, float pitch_cmd, float roll_cmd)
{
    (void)pitch_cmd;
    (void)roll_cmd;

    control_leg_legacy_stance_cmd = legacy_stance_cmd;
    control_leg_pitch_cmd = 0.0f;
    control_leg_roll_cmd = 0.0f;

    if((0.0f != legacy_stance_cmd) && (APP_TRUE == control_leg_is_finite(legacy_stance_cmd)))
    {
        control_leg_set_legacy_stance(legacy_stance_cmd, 0U);
    }
    else
    {
        control_leg_mode = LEG_MODE_LOCK;
    }
}

const servo_cmd_struct *control_leg_get_servo_cmd(void)
{
    return &control_leg_servo_cmd;
}

uint8 control_leg_set_legacy_stance(float stance_units, uint32 now_ms)
{
    const leg_stance_profile_struct *profile;

    profile = leg_config_get_stance_profile();
    if((APP_FALSE == control_leg_is_finite(stance_units)) ||
       (profile->legacy_low_units > stance_units) ||
       (profile->legacy_high_units < stance_units))
    {
        return APP_FALSE;
    }

    if((LEG_MOTION_FAULT == control_leg_motion_state) ||
       (LEG_MODE_LEGACY_STANCE != control_leg_mode))
    {
        control_leg_legacy_stance_ref_units = profile->legacy_safe_support_units;
        control_leg_legacy_stance_rate_units_s = 0.0f;
        control_leg_legacy_stance_accel_units_s2 = 0.0f;
        control_leg_last_update_ms = now_ms;
        control_leg_fault_reason = LEG_FAULT_NONE;
        control_leg_motion_state = LEG_MOTION_TRANSITION;
        control_leg_settle_start_ms = now_ms;
    }
    control_leg_legacy_stance_target_units = stance_units;
    control_leg_mode = LEG_MODE_LEGACY_STANCE;
    control_leg_trajectory_mode = LEG_TRAJECTORY_NONE;
    control_leg_s7_progress = 0.0f;
    control_leg_s7_remaining_ms = 0U;
    return APP_TRUE;
}

uint8 control_leg_set_direct_legacy_stance(float stance_units, uint32 now_ms)
{
    const leg_stance_profile_struct *profile;

    profile = leg_config_get_stance_profile();
    if((APP_FALSE == control_leg_is_finite(stance_units)) ||
       (profile->legacy_low_units > stance_units) ||
       (profile->legacy_high_units < stance_units) ||
       (LEG_MOTION_FAULT == control_leg_motion_state))
    {
        return APP_FALSE;
    }

    control_leg_legacy_stance_target_units = stance_units;
    control_leg_legacy_stance_ref_units = stance_units;
    control_leg_legacy_stance_rate_units_s = 0.0f;
    control_leg_legacy_stance_accel_units_s2 = 0.0f;
    control_leg_settle_start_ms = now_ms;
    control_leg_motion_state = LEG_MOTION_TRANSITION;
    control_leg_fault_reason = LEG_FAULT_NONE;
    control_leg_mode = LEG_MODE_DIRECT_LEGACY_STANCE;
    return APP_TRUE;
}

uint8 control_leg_set_ik_reference(uint32 now_ms)
{
    const leg_kinematics_config_struct *kinematics;
    leg_ik_result_struct left_reference;
    leg_ik_result_struct right_reference;
    float servo_deg[LEG_SERVO_COUNT];

    (void)now_ms;
    if(LEG_MOTION_FAULT == control_leg_motion_state)
    {
        return APP_FALSE;
    }
    kinematics = leg_config_get_kinematics();
    if((NULL == kinematics) ||
       (APP_TRUE != leg_kinematics_solve(APP_FALSE,
                                          kinematics->physical_reference_x_mm,
                                          kinematics->physical_reference_y_mm,
                                          NULL,
                                          &left_reference)) ||
       (APP_TRUE != leg_kinematics_solve(APP_TRUE,
                                          kinematics->physical_reference_x_mm,
                                          kinematics->physical_reference_y_mm,
                                          NULL,
                                          &right_reference)) ||
       (APP_TRUE != leg_kinematics_map_reference_pose(&left_reference,
                                                       &right_reference,
                                                       servo_deg)))
    {
        return APP_FALSE;
    }

    control_leg_ik_reference_left = left_reference;
    control_leg_ik_reference_right = right_reference;
    control_leg_ik_previous_left = left_reference;
    control_leg_ik_previous_right = right_reference;
    control_leg_ik_reference_valid = APP_TRUE;
    control_leg_ik_target_x_mm = kinematics->physical_reference_x_mm;
    control_leg_ik_target_y_mm = kinematics->physical_reference_y_mm;
    control_leg_motion_state = LEG_MOTION_TRANSITION;
    control_leg_fault_reason = LEG_FAULT_NONE;
    control_leg_mode = LEG_MODE_IK_REFERENCE;
    return APP_TRUE;
}

uint8 control_leg_set_xy(float x_mm, float y_mm, uint32 now_ms)
{
    leg_ik_result_struct left_target;
    leg_ik_result_struct right_target;

    (void)now_ms;
    if((LEG_MOTION_FAULT == control_leg_motion_state) ||
       (APP_FALSE == control_leg_ik_reference_valid))
    {
        return APP_FALSE;
    }
    if((APP_FALSE == leg_kinematics_target_valid(x_mm, y_mm)) ||
       (APP_TRUE != leg_kinematics_solve(APP_FALSE,
                                          x_mm,
                                          y_mm,
                                          &control_leg_ik_previous_left,
                                          &left_target)) ||
       (APP_TRUE != leg_kinematics_solve(APP_TRUE,
                                          x_mm,
                                          y_mm,
                                          &control_leg_ik_previous_right,
                                          &right_target)))
    {
        return APP_FALSE;
    }

    control_leg_ik_target_x_mm = x_mm;
    control_leg_ik_target_y_mm = y_mm;
    control_leg_motion_state = LEG_MOTION_TRANSITION;
    control_leg_fault_reason = LEG_FAULT_NONE;
    control_leg_mode = LEG_MODE_IK_VALIDATE;
    return APP_TRUE;
}

uint8 control_leg_set_race_assist_request(float u_request,
                                          float dx_mm,
                                          float dy_mm,
                                          uint32 now_ms)
{
    float previous_dx_mm;
    float previous_dy_mm;
    float requested_before;

    if((APP_FALSE == control_leg_is_finite(u_request)) ||
       (APP_FALSE == control_leg_is_finite(dx_mm)) ||
       (APP_FALSE == control_leg_is_finite(dy_mm)) ||
       (1.0f < control_leg_absf(u_request)) ||
       (0.0f >= dx_mm) ||
       (0.0f >= dy_mm) ||
       (APP_FALSE == control_leg_ik_reference_valid) ||
       (LEG_MOTION_FAULT == control_leg_motion_state) ||
       (LEG_MOTION_RACE_FAULT_HOLD == control_leg_motion_state))
    {
        return APP_FALSE;
    }

    if((LEG_MODE_RACE_ASSIST != control_leg_mode) &&
       (APP_TRUE != control_leg_race_entry_ready()))
    {
        return APP_FALSE;
    }

    if((LEG_MODE_RACE_ASSIST == control_leg_mode) &&
       ((0.001f < control_leg_absf(dx_mm - control_leg_race_dx_mm)) ||
        (0.001f < control_leg_absf(dy_mm - control_leg_race_dy_mm))))
    {
        /* A running Cartesian segment never changes geometric scale mid-curve. */
        return APP_FALSE;
    }

    if(LEG_MODE_RACE_ASSIST != control_leg_mode)
    {
        control_leg_race_preflight_cache_invalidate();
    }

    previous_dx_mm = control_leg_race_dx_mm;
    previous_dy_mm = control_leg_race_dy_mm;
    control_leg_race_dx_mm = dx_mm;
    control_leg_race_dy_mm = dy_mm;
    if(APP_TRUE != control_leg_race_preflight(dx_mm, dy_mm))
    {
        control_leg_race_dx_mm = previous_dx_mm;
        control_leg_race_dy_mm = previous_dy_mm;
        return APP_FALSE;
    }

    if(LEG_MODE_RACE_ASSIST != control_leg_mode)
    {
        control_leg_race_u_start = 0.0f;
        control_leg_race_u_target = 0.0f;
        control_leg_race_u_pending = 0.0f;
        control_leg_race_u_actual = 0.0f;
        control_leg_race_start_ms = now_ms;
        control_leg_race_duration_ms = 0U;
        control_leg_race_pending_valid = APP_FALSE;
        control_leg_race_path_valid = APP_TRUE;
        if(APP_TRUE != control_leg_race_start_segment(u_request, now_ms))
        {
            control_leg_race_dx_mm = previous_dx_mm;
            control_leg_race_dy_mm = previous_dy_mm;
            control_leg_race_path_valid = APP_FALSE;
            return APP_FALSE;
        }
        control_leg_race_held_command_valid = control_leg_race_command_valid();
        control_leg_motion_state = LEG_MOTION_RACE_ASSIST;
        control_leg_fault_reason = LEG_FAULT_NONE;
        control_leg_mode = LEG_MODE_RACE_ASSIST;
        return APP_TRUE;
    }

    requested_before = (APP_TRUE == control_leg_race_pending_valid) ?
                       control_leg_race_u_pending : control_leg_race_u_target;
    if(APP_RACE_ASSIST_REQUEST_DEADBAND >
       control_leg_absf(u_request - requested_before))
    {
        return APP_TRUE;
    }

    control_leg_race_u_pending = u_request;
    control_leg_race_pending_valid = APP_TRUE;
    if((0U == control_leg_race_duration_ms) ||
       ((now_ms - control_leg_race_start_ms) >= control_leg_race_duration_ms))
    {
        if(APP_TRUE != control_leg_race_start_pending(now_ms))
        {
            control_leg_enter_race_fault_hold(LEG_FAULT_IK_INVALID);
            return APP_FALSE;
        }
    }
    return APP_TRUE;
}

void control_leg_disable_race_assist(uint32 now_ms)
{
    if((LEG_MODE_RACE_ASSIST != control_leg_mode) ||
       (LEG_MOTION_RACE_FAULT_HOLD == control_leg_motion_state))
    {
        return;
    }

    control_leg_race_u_pending = 0.0f;
    control_leg_race_pending_valid = APP_TRUE;
    if((0U == control_leg_race_duration_ms) ||
       ((now_ms - control_leg_race_start_ms) >= control_leg_race_duration_ms))
    {
        if(APP_TRUE != control_leg_race_start_pending(now_ms))
        {
            control_leg_enter_race_fault_hold(LEG_FAULT_IK_INVALID);
        }
    }
}

uint8 control_leg_set_fast_legacy_stance(float stance_units, uint32 now_ms)
{
    const leg_stance_profile_struct *profile;

    profile = leg_config_get_stance_profile();
    if((APP_FALSE == control_leg_is_finite(stance_units)) ||
       (profile->legacy_low_units > stance_units) ||
       (profile->legacy_high_units < stance_units) ||
       (LEG_MOTION_FAULT == control_leg_motion_state))
    {
        return APP_FALSE;
    }
    control_leg_fast_stance_start_units = control_leg_clamp(control_leg_legacy_stance_ref_units,
                                                            profile->legacy_low_units,
                                                            profile->legacy_high_units);
    control_leg_fast_stance_start_ms = now_ms;
    control_leg_legacy_stance_target_units = stance_units;
    control_leg_legacy_stance_rate_units_s = 0.0f;
    control_leg_legacy_stance_accel_units_s2 = 0.0f;
    control_leg_settle_start_ms = now_ms;
    control_leg_motion_state = LEG_MOTION_TRANSITION;
    control_leg_fault_reason = LEG_FAULT_NONE;
    control_leg_mode = LEG_MODE_FAST_LEGACY_STANCE;
    return APP_TRUE;
}

uint8 control_leg_set_calib_angles(float servo0_deg,
                                   float servo1_deg,
                                   float servo2_deg,
                                   float servo3_deg)
{
    if(LEG_MOTION_FAULT == control_leg_motion_state)
    {
        return APP_FALSE;
    }

    if((APP_FALSE == control_leg_servo_angle_valid(0U, servo0_deg)) ||
       (APP_FALSE == control_leg_servo_angle_valid(1U, servo1_deg)) ||
       (APP_FALSE == control_leg_servo_angle_valid(2U, servo2_deg)) ||
       (APP_FALSE == control_leg_servo_angle_valid(3U, servo3_deg)))
    {
        return APP_FALSE;
    }

    control_leg_set_manual_angle(0U, servo0_deg);
    control_leg_set_manual_angle(1U, servo1_deg);
    control_leg_set_manual_angle(2U, servo2_deg);
    control_leg_set_manual_angle(3U, servo3_deg);

    control_leg_mode = LEG_MODE_IK_CALIB;
    control_leg_motion_state = LEG_MOTION_TRANSITION;
    control_leg_fault_reason = LEG_FAULT_NONE;
    return APP_TRUE;
}

const leg_diag_struct *control_leg_get_diag(void)
{
    return &control_leg_diag;
}

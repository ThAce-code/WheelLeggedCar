/*********************************************************************************************************************
* File: control_leg.c
* Description: Leg controller: mix body attitude commands into servo angles.
********************************************************************************************************************/

#include "control_leg.h"
#include "app_config.h"
#include "app_safety.h"
#include "app_state.h"
#include "actuator_servo.h"

static servo_cmd_struct    control_leg_servo_cmd;
static leg_mode_enum       control_leg_mode;
static float               control_leg_manual_angle[APP_SERVO_COUNT];
static float               control_leg_height_cmd;
static float               control_leg_pitch_cmd;
static float               control_leg_roll_cmd;

#if (APP_LEG_VERIFY_ENABLE == 1U)
static uint32              control_leg_verify_start_ms = 0;
static uint8               control_leg_verify_active = APP_FALSE;
#endif

static leg_diag_struct     control_leg_diag;
static float               control_leg_target_height_mm;
static float               control_leg_height_ref_mm;
static float               control_leg_height_rate_mm_s;
static float               control_leg_height_accel_mm_s2;
static leg_motion_state_enum control_leg_motion_state;
static leg_fault_reason_enum  control_leg_fault_reason;
static uint32              control_leg_settle_start_ms;
static uint32              control_leg_last_update_ms;

#define CONTROL_LEG_EMPIRICAL_CENTER_HEIGHT_MM   (55.0f)
#define CONTROL_LEG_EMPIRICAL_CENTER_SERVO_DEG   (90.0f)
#define CONTROL_LEG_EMPIRICAL_MM_PER_DELTA_DEG   (0.595f)
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

static void control_leg_publish_diag(uint8 ik_valid, uint8 output_enable)
{
    uint8 i;
    const leg_height_profile_struct *profile;
    float configured_forward_limit_rpm;

    profile = leg_config_get_height_profile();
    control_leg_diag.target_height_mm = control_leg_target_height_mm;
    /* This is an open-loop command estimate; this PWM-only platform has no height feedback. */
    control_leg_diag.actual_height_mm = control_leg_height_ref_mm;
    control_leg_diag.height_ref_mm = control_leg_height_ref_mm;
    control_leg_diag.height_rate_mm_s = control_leg_height_rate_mm_s;
    if(profile->high_height_mm > profile->low_height_mm)
    {
        control_leg_diag.height_norm =
            (control_leg_height_ref_mm - profile->low_height_mm) /
            (profile->high_height_mm - profile->low_height_mm);
    }
    else
    {
        control_leg_diag.height_norm = 0.0f;
    }
    control_leg_diag.height_norm = control_leg_clamp(control_leg_diag.height_norm, 0.0f, 1.0f);
    control_leg_diag.mode = (uint8)control_leg_mode;
    control_leg_diag.ik_valid = ik_valid;
    control_leg_diag.output_enable = output_enable;
    control_leg_diag.motion_state = control_leg_motion_state;
    control_leg_diag.fault_reason = control_leg_fault_reason;
    configured_forward_limit_rpm =
        profile->chassis_forward_limit_low_rpm +
        ((profile->chassis_forward_limit_high_rpm - profile->chassis_forward_limit_low_rpm) *
         control_leg_diag.height_norm);
    if((LEG_MOTION_FAULT == control_leg_motion_state) ||
       (LEG_MODE_DIRECT_STEP == control_leg_mode))
    {
        control_leg_diag.drive_allowed = APP_FALSE;
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
        else
        {
            control_leg_diag.drive_forward_limit_rpm = configured_forward_limit_rpm;
        }
    }
    for(i = 0; i < APP_SERVO_COUNT; i++)
    {
        control_leg_diag.servo_target_deg[i] = control_leg_servo_cmd.angle_deg[i];
        control_leg_diag.servo_actual_deg[i] = actuator_servo_get_current_angle(i);
    }
}

static uint8 control_leg_apply_empirical_height(float height_mm,
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
       (APP_FALSE == control_leg_is_finite(height_mm)) ||
       (0.0f == CONTROL_LEG_EMPIRICAL_MM_PER_DELTA_DEG))
    {
        return APP_FALSE;
    }

    /*
     * Phase 1 uses the measured differential height map instead of the
     * uncalibrated five-bar leg_kinematics_solve model:
     *   Y_real ~= 55mm + 0.595mm/deg * d
     *   d = a0 - a1 = a3 - a2
     */
    delta_deg = (height_mm - CONTROL_LEG_EMPIRICAL_CENTER_HEIGHT_MM) / CONTROL_LEG_EMPIRICAL_MM_PER_DELTA_DEG;
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
    control_leg_height_rate_mm_s = 0.0f;
    control_leg_height_accel_mm_s2 = 0.0f;
    control_leg_settle_start_ms = 0U;
    control_leg_diag.ik_error_count++;
    /* Keep PWM enabled when the application is otherwise runnable; actuator_servo limits this move. */
    control_leg_write_safe_angles(config);
}

void control_leg_init(void)
{
    uint8 i;
    const leg_config_struct *config;

    config = leg_config_get();
    control_leg_mode = LEG_MODE_LOCK;
    control_leg_height_cmd = 0.0f;
    control_leg_pitch_cmd = 0.0f;
    control_leg_roll_cmd = 0.0f;

    for(i = 0; i < APP_SERVO_COUNT; i++)
    {
        control_leg_manual_angle[i] = config->servo[i].safe_deg;
        control_leg_servo_cmd.angle_deg[i] = config->servo[i].safe_deg;
        control_leg_servo_cmd.enable[i] = APP_FALSE;
    }

    {
        const leg_height_profile_struct *profile;
        profile = leg_config_get_height_profile();
        control_leg_target_height_mm = profile->default_height_mm;
        control_leg_height_ref_mm = profile->default_height_mm;
        control_leg_height_rate_mm_s = 0.0f;
        control_leg_height_accel_mm_s2 = 0.0f;
        control_leg_motion_state = LEG_MOTION_LOCKED;
        control_leg_fault_reason = LEG_FAULT_NONE;
        control_leg_settle_start_ms = 0U;
        control_leg_last_update_ms = 0U;
        control_leg_diag.ik_error_count = 0U;
        control_leg_diag.left_x_mm = 0.0f;
        control_leg_diag.right_x_mm = 0.0f;
        control_leg_diag.left_y_mm = control_leg_height_ref_mm;
        control_leg_diag.right_y_mm = control_leg_height_ref_mm;
    }

    control_leg_publish_diag(APP_FALSE, APP_FALSE);
}

void control_leg_update(uint32 now_ms)
{
    uint8 i;
    const leg_config_struct *config;
    const leg_servo_config_struct *servo_cfg;

    config = leg_config_get();

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
        control_leg_mode = LEG_MODE_HEIGHT;
        control_leg_height_cmd = APP_LEG_VERIFY_HEIGHT_CMD;
        control_leg_pitch_cmd = APP_LEG_VERIFY_PITCH_CMD;
        control_leg_roll_cmd = APP_LEG_VERIFY_ROLL_CMD;
    }
#endif

    if(LEG_MOTION_FAULT == control_leg_motion_state)
    {
        control_leg_write_safe_angles(config);
        control_leg_publish_diag(APP_FALSE, control_leg_run_enabled());
    }
    else
    {
        switch(control_leg_mode)
        {
            case LEG_MODE_MANUAL:
#if (APP_LEG_CALIB_ENABLE == 1U)
                servo_cfg = &config->servo[APP_LEG_CALIB_SERVO_ID];
                control_leg_manual_angle[APP_LEG_CALIB_SERVO_ID] = control_leg_clamp(servo_cfg->safe_deg + APP_LEG_CALIB_OFFSET_DEG,
                                                                                      servo_cfg->min_deg,
                                                                                      servo_cfg->max_deg);
#endif
                for(i = 0; i < APP_SERVO_COUNT; i++)
                {
                    servo_cfg = &config->servo[i];
                    control_leg_servo_cmd.angle_deg[i] = control_leg_clamp(control_leg_manual_angle[i],
                                                                           servo_cfg->min_deg,
                                                                           servo_cfg->max_deg);
                }
                break;

            case LEG_MODE_HEIGHT:
            {
                const leg_height_profile_struct *profile;
                float servo_fl_deg;
                float servo_fr_deg;
                float servo_rl_deg;
                float servo_rr_deg;
                float error_mm;
                float desired_rate_mm_s;
                float desired_accel_mm_s2;
                float accel_delta_mm_s2;
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

                profile = leg_config_get_height_profile();
                if((APP_FALSE == control_leg_is_finite(control_leg_height_ref_mm)) ||
                   (APP_FALSE == control_leg_is_finite(control_leg_height_rate_mm_s)) ||
                   (APP_FALSE == control_leg_is_finite(control_leg_height_accel_mm_s2)) ||
                   (0.0f >= profile->max_height_speed_mm_s) ||
                   (0.0f >= profile->max_height_accel_mm_s2) ||
                   (0.0f >= profile->max_height_jerk_mm_s3) ||
                   (0.0f >= profile->height_position_kp_s) ||
                   (0.0f >= profile->height_rate_kp_s))
                {
                    control_leg_enter_fault(LEG_FAULT_SERVO_LIMIT);
                    control_leg_publish_diag(APP_FALSE, control_leg_run_enabled());
                    break;
                }
                error_mm = control_leg_target_height_mm - control_leg_height_ref_mm;
                desired_rate_mm_s = control_leg_clamp(error_mm * profile->height_position_kp_s,
                                                       -profile->max_height_speed_mm_s,
                                                       profile->max_height_speed_mm_s);
                desired_accel_mm_s2 = control_leg_clamp(
                    (desired_rate_mm_s - control_leg_height_rate_mm_s) * profile->height_rate_kp_s,
                    -profile->max_height_accel_mm_s2,
                    profile->max_height_accel_mm_s2);
                accel_delta_mm_s2 = control_leg_clamp(desired_accel_mm_s2 - control_leg_height_accel_mm_s2,
                                                       -profile->max_height_jerk_mm_s3 * dt_s,
                                                       profile->max_height_jerk_mm_s3 * dt_s);
                control_leg_height_accel_mm_s2 = control_leg_clamp(control_leg_height_accel_mm_s2 + accel_delta_mm_s2,
                                                                    -profile->max_height_accel_mm_s2,
                                                                    profile->max_height_accel_mm_s2);
                control_leg_height_rate_mm_s = control_leg_clamp(control_leg_height_rate_mm_s +
                                                                  (control_leg_height_accel_mm_s2 * dt_s),
                                                                  -profile->max_height_speed_mm_s,
                                                                  profile->max_height_speed_mm_s);
                control_leg_height_ref_mm += control_leg_height_rate_mm_s * dt_s;
                if((0.01f >= control_leg_absf(control_leg_target_height_mm - control_leg_height_ref_mm)) &&
                   (0.05f >= control_leg_absf(control_leg_height_rate_mm_s)) &&
                   ((profile->max_height_jerk_mm_s3 * dt_s) >=
                    control_leg_absf(control_leg_height_accel_mm_s2)))
                {
                    control_leg_height_ref_mm = control_leg_target_height_mm;
                    control_leg_height_rate_mm_s = 0.0f;
                    control_leg_height_accel_mm_s2 = 0.0f;
                }

                control_leg_diag.left_x_mm = 0.0f;
                control_leg_diag.right_x_mm = 0.0f;
                control_leg_diag.left_y_mm = control_leg_height_ref_mm;
                control_leg_diag.right_y_mm = control_leg_height_ref_mm;

                if(APP_TRUE == control_leg_apply_empirical_height(control_leg_height_ref_mm,
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
                    if((profile->height_settle_error_mm >=
                        control_leg_absf(control_leg_target_height_mm - control_leg_height_ref_mm)) &&
                       (0.0f == control_leg_height_rate_mm_s))
                    {
                        if(LEG_MOTION_TRANSITION != control_leg_motion_state)
                        {
                            control_leg_settle_start_ms = now_ms;
                            control_leg_motion_state = LEG_MOTION_TRANSITION;
                        }
                        if((now_ms - control_leg_settle_start_ms) >= profile->height_settle_ms)
                        {
                            control_leg_motion_state = LEG_MOTION_STABLE;
                        }
                    }
                    else
                    {
                        control_leg_settle_start_ms = now_ms;
                        control_leg_motion_state = LEG_MOTION_TRANSITION;
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
                break;

            case LEG_MODE_DIRECT_STEP:
            {
                float servo_fl_deg;
                float servo_fr_deg;
                float servo_rl_deg;
                float servo_rr_deg;
                uint8 output_enable;

                control_leg_height_rate_mm_s = 0.0f;
                control_leg_height_accel_mm_s2 = 0.0f;
                control_leg_diag.left_x_mm = 0.0f;
                control_leg_diag.right_x_mm = 0.0f;
                control_leg_diag.left_y_mm = control_leg_height_ref_mm;
                control_leg_diag.right_y_mm = control_leg_height_ref_mm;
                if(APP_TRUE == control_leg_apply_empirical_height(control_leg_height_ref_mm,
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

            case LEG_MODE_LOCK:
            default:
            {
                const leg_height_profile_struct *profile;

                profile = leg_config_get_height_profile();
                control_leg_target_height_mm = profile->safe_support_height_mm;
                control_leg_height_ref_mm = profile->safe_support_height_mm;
                control_leg_height_rate_mm_s = 0.0f;
                control_leg_height_accel_mm_s2 = 0.0f;
                control_leg_write_safe_angles(config);
                control_leg_motion_state = LEG_MOTION_LOCKED;
                control_leg_fault_reason = LEG_FAULT_NONE;
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

    actuator_servo_set_cmd(&control_leg_servo_cmd);
    if(LEG_MODE_DIRECT_STEP == control_leg_mode)
    {
        actuator_servo_apply_immediate();
    }
}

void control_leg_set_mode(leg_mode_enum mode)
{
    if(mode > LEG_MODE_DIRECT_STEP)
    {
        control_leg_mode = LEG_MODE_LOCK;
        return;
    }
    control_leg_mode = mode;
    if((LEG_MODE_LOCK == mode) && (LEG_MOTION_FAULT != control_leg_motion_state))
    {
        control_leg_motion_state = LEG_MOTION_LOCKED;
        control_leg_fault_reason = LEG_FAULT_NONE;
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

void control_leg_set_body_cmd(float height_cmd, float pitch_cmd, float roll_cmd)
{
    (void)pitch_cmd;
    (void)roll_cmd;

    control_leg_height_cmd = height_cmd;
    control_leg_pitch_cmd = 0.0f;
    control_leg_roll_cmd = 0.0f;

    if((0.0f != height_cmd) && (APP_TRUE == control_leg_is_finite(height_cmd)))
    {
        control_leg_set_height(height_cmd, 0U);
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

uint8 control_leg_set_height(float height_mm, uint32 now_ms)
{
    const leg_height_profile_struct *profile;

    profile = leg_config_get_height_profile();
    if((APP_FALSE == control_leg_is_finite(height_mm)) ||
       (profile->low_height_mm > height_mm) ||
       (profile->high_height_mm < height_mm))
    {
        return APP_FALSE;
    }

    if((LEG_MOTION_FAULT == control_leg_motion_state) ||
       (LEG_MODE_HEIGHT != control_leg_mode))
    {
        control_leg_height_ref_mm = profile->safe_support_height_mm;
        control_leg_height_rate_mm_s = 0.0f;
        control_leg_height_accel_mm_s2 = 0.0f;
        control_leg_last_update_ms = now_ms;
        control_leg_fault_reason = LEG_FAULT_NONE;
        control_leg_motion_state = LEG_MOTION_TRANSITION;
        control_leg_settle_start_ms = now_ms;
    }
    control_leg_target_height_mm = height_mm;
    control_leg_mode = LEG_MODE_HEIGHT;
    return APP_TRUE;
}

uint8 control_leg_set_direct_step_height(float height_mm, uint32 now_ms)
{
    const leg_height_profile_struct *profile;

    profile = leg_config_get_height_profile();
    if((APP_FALSE == control_leg_is_finite(height_mm)) ||
       (profile->low_height_mm > height_mm) ||
       (profile->high_height_mm < height_mm) ||
       (LEG_MOTION_FAULT == control_leg_motion_state))
    {
        return APP_FALSE;
    }

    control_leg_target_height_mm = height_mm;
    control_leg_height_ref_mm = height_mm;
    control_leg_height_rate_mm_s = 0.0f;
    control_leg_height_accel_mm_s2 = 0.0f;
    control_leg_settle_start_ms = now_ms;
    control_leg_motion_state = LEG_MOTION_TRANSITION;
    control_leg_fault_reason = LEG_FAULT_NONE;
    control_leg_mode = LEG_MODE_DIRECT_STEP;
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

    control_leg_servo_cmd.angle_deg[0] = servo0_deg;
    control_leg_servo_cmd.angle_deg[1] = servo1_deg;
    control_leg_servo_cmd.angle_deg[2] = servo2_deg;
    control_leg_servo_cmd.angle_deg[3] = servo3_deg;

    control_leg_mode = LEG_MODE_IK_CALIB;
    control_leg_publish_diag(APP_FALSE, control_leg_run_enabled());
    return APP_TRUE;
}

const leg_diag_struct *control_leg_get_diag(void)
{
    return &control_leg_diag;
}

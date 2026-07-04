/*********************************************************************************************************************
* File: control_leg.c
* Description: Leg controller: mix body attitude commands into servo angles.
********************************************************************************************************************/

#include "control_leg.h"
#include "app_config.h"
#include "app_state.h"
#include "actuator_servo.h"
#include "leg_kinematics.h"

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
static float               control_leg_actual_height_mm;
static uint32              control_leg_last_update_ms;

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

static float control_leg_absf(float value)
{
    return (0.0f > value) ? -value : value;
}

static float control_leg_ramp_toward(float current, float target, float max_delta)
{
    float delta;

    delta = target - current;
    if(max_delta >= control_leg_absf(delta))
    {
        return target;
    }
    return current + ((0.0f < delta) ? max_delta : -max_delta);
}

static void control_leg_publish_diag(uint8 ik_valid, uint8 output_enable)
{
    uint8 i;
    const leg_height_profile_struct *profile;

    profile = leg_config_get_height_profile();
    control_leg_diag.target_height_mm = control_leg_target_height_mm;
    control_leg_diag.actual_height_mm = control_leg_actual_height_mm;
    if(profile->high_height_mm > profile->low_height_mm)
    {
        control_leg_diag.height_norm =
            (control_leg_actual_height_mm - profile->low_height_mm) /
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
    for(i = 0; i < APP_SERVO_COUNT; i++)
    {
        control_leg_diag.servo_target_deg[i] = control_leg_servo_cmd.angle_deg[i];
        control_leg_diag.servo_actual_deg[i] = control_leg_servo_cmd.angle_deg[i];
    }
}

static float control_leg_apply_calib(uint8 servo_index, float ik_angle_deg)
{
    const leg_servo_config_struct *servo_cfg;
    float calibrated;

    servo_cfg = leg_config_get_servo(servo_index);
    if(NULL == servo_cfg)
    {
        return ik_angle_deg;
    }
    calibrated = servo_cfg->neutral_deg + (servo_cfg->direction * (ik_angle_deg - servo_cfg->neutral_deg));
    return control_leg_clamp(calibrated, servo_cfg->min_deg, servo_cfg->max_deg);
}

static uint8 control_leg_run_enabled(void)
{
    app_run_state_enum state;

    state = app_state_get();
    return ((APP_STATE_STANDBY == state) || (APP_STATE_RUN == state)) ? APP_TRUE : APP_FALSE;
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
        control_leg_actual_height_mm = profile->default_height_mm;
        control_leg_last_update_ms = 0U;
        control_leg_diag.ik_error_count = 0U;
        control_leg_diag.left_x_mm = 0.0f;
        control_leg_diag.right_x_mm = 0.0f;
        control_leg_diag.left_y_mm = control_leg_actual_height_mm;
        control_leg_diag.right_y_mm = control_leg_actual_height_mm;
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
            leg_ik_result_struct left_ik;
            leg_ik_result_struct right_ik;
            const leg_height_profile_struct *profile;
            float max_delta;
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
            max_delta = profile->max_height_speed_mm_s * dt_s;
            control_leg_actual_height_mm =
                control_leg_ramp_toward(control_leg_actual_height_mm,
                                        control_leg_target_height_mm,
                                        max_delta);

            control_leg_diag.left_x_mm = 0.0f;
            control_leg_diag.right_x_mm = 0.0f;
            control_leg_diag.left_y_mm = control_leg_actual_height_mm;
            control_leg_diag.right_y_mm = control_leg_actual_height_mm;

            if((APP_TRUE == leg_kinematics_solve(APP_FALSE, 0.0f, control_leg_actual_height_mm, &left_ik)) &&
               (APP_TRUE == leg_kinematics_solve(APP_TRUE, 0.0f, control_leg_actual_height_mm, &right_ik)))
            {
                control_leg_servo_cmd.angle_deg[LEG_SERVO_FL] =
                    control_leg_apply_calib(LEG_SERVO_FL, left_ik.servo_deg[0]);
                control_leg_servo_cmd.angle_deg[LEG_SERVO_RL] =
                    control_leg_apply_calib(LEG_SERVO_RL, left_ik.servo_deg[1]);
                control_leg_servo_cmd.angle_deg[LEG_SERVO_FR] =
                    control_leg_apply_calib(LEG_SERVO_FR, right_ik.servo_deg[0]);
                control_leg_servo_cmd.angle_deg[LEG_SERVO_RR] =
                    control_leg_apply_calib(LEG_SERVO_RR, right_ik.servo_deg[1]);
                output_enable = control_leg_run_enabled();
                control_leg_publish_diag(APP_TRUE, output_enable);
            }
            else
            {
                control_leg_diag.ik_error_count++;
                control_leg_mode = LEG_MODE_LOCK;
                for(i = 0; i < APP_SERVO_COUNT; i++)
                {
                    servo_cfg = &config->servo[i];
                    control_leg_servo_cmd.angle_deg[i] = control_leg_clamp(servo_cfg->safe_deg,
                                                                           servo_cfg->min_deg,
                                                                           servo_cfg->max_deg);
                }
                output_enable = APP_FALSE;
                control_leg_publish_diag(APP_FALSE, output_enable);
            }
            break;
        }

        case LEG_MODE_IK_CALIB:
            break;

        case LEG_MODE_LOCK:
        default:
            for(i = 0; i < APP_SERVO_COUNT; i++)
            {
                servo_cfg = &config->servo[i];
                control_leg_servo_cmd.angle_deg[i] = control_leg_clamp(servo_cfg->safe_deg,
                                                                       servo_cfg->min_deg,
                                                                       servo_cfg->max_deg);
            }
            break;
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
}

void control_leg_set_mode(leg_mode_enum mode)
{
    if(mode > LEG_MODE_HEIGHT)
    {
        control_leg_mode = LEG_MODE_LOCK;
        return;
    }
    control_leg_mode = mode;
}

void control_leg_set_manual_angle(uint8 leg_id, float angle_deg)
{
    const leg_servo_config_struct *servo_cfg;

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

    (void)now_ms;
    profile = leg_config_get_height_profile();
    if((APP_FALSE == control_leg_is_finite(height_mm)) ||
       (profile->low_height_mm > height_mm) ||
       (profile->high_height_mm < height_mm))
    {
        return APP_FALSE;
    }

    control_leg_target_height_mm = height_mm;
    control_leg_mode = LEG_MODE_HEIGHT;
    return APP_TRUE;
}

uint8 control_leg_set_calib_angles(float servo0_deg,
                                   float servo1_deg,
                                   float servo2_deg,
                                   float servo3_deg)
{
    const leg_servo_config_struct *servo_cfg;

    control_leg_set_manual_angle(0U, servo0_deg);
    control_leg_set_manual_angle(1U, servo1_deg);
    control_leg_set_manual_angle(2U, servo2_deg);
    control_leg_set_manual_angle(3U, servo3_deg);

    servo_cfg = leg_config_get_servo(0U);
    control_leg_servo_cmd.angle_deg[0] = (NULL != servo_cfg)
        ? control_leg_clamp(servo0_deg, servo_cfg->min_deg, servo_cfg->max_deg) : servo0_deg;
    servo_cfg = leg_config_get_servo(1U);
    control_leg_servo_cmd.angle_deg[1] = (NULL != servo_cfg)
        ? control_leg_clamp(servo1_deg, servo_cfg->min_deg, servo_cfg->max_deg) : servo1_deg;
    servo_cfg = leg_config_get_servo(2U);
    control_leg_servo_cmd.angle_deg[2] = (NULL != servo_cfg)
        ? control_leg_clamp(servo2_deg, servo_cfg->min_deg, servo_cfg->max_deg) : servo2_deg;
    servo_cfg = leg_config_get_servo(3U);
    control_leg_servo_cmd.angle_deg[3] = (NULL != servo_cfg)
        ? control_leg_clamp(servo3_deg, servo_cfg->min_deg, servo_cfg->max_deg) : servo3_deg;

    control_leg_mode = LEG_MODE_IK_CALIB;
    control_leg_publish_diag(APP_FALSE, control_leg_run_enabled());
    return APP_TRUE;
}

const leg_diag_struct *control_leg_get_diag(void)
{
    return &control_leg_diag;
}

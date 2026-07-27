/*********************************************************************************************************************
* File: control_leg.c
* Description: Leg controller: mix body attitude commands into servo angles.
********************************************************************************************************************/

#include "control_leg.h"
#include "app_config.h"
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

static float control_leg_compute_target(const leg_servo_config_struct *servo_cfg,
                                        float height,
                                        float pitch,
                                        float roll)
{
    float target;
    float mix;

    mix = height + (pitch * servo_cfg->mount_x) + (roll * servo_cfg->mount_y);
    target = servo_cfg->neutral_deg + (servo_cfg->direction * mix);
    return control_leg_clamp(target, servo_cfg->min_deg, servo_cfg->max_deg);
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
}

void control_leg_update(uint32 now_ms)
{
    uint8 i;
    const leg_config_struct *config;
    const leg_servo_config_struct *servo_cfg;
    float height;
    float pitch;
    float roll;
    float target;

    (void)now_ms;
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
        control_leg_mode = LEG_MODE_ATTITUDE;
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

        case LEG_MODE_ATTITUDE:
            height = control_leg_clamp(control_leg_height_cmd, config->height_min, config->height_max);
            pitch  = control_leg_clamp(control_leg_pitch_cmd, -config->pitch_limit, config->pitch_limit);
            roll   = control_leg_clamp(control_leg_roll_cmd, -config->roll_limit, config->roll_limit);
            for(i = 0; i < APP_SERVO_COUNT; i++)
            {
                servo_cfg = &config->servo[i];
                target = control_leg_compute_target(servo_cfg, height, pitch, roll);
                control_leg_servo_cmd.angle_deg[i] = control_leg_clamp(target,
                                                                       servo_cfg->min_deg,
                                                                       servo_cfg->max_deg);
            }
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
    if(mode > LEG_MODE_ATTITUDE)
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
    control_leg_height_cmd = height_cmd;
    control_leg_pitch_cmd = pitch_cmd;
    control_leg_roll_cmd = roll_cmd;
}

const servo_cmd_struct *control_leg_get_servo_cmd(void)
{
    return &control_leg_servo_cmd;
}

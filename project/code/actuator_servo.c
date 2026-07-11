/*********************************************************************************************************************
* File: actuator_servo.c
* Description: PWM servo actuator with 300 Hz first-order low-pass execution.
*********************************************************************************************************************/

#include "actuator_servo.h"
#include "app_config.h"
#include "servo_motion.h"

typedef struct
{
    servo_cmd_struct cmd;
    float speed_limit_dps;
    uint8 direct_bypass;
}actuator_servo_frame_struct;

static actuator_servo_frame_struct actuator_servo_frame[2];
static volatile uint8 actuator_servo_active_frame = 0U;
static servo_motion_state_struct actuator_servo_motion[APP_SERVO_COUNT];
static actuator_servo_diag_struct actuator_servo_diag;

static const pwm_channel_enum actuator_servo_pwm_ch[APP_SERVO_COUNT] =
{
    APP_SERVO0_PWM_CH,
    APP_SERVO1_PWM_CH,
    APP_SERVO2_PWM_CH,
    APP_SERVO3_PWM_CH
};

static float actuator_servo_limit(float value)
{
    if(APP_SERVO_MIN_DEG > value)
    {
        return APP_SERVO_MIN_DEG;
    }
    if(APP_SERVO_MAX_DEG < value)
    {
        return APP_SERVO_MAX_DEG;
    }
    return value;
}

static float actuator_servo_absf(float value)
{
    return (0.0f > value) ? -value : value;
}

static uint8 actuator_servo_is_active(uint8 index)
{
    return (0U != (APP_SERVO_ACTIVE_MASK & (1U << index))) ? APP_TRUE : APP_FALSE;
}

static void actuator_servo_write(uint8 index, float angle_deg)
{
    if(APP_FALSE == actuator_servo_is_active(index))
    {
        return;
    }
    pwm_set_duty(actuator_servo_pwm_ch[index], actuator_servo_angle_to_duty(angle_deg));
}

void actuator_servo_init(void)
{
    uint8 i;

    for(i = 0; i < APP_SERVO_COUNT; i++)
    {
        servo_motion_init(&actuator_servo_motion[i], APP_SERVO_MID_DEG);
        actuator_servo_frame[0].cmd.angle_deg[i] = APP_SERVO_MID_DEG;
        actuator_servo_frame[0].cmd.enable[i] = APP_FALSE;
        actuator_servo_frame[1].cmd.angle_deg[i] = APP_SERVO_MID_DEG;
        actuator_servo_frame[1].cmd.enable[i] = APP_FALSE;
        actuator_servo_diag.target_deg[i] = APP_SERVO_MID_DEG;
        actuator_servo_diag.filtered_deg[i] = APP_SERVO_MID_DEG;
        actuator_servo_diag.output_deg[i] = APP_SERVO_MID_DEG;
        if(APP_TRUE == actuator_servo_is_active(i))
        {
            pwm_init(actuator_servo_pwm_ch[i], APP_SERVO_PWM_FREQ_HZ, 0);
            pwm_set_duty(actuator_servo_pwm_ch[i], 0);
        }
    }
    actuator_servo_frame[0].speed_limit_dps = APP_SERVO_MAX_SPEED_DPS;
    actuator_servo_frame[0].direct_bypass = APP_FALSE;
    actuator_servo_frame[1].speed_limit_dps = APP_SERVO_MAX_SPEED_DPS;
    actuator_servo_frame[1].direct_bypass = APP_FALSE;
    actuator_servo_active_frame = 0U;
    actuator_servo_diag.max_error_deg = 0.0f;
    actuator_servo_diag.settled = APP_FALSE;
    actuator_servo_diag.fast_mode = APP_FALSE;
    actuator_servo_diag.direct_bypass = APP_FALSE;
}

void actuator_servo_publish_cmd(const servo_cmd_struct *cmd,
                                float speed_limit_dps,
                                uint8 direct_bypass)
{
    uint8 i;
    uint8 inactive;

    if(NULL == cmd)
    {
        return;
    }

    inactive = (0U == actuator_servo_active_frame) ? 1U : 0U;

    for(i = 0; i < APP_SERVO_COUNT; i++)
    {
        actuator_servo_frame[inactive].cmd.angle_deg[i] = actuator_servo_limit(cmd->angle_deg[i]);
        actuator_servo_frame[inactive].cmd.enable[i] = cmd->enable[i];
    }

    if((speed_limit_dps != speed_limit_dps) || (0.0f >= speed_limit_dps))
    {
        actuator_servo_frame[inactive].speed_limit_dps = APP_SERVO_MAX_SPEED_DPS;
    }
    else if(APP_LEG_FAST_SERVO_MAX_SPEED_DPS < speed_limit_dps)
    {
        actuator_servo_frame[inactive].speed_limit_dps = APP_LEG_FAST_SERVO_MAX_SPEED_DPS;
    }
    else
    {
        actuator_servo_frame[inactive].speed_limit_dps = speed_limit_dps;
    }

    actuator_servo_frame[inactive].direct_bypass = direct_bypass;

    if(APP_TRUE == direct_bypass)
    {
        interrupt_global_disable();
        actuator_servo_active_frame = inactive;
        for(i = 0; i < APP_SERVO_COUNT; i++)
        {
            servo_motion_apply_immediate(&actuator_servo_motion[i],
                                         actuator_servo_frame[inactive].cmd.angle_deg[i]);
            if((APP_TRUE == actuator_servo_frame[inactive].cmd.enable[i]) &&
               (APP_TRUE == actuator_servo_is_active(i)))
            {
                actuator_servo_write(i, actuator_servo_motion[i].output_deg);
            }
        }
        interrupt_global_enable();
    }
    else
    {
        interrupt_global_disable();
        actuator_servo_active_frame = inactive;
        interrupt_global_enable();
    }
}

void actuator_servo_tick_300hz(void)
{
    uint8 i;
    uint8 active;
    const actuator_servo_frame_struct *frame;
    float max_step_deg;
    float max_error_deg;
    uint8 all_settled;

    active = actuator_servo_active_frame;
    frame = &actuator_servo_frame[active];

    max_step_deg = frame->speed_limit_dps / (float)APP_SERVO_PWM_FREQ_HZ;

    for(i = 0; i < APP_SERVO_COUNT; i++)
    {
        if(APP_FALSE == frame->cmd.enable[i])
        {
            if(APP_TRUE == actuator_servo_is_active(i))
            {
                pwm_set_duty(actuator_servo_pwm_ch[i], 0);
            }
            /* Align filtered state to saved output so a later enable
               resumes from the last held position without a stale jump. */
            actuator_servo_motion[i].filtered_deg = actuator_servo_motion[i].output_deg;
            actuator_servo_motion[i].target_deg = actuator_servo_motion[i].output_deg;
            actuator_servo_motion[i].settled_ticks = 0U;
            actuator_servo_motion[i].settled = APP_FALSE;
            continue;
        }

        servo_motion_step(&actuator_servo_motion[i],
                          frame->cmd.angle_deg[i],
                          APP_SERVO_LPF_ALPHA,
                          max_step_deg,
                          APP_SERVO_SETTLE_ERROR_DEG,
                          (uint16)APP_SERVO_SETTLE_TICKS);

        if(APP_TRUE == actuator_servo_is_active(i))
        {
            actuator_servo_write(i, actuator_servo_motion[i].output_deg);
        }
    }

    /* Build diagnostic snapshot */
    max_error_deg = 0.0f;
    all_settled = APP_TRUE;
    for(i = 0; i < APP_SERVO_COUNT; i++)
    {
        float error_deg;

        actuator_servo_diag.target_deg[i] = actuator_servo_motion[i].target_deg;
        actuator_servo_diag.filtered_deg[i] = actuator_servo_motion[i].filtered_deg;
        actuator_servo_diag.output_deg[i] = actuator_servo_motion[i].output_deg;
        error_deg = actuator_servo_absf(actuator_servo_motion[i].target_deg -
                                        actuator_servo_motion[i].output_deg);
        if(error_deg > max_error_deg)
        {
            max_error_deg = error_deg;
        }
        if((APP_TRUE == frame->cmd.enable[i]) &&
           (APP_TRUE != actuator_servo_motion[i].settled))
        {
            all_settled = APP_FALSE;
        }
    }
    actuator_servo_diag.max_error_deg = max_error_deg;
    actuator_servo_diag.settled = all_settled;
    actuator_servo_diag.fast_mode =
        (frame->speed_limit_dps >= (APP_LEG_FAST_SERVO_MAX_SPEED_DPS - 0.1f)) ? APP_TRUE : APP_FALSE;
    actuator_servo_diag.direct_bypass = frame->direct_bypass;
}

void actuator_servo_get_diag(actuator_servo_diag_struct *diag)
{
    if(NULL == diag)
    {
        return;
    }
    interrupt_global_disable();
    *diag = actuator_servo_diag;
    interrupt_global_enable();
}

uint8 actuator_servo_is_settled(void)
{
    uint8 settled;

    interrupt_global_disable();
    settled = actuator_servo_diag.settled;
    interrupt_global_enable();
    return settled;
}

void actuator_servo_enable(void)
{
    uint8 i;
    uint8 active;

    active = actuator_servo_active_frame;
    for(i = 0; i < APP_SERVO_COUNT; i++)
    {
        if(APP_TRUE == actuator_servo_frame[active].cmd.enable[i])
        {
            actuator_servo_write(i, actuator_servo_motion[i].output_deg);
        }
    }
}

void actuator_servo_disable(void)
{
    uint8 i;
    uint8 inactive;

    inactive = (0U == actuator_servo_active_frame) ? 1U : 0U;
    for(i = 0; i < APP_SERVO_COUNT; i++)
    {
        actuator_servo_frame[inactive].cmd.enable[i] = APP_FALSE;
        if(APP_TRUE == actuator_servo_is_active(i))
        {
            pwm_set_duty(actuator_servo_pwm_ch[i], 0);
        }
    }
    interrupt_global_disable();
    actuator_servo_active_frame = inactive;
    interrupt_global_enable();
}

uint32 actuator_servo_angle_to_duty(float angle_deg)
{
    float limited_angle;
    float pulse_us;

    limited_angle = actuator_servo_limit(angle_deg);
    pulse_us = (float)APP_SERVO_MIN_PULSE_US +
               (limited_angle - APP_SERVO_MIN_DEG) *
               (float)(APP_SERVO_MAX_PULSE_US - APP_SERVO_MIN_PULSE_US) /
               (APP_SERVO_MAX_DEG - APP_SERVO_MIN_DEG);

    return (uint32)(pulse_us * (float)PWM_DUTY_MAX *
                    (float)APP_SERVO_PWM_FREQ_HZ / 1000000.0f);
}

float actuator_servo_get_current_angle(uint8 index)
{
    uint8 active;
    float angle;

    if(APP_SERVO_COUNT <= index)
    {
        return APP_SERVO_MID_DEG;
    }

    /* This is the low-pass-filtered PWM output command, not encoder feedback. */
    interrupt_global_disable();
    active = actuator_servo_active_frame;
    angle = actuator_servo_motion[index].output_deg;
    interrupt_global_enable();
    (void)active;
    return angle;
}

/*********************************************************************************************************************
* File: actuator_servo.c
* Description: PWM servo actuator.
********************************************************************************************************************/

#include "actuator_servo.h"
#include "app_config.h"

static servo_cmd_struct actuator_servo_cmd;
static float actuator_servo_current_angle[APP_SERVO_COUNT];
static uint32 actuator_servo_last_update_ms = 0;
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

static float actuator_servo_abs(float value)
{
    return (0.0f > value) ? -value : value;
}

static float actuator_servo_step_to_target(float current, float target, float max_step)
{
    float delta;

    delta = target - current;
    if(max_step >= actuator_servo_abs(delta))
    {
        return target;
    }
    return current + ((0.0f < delta) ? max_step : -max_step);
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
        actuator_servo_current_angle[i] = APP_SERVO_MID_DEG;
        actuator_servo_cmd.angle_deg[i] = APP_SERVO_MID_DEG;
        actuator_servo_cmd.enable[i] = APP_FALSE;
        if(APP_TRUE == actuator_servo_is_active(i))
        {
            pwm_init(actuator_servo_pwm_ch[i], APP_SERVO_PWM_FREQ_HZ, 0);
            pwm_set_duty(actuator_servo_pwm_ch[i], 0);
        }
    }
    actuator_servo_last_update_ms = 0;
}

void actuator_servo_set_cmd(const servo_cmd_struct *cmd)
{
    uint8 i;

    for(i = 0; i < APP_SERVO_COUNT; i++)
    {
        actuator_servo_cmd.angle_deg[i] = actuator_servo_limit(cmd->angle_deg[i]);
        actuator_servo_cmd.enable[i] = cmd->enable[i];
    }
}

void actuator_servo_set_angle(uint8 index, float angle_deg)
{
    if(APP_SERVO_COUNT <= index)
    {
        return;
    }
    actuator_servo_cmd.angle_deg[index] = actuator_servo_limit(angle_deg);
}

void actuator_servo_update(uint32 now_ms)
{
    uint8 i;
    uint32 elapsed_ms;
    float max_step;

    elapsed_ms = now_ms - actuator_servo_last_update_ms;
    actuator_servo_last_update_ms = now_ms;
    if((0U == elapsed_ms) || (APP_SERVO_MAX_UPDATE_GAP_MS < elapsed_ms))
    {
        elapsed_ms = APP_SERVO_PERIOD_MS;
    }
    max_step = APP_SERVO_MAX_SPEED_DPS * (float)elapsed_ms / 1000.0f;

    for(i = 0; i < APP_SERVO_COUNT; i++)
    {
        if(APP_FALSE == actuator_servo_cmd.enable[i])
        {
            continue;
        }
        actuator_servo_current_angle[i] = actuator_servo_step_to_target(actuator_servo_current_angle[i],
                                                                        actuator_servo_cmd.angle_deg[i],
                                                                        max_step);
        actuator_servo_write(i, actuator_servo_current_angle[i]);
    }
}

void actuator_servo_enable(void)
{
    uint8 i;

    for(i = 0; i < APP_SERVO_COUNT; i++)
    {
        if(APP_TRUE == actuator_servo_cmd.enable[i])
        {
            actuator_servo_write(i, actuator_servo_current_angle[i]);
        }
    }
}

void actuator_servo_disable(void)
{
    uint8 i;

    for(i = 0; i < APP_SERVO_COUNT; i++)
    {
        actuator_servo_cmd.enable[i] = APP_FALSE;
        if(APP_TRUE == actuator_servo_is_active(i))
        {
            pwm_set_duty(actuator_servo_pwm_ch[i], 0);
        }
    }
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

    return (uint32)(pulse_us * (float)PWM_DUTY_MAX / (float)APP_SERVO_PWM_PERIOD_US);
}

const servo_cmd_struct *actuator_servo_get_cmd(void)
{
    return &actuator_servo_cmd;
}

float actuator_servo_get_current_angle(uint8 index)
{
    if(APP_SERVO_COUNT <= index)
    {
        return APP_SERVO_MID_DEG;
    }

    /* This is the speed-limited PWM output command, not encoder feedback. */
    return actuator_servo_current_angle[index];
}

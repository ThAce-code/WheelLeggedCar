/*********************************************************************************************************************
* File: control_pid.c
* Description: Reusable bounded PID controller.
********************************************************************************************************************/

#include "control_pid.h"

static float control_pid_absf(float value)
{
    return (0.0f > value) ? -value : value;
}

static float control_pid_limit(float value, float limit)
{
    float abs_limit;

    abs_limit = control_pid_absf(limit);
    if(abs_limit <= 0.0f)
    {
        return value;
    }
    if(abs_limit < value)
    {
        return abs_limit;
    }
    if((-abs_limit) > value)
    {
        return -abs_limit;
    }
    return value;
}

void control_pid_init(pid_controller_struct *pid)
{
    pid->kp = 0.0f;
    pid->ki = 0.0f;
    pid->kd = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_limit = 0.0f;
    pid->output_limit = 0.0f;
    pid->first_update = APP_TRUE;
}

void control_pid_set_gain(pid_controller_struct *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void control_pid_set_limit(pid_controller_struct *pid, float integral_limit, float output_limit)
{
    pid->integral_limit = control_pid_absf(integral_limit);
    pid->output_limit = control_pid_absf(output_limit);
}

void control_pid_reset(pid_controller_struct *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->first_update = APP_TRUE;
}

float control_pid_update(pid_controller_struct *pid, float target, float feedback, float dt_s)
{
    float error;
    float derivative;
    float output;

    error = target - feedback;
    derivative = 0.0f;

    if(0.0f < dt_s)
    {
        pid->integral += error * dt_s;
        pid->integral = control_pid_limit(pid->integral, pid->integral_limit);

        if(APP_FALSE == pid->first_update)
        {
            derivative = (error - pid->prev_error) / dt_s;
        }
    }

    output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    output = control_pid_limit(output, pid->output_limit);

    pid->prev_error = error;
    pid->first_update = APP_FALSE;

    return output;
}

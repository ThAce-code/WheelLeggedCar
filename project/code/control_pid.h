/*********************************************************************************************************************
* File: control_pid.h
* Description: Reusable bounded PID controller.
********************************************************************************************************************/

#ifndef _control_pid_h_
#define _control_pid_h_

#include "app_types.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float integral_limit;
    float output_limit;
    uint8 first_update;
}pid_controller_struct;

void control_pid_init(pid_controller_struct *pid);
void control_pid_set_gain(pid_controller_struct *pid, float kp, float ki, float kd);
void control_pid_set_limit(pid_controller_struct *pid, float integral_limit, float output_limit);
void control_pid_reset(pid_controller_struct *pid);
float control_pid_update(pid_controller_struct *pid, float target, float feedback, float dt_s);

#endif

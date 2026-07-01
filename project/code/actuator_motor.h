/*********************************************************************************************************************
* File: actuator_motor.h
* Description: Brushless motor actuator interface.
********************************************************************************************************************/

#ifndef _actuator_motor_h_
#define _actuator_motor_h_

#include "app_types.h"

void actuator_motor_init(void);
void actuator_motor_set_cmd(const motor_cmd_struct *cmd);
void actuator_motor_set_actuator_cmd(const motor_actuator_cmd_struct *cmd);
void actuator_motor_set_mode_stop(void);
void actuator_motor_set_mode_open_duty(float left_duty, float right_duty);
void actuator_motor_set_mode_motor_rpm(float left_motor_rpm, float right_motor_rpm);
void actuator_motor_record_host_motion(uint32 now_ms);
/* Compatibility wrappers — prefer actuator_motor_set_mode_*() for new code */
void actuator_motor_set_motor_rpm_target(float left_motor_rpm, float right_motor_rpm, uint8 enable);
void actuator_motor_set_open_loop_duty(float left_duty, float right_duty, uint8 enable);
void actuator_motor_set_rpm_pid_gain(uint8 left_enable, uint8 right_enable, float kp, float ki, float kd);
void actuator_motor_record_command_error(uint8 is_error);
void actuator_motor_update(uint32 now_ms);
void actuator_motor_stop(void);
const motor_cmd_struct *actuator_motor_get_cmd(void);
const wheel_feedback_struct *actuator_motor_get_feedback(void);
const motor_diag_struct *actuator_motor_get_diag(void);
const motor_rpm_loop_diag_struct *actuator_motor_get_motor_rpm_loop_diag(void);

#endif

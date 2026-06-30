/*********************************************************************************************************************
* File: actuator_motor.h
* Description: Brushless motor actuator interface.
********************************************************************************************************************/

#ifndef _actuator_motor_h_
#define _actuator_motor_h_

#include "app_types.h"

void actuator_motor_init(void);
void actuator_motor_set_cmd(const motor_cmd_struct *cmd);
void actuator_motor_set_speed_target(float left_speed, float right_speed, uint8 enable);
void actuator_motor_record_command_error(uint8 is_error);
void actuator_motor_update(uint32 now_ms);
void actuator_motor_stop(void);
const motor_cmd_struct *actuator_motor_get_cmd(void);
const wheel_feedback_struct *actuator_motor_get_feedback(void);
const motor_diag_struct *actuator_motor_get_diag(void);
const motor_speed_loop_diag_struct *actuator_motor_get_speed_loop_diag(void);

#endif

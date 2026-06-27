/*********************************************************************************************************************
* File: actuator_motor.h
* Description: Brushless motor actuator interface.
********************************************************************************************************************/

#ifndef _actuator_motor_h_
#define _actuator_motor_h_

#include "app_types.h"

void actuator_motor_init(void);
void actuator_motor_set_cmd(const motor_cmd_struct *cmd);
void actuator_motor_update(uint32 now_ms);
void actuator_motor_stop(void);
const motor_cmd_struct *actuator_motor_get_cmd(void);

#endif

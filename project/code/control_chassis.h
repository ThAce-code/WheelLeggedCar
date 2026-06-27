/*********************************************************************************************************************
* File: control_chassis.h
* Description: Chassis controller interface.
********************************************************************************************************************/

#ifndef _control_chassis_h_
#define _control_chassis_h_

#include "app_types.h"

void control_chassis_init(void);
void control_chassis_update(uint32 now_ms);
const motor_cmd_struct *control_chassis_get_motor_cmd(void);

#endif

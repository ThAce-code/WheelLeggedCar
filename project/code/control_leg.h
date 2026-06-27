/*********************************************************************************************************************
* File: control_leg.h
* Description: Leg controller interface.
********************************************************************************************************************/

#ifndef _control_leg_h_
#define _control_leg_h_

#include "app_types.h"

void control_leg_init(void);
void control_leg_update(uint32 now_ms);
const servo_cmd_struct *control_leg_get_servo_cmd(void);

#endif

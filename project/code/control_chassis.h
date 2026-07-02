/*********************************************************************************************************************
* File: control_chassis.h
* Description: Chassis command mixing interface.
********************************************************************************************************************/

#ifndef _control_chassis_h_
#define _control_chassis_h_

#include "app_types.h"

void control_chassis_init(void);
void control_chassis_update(uint32 now_ms);
void control_chassis_set_cmd(float forward_rpm, float turn_rpm, uint8 enable, uint32 now_ms);
void control_chassis_stop(uint32 now_ms);
uint8 control_chassis_set_drive_gain(float speed_kp, float speed_ki, float turn_kp);
const chassis_cmd_struct *control_chassis_get_cmd(void);
const chassis_output_struct *control_chassis_get_output(void);

#endif

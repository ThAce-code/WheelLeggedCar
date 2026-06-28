/*********************************************************************************************************************
* File: actuator_servo.h
* Description: Servo actuator interface.
********************************************************************************************************************/

#ifndef _actuator_servo_h_
#define _actuator_servo_h_

#include "app_types.h"

void actuator_servo_init(void);
void actuator_servo_set_cmd(const servo_cmd_struct *cmd);
void actuator_servo_set_angle(uint8 index, float angle_deg);
void actuator_servo_update(uint32 now_ms);
void actuator_servo_enable(void);
void actuator_servo_disable(void);
uint32 actuator_servo_angle_to_duty(float angle_deg);
const servo_cmd_struct *actuator_servo_get_cmd(void);

#endif

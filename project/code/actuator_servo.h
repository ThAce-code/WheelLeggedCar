/*********************************************************************************************************************
* File: actuator_servo.h
* Description: Servo actuator interface.
*********************************************************************************************************************/

#ifndef _actuator_servo_h_
#define _actuator_servo_h_

#include "app_types.h"

void actuator_servo_init(void);
void actuator_servo_enable(void);
void actuator_servo_disable(void);
uint32 actuator_servo_angle_to_duty(float angle_deg);
float actuator_servo_get_current_angle(uint8 index);

void actuator_servo_publish_cmd(const servo_cmd_struct *cmd,
                                float speed_limit_dps,
                                uint8 direct_bypass);
void actuator_servo_tick_300hz(void);
void actuator_servo_get_diag(actuator_servo_diag_struct *diag);
uint8 actuator_servo_is_settled(void);

#endif

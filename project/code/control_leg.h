/*********************************************************************************************************************
* File: control_leg.h
* Description: Leg controller interface.
********************************************************************************************************************/

#ifndef _control_leg_h_
#define _control_leg_h_

#include "app_types.h"
#include "leg_config.h"

typedef enum
{
    LEG_MODE_LOCK = 0,
    LEG_MODE_MANUAL,
    LEG_MODE_ATTITUDE
}leg_mode_enum;

void control_leg_init(void);
void control_leg_update(uint32 now_ms);
void control_leg_set_mode(leg_mode_enum mode);
void control_leg_set_manual_angle(uint8 leg_id, float angle_deg);
void control_leg_set_body_cmd(float height_cmd, float pitch_cmd, float roll_cmd);
const servo_cmd_struct *control_leg_get_servo_cmd(void);
uint8 control_leg_get_safe_ready(void);

#endif

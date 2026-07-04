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
    LEG_MODE_IK_CALIB,
    LEG_MODE_HEIGHT
}leg_mode_enum;

void control_leg_init(void);
void control_leg_update(uint32 now_ms);
void control_leg_set_mode(leg_mode_enum mode);
void control_leg_set_manual_angle(uint8 leg_id, float angle_deg);
void control_leg_set_body_cmd(float height_cmd, float pitch_cmd, float roll_cmd);
uint8 control_leg_set_height(float height_mm, uint32 now_ms);
uint8 control_leg_set_calib_angles(float servo0_deg,
                                   float servo1_deg,
                                   float servo2_deg,
                                   float servo3_deg);
const servo_cmd_struct *control_leg_get_servo_cmd(void);
const leg_diag_struct *control_leg_get_diag(void);

#endif

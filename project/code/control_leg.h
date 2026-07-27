/*********************************************************************************************************************
* File: control_leg.h
* Description: Leg controller interface.
********************************************************************************************************************/

#ifndef _control_leg_h_
#define _control_leg_h_

#include "app_config.h"
#include "app_types.h"
#include "leg_config.h"

typedef enum
{
    LEG_MODE_LOCK = 0,
    LEG_MODE_MANUAL,
    LEG_MODE_IK_CALIB,
    LEG_MODE_LEGACY_STANCE,
    LEG_MODE_FAST_LEGACY_STANCE,
    LEG_MODE_DIRECT_LEGACY_STANCE,
    LEG_MODE_IK_REFERENCE,
    LEG_MODE_IK_VALIDATE,
    LEG_MODE_RACE_ASSIST
}leg_mode_enum;

void control_leg_init(void);
void control_leg_update(uint32 now_ms);
void control_leg_set_mode(leg_mode_enum mode);
void control_leg_set_manual_angle(uint8 leg_id, float angle_deg);
void control_leg_set_body_cmd(float legacy_stance_cmd, float pitch_cmd, float roll_cmd);
/* A valid LH command clears only the leg controller's soft fault. */
uint8 control_leg_set_legacy_stance(float stance_units, uint32 now_ms);
uint8 control_leg_set_fast_legacy_stance(float stance_units, uint32 now_ms);
uint8 control_leg_set_direct_legacy_stance(float stance_units, uint32 now_ms);
uint8 control_leg_set_ik_reference(uint32 now_ms);
uint8 control_leg_set_xy(float x_mm, float y_mm, uint32 now_ms);
uint8 control_leg_set_race_assist_request(float u_request,
                                          float dx_mm,
                                          float dy_mm,
                                          uint32 now_ms);
void control_leg_disable_race_assist(uint32 now_ms);
#if (APP_RACE_ASSIST_PREFLIGHT_WCET_ENABLE == 1U)
extern volatile uint32 control_leg_race_preflight_count;
extern volatile uint32 control_leg_race_preflight_last_cycles;
extern volatile uint32 control_leg_race_preflight_max_cycles;
#endif
uint8 control_leg_set_calib_angles(float servo0_deg,
                                   float servo1_deg,
                                   float servo2_deg,
                                   float servo3_deg);
const servo_cmd_struct *control_leg_get_servo_cmd(void);
const leg_diag_struct *control_leg_get_diag(void);

#endif

/*********************************************************************************************************************
* File: control_balance.h
* Description: Low-power balance control interface.
********************************************************************************************************************/

#ifndef _control_balance_h_
#define _control_balance_h_

#include "app_types.h"

void control_balance_init(void);
void control_balance_update(uint32 now_ms);
void control_balance_set_mode(balance_mode_enum mode);
void control_balance_set_gain(float pitch_kp, float pitch_rate_kd);
void control_balance_set_full_gain(float pitch_kp, float pitch_rate_kd, float wheel_speed_ks, float wheel_pos_kp);
void control_balance_reset_motion_state_public(void);
void control_balance_set_ident_excitation(float amp_rpm, uint32 period_ms, uint32 now_ms);
balance_mode_enum control_balance_get_mode(void);
const balance_diag_struct *control_balance_get_diag(void);

#endif

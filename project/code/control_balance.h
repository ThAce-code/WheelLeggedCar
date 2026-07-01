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
balance_mode_enum control_balance_get_mode(void);
const balance_diag_struct *control_balance_get_diag(void);

#endif

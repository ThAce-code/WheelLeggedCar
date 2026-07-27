/*********************************************************************************************************************
* File: servo_motion.h
* Description: Pure first-order low-pass servo motion unit (deterministic, hardware-free).
*********************************************************************************************************************/

#ifndef _servo_motion_h_
#define _servo_motion_h_

#include "app_types.h"

typedef struct
{
    float target_deg;
    float filtered_deg;
    float output_deg;
    uint16 settled_ticks;
    uint8 settled;
}servo_motion_state_struct;

void servo_motion_init(servo_motion_state_struct *state, float angle_deg);
void servo_motion_step(servo_motion_state_struct *state,
                       float target_deg,
                       float alpha,
                       float max_step_deg,
                       float settle_error_deg,
                       uint16 settle_ticks_required);
void servo_motion_apply_immediate(servo_motion_state_struct *state, float angle_deg);

#endif

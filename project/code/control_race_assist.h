/*********************************************************************************************************************
* File: control_race_assist.h
* Description: Pure fail-closed supervisor for a future low-pose race assist integration.
*********************************************************************************************************************/

#ifndef _control_race_assist_h_
#define _control_race_assist_h_

#include "app_types.h"

typedef enum
{
    RACE_ASSIST_DISABLED = 0,
    RACE_ASSIST_LOW_RACE,
    RACE_ASSIST_ARMED,
    RACE_ASSIST_BOOST,
    RACE_ASSIST_CRUISE_HOLD,
    RACE_ASSIST_BRAKE,
    RACE_ASSIST_RECENTER,
    RACE_ASSIST_FAULT_HOLD
}race_assist_state_enum;

typedef enum
{
    RACE_ASSIST_FAULT_NONE = 0,
    RACE_ASSIST_FAULT_INPUT_INVALID,
    RACE_ASSIST_FAULT_LEG_NOT_READY,
    RACE_ASSIST_FAULT_LEG_PATH,
    RACE_ASSIST_FAULT_PITCH_LIMIT
}race_assist_fault_reason_enum;

typedef struct
{
    float forward_limit_rpm;
    float balance_limit_rpm;
    float dx_mm;
    float dy_mm;
}race_assist_level_profile_struct;

typedef struct
{
    float target_rpm;
    float ramped_rpm;
    float measured_rpm;
    float pitch_deg;
    float pitch_rate_dps;
    float leg_u_actual;
    float dt_s;
    uint8 fast_enable;
    uint8 feedback_healthy;
    uint8 low_pose_ready;
    uint8 leg_path_fault;
}race_assist_input_struct;

typedef struct
{
    race_assist_state_enum state;
    race_assist_fault_reason_enum fault_reason;
    float requested_accel_rpm_s;
    float speed_error_rpm;
    float speed_blend;
    float u_request;
    float forward_limit_rpm;
    float balance_limit_rpm;
    float turn_scale;
    float pitch_offset_limit_deg;
    float dx_mm;
    float dy_mm;
    uint8 enable;
    uint8 level;
    uint8 leg_command_enable;
}race_assist_output_struct;

void control_race_assist_init(void);
uint8 control_race_assist_set_level(uint8 level);
uint8 control_race_assist_set_gains(float accel_gain, float error_gain, float hold_bias);
void control_race_assist_report_leg_path_fault(void);
void control_race_assist_update(const race_assist_input_struct *input);
const race_assist_output_struct *control_race_assist_get_output(void);
uint8 control_race_assist_get_level_profile(uint8 level,
                                            race_assist_level_profile_struct *profile);

#endif

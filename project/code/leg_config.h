/*********************************************************************************************************************
* File: leg_config.h
* Description: Static configuration for the four wheel-leg servos.
********************************************************************************************************************/

#ifndef _leg_config_h_
#define _leg_config_h_

#include "zf_common_headfile.h"

typedef enum
{
    LEG_SERVO_FL = 0,
    LEG_SERVO_FR = 1,
    LEG_SERVO_RL = 2,
    LEG_SERVO_RR = 3,
    LEG_SERVO_COUNT = 4
}leg_servo_id_enum;

typedef struct
{
    uint8  servo_index;
    float  safe_deg;
    float  neutral_deg;
    float  min_deg;
    float  max_deg;
    float  direction;
    float  mount_x;
    float  mount_y;
}leg_servo_config_struct;

typedef struct
{
    leg_servo_config_struct servo[LEG_SERVO_COUNT];
    float  height_min;
    float  height_max;
    float  pitch_limit;
    float  roll_limit;
}leg_config_struct;

const leg_config_struct      *leg_config_get(void);
const leg_servo_config_struct *leg_config_get_servo(uint8 leg_id);

#endif

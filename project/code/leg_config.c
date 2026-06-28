/*********************************************************************************************************************
* File: leg_config.c
* Description: Default wheel-leg servo configuration.
********************************************************************************************************************/

#include "leg_config.h"

static const leg_config_struct leg_config_default =
{
    {
        {0,  90.0f,  90.0f, 0.0f, 180.0f, 1.0f,  1.0f,  1.0f},
        {1,  90.0f,  90.0f, 0.0f, 180.0f, -1.0f,  1.0f, -1.0f},
        {2,  90.0f,  90.0f, 0.0f, 180.0f, -1.0f, -1.0f,  1.0f},
        {3,  90.0f,  90.0f, 0.0f, 180.0f, 1.0f, -1.0f, -1.0f}
    },
    -30.0f,
     30.0f,
     30.0f,
     30.0f
};

const leg_config_struct *leg_config_get(void)
{
    return &leg_config_default;
}

const leg_servo_config_struct *leg_config_get_servo(uint8 leg_id)
{
    if(LEG_SERVO_COUNT <= leg_id)
    {
        return NULL;
    }
    return &leg_config_default.servo[leg_id];
}

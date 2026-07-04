/*********************************************************************************************************************
* File: leg_config.c
* Description: Default wheel-leg servo configuration.
********************************************************************************************************************/

#include "leg_config.h"

static const leg_config_struct leg_config_default =
{
    {
        {0,  90.0f,  90.0f, 15.0f, 165.0f,  1.0f,  1.0f,  1.0f},
        {1,  90.0f,  90.0f, 15.0f, 165.0f, -1.0f,  1.0f, -1.0f},
        {2,  90.0f,  90.0f, 15.0f, 165.0f, -1.0f, -1.0f,  1.0f},
        {3,  90.0f,  90.0f, 15.0f, 165.0f,  1.0f, -1.0f, -1.0f}
    },
    {
        60.0f,   /* L1 — driven link (measured) */
        90.0f,   /* L2 — passive link (measured) */
        90.0f,   /* L3 — passive link (measured) */
        60.0f,   /* L4 — driven link (measured) */
        38.0f,   /* L5 — base spacing (measured) */
        -35.0f,
        35.0f,
        70.0f,
        145.0f,
        0.0f,
        0.0f,
        LEG_IK_BRANCH_PLUS,
        LEG_IK_BRANCH_MINUS,
        LEG_IK_BRANCH_PLUS,
        LEG_IK_BRANCH_MINUS
    },
    {
        80.0f,
        130.0f,
        100.0f,
        60.0f,
        30.0f,
        18.0f,
        22.0f,
        8.0f,
        10.0f,
        3.0f,
        2.0f,
        -1.35f,
        -1.35f,
        80.0f,
        40.0f,
        220.0f,
        120.0f
    },
    80.0f,
    130.0f,
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

const leg_kinematics_config_struct *leg_config_get_kinematics(void)
{
    return &leg_config_default.kinematics;
}

const leg_height_profile_struct *leg_config_get_height_profile(void)
{
    return &leg_config_default.height_profile;
}

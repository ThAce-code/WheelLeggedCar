/*********************************************************************************************************************
* File: leg_kinematics.h
* Description: Five-bar wheel-leg inverse kinematics.
********************************************************************************************************************/

#ifndef _leg_kinematics_h_
#define _leg_kinematics_h_

#include "app_types.h"
#include "leg_config.h"

typedef struct
{
    float servo_deg[2];
    float alpha_rad;
    float beta_rad;
    float singularity_margin;
    uint8 valid;
}leg_ik_result_struct;

uint8 leg_kinematics_solve(uint8 right_side,
                           float x_mm,
                           float y_mm,
                           const leg_ik_result_struct *previous,
                           leg_ik_result_struct *result);
uint8 leg_kinematics_forward(uint8 right_side,
                              float servo_a_deg,
                              float servo_b_deg,
                              float *x_mm,
                              float *y_mm);
uint8 leg_kinematics_map_reference_pose(const leg_ik_result_struct *left,
                                        const leg_ik_result_struct *right,
                                        float servo_deg[LEG_SERVO_COUNT]);
uint8 leg_kinematics_map_target_pose(const leg_ik_result_struct *left_reference,
                                     const leg_ik_result_struct *right_reference,
                                     const leg_ik_result_struct *left_target,
                                     const leg_ik_result_struct *right_target,
                                     float servo_deg[LEG_SERVO_COUNT]);

#endif

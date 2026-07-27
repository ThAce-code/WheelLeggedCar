/*********************************************************************************************************************
* File: single_gap_controller.h
* Description: Fail-closed image-space controller for one cone gap.
*********************************************************************************************************************/

#ifndef _single_gap_controller_h_
#define _single_gap_controller_h_

#include "single_gap_types.h"

void single_gap_controller_init(single_gap_controller_struct *controller);
void single_gap_controller_set_armed(single_gap_controller_struct *controller,
                                      uint8 armed,
                                      uint32 now_ms);
void single_gap_controller_update(single_gap_controller_struct *controller,
                                  const single_gap_observation_struct *observation,
                                  const single_gap_tof_snapshot_struct *tof,
                                  float odometry_m,
                                  uint8 odometry_valid,
                                  float forward_rpm,
                                  uint32 now_ms,
                                  single_gap_output_struct *output);

#endif

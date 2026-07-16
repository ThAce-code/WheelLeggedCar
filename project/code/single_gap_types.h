/*********************************************************************************************************************
* File: single_gap_types.h
* Description: Pointer-free contracts for single-gap sensing and control.
*********************************************************************************************************************/

#ifndef _single_gap_types_h_
#define _single_gap_types_h_

#include "perception_types.h"

typedef enum
{
    SINGLE_GAP_STATE_DISABLED = 0,
    SINGLE_GAP_STATE_ACQUIRE,
    SINGLE_GAP_STATE_APPROACH,
    SINGLE_GAP_STATE_PASS_CANDIDATE,
    SINGLE_GAP_STATE_PASSED,
    SINGLE_GAP_STATE_FAULT_STOP
} single_gap_state_enum;

typedef enum
{
    SINGLE_GAP_STOP_NONE = 0,
    SINGLE_GAP_STOP_DISABLED,
    SINGLE_GAP_STOP_FRAME_STALE,
    SINGLE_GAP_STOP_TARGET_LOST,
    SINGLE_GAP_STOP_TARGET_AMBIGUOUS,
    SINGLE_GAP_STOP_GAP_NARROW,
    SINGLE_GAP_STOP_TOF_INVALID,
    SINGLE_GAP_STOP_TOF_STALE,
    SINGLE_GAP_STOP_TOF_NEAR,
    SINGLE_GAP_STOP_ODOMETRY,
    SINGLE_GAP_STOP_PASS_TIMEOUT,
    SINGLE_GAP_STOP_PASSED
} single_gap_stop_reason_enum;

typedef struct
{
    uint32 sequence;
    uint32 capture_ms;
    uint16 accepted_count;
    uint16 gap_center_x;
    uint16 gap_width_px;
    uint16 left_bottom_y;
    uint16 right_bottom_y;
    uint8 valid;
    uint8 ambiguous;
} single_gap_observation_struct;

typedef struct
{
    uint32 sample_ms;
    uint16 distance_mm;
    uint8 initialized;
    uint8 valid;
} single_gap_tof_snapshot_struct;

typedef struct
{
    uint32 acquire_bits;
    uint32 state_enter_ms;
    uint32 last_frame_ms;
    float pass_start_odometry_m;
    float previous_error;
    float previous_turn_dps;
    uint16 gap_center_history[3];
    uint8 acquire_count;
    uint8 lost_count;
    uint8 bottom_count;
    uint8 gap_history_count;
    uint8 gap_history_index;
    uint8 armed;
    single_gap_state_enum state;
    single_gap_stop_reason_enum stop_reason;
} single_gap_controller_struct;

typedef struct
{
    float forward_rpm;
    float turn_rate_dps;
    uint16 gap_center_x;
    uint16 gap_width_px;
    uint8 enable;
    single_gap_state_enum state;
    single_gap_stop_reason_enum stop_reason;
} single_gap_output_struct;

#endif

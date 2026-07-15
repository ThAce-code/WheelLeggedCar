/*********************************************************************************************************************
* File: perception_types.h
* Description: Pointer-free public data contracts for cone perception.
*********************************************************************************************************************/

#ifndef _perception_types_h_
#define _perception_types_h_

#include "perception_config.h"

#define PERCEPTION_POSE_VALID_IMU              (0x0001U)
#define PERCEPTION_POSE_VALID_WHEEL            (0x0002U)
#define PERCEPTION_POSE_VALID_LEG_HEIGHT       (0x0004U)
#define PERCEPTION_POSE_VALID_ODOMETRY         (0x0008U)

#define PERCEPTION_TARGET_VALID                (0x0001U)
#define PERCEPTION_TARGET_FROZEN               (0x0002U)
#define PERCEPTION_TARGET_PREDICTION_ONLY      (0x0004U)

#define PERCEPTION_TRACK_VALID                 (0x0001U)
#define PERCEPTION_TRACK_CONFIRMED             (0x0002U)
#define PERCEPTION_TRACK_FROZEN                (0x0004U)

typedef enum
{
    PERCEPTION_STATE_BOOT = 0,
    PERCEPTION_STATE_READY,
    PERCEPTION_STATE_OUTBOUND_MAP,
    PERCEPTION_STATE_TURN_LOCK,
    PERCEPTION_STATE_RETURN_GAPS,
    PERCEPTION_STATE_FINISH,
    PERCEPTION_STATE_SAFE_STOP
} perception_state_enum;

typedef struct
{
    uint16 left;
    uint16 top;
    uint16 right;
    uint16 bottom;
} perception_track_window_struct;

typedef struct
{
    uint16 left;
    uint16 top;
    uint16 right;
    uint16 bottom;
    uint16 bottom_center_x;
    uint16 bottom_center_y;
    uint16 area_px;
    uint16 score;
    uint8 taper_score;
    uint8 band_score;
    uint8 symmetry_score;
    uint8 aspect_score;
    uint8 base_score;
    uint8 contrast_score;
    uint8 slice_count;
    uint8 reserved;
} perception_candidate_struct;

typedef struct
{
    uint32 sequence;
    uint32 timestamp_us;
    float position_x_m;
    float position_y_m;
    float yaw_rad;
    float roll_rad;
    float pitch_rad;
    float speed_mps;
    float camera_height_m;
    uint16 validity_flags;
    uint16 reserved;
    uint32 crc32;
} perception_pose_snapshot_struct;

typedef struct
{
    uint32 timestamp_us;
    uint16 source_score;
    uint16 validity_flags;
    float position_x_m;
    float position_y_m;
    float covariance_xx;
    float covariance_xy;
    float covariance_yy;
} perception_ground_observation_struct;

typedef struct
{
    uint16 cone_id;
    uint16 validity_flags;
    uint16 hit_count;
    uint16 miss_count;
    float position_x_m;
    float position_y_m;
    float covariance_xx;
    float covariance_xy;
    float covariance_yy;
} perception_track_summary_struct;

typedef struct
{
    uint16 gap_id;
    uint16 left_cone_id;
    uint16 right_cone_id;
    uint16 validity_flags;
    float center_x_m;
    float center_y_m;
    float heading_rad;
    float approach_x_m;
    float approach_y_m;
    float exit_x_m;
    float exit_y_m;
} perception_gap_target_struct;

typedef struct
{
    uint32 frame_count;
    uint32 roi_pixel_count;
    uint32 candidate_overflow_count;
    uint32 track_overflow_count;
    uint32 quality_reject_count;
    uint32 crc_error_count;
    uint32 max_frame_time_us;
    uint16 accepted_candidate_count;
    uint16 confirmed_track_count;
} perception_diagnostics_struct;

typedef struct
{
    uint32 sequence;
    uint32 timestamp_us;
    uint16 state;
    uint16 validity_flags;
    uint16 map_count;
    uint16 gap_count;
    perception_gap_target_struct current_target;
    perception_track_summary_struct map[PERCEPTION_MAX_TRACKS];
    perception_diagnostics_struct diagnostics;
    uint32 crc32;
} perception_snapshot_struct;

#endif

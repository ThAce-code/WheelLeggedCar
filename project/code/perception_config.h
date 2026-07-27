/*********************************************************************************************************************
* File: perception_config.h
* Description: Fixed limits and timing contracts for cone perception.
*********************************************************************************************************************/

#ifndef _perception_config_h_
#define _perception_config_h_

#include "zf_common_typedef.h"

#define PERCEPTION_IMAGE_WIDTH                 (188U)
#define PERCEPTION_IMAGE_HEIGHT                (120U)
#define PERCEPTION_FRAME_RATE_HZ               (50U)
#define PERCEPTION_DISCOVERY_DIVIDER           (2U)
#define PERCEPTION_MAP_RATE_HZ                 (20U)

#define PERCEPTION_MAX_CANDIDATES              (24U)
#define PERCEPTION_MAX_TRACKS                  (32U)
#define PERCEPTION_MAX_GAPS                    (31U)
#define PERCEPTION_MAX_TRACK_WINDOWS           (8U)
#define PERCEPTION_MAX_SLICES                  (5U)
#define PERCEPTION_HORIZON_SAMPLES             (16U)

#define PERCEPTION_SCORE_ACCEPT                (166U)
#define PERCEPTION_SCORE_HIGH                  (204U)
#define PERCEPTION_WEIGHT_TAPER                (77U)
#define PERCEPTION_WEIGHT_BAND                 (64U)
#define PERCEPTION_WEIGHT_SYMMETRY             (38U)
#define PERCEPTION_WEIGHT_ASPECT               (26U)
#define PERCEPTION_WEIGHT_BASE                 (26U)
#define PERCEPTION_WEIGHT_CONTRAST             (25U)
#define PERCEPTION_WEIGHT_SUM                  (PERCEPTION_WEIGHT_TAPER + PERCEPTION_WEIGHT_BAND + \
                                                PERCEPTION_WEIGHT_SYMMETRY + PERCEPTION_WEIGHT_ASPECT + \
                                                PERCEPTION_WEIGHT_BASE + PERCEPTION_WEIGHT_CONTRAST)

#if (PERCEPTION_WEIGHT_SUM != 256U)
#error "Perception score weights must sum to 256"
#endif

#define PERCEPTION_GATE_CHI2_Q16               (392561UL)
#define PERCEPTION_TRACK_CONFIRM_HITS          (3U)
#define PERCEPTION_TRACK_CONFIRM_WINDOW        (5U)
#define PERCEPTION_TRACK_FAR_CONFIRM_HITS      (4U)
#define PERCEPTION_TRACK_FAR_CONFIRM_WINDOW    (6U)
#define PERCEPTION_TRACK_PREDICT_HOLD_FRAMES   (8U)
#define PERCEPTION_TRACK_DELETE_FRAMES         (20U)

#define PERCEPTION_IMU_STALE_MS                (30U)
#define PERCEPTION_WHEEL_STALE_MS              (100U)
#define PERCEPTION_CAMERA_PREDICT_MS           (100U)
#define PERCEPTION_CAMERA_INVALID_MS           (300U)
#define PERCEPTION_FRAME_BUDGET_US             (20000U)
#define PERCEPTION_P99_TARGET_US               (15000U)

#define PERCEPTION_RANGE_MIN_M                 (0.25f)
#define PERCEPTION_RANGE_MAX_M                 (8.00f)
#define PERCEPTION_HORIZON_MARGIN_PX           (4)
#define PERCEPTION_VEHICLE_MASK_TOP_PX         (108U)
#define PERCEPTION_GAP_OFFSET_M                (0.60f)

#endif

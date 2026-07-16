/*********************************************************************************************************************
* File: single_gap_config.h
* Description: Fixed limits and fail-closed compile gates for single-gap validation.
*********************************************************************************************************************/

#ifndef _single_gap_config_h_
#define _single_gap_config_h_

#define SINGLE_GAP_ENABLE                    (0U)
#define SINGLE_GAP_MOTION_ENABLE             (0U)
#define SINGLE_GAP_IMAGE_WIDTH               (188U)
#define SINGLE_GAP_IMAGE_HEIGHT              (120U)
#define SINGLE_GAP_MAX_RUNS                  (96U)
#define SINGLE_GAP_MAX_COMPONENTS            (24U)
#define SINGLE_GAP_MAX_ACCEPTED              (3U)
#define SINGLE_GAP_ROI_TOP_PX                (20U)
#define SINGLE_GAP_ROI_BOTTOM_PX             (107U)
#define SINGLE_GAP_CONTROL_PERIOD_MS          (40U)
#define SINGLE_GAP_TOF_PERIOD_MS              (50U)
#define SINGLE_GAP_POSE_PERIOD_MS             (50U)
#define SINGLE_GAP_SENSOR_STALE_MS            (100U)
#define SINGLE_GAP_ACQUIRE_WINDOW_FRAMES      (7U)
#define SINGLE_GAP_ACQUIRE_HITS               (5U)
#define SINGLE_GAP_LOST_FRAMES                (5U)
#define SINGLE_GAP_MIN_WIDTH_PX               (24U)
#define SINGLE_GAP_BOTTOM_ENTER_PX            (96U)
#define SINGLE_GAP_BOTTOM_CONFIRM_FRAMES      (2U)
#define SINGLE_GAP_DEADBAND_PX                (3)
#define SINGLE_GAP_TURN_LIMIT_DPS             (15.0f)
#define SINGLE_GAP_TURN_STEP_DPS              (5.0f)
#define SINGLE_GAP_TURN_DECAY_MS              (400U)
#define SINGLE_GAP_PASS_TIMEOUT_MS            (1500U)
#define SINGLE_GAP_PASS_DISTANCE_M            (0.20f)
#define SINGLE_GAP_FORWARD_SPEED_MPS           (0.20f)
#define SINGLE_GAP_MAX_SPEED_MPS               (0.30f)
#define SINGLE_GAP_FORWARD_LIMIT_RPM           (60.0f)
#define SINGLE_GAP_TOF_STOP_MM                 (350U)
#define SINGLE_GAP_WHEEL_CIRCUMFERENCE_MM      (0U)
#define SINGLE_GAP_NAV_VALID_MS                (100U)

#if (SINGLE_GAP_MOTION_ENABLE && !SINGLE_GAP_ENABLE)
#error "single-gap motion requires single-gap sensing"
#endif

#if (SINGLE_GAP_MOTION_ENABLE && (SINGLE_GAP_WHEEL_CIRCUMFERENCE_MM == 0U))
#error "measure wheel circumference before enabling motion"
#endif

#endif

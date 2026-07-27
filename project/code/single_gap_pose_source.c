/*********************************************************************************************************************
* File: single_gap_pose_source.c
* Description: CM7_0 short-range wheel-odometry publisher for single-gap validation.
*********************************************************************************************************************/

#include "single_gap_pose_source.h"

#include <actuator_motor.h>
#include <control_balance.h>
#include "intercore_memory.h"
#include "perception_intercore.h"
#include "single_gap_config.h"

#include <string.h>

static perception_intercore_transport_struct single_gap_pose_transport;
static float single_gap_distance_m;
static uint32 single_gap_pose_last_ms;
static uint32 single_gap_pose_sequence;
static uint8 single_gap_pose_initialized;

static uint8 single_gap_pose_float_is_finite(float value)
{
    return ((value == value) &&
            (3.402823466e+38F >= value) &&
            (-3.402823466e+38F <= value)) ? 1U : 0U;
}

float single_gap_pose_integrate_distance(float previous_m,
                                         float wheel_rpm,
                                         float circumference_m,
                                         uint32 dt_ms)
{
    if((0U == single_gap_pose_float_is_finite(previous_m)) ||
       (0U == single_gap_pose_float_is_finite(wheel_rpm)) ||
       (0U == single_gap_pose_float_is_finite(circumference_m)) ||
       (0.0f >= circumference_m) || (0U == dt_ms))
    {
        return previous_m;
    }
    return previous_m + wheel_rpm * ((float)dt_ms / 60000.0f) * circumference_m;
}

uint8 single_gap_pose_source_init(void)
{
    volatile intercore_shared_layout_struct *shared = intercore_memory_get_layout();

    memset(&single_gap_pose_transport, 0, sizeof(single_gap_pose_transport));
    single_gap_distance_m = 0.0f;
    single_gap_pose_last_ms = 0U;
    single_gap_pose_sequence = 0U;
    single_gap_pose_initialized = 0U;
    if((NULL == shared) ||
       (0U == perception_intercore_cm7_0_init(&single_gap_pose_transport, shared)))
    {
        return 0U;
    }
    single_gap_pose_initialized = 1U;
    return 1U;
}

void single_gap_pose_source_update(uint32 now_ms)
{
    const balance_diag_struct *balance;
    const wheel_feedback_struct *wheel;
    perception_pose_snapshot_struct snapshot;
    float circumference_m;
    float wheel_rpm;
    uint32 dt_ms;
    uint8 wheel_valid;

    if((0U == single_gap_pose_initialized) ||
       (SINGLE_GAP_POSE_PERIOD_MS > (now_ms - single_gap_pose_last_ms)))
    {
        return;
    }

    dt_ms = now_ms - single_gap_pose_last_ms;
    single_gap_pose_last_ms = now_ms;
    balance = control_balance_get_diag();
    wheel = actuator_motor_get_feedback();
    wheel_rpm = (NULL != balance) ? balance->wheel_speed_rpm : 0.0f;
    circumference_m = (float)SINGLE_GAP_WHEEL_CIRCUMFERENCE_MM / 1000.0f;
    wheel_valid = ((NULL != balance) && (NULL != wheel) &&
                   (0U != wheel->online) && (0U != wheel->left_online) &&
                   (0U != wheel->right_online) &&
                   (SINGLE_GAP_SENSOR_STALE_MS >= wheel->age_ms) &&
                   (0U != single_gap_pose_float_is_finite(wheel_rpm)) &&
                   (0.0f < circumference_m)) ? 1U : 0U;
    if(0U != wheel_valid)
    {
        single_gap_distance_m = single_gap_pose_integrate_distance(
                                    single_gap_distance_m,
                                    wheel_rpm,
                                    circumference_m,
                                    dt_ms);
    }

    memset(&snapshot, 0, sizeof(snapshot));
    single_gap_pose_sequence++;
    if(0U == single_gap_pose_sequence)
    {
        single_gap_pose_sequence = 1U;
    }
    snapshot.sequence = single_gap_pose_sequence;
    snapshot.timestamp_us = now_ms * 1000U;
    snapshot.position_x_m = single_gap_distance_m;
    if(0U != wheel_valid)
    {
        snapshot.speed_mps = wheel_rpm * circumference_m / 60.0f;
        snapshot.validity_flags = PERCEPTION_POSE_VALID_WHEEL |
                                  PERCEPTION_POSE_VALID_ODOMETRY;
    }
    (void)perception_intercore_publish_pose(&single_gap_pose_transport, &snapshot);
}

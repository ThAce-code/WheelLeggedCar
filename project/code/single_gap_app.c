/*********************************************************************************************************************
* File: single_gap_app.c
* Description: CM7_1 single-gap sensing and fail-closed navigation publisher.
*********************************************************************************************************************/

#include "single_gap_app.h"

#include "dl1b_safety.h"
#include "intercore_memory.h"
#include "intercore_transport.h"
#include "perception_intercore.h"
#include "single_gap_config.h"
#include "single_gap_controller.h"
#include "single_gap_detector.h"

#include <stdint.h>
#include <string.h>

static uint8 single_gap_app_float_is_finite(float value)
{
    return ((value == value) &&
            (3.402823466e+38F >= value) &&
            (-3.402823466e+38F <= value)) ? 1U : 0U;
}

float single_gap_speed_mps_to_rpm(float speed_mps, float circumference_m)
{
    if((0U == single_gap_app_float_is_finite(speed_mps)) ||
       (0U == single_gap_app_float_is_finite(circumference_m)) ||
       (0.0f > speed_mps) || (0.0f >= circumference_m))
    {
        return 0.0f;
    }
    return speed_mps * 60.0f / circumference_m;
}

#if (SINGLE_GAP_ENABLE == 1U)
static intercore_transport_struct single_gap_navigation_transport;
static perception_intercore_transport_struct single_gap_pose_transport;
static single_gap_controller_struct single_gap_controller;
static single_gap_observation_struct single_gap_observation;
static perception_pose_snapshot_struct single_gap_pose;
static uint32 single_gap_last_service_ms;
static uint32 single_gap_pose_received_ms;
static uint32 single_gap_command_sequence;
static uint8 single_gap_has_observation;
static uint8 single_gap_initialized;

static uint32 single_gap_next_sequence(void)
{
    single_gap_command_sequence++;
    if(0U == single_gap_command_sequence)
    {
        single_gap_command_sequence = 1U;
    }
    return single_gap_command_sequence;
}

static uint8 single_gap_map_stop_reason(single_gap_stop_reason_enum reason)
{
    switch(reason)
    {
        case SINGLE_GAP_STOP_NONE:
            return (uint8)NAVIGATION_STOP_NONE;
        case SINGLE_GAP_STOP_DISABLED:
        case SINGLE_GAP_STOP_PASSED:
            return (uint8)NAVIGATION_STOP_DISABLED;
        case SINGLE_GAP_STOP_FRAME_STALE:
        case SINGLE_GAP_STOP_TOF_STALE:
            return (uint8)NAVIGATION_STOP_STALE;
        case SINGLE_GAP_STOP_TOF_NEAR:
            return (uint8)NAVIGATION_STOP_EMERGENCY;
        default:
            return (uint8)NAVIGATION_STOP_INVALID;
    }
}

static uint8 single_gap_diagnostic_stop_reason(
    const single_gap_tof_snapshot_struct *tof,
    float forward_rpm,
    uint32 now_ms,
    uint8 controller_reason)
{
    if((NULL == tof) || (0U == tof->initialized) || (0U == tof->valid))
    {
        return (uint8)NAVIGATION_STOP_INVALID;
    }
    if(SINGLE_GAP_SENSOR_STALE_MS < (now_ms - tof->sample_ms))
    {
        return (uint8)NAVIGATION_STOP_STALE;
    }
    if(SINGLE_GAP_TOF_STOP_MM >= tof->distance_mm)
    {
        return (uint8)NAVIGATION_STOP_EMERGENCY;
    }
    if((0U == single_gap_has_observation) ||
       (SINGLE_GAP_SENSOR_STALE_MS <
        (now_ms - single_gap_observation.capture_ms)))
    {
        return (uint8)NAVIGATION_STOP_STALE;
    }
    if((0U == single_gap_app_float_is_finite(forward_rpm)) ||
       (0.0f >= forward_rpm) ||
       (SINGLE_GAP_FORWARD_LIMIT_RPM < forward_rpm))
    {
        return (uint8)NAVIGATION_STOP_INVALID;
    }
    return controller_reason;
}
#endif

uint8 single_gap_app_init(void)
{
#if (SINGLE_GAP_ENABLE == 1U)
    volatile intercore_shared_layout_struct *shared = intercore_memory_get_layout();

    memset(&single_gap_navigation_transport, 0,
           sizeof(single_gap_navigation_transport));
    memset(&single_gap_pose_transport, 0, sizeof(single_gap_pose_transport));
    memset(&single_gap_observation, 0, sizeof(single_gap_observation));
    memset(&single_gap_pose, 0, sizeof(single_gap_pose));
    single_gap_controller_init(&single_gap_controller);
    single_gap_controller_set_armed(&single_gap_controller,
                                    (uint8)SINGLE_GAP_MOTION_ENABLE,
                                    0U);
    single_gap_last_service_ms = 0U;
    single_gap_pose_received_ms = 0U;
    single_gap_command_sequence = 0U;
    single_gap_has_observation = 0U;
    single_gap_initialized = 0U;
    camera_frame_consumer_set_handler(NULL);

    if((NULL == shared) ||
       (0U == intercore_transport_cm7_1_attach(
                  &single_gap_navigation_transport, shared)) ||
       (0U == perception_intercore_cm7_1_attach(
                  &single_gap_pose_transport, shared)) ||
       (0U == dl1b_safety_init(0U)))
    {
        return 1U;
    }
    camera_frame_consumer_set_handler(single_gap_app_on_frame);
    single_gap_initialized = 1U;
#endif
    return 0U;
}

void single_gap_app_on_frame(const camera_vision_frame_view_struct *frame)
{
#if (SINGLE_GAP_ENABLE == 1U)
    single_gap_observation_struct observation;
    uint32 local_capture_ms;
    uint32 local_now_ms;

    if((0U == single_gap_initialized) || (NULL == frame) ||
       (NULL == frame->pixels))
    {
        return;
    }
    local_now_ms = camera_frame_consumer_now_ms();
    local_capture_ms = (local_now_ms >= frame->frame_age_ms) ?
                       (local_now_ms - frame->frame_age_ms) : 0U;
    if(0U == single_gap_detector_process(
                 (const uint8 *)(uintptr_t)frame->pixels,
                 frame->width,
                 frame->height,
                 frame->stride,
                 frame->sequence,
                 local_capture_ms,
                 &observation))
    {
        return;
    }
    single_gap_observation = observation;
    single_gap_has_observation = 1U;
#else
    (void)frame;
#endif
}

void single_gap_app_service(uint32 now_ms)
{
#if (SINGLE_GAP_ENABLE == 1U)
    navigation_command_struct command;
    single_gap_output_struct output;
    single_gap_tof_snapshot_struct tof;
    float circumference_m;
    float forward_rpm;
    uint8 odometry_valid;
    uint8 pose_updated;
    uint8 stop_reason;

    if(0U == single_gap_initialized)
    {
        return;
    }
    dl1b_safety_update(now_ms);
    tof = dl1b_safety_get_snapshot();
    pose_updated = perception_intercore_read_pose(&single_gap_pose_transport,
                                                   &single_gap_pose);
    if(0U != pose_updated)
    {
        single_gap_pose_received_ms = now_ms;
    }
    if(SINGLE_GAP_CONTROL_PERIOD_MS >
       (now_ms - single_gap_last_service_ms))
    {
        return;
    }
    single_gap_last_service_ms = now_ms;
    circumference_m = (float)SINGLE_GAP_WHEEL_CIRCUMFERENCE_MM / 1000.0f;
    forward_rpm = single_gap_speed_mps_to_rpm(
                      SINGLE_GAP_FORWARD_SPEED_MPS,
                      circumference_m);
    odometry_valid = ((0U != single_gap_pose.sequence) &&
                      ((single_gap_pose.validity_flags &
                        PERCEPTION_POSE_VALID_ODOMETRY) != 0U) &&
                      (SINGLE_GAP_SENSOR_STALE_MS >=
                       (now_ms - single_gap_pose_received_ms))) ? 1U : 0U;
    single_gap_controller_update(
        &single_gap_controller,
        (0U != single_gap_has_observation) ? &single_gap_observation : NULL,
        &tof,
        single_gap_pose.position_x_m,
        odometry_valid,
        forward_rpm,
        now_ms,
        &output);

    stop_reason = single_gap_diagnostic_stop_reason(
                      &tof,
                      forward_rpm,
                      now_ms,
                      single_gap_map_stop_reason(output.stop_reason));
    memset(&command, 0, sizeof(command));
    command.forward_rpm = (0U != output.enable) ? output.forward_rpm : 0.0f;
    command.turn_rate_dps = (0U != output.enable) ? output.turn_rate_dps : 0.0f;
    command.confidence = (0U != output.enable) ? 1.0f : 0.0f;
    command.source_sequence = single_gap_next_sequence();
    command.valid_for_ms = SINGLE_GAP_NAV_VALID_MS;
    command.enable = (uint8)((0U != output.enable) &&
                             (SINGLE_GAP_MOTION_ENABLE != 0U) &&
                             ((uint8)NAVIGATION_STOP_NONE == stop_reason));
    if(0U == command.enable)
    {
        command.forward_rpm = 0.0f;
        command.turn_rate_dps = 0.0f;
        command.confidence = 0.0f;
    }
    command.source = (uint8)NAVIGATION_SOURCE_VISION;
    command.mode = (uint8)NAVIGATION_MODE_VISION_ASSIST;
    command.stop_reason = stop_reason;
    command.reserved[0] = 0U;
    command.reserved[1] = 0U;
    (void)intercore_transport_publish_navigation(
              &single_gap_navigation_transport, &command, now_ms);
#else
    (void)now_ms;
#endif
}

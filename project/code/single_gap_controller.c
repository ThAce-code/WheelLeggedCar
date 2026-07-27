/*********************************************************************************************************************
* File: single_gap_controller.c
* Description: Fail-closed image-space controller for one cone gap.
*********************************************************************************************************************/

#include "single_gap_controller.h"
#include "single_gap_config.h"

#include <string.h>

#define SINGLE_GAP_IMAGE_CENTER_X       (93.5f)
#define SINGLE_GAP_HALF_IMAGE_WIDTH     (94.0f)
#define SINGLE_GAP_KP_DPS               (15.0f)
#define SINGLE_GAP_ODOMETRY_EPSILON_M   (0.001f)

static uint8 single_gap_float_is_finite(float value)
{
    return ((value == value) &&
            (3.402823466e+38F >= value) &&
            (-3.402823466e+38F <= value)) ? 1U : 0U;
}

static float single_gap_absf(float value)
{
    return (0.0f > value) ? -value : value;
}

static float single_gap_limit(float value, float limit)
{
    if(limit < value)
    {
        return limit;
    }
    if(-limit > value)
    {
        return -limit;
    }
    return value;
}

static uint8 single_gap_popcount7(uint32 bits)
{
    uint8 count = 0U;
    uint8 index;

    for(index = 0U; index < SINGLE_GAP_ACQUIRE_WINDOW_FRAMES; index++)
    {
        count += (uint8)((bits >> index) & 1U);
    }
    return count;
}

static uint16 single_gap_median_center(const single_gap_controller_struct *controller)
{
    uint16 first;
    uint16 second;
    uint16 third;
    uint16 temporary;

    if(0U == controller->gap_history_count)
    {
        return (uint16)SINGLE_GAP_IMAGE_CENTER_X;
    }
    if(1U == controller->gap_history_count)
    {
        return controller->gap_center_history[0];
    }
    if(2U == controller->gap_history_count)
    {
        return (uint16)((controller->gap_center_history[0] +
                         controller->gap_center_history[1]) / 2U);
    }

    first = controller->gap_center_history[0];
    second = controller->gap_center_history[1];
    third = controller->gap_center_history[2];
    if(first > second)
    {
        temporary = first;
        first = second;
        second = temporary;
    }
    if(second > third)
    {
        temporary = second;
        second = third;
        third = temporary;
    }
    if(first > second)
    {
        second = first;
    }
    return second;
}

static void single_gap_push_center(single_gap_controller_struct *controller,
                                   uint16 center_x)
{
    controller->gap_center_history[controller->gap_history_index] = center_x;
    controller->gap_history_index++;
    if(3U <= controller->gap_history_index)
    {
        controller->gap_history_index = 0U;
    }
    if(3U > controller->gap_history_count)
    {
        controller->gap_history_count++;
    }
}

static void single_gap_write_output(const single_gap_controller_struct *controller,
                                    single_gap_output_struct *output)
{
    memset(output, 0, sizeof(*output));
    output->state = controller->state;
    output->stop_reason = controller->stop_reason;
    output->gap_center_x = single_gap_median_center(controller);
}

static void single_gap_latch_stop(single_gap_controller_struct *controller,
                                  single_gap_stop_reason_enum reason,
                                  single_gap_output_struct *output)
{
    controller->state = SINGLE_GAP_STATE_FAULT_STOP;
    controller->stop_reason = reason;
    single_gap_write_output(controller, output);
}

static float single_gap_compute_turn(single_gap_controller_struct *controller,
                                     uint16 center_x)
{
    float desired_turn;
    float error_px = (float)center_x - SINGLE_GAP_IMAGE_CENTER_X;
    float delta;

    if((float)SINGLE_GAP_DEADBAND_PX >= single_gap_absf(error_px))
    {
        desired_turn = 0.0f;
    }
    else
    {
        controller->previous_error = error_px / SINGLE_GAP_HALF_IMAGE_WIDTH;
        desired_turn = SINGLE_GAP_KP_DPS * controller->previous_error;
    }
    desired_turn = single_gap_limit(desired_turn, SINGLE_GAP_TURN_LIMIT_DPS);
    delta = desired_turn - controller->previous_turn_dps;
    delta = single_gap_limit(delta, SINGLE_GAP_TURN_STEP_DPS);
    controller->previous_turn_dps += delta;
    return controller->previous_turn_dps;
}

void single_gap_controller_init(single_gap_controller_struct *controller)
{
    if(NULL == controller)
    {
        return;
    }
    memset(controller, 0, sizeof(*controller));
    controller->state = SINGLE_GAP_STATE_DISABLED;
    controller->stop_reason = SINGLE_GAP_STOP_DISABLED;
}

void single_gap_controller_set_armed(single_gap_controller_struct *controller,
                                      uint8 armed,
                                      uint32 now_ms)
{
    if(NULL == controller)
    {
        return;
    }
    single_gap_controller_init(controller);
    if(0U != armed)
    {
        controller->armed = 1U;
        controller->state = SINGLE_GAP_STATE_ACQUIRE;
        controller->stop_reason = SINGLE_GAP_STOP_NONE;
        controller->state_enter_ms = now_ms;
    }
}

void single_gap_controller_update(single_gap_controller_struct *controller,
                                  const single_gap_observation_struct *observation,
                                  const single_gap_tof_snapshot_struct *tof,
                                  float odometry_m,
                                  uint8 odometry_valid,
                                  float forward_rpm,
                                  uint32 now_ms,
                                  single_gap_output_struct *output)
{
    uint16 filtered_center;
    uint32 pass_elapsed_ms;
    float pass_distance_m;
    float decay;
    uint8 new_observation;

    if((NULL == controller) || (NULL == output))
    {
        return;
    }
    single_gap_write_output(controller, output);
    if(0U == controller->armed)
    {
        controller->state = SINGLE_GAP_STATE_DISABLED;
        controller->stop_reason = SINGLE_GAP_STOP_DISABLED;
        single_gap_write_output(controller, output);
        return;
    }
    if((SINGLE_GAP_STATE_FAULT_STOP == controller->state) ||
       (SINGLE_GAP_STATE_PASSED == controller->state))
    {
        return;
    }
    if((NULL == tof) || (0U == tof->initialized) || (0U == tof->valid))
    {
        single_gap_latch_stop(controller, SINGLE_GAP_STOP_TOF_INVALID, output);
        return;
    }
    if(SINGLE_GAP_SENSOR_STALE_MS < (now_ms - tof->sample_ms))
    {
        single_gap_latch_stop(controller, SINGLE_GAP_STOP_TOF_STALE, output);
        return;
    }
    if(SINGLE_GAP_TOF_STOP_MM >= tof->distance_mm)
    {
        single_gap_latch_stop(controller, SINGLE_GAP_STOP_TOF_NEAR, output);
        return;
    }
    if((0U == odometry_valid) || (0U == single_gap_float_is_finite(odometry_m)) ||
       (0U == single_gap_float_is_finite(forward_rpm)))
    {
        single_gap_latch_stop(controller, SINGLE_GAP_STOP_ODOMETRY, output);
        return;
    }
    if(NULL == observation)
    {
        single_gap_latch_stop(controller, SINGLE_GAP_STOP_FRAME_STALE, output);
        return;
    }
    if(SINGLE_GAP_SENSOR_STALE_MS < (now_ms - observation->capture_ms))
    {
        single_gap_latch_stop(controller, SINGLE_GAP_STOP_FRAME_STALE, output);
        return;
    }

    new_observation = (observation->sequence != controller->last_observation_sequence) ? 1U : 0U;
    if(0U != new_observation)
    {
        controller->last_observation_sequence = observation->sequence;
        controller->last_frame_ms = observation->capture_ms;
        if((0U != observation->ambiguous) || (2U < observation->accepted_count))
        {
            single_gap_latch_stop(controller, SINGLE_GAP_STOP_TARGET_AMBIGUOUS, output);
            return;
        }
        if((2U == observation->accepted_count) &&
           (SINGLE_GAP_MIN_WIDTH_PX > observation->gap_width_px))
        {
            single_gap_latch_stop(controller, SINGLE_GAP_STOP_GAP_NARROW, output);
            return;
        }

        if(SINGLE_GAP_STATE_ACQUIRE == controller->state)
        {
            controller->acquire_bits =
                ((controller->acquire_bits << 1U) |
                 ((0U != observation->valid) ? 1U : 0U)) & 0x7FU;
            controller->acquire_count = single_gap_popcount7(controller->acquire_bits);
        }

        if(0U != observation->valid)
        {
            controller->lost_count = 0U;
            single_gap_push_center(controller, observation->gap_center_x);
        }
        else if(SINGLE_GAP_STATE_PASS_CANDIDATE != controller->state)
        {
            controller->lost_count++;
            if(SINGLE_GAP_LOST_FRAMES <= controller->lost_count)
            {
                single_gap_latch_stop(controller, SINGLE_GAP_STOP_TARGET_LOST, output);
                return;
            }
        }
    }

    if((SINGLE_GAP_STATE_ACQUIRE == controller->state) &&
       (SINGLE_GAP_ACQUIRE_HITS <= controller->acquire_count))
    {
        controller->state = SINGLE_GAP_STATE_APPROACH;
        controller->state_enter_ms = now_ms;
    }

    filtered_center = single_gap_median_center(controller);
    output->gap_center_x = filtered_center;
    output->gap_width_px = observation->gap_width_px;

    if(SINGLE_GAP_STATE_APPROACH == controller->state)
    {
        if((0U != new_observation) && (0U != observation->valid) &&
           (SINGLE_GAP_BOTTOM_ENTER_PX <= observation->left_bottom_y) &&
           (SINGLE_GAP_BOTTOM_ENTER_PX <= observation->right_bottom_y))
        {
            controller->bottom_count++;
        }
        else if(0U != new_observation)
        {
            controller->bottom_count = 0U;
        }
        output->enable = 1U;
        output->forward_rpm = forward_rpm;
        output->turn_rate_dps = single_gap_compute_turn(controller, filtered_center);
        if(SINGLE_GAP_BOTTOM_CONFIRM_FRAMES <= controller->bottom_count)
        {
            controller->state = SINGLE_GAP_STATE_PASS_CANDIDATE;
            controller->state_enter_ms = now_ms;
            controller->pass_start_odometry_m = odometry_m;
            output->state = controller->state;
        }
        return;
    }

    if(SINGLE_GAP_STATE_PASS_CANDIDATE == controller->state)
    {
        pass_elapsed_ms = now_ms - controller->state_enter_ms;
        pass_distance_m = odometry_m - controller->pass_start_odometry_m;
        if((-SINGLE_GAP_ODOMETRY_EPSILON_M) > pass_distance_m)
        {
            single_gap_latch_stop(controller, SINGLE_GAP_STOP_ODOMETRY, output);
            return;
        }
        if(SINGLE_GAP_PASS_TIMEOUT_MS < pass_elapsed_ms)
        {
            single_gap_latch_stop(controller, SINGLE_GAP_STOP_PASS_TIMEOUT, output);
            return;
        }
        if((0U != new_observation) && (0U != observation->valid))
        {
            if((SINGLE_GAP_BOTTOM_ENTER_PX > observation->left_bottom_y) ||
               (SINGLE_GAP_BOTTOM_ENTER_PX > observation->right_bottom_y))
            {
                single_gap_latch_stop(controller, SINGLE_GAP_STOP_TARGET_LOST, output);
                return;
            }
            controller->bottom_count = SINGLE_GAP_BOTTOM_CONFIRM_FRAMES;
        }
        else if((0U != new_observation) && (0U == observation->valid))
        {
            controller->bottom_count = 0U;
        }
        if((0U == controller->bottom_count) &&
           (SINGLE_GAP_PASS_DISTANCE_M <= pass_distance_m))
        {
            controller->state = SINGLE_GAP_STATE_PASSED;
            controller->stop_reason = SINGLE_GAP_STOP_PASSED;
            single_gap_write_output(controller, output);
            return;
        }

        decay = (SINGLE_GAP_TURN_DECAY_MS >= pass_elapsed_ms) ?
                ((float)(SINGLE_GAP_TURN_DECAY_MS - pass_elapsed_ms) /
                 (float)SINGLE_GAP_TURN_DECAY_MS) : 0.0f;
        output->state = controller->state;
        output->enable = 1U;
        output->forward_rpm = forward_rpm;
        output->turn_rate_dps = controller->previous_turn_dps * decay;
        return;
    }

    single_gap_write_output(controller, output);
}

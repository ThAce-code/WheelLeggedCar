#include <stdio.h>
#include <string.h>

#include "single_gap_controller.h"

static uint32 failure_count;

#define TEST_CHECK(condition)                                                        \
    do                                                                               \
    {                                                                                \
        if(!(condition))                                                             \
        {                                                                            \
            failure_count++;                                                         \
            printf("FAIL:%u: %s\n", (unsigned)__LINE__, #condition);               \
        }                                                                            \
    } while(0)

static single_gap_observation_struct make_observation(uint32 sequence,
                                                       uint32 capture_ms,
                                                       uint8 valid,
                                                       uint16 center_x,
                                                       uint16 width_px,
                                                       uint16 bottom_y)
{
    single_gap_observation_struct observation;

    memset(&observation, 0, sizeof(observation));
    observation.sequence = sequence;
    observation.capture_ms = capture_ms;
    observation.accepted_count = (0U != valid) ? 2U : 0U;
    observation.gap_center_x = center_x;
    observation.gap_width_px = width_px;
    observation.left_bottom_y = bottom_y;
    observation.right_bottom_y = bottom_y;
    observation.valid = valid;
    return observation;
}

static single_gap_tof_snapshot_struct clear_tof(uint32 now_ms)
{
    single_gap_tof_snapshot_struct tof;

    tof.sample_ms = now_ms;
    tof.distance_mm = 1000U;
    tof.initialized = 1U;
    tof.valid = 1U;
    return tof;
}

static void acquire_target(single_gap_controller_struct *controller,
                           single_gap_output_struct *output)
{
    static const uint8 pattern[7] = {1U, 1U, 1U, 0U, 1U, 1U, 0U};
    single_gap_observation_struct observation;
    single_gap_tof_snapshot_struct tof;
    uint32 index;
    uint32 now_ms;

    single_gap_controller_init(controller);
    single_gap_controller_set_armed(controller, 1U, 0U);
    for(index = 0U; index < 7U; index++)
    {
        now_ms = index * 40U;
        observation = make_observation(index + 1U, now_ms, pattern[index],
                                       94U, 58U, 80U);
        tof = clear_tof(now_ms);
        single_gap_controller_update(controller, &observation, &tof,
                                     0.0f, 1U, 50.0f, now_ms, output);
    }
}

static void test_five_of_seven_acquires_and_duplicate_does_not(void)
{
    single_gap_controller_struct controller;
    single_gap_output_struct output;
    single_gap_observation_struct observation;
    single_gap_tof_snapshot_struct tof = clear_tof(0U);
    uint32 index;

    single_gap_controller_init(&controller);
    single_gap_controller_set_armed(&controller, 1U, 0U);
    observation = make_observation(1U, 0U, 1U, 94U, 58U, 80U);
    for(index = 0U; index < 5U; index++)
    {
        single_gap_controller_update(&controller, &observation, &tof,
                                     0.0f, 1U, 50.0f, 0U, &output);
    }
    TEST_CHECK(SINGLE_GAP_STATE_ACQUIRE == output.state);
    TEST_CHECK(1U == controller.acquire_count);

    acquire_target(&controller, &output);
    TEST_CHECK(SINGLE_GAP_STATE_APPROACH == output.state);
    TEST_CHECK(1U == output.enable);
    TEST_CHECK(50.0f == output.forward_rpm);
}

static void test_pd_sign_deadband_median_and_slew_limit(void)
{
    single_gap_controller_struct controller;
    single_gap_output_struct output;
    single_gap_observation_struct observation;
    single_gap_tof_snapshot_struct tof;

    acquire_target(&controller, &output);
    observation = make_observation(8U, 280U, 1U, 96U, 58U, 80U);
    tof = clear_tof(280U);
    single_gap_controller_update(&controller, &observation, &tof,
                                 0.0f, 1U, 50.0f, 280U, &output);
    TEST_CHECK(0.0f == output.turn_rate_dps);

    observation = make_observation(9U, 320U, 1U, 188U, 58U, 80U);
    tof = clear_tof(320U);
    single_gap_controller_update(&controller, &observation, &tof,
                                 0.0f, 1U, 50.0f, 320U, &output);
    TEST_CHECK(0.0f == output.turn_rate_dps);

    observation = make_observation(10U, 360U, 1U, 188U, 58U, 80U);
    tof = clear_tof(360U);
    single_gap_controller_update(&controller, &observation, &tof,
                                 0.0f, 1U, 50.0f, 360U, &output);
    TEST_CHECK(output.turn_rate_dps > 0.0f);
    TEST_CHECK(output.turn_rate_dps <= 5.0f);

    single_gap_controller_init(&controller);
    single_gap_controller_set_armed(&controller, 1U, 0U);
    observation = make_observation(1U, 0U, 1U, 90U, 58U, 80U);
    tof = clear_tof(0U);
    single_gap_controller_update(&controller, &observation, &tof,
                                 0.0f, 1U, 50.0f, 0U, &output);
    observation = make_observation(2U, 40U, 1U, 150U, 58U, 80U);
    tof = clear_tof(40U);
    single_gap_controller_update(&controller, &observation, &tof,
                                 0.0f, 1U, 50.0f, 40U, &output);
    observation = make_observation(3U, 80U, 1U, 92U, 58U, 80U);
    tof = clear_tof(80U);
    single_gap_controller_update(&controller, &observation, &tof,
                                 0.0f, 1U, 50.0f, 80U, &output);
    TEST_CHECK(92U == output.gap_center_x);
}

static void test_safety_faults_latch(void)
{
    single_gap_controller_struct controller;
    single_gap_output_struct output;
    single_gap_observation_struct observation;
    single_gap_tof_snapshot_struct tof;

    acquire_target(&controller, &output);
    observation = make_observation(8U, 280U, 1U, 94U, 58U, 80U);
    tof = clear_tof(280U);
    tof.distance_mm = 350U;
    single_gap_controller_update(&controller, &observation, &tof,
                                 0.0f, 1U, 50.0f, 280U, &output);
    TEST_CHECK(SINGLE_GAP_STATE_FAULT_STOP == output.state);
    TEST_CHECK(SINGLE_GAP_STOP_TOF_NEAR == output.stop_reason);
    TEST_CHECK(0U == output.enable);

    tof = clear_tof(320U);
    single_gap_controller_update(&controller, &observation, &tof,
                                 0.0f, 1U, 50.0f, 320U, &output);
    TEST_CHECK(SINGLE_GAP_STOP_TOF_NEAR == output.stop_reason);

    acquire_target(&controller, &output);
    observation = make_observation(8U, 280U, 1U, 94U, 58U, 80U);
    tof = clear_tof(280U);
    observation.ambiguous = 1U;
    observation.accepted_count = 3U;
    single_gap_controller_update(&controller, &observation, &tof,
                                 0.0f, 1U, 50.0f, 280U, &output);
    TEST_CHECK(SINGLE_GAP_STOP_TARGET_AMBIGUOUS == output.stop_reason);

    acquire_target(&controller, &output);
    observation = make_observation(8U, 280U, 1U, 94U, 58U, 80U);
    tof = clear_tof(280U);
    single_gap_controller_update(&controller, &observation, &tof,
                                 0.0f, 0U, 50.0f, 280U, &output);
    TEST_CHECK(SINGLE_GAP_STOP_ODOMETRY == output.stop_reason);
}

static void test_bottom_exit_requires_distance(void)
{
    single_gap_controller_struct controller;
    single_gap_output_struct output;
    single_gap_observation_struct observation;
    single_gap_tof_snapshot_struct tof;

    acquire_target(&controller, &output);
    observation = make_observation(8U, 280U, 1U, 110U, 58U, 100U);
    tof = clear_tof(280U);
    single_gap_controller_update(&controller, &observation, &tof,
                                 1.00f, 1U, 50.0f, 280U, &output);
    observation = make_observation(9U, 320U, 1U, 110U, 58U, 100U);
    tof = clear_tof(320U);
    single_gap_controller_update(&controller, &observation, &tof,
                                 1.02f, 1U, 50.0f, 320U, &output);
    TEST_CHECK(SINGLE_GAP_STATE_PASS_CANDIDATE == output.state);

    observation = make_observation(10U, 360U, 0U, 0U, 0U, 0U);
    tof = clear_tof(360U);
    single_gap_controller_update(&controller, &observation, &tof,
                                 1.10f, 1U, 50.0f, 360U, &output);
    TEST_CHECK(SINGLE_GAP_STATE_PASS_CANDIDATE == output.state);
    TEST_CHECK(1U == output.enable);

    observation = make_observation(11U, 400U, 0U, 0U, 0U, 0U);
    tof = clear_tof(400U);
    single_gap_controller_update(&controller, &observation, &tof,
                                 1.23f, 1U, 50.0f, 400U, &output);
    TEST_CHECK(SINGLE_GAP_STATE_PASSED == output.state);
    TEST_CHECK(SINGLE_GAP_STOP_PASSED == output.stop_reason);
    TEST_CHECK(0U == output.enable);
}

static void enter_pass_candidate(single_gap_controller_struct *controller,
                                 single_gap_output_struct *output)
{
    single_gap_observation_struct observation;
    single_gap_tof_snapshot_struct tof;

    acquire_target(controller, output);
    observation = make_observation(8U, 280U, 1U, 110U, 58U, 100U);
    tof = clear_tof(280U);
    single_gap_controller_update(controller, &observation, &tof,
                                 1.00f, 1U, 50.0f, 280U, output);
    observation = make_observation(9U, 320U, 1U, 110U, 58U, 100U);
    tof = clear_tof(320U);
    single_gap_controller_update(controller, &observation, &tof,
                                 1.02f, 1U, 50.0f, 320U, output);
    TEST_CHECK(SINGLE_GAP_STATE_PASS_CANDIDATE == output->state);
}

static void test_stale_lost_narrow_and_pass_faults(void)
{
    single_gap_controller_struct controller;
    single_gap_output_struct output;
    single_gap_observation_struct observation;
    single_gap_tof_snapshot_struct tof;
    uint32 index;
    uint32 now_ms;

    acquire_target(&controller, &output);
    observation = make_observation(8U, 280U, 1U, 94U, 58U, 80U);
    tof = clear_tof(179U);
    single_gap_controller_update(&controller, &observation, &tof,
                                 0.0f, 1U, 50.0f, 280U, &output);
    TEST_CHECK(SINGLE_GAP_STOP_TOF_STALE == output.stop_reason);

    acquire_target(&controller, &output);
    observation = make_observation(8U, 179U, 1U, 94U, 58U, 80U);
    tof = clear_tof(280U);
    single_gap_controller_update(&controller, &observation, &tof,
                                 0.0f, 1U, 50.0f, 280U, &output);
    TEST_CHECK(SINGLE_GAP_STOP_FRAME_STALE == output.stop_reason);

    acquire_target(&controller, &output);
    for(index = 0U; index < 5U; index++)
    {
        now_ms = 280U + index * 40U;
        observation = make_observation(8U + index, now_ms, 0U, 0U, 0U, 0U);
        tof = clear_tof(now_ms);
        single_gap_controller_update(&controller, &observation, &tof,
                                     0.0f, 1U, 50.0f, now_ms, &output);
    }
    TEST_CHECK(SINGLE_GAP_STOP_TARGET_LOST == output.stop_reason);

    acquire_target(&controller, &output);
    observation = make_observation(8U, 280U, 1U, 94U, 23U, 80U);
    tof = clear_tof(280U);
    single_gap_controller_update(&controller, &observation, &tof,
                                 0.0f, 1U, 50.0f, 280U, &output);
    TEST_CHECK(SINGLE_GAP_STOP_GAP_NARROW == output.stop_reason);

    enter_pass_candidate(&controller, &output);
    observation = make_observation(10U, 1821U, 0U, 0U, 0U, 0U);
    tof = clear_tof(1821U);
    single_gap_controller_update(&controller, &observation, &tof,
                                 1.10f, 1U, 50.0f, 1821U, &output);
    TEST_CHECK(SINGLE_GAP_STOP_PASS_TIMEOUT == output.stop_reason);

    enter_pass_candidate(&controller, &output);
    observation = make_observation(10U, 360U, 0U, 0U, 0U, 0U);
    tof = clear_tof(360U);
    single_gap_controller_update(&controller, &observation, &tof,
                                 1.00f, 1U, 50.0f, 360U, &output);
    TEST_CHECK(SINGLE_GAP_STOP_ODOMETRY == output.stop_reason);
}

int main(void)
{
    test_five_of_seven_acquires_and_duplicate_does_not();
    test_pd_sign_deadband_median_and_slew_limit();
    test_safety_faults_latch();
    test_bottom_exit_requires_distance();
    test_stale_lost_narrow_and_pass_faults();

    if(0U != failure_count)
    {
        printf("single_gap_controller_test: %u failure(s)\n", (unsigned)failure_count);
        return 1;
    }
    puts("single_gap_controller_test: PASS");
    return 0;
}

#include <stdio.h>
#include <string.h>

#include "actuator_motor.h"
#include "control_balance.h"
#include "intercore_memory.h"
#include "perception_intercore.h"
#include "single_gap_pose_source.h"

static uint32 failure_count;
static uint32 publish_count;
static balance_diag_struct mock_balance;
static wheel_feedback_struct mock_wheel;
static perception_pose_snapshot_struct published_snapshot;
static volatile intercore_shared_layout_struct mock_shared;

#define TEST_CHECK(condition)                                                        \
    do                                                                               \
    {                                                                                \
        if(!(condition))                                                             \
        {                                                                            \
            failure_count++;                                                         \
            printf("FAIL:%u: %s\n", (unsigned)__LINE__, #condition);               \
        }                                                                            \
    } while(0)

static uint8 float_near(float expected, float actual, float tolerance)
{
    float error = expected - actual;
    if(0.0f > error)
    {
        error = -error;
    }
    return (tolerance >= error) ? 1U : 0U;
}

volatile intercore_shared_layout_struct *intercore_memory_get_layout(void)
{
    return &mock_shared;
}

uint8 perception_intercore_cm7_0_init(perception_intercore_transport_struct *transport,
                                      volatile intercore_shared_layout_struct *shared)
{
    (void)transport;
    (void)shared;
    return 1U;
}

uint8 perception_intercore_publish_pose(perception_intercore_transport_struct *transport,
                                        const perception_pose_snapshot_struct *snapshot)
{
    (void)transport;
    published_snapshot = *snapshot;
    publish_count++;
    return 1U;
}

const balance_diag_struct *control_balance_get_diag(void)
{
    return &mock_balance;
}

const wheel_feedback_struct *actuator_motor_get_feedback(void)
{
    return &mock_wheel;
}

static void test_distance_integration(void)
{
    union
    {
        uint32 bits;
        float value;
    } not_a_number;

    TEST_CHECK(float_near(0.20f,
        single_gap_pose_integrate_distance(0.0f, 60.0f, 0.20f, 1000U),
        0.0001f));
    TEST_CHECK(float_near(-0.20f,
        single_gap_pose_integrate_distance(0.0f, -60.0f, 0.20f, 1000U),
        0.0001f));
    TEST_CHECK(1.25f ==
        single_gap_pose_integrate_distance(1.25f, 60.0f, 0.0f, 1000U));
    not_a_number.bits = 0x7FC00000UL;
    TEST_CHECK(1.25f ==
        single_gap_pose_integrate_distance(1.25f, not_a_number.value, 0.20f, 1000U));
}

static void test_unmeasured_circumference_publishes_invalid_odometry(void)
{
    memset((void *)&mock_shared, 0, sizeof(mock_shared));
    memset(&mock_balance, 0, sizeof(mock_balance));
    memset(&mock_wheel, 0, sizeof(mock_wheel));
    memset(&published_snapshot, 0, sizeof(published_snapshot));
    publish_count = 0U;
    mock_balance.wheel_speed_rpm = 60.0f;
    mock_wheel.online = 1U;
    mock_wheel.left_online = 1U;
    mock_wheel.right_online = 1U;
    mock_wheel.age_ms = 0U;

    TEST_CHECK(1U == single_gap_pose_source_init());
    single_gap_pose_source_update(49U);
    TEST_CHECK(0U == publish_count);
    single_gap_pose_source_update(50U);
    TEST_CHECK(1U == publish_count);
    TEST_CHECK(1U == published_snapshot.sequence);
    TEST_CHECK(50000U == published_snapshot.timestamp_us);
    TEST_CHECK(0U == (published_snapshot.validity_flags &
                      PERCEPTION_POSE_VALID_ODOMETRY));
    TEST_CHECK(0.0f == published_snapshot.position_x_m);
}

int main(void)
{
    test_distance_integration();
    test_unmeasured_circumference_publishes_invalid_odometry();

    if(0U != failure_count)
    {
        printf("single_gap_pose_source_test: %u failure(s)\n", (unsigned)failure_count);
        return 1;
    }
    puts("single_gap_pose_source_test: PASS");
    return 0;
}

#include <stdio.h>
#include <string.h>

#include "perception_intercore.h"

static uint32 test_failure_count = 0U;
static _Alignas(32) intercore_shared_layout_struct shared;
static perception_intercore_transport_struct cm7_0_transport;
static perception_intercore_transport_struct cm7_1_transport;

#define TEST_CHECK(condition)                                                   \
    do                                                                          \
    {                                                                           \
        if(!(condition))                                                        \
        {                                                                       \
            printf("FAIL:%s:%d: %s\n", __FILE__, __LINE__, #condition);       \
            test_failure_count++;                                               \
        }                                                                       \
    }while(0)

static void fixture_init(void)
{
    memset(&shared, 0, sizeof(shared));
    shared.metadata.magic = INTERCORE_PROTOCOL_MAGIC;
    shared.metadata.version = INTERCORE_PROTOCOL_VERSION;
    shared.metadata.layout_size = (uint16)sizeof(shared);
    shared.metadata.boot_epoch = 1U;
    shared.metadata.cm7_0_ready = 1U;

    TEST_CHECK(1U == perception_intercore_cm7_0_init(&cm7_0_transport, &shared));
    TEST_CHECK(1U == perception_intercore_cm7_1_attach(&cm7_1_transport, &shared));
}

static void test_pose_round_trip(void)
{
    perception_pose_snapshot_struct published = {0};
    perception_pose_snapshot_struct received = {0};

    fixture_init();
    published.sequence = 1U;
    published.timestamp_us = 12000U;
    published.position_x_m = 1.25F;
    published.camera_height_m = 0.18F;
    published.validity_flags = PERCEPTION_POSE_VALID_IMU;

    TEST_CHECK(1U == perception_intercore_publish_pose(&cm7_0_transport, &published));
    TEST_CHECK(1U == perception_intercore_read_pose(&cm7_1_transport, &received));
    TEST_CHECK(1U == received.sequence);
    TEST_CHECK(12000U == received.timestamp_us);
    TEST_CHECK(1.25F == received.position_x_m);
    TEST_CHECK(0.18F == received.camera_height_m);
}

static void test_perception_round_trip_and_crc_rejection(void)
{
    perception_snapshot_struct published = {0};
    perception_snapshot_struct received = {0};

    fixture_init();
    published.sequence = 7U;
    published.timestamp_us = 25000U;
    published.state = (uint16)PERCEPTION_STATE_RETURN_GAPS;
    published.validity_flags = PERCEPTION_TARGET_VALID;
    published.current_target.gap_id = 3U;
    published.current_target.center_x_m = 2.50F;

    TEST_CHECK(1U == perception_intercore_publish_perception(&cm7_1_transport,
                                                              &published));
    TEST_CHECK(1U == perception_intercore_read_perception(&cm7_0_transport,
                                                           &received));
    TEST_CHECK(7U == received.sequence);
    TEST_CHECK(3U == received.current_target.gap_id);
    TEST_CHECK(2.50F == received.current_target.center_x_m);

    cm7_0_transport.last_perception_sequence = 0U;
    shared.perception.perception[shared.perception.perception_active_index].payload.current_target.gap_id++;
    TEST_CHECK(0U == perception_intercore_read_perception(&cm7_0_transport,
                                                           &received));
}

int main(void)
{
    TEST_CHECK(0xD00U == offsetof(intercore_shared_layout_struct, perception));
    TEST_CHECK(8192U == sizeof(intercore_shared_layout_struct));
    TEST_CHECK(256U == sizeof(intercore_camera_control_struct));
    test_pose_round_trip();
    test_perception_round_trip_and_crc_rejection();

    if(0U != test_failure_count)
    {
        printf("perception_intercore_test: %u failure(s)\n",
               (unsigned int)test_failure_count);
        return 1;
    }

    printf("perception_intercore_test: PASS\n");
    return 0;
}

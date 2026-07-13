#include <math.h>
#include <stdio.h>
#include <string.h>
#include "intercore_protocol.h"
#include "intercore_transport.h"
#include "intercore_notify.h"
#include "intercore_notify_port.h"

static uint32 test_failure_count = 0U;
static intercore_shared_layout_struct shared __attribute__((aligned(32)));
static uint8 mock_notify_send_result = 0U;
static uint32 mock_notify_send_count = 0U;

uint8 intercore_notify_port_init(intercore_role_enum role)
{
    (void)role;
    return 1U;
}

uint8 intercore_notify_port_send(const intercore_doorbell_struct *message)
{
    (void)message;
    mock_notify_send_count++;
    return mock_notify_send_result;
}

#define TEST_CHECK(condition)                                                   \
    do                                                                          \
    {                                                                           \
        if(!(condition))                                                        \
        {                                                                       \
            printf("FAIL:%s:%d: %s\n", __FILE__, __LINE__, #condition);         \
            test_failure_count++;                                               \
        }                                                                       \
    }while(0)

static void test_protocol_crc_and_sizes(void)
{
    static const uint8 sample[] = "123456789";

    TEST_CHECK(0xCBF43926UL == intercore_crc32(sample, 9U));
    TEST_CHECK(24U == sizeof(intercore_header_struct));
    TEST_CHECK(24U == sizeof(navigation_command_struct));
    TEST_CHECK(8192U == sizeof(intercore_shared_layout_struct));
}

static void test_navigation_structural_validation(void)
{
    navigation_command_struct command = {0};

    command.forward_rpm = 20.0f;
    command.turn_rate_dps = -10.0f;
    command.confidence = 0.75f;
    command.source_sequence = 1U;
    command.valid_for_ms = 200U;
    command.enable = 1U;
    command.source = NAVIGATION_SOURCE_VISION;
    command.mode = NAVIGATION_MODE_VISION_ASSIST;
    TEST_CHECK(1U == intercore_navigation_is_structurally_valid(&command));

    command.forward_rpm = NAN;
    TEST_CHECK(0U == intercore_navigation_is_structurally_valid(&command));
    command.forward_rpm = 20.0f;
    command.valid_for_ms = 201U;
    TEST_CHECK(0U == intercore_navigation_is_structurally_valid(&command));
    command.valid_for_ms = 200U;
    command.confidence = 1.1f;
    TEST_CHECK(0U == intercore_navigation_is_structurally_valid(&command));
}

static void test_record_rejects_corruption(void)
{
    intercore_header_struct header = {0};
    navigation_command_struct command = {0};

    command.forward_rpm = 15.0f;
    command.turn_rate_dps = 5.0f;
    command.confidence = 0.8f;
    command.source_sequence = 4U;
    command.valid_for_ms = 200U;
    command.enable = 1U;
    command.source = NAVIGATION_SOURCE_VISION;
    command.mode = NAVIGATION_MODE_VISION_ASSIST;

    intercore_record_prepare(&header,
                             INTERCORE_RECORD_NAVIGATION,
                             9U,
                             50U,
                             &command,
                             sizeof(command));
    TEST_CHECK(1U == intercore_record_validate(&header,
                                               INTERCORE_RECORD_NAVIGATION,
                                               &command,
                                               sizeof(command)));
    ((uint8 *)&command)[0] ^= 0x01U;
    TEST_CHECK(0U == intercore_record_validate(&header,
                                               INTERCORE_RECORD_NAVIGATION,
                                               &command,
                                               sizeof(command)));
}

static navigation_command_struct test_navigation_command(void)
{
    navigation_command_struct command = {0};

    command.forward_rpm = 20.0f;
    command.turn_rate_dps = -5.0f;
    command.confidence = 0.9f;
    command.source_sequence = 7U;
    command.valid_for_ms = 100U;
    command.enable = 1U;
    command.source = NAVIGATION_SOURCE_VISION;
    command.mode = NAVIGATION_MODE_VISION_ASSIST;
    return command;
}

static void test_transport_publish_and_consume(void)
{
    intercore_transport_struct receiver = {0};
    intercore_transport_struct sender = {0};
    navigation_command_struct sent = test_navigation_command();
    navigation_command_struct received = {0};
    uint32 record_sequence = 0U;

    memset(&shared, 0, sizeof(shared));
    TEST_CHECK(1U == intercore_transport_cm7_0_init(&receiver, &shared));
    TEST_CHECK(INTERCORE_PROTOCOL_MAGIC == shared.metadata.magic);
    TEST_CHECK(INTERCORE_PROTOCOL_VERSION == shared.metadata.version);
    TEST_CHECK(1U == shared.metadata.boot_epoch);
    TEST_CHECK(1U == intercore_transport_cm7_1_attach(&sender, &shared));
    TEST_CHECK(1U == shared.metadata.cm7_1_ready);
    TEST_CHECK(1U == intercore_transport_publish_navigation(&sender, &sent, 50U));
    TEST_CHECK(INTERCORE_TRANSPORT_OK ==
               intercore_transport_read_navigation(&receiver, &received, &record_sequence));
    TEST_CHECK(0 == memcmp(&sent, &received, sizeof(sent)));
    TEST_CHECK(1U == record_sequence);
    TEST_CHECK(INTERCORE_TRANSPORT_NO_DATA ==
               intercore_transport_read_navigation(&receiver, &received, &record_sequence));
}

static void test_transport_rejects_corruption_and_bad_metadata(void)
{
    intercore_transport_struct receiver = {0};
    intercore_transport_struct sender = {0};
    navigation_command_struct command = test_navigation_command();
    navigation_command_struct received = {0};
    uint32 record_sequence = 0U;
    uint32 active_index;

    memset(&shared, 0, sizeof(shared));
    TEST_CHECK(1U == intercore_transport_cm7_0_init(&receiver, &shared));
    TEST_CHECK(1U == intercore_transport_cm7_1_attach(&sender, &shared));
    TEST_CHECK(1U == intercore_transport_publish_navigation(&sender, &command, 50U));
    active_index = shared.metadata.navigation_active_index;
    shared.navigation[active_index].payload.forward_rpm += 1.0f;
    TEST_CHECK(INTERCORE_TRANSPORT_INVALID ==
               intercore_transport_read_navigation(&receiver, &received, &record_sequence));
    TEST_CHECK(1U == shared.health.crc_error_count);

    shared.metadata.navigation_active_index = 2U;
    TEST_CHECK(INTERCORE_TRANSPORT_INVALID ==
               intercore_transport_read_navigation(&receiver, &received, &record_sequence));
    TEST_CHECK(0U == shared.health.version_error_count);

    shared.metadata.navigation_active_index = active_index;
    shared.metadata.version = (uint16)(INTERCORE_PROTOCOL_VERSION + 1U);
    TEST_CHECK(INTERCORE_TRANSPORT_INVALID ==
               intercore_transport_read_navigation(&receiver, &received, &record_sequence));
    TEST_CHECK(1U == shared.health.version_error_count);
}

static void test_transport_rejects_slot_sequence_mismatch(void)
{
    intercore_transport_struct receiver = {0};
    intercore_transport_struct sender = {0};
    navigation_command_struct command = test_navigation_command();
    navigation_command_struct received = {0};
    uint32 record_sequence = 0U;
    uint32 active_index;

    memset(&shared, 0, sizeof(shared));
    TEST_CHECK(1U == intercore_transport_cm7_0_init(&receiver, &shared));
    TEST_CHECK(1U == intercore_transport_cm7_1_attach(&sender, &shared));
    TEST_CHECK(1U == intercore_transport_publish_navigation(&sender, &command, 50U));

    active_index = shared.metadata.navigation_active_index;
    shared.metadata.navigation_sequence =
        shared.navigation[active_index].header.sequence + 1U;
    TEST_CHECK(INTERCORE_TRANSPORT_INVALID ==
               intercore_transport_read_navigation(&receiver, &received, &record_sequence));
    TEST_CHECK(0U == shared.health.crc_error_count);
    TEST_CHECK(0U == shared.health.version_error_count);
    TEST_CHECK(0U == shared.health.cm7_0_consume_count);
}

static void test_transport_epoch_reinitialization(void)
{
    intercore_transport_struct receiver = {0};
    intercore_transport_struct sender = {0};
    navigation_command_struct command = test_navigation_command();
    navigation_command_struct received = {0};
    uint32 record_sequence = 0U;

    memset(&shared, 0, sizeof(shared));
    TEST_CHECK(1U == intercore_transport_cm7_0_init(&receiver, &shared));
    TEST_CHECK(1U == intercore_transport_cm7_1_attach(&sender, &shared));
    TEST_CHECK(1U == intercore_transport_cm7_0_init(&receiver, &shared));
    TEST_CHECK(2U == shared.metadata.boot_epoch);
    TEST_CHECK(0U == intercore_transport_publish_navigation(&sender, &command, 100U));
    TEST_CHECK(1U == intercore_transport_cm7_1_attach(&sender, &shared));
    TEST_CHECK(1U == intercore_transport_publish_navigation(&sender, &command, 100U));

    memset(&shared, 0, sizeof(shared));
    TEST_CHECK(1U == intercore_transport_cm7_0_init(&receiver, &shared));
    TEST_CHECK(1U == intercore_transport_cm7_1_attach(&sender, &shared));
    shared.metadata.boot_epoch = 2U;
    TEST_CHECK(INTERCORE_TRANSPORT_EPOCH_CHANGED ==
               intercore_transport_read_navigation(&receiver, &received, &record_sequence));
    TEST_CHECK(2U == receiver.boot_epoch);
}

static void test_nonblocking_notify_state(void)
{
    mock_notify_send_result = 1U;
    mock_notify_send_count = 0U;
    TEST_CHECK(1U == intercore_notify_init(INTERCORE_ROLE_CM7_1));
    TEST_CHECK(1U == intercore_notify_try(INTERCORE_NOTIFY_NAVIGATION));
    TEST_CHECK(1U == mock_notify_send_count);
    TEST_CHECK(0U == intercore_notify_try(INTERCORE_NOTIFY_HEARTBEAT));
    TEST_CHECK(1U == mock_notify_send_count);
    intercore_notify_release_callback();
    TEST_CHECK(1U == intercore_notify_try(INTERCORE_NOTIFY_HEARTBEAT));
    intercore_notify_release_callback();
    mock_notify_send_result = 0U;
    TEST_CHECK(0U == intercore_notify_try(INTERCORE_NOTIFY_CONTROL_STATUS));
    TEST_CHECK(0U == intercore_notify_get_diag()->in_flight);
    TEST_CHECK(2U == intercore_notify_get_diag()->busy_count);
    intercore_notify_receive_callback(INTERCORE_NOTIFY_NAVIGATION);
    TEST_CHECK(INTERCORE_NOTIFY_NAVIGATION == intercore_notify_take_pending());
    TEST_CHECK(0U == intercore_notify_take_pending());
}

int main(void)
{
    test_protocol_crc_and_sizes();
    test_navigation_structural_validation();
    test_record_rejects_corruption();
    test_transport_publish_and_consume();
    test_transport_rejects_corruption_and_bad_metadata();
    test_transport_rejects_slot_sequence_mismatch();
    test_transport_epoch_reinitialization();
    test_nonblocking_notify_state();

    if(0U != test_failure_count)
    {
        printf("intercore_control_foundation_test: %u failure(s)\n",
               (unsigned int)test_failure_count);
        return 1;
    }

    printf("intercore_control_foundation_test: PASS\n");
    return 0;
}

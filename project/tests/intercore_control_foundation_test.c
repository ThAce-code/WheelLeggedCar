#include <math.h>
#include <stdio.h>
#include <string.h>
#include "intercore_protocol.h"
#include "intercore_transport.h"
#include "intercore_notify.h"
#include "intercore_notify_port.h"
#include "motion_command_router.h"

static uint32 test_failure_count = 0U;
static intercore_shared_layout_struct shared __attribute__((aligned(32)));
static uint8 mock_notify_send_result = 0U;
static uint32 mock_notify_send_count = 0U;
static float mock_router_last_forward_rpm = 0.0f;
static float mock_router_last_turn_rate_dps = 0.0f;
static uint8 mock_router_last_enable = 0U;
static uint32 mock_router_last_now_ms = 0U;
static uint32 mock_router_apply_count = 0U;
static uint32 mock_router_stop_count = 0U;

void motion_command_router_port_apply(float forward_rpm,
                                      float turn_rate_dps,
                                      uint8 enable,
                                      uint32 now_ms)
{
    mock_router_last_forward_rpm = forward_rpm;
    mock_router_last_turn_rate_dps = turn_rate_dps;
    mock_router_last_enable = enable;
    mock_router_last_now_ms = now_ms;
    mock_router_apply_count++;
}

void motion_command_router_port_stop(uint32 now_ms)
{
    mock_router_last_now_ms = now_ms;
    mock_router_stop_count++;
}

uint8 intercore_notify_port_init(intercore_role_enum role)
{
    return ((INTERCORE_ROLE_CM7_0 == role) ||
            (INTERCORE_ROLE_CM7_1 == role)) ? 1U : 0U;
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
    TEST_CHECK(INTERCORE_NOTIFY_CLIENT_ID < 8U);
    TEST_CHECK(0x28081FF8UL == INTERCORE_NOTIFY_MESSAGE_ADDRESS);
    TEST_CHECK(INTERCORE_SHARED_BASE_ADDRESS <= INTERCORE_NOTIFY_MESSAGE_ADDRESS);
    TEST_CHECK((INTERCORE_NOTIFY_MESSAGE_ADDRESS + sizeof(intercore_doorbell_struct)) <=
               (INTERCORE_SHARED_BASE_ADDRESS + INTERCORE_SHARED_SIZE_BYTES));
    TEST_CHECK(0U == intercore_notify_init((intercore_role_enum)2));
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

static motion_command_request_struct test_motion_request(float forward_rpm,
                                                          uint8 source_sequence,
                                                          uint16 valid_for_ms)
{
    motion_command_request_struct request = {0};
    request.forward_rpm = forward_rpm;
    request.turn_rate_dps = forward_rpm / 10.0f;
    request.enable = 1U;
    request.received_ms = 100U;
    request.valid_for_ms = valid_for_ms;
    request.source_sequence = source_sequence;
    return request;
}

static void test_motion_router_priority_and_timeout(void)
{
    motion_command_request_struct autonomous = {10.0f, 1.0f, 1U, 100U, 200U, 1U};
    motion_command_request_struct wireless = {20.0f, 2.0f, 2U, 100U, 200U, 1U};
    motion_command_request_struct uart = {30.0f, 3.0f, 3U, 100U, 500U, 1U};

    motion_command_router_init();
    mock_router_apply_count = 0U;
    mock_router_stop_count = 0U;
    TEST_CHECK(1U == motion_command_router_arm_remote(100U, 0U));
    TEST_CHECK(1U == motion_command_router_submit(MOTION_SOURCE_AUTONOMOUS, &autonomous));
    TEST_CHECK(1U == motion_command_router_submit(MOTION_SOURCE_WIRELESS_MANUAL, &wireless));
    TEST_CHECK(1U == motion_command_router_submit(MOTION_SOURCE_UART_LOCAL, &uart));
    motion_command_router_update(101U, 0U);
    TEST_CHECK(30.0f == mock_router_last_forward_rpm);
    TEST_CHECK(MOTION_SOURCE_UART_LOCAL ==
               motion_command_router_get_diag()->active_source);
    motion_command_router_update(600U, 0U);
    TEST_CHECK(0U < mock_router_stop_count);
    TEST_CHECK(MOTION_SOURCE_NONE ==
               motion_command_router_get_diag()->active_source);
}

static void test_motion_router_rejects_invalid_and_disarmed_remote(void)
{
    motion_command_request_struct request = test_motion_request(12.0f, 1U, 100U);
    motion_command_router_init();
    TEST_CHECK(0U == motion_command_router_submit(MOTION_SOURCE_WIRELESS_MANUAL, &request));
    TEST_CHECK(0U == motion_command_router_submit(MOTION_SOURCE_AUTONOMOUS, NULL));
    request.forward_rpm = NAN;
    TEST_CHECK(0U == motion_command_router_submit(MOTION_SOURCE_UART_LOCAL, &request));
    request.forward_rpm = 12.0f;
    request.valid_for_ms = 0U;
    TEST_CHECK(0U == motion_command_router_submit(MOTION_SOURCE_UART_LOCAL, &request));
}

static void test_motion_router_maintenance_and_rearm(void)
{
    motion_command_request_struct request = test_motion_request(12.0f, 1U, 100U);
    motion_command_router_init();
    TEST_CHECK(1U == motion_command_router_arm_remote(100U, 0U));
    TEST_CHECK(1U == motion_command_router_submit(MOTION_SOURCE_WIRELESS_MANUAL, &request));
    TEST_CHECK(1U == motion_command_router_set_maintenance(1U, 101U));
    TEST_CHECK(0U == motion_command_router_get_diag()->remote_armed);
    TEST_CHECK(0U == motion_command_router_submit(MOTION_SOURCE_WIRELESS_MANUAL, &request));
    TEST_CHECK(1U == motion_command_router_set_maintenance(0U, 102U));
    motion_command_router_update(103U, 0U);
    TEST_CHECK(MOTION_SOURCE_NONE == motion_command_router_get_diag()->active_source);
    TEST_CHECK(0U == motion_command_router_arm_remote(103U, 1U));
    TEST_CHECK(1U == motion_command_router_arm_remote(103U, 0U));
}

static void test_motion_router_emergency_and_safety_latches(void)
{
    motion_command_request_struct request = test_motion_request(12.0f, 1U, 100U);
    motion_command_router_init();
    mock_router_stop_count = 0U;
    TEST_CHECK(1U == motion_command_router_arm_remote(100U, 0U));
    TEST_CHECK(1U == motion_command_router_submit(MOTION_SOURCE_WIRELESS_MANUAL, &request));
    TEST_CHECK(1U == motion_command_router_latch_emergency_stop(101U));
    TEST_CHECK(0U == motion_command_router_submit(MOTION_SOURCE_UART_LOCAL, &request));
    TEST_CHECK(1U == motion_command_router_clear_emergency_stop(102U));
    motion_command_router_update(103U, 1U);
    TEST_CHECK(MOTION_STOP_SAFETY == motion_command_router_get_diag()->stop_reason);
    TEST_CHECK(0U == motion_command_router_get_diag()->remote_armed);
    TEST_CHECK(0U < mock_router_stop_count);
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
    test_motion_router_priority_and_timeout();
    test_motion_router_rejects_invalid_and_disarmed_remote();
    test_motion_router_maintenance_and_rearm();
    test_motion_router_emergency_and_safety_latches();

    if(0U != test_failure_count)
    {
        printf("intercore_control_foundation_test: %u failure(s)\n",
               (unsigned int)test_failure_count);
        return 1;
    }

    printf("intercore_control_foundation_test: PASS\n");
    return 0;
}

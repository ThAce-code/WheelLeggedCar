#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "intercore_transport.h"

static uint32 failures;
static _Alignas(32) intercore_shared_layout_struct shared;
static intercore_transport_struct sender;
static intercore_transport_struct receiver;

#define TEST_CHECK(condition) do {                                             \
    if(!(condition)) {                                                         \
        printf("FAIL:%s:%d: %s\n", __FILE__, __LINE__, #condition);         \
        failures++;                                                            \
    }                                                                          \
}while(0)

static void fixture_init(void)
{
    memset(&shared, 0, sizeof(shared));
    TEST_CHECK(1U == intercore_transport_cm7_0_init(&receiver, &shared));
    TEST_CHECK(1U == intercore_transport_cm7_1_attach(&sender, &shared));
}

static intercore_gnss_payload_struct sample_payload(void)
{
    intercore_gnss_payload_struct sent = {0};

    sent.local_x_m = 1.25F;
    sent.local_y_m = -0.50F;
    sent.hdop = 0.85F;
    sent.satellite_count = 12U;
    sent.fix_valid = 1U;
    sent.origin_valid = 1U;
    return sent;
}

static void test_round_trip_duplicate_and_crc(void)
{
    intercore_gnss_payload_struct sent;
    intercore_gnss_payload_struct received = {0};
    uint32 source_ms = 0U;
    uint32 record_sequence = 0U;

    fixture_init();
    sent = sample_payload();
    TEST_CHECK(1U == intercore_transport_publish_gnss(&sender, &sent, 100U));
    TEST_CHECK(INTERCORE_TRANSPORT_OK == intercore_transport_read_gnss(
        &receiver, &received, &source_ms, &record_sequence));
    TEST_CHECK(1.25F == received.local_x_m);
    TEST_CHECK(-0.50F == received.local_y_m);
    TEST_CHECK(12U == received.satellite_count);
    TEST_CHECK(100U == source_ms);
    TEST_CHECK(INTERCORE_TRANSPORT_NO_DATA == intercore_transport_read_gnss(
        &receiver, &received, &source_ms, &record_sequence));

    receiver.last_gnss_sequence = 0U;
    shared.gnss[shared.metadata.gnss_active_index].payload.local_x_m += 1.0F;
    TEST_CHECK(INTERCORE_TRANSPORT_INVALID == intercore_transport_read_gnss(
        &receiver, &received, &source_ms, &record_sequence));
    TEST_CHECK(1U == shared.health.crc_error_count);
}

static void test_invalid_headers_and_epoch(void)
{
    intercore_gnss_payload_struct sent;
    intercore_gnss_payload_struct received = {0};
    uint32 source_ms = 0U;
    uint32 record_sequence = 0U;
    uint32 active_index;

    fixture_init();
    shared.metadata.gnss_active_index = 2U;
    TEST_CHECK(INTERCORE_TRANSPORT_INVALID == intercore_transport_read_gnss(
        &receiver, &received, &source_ms, &record_sequence));

    fixture_init();
    sent = sample_payload();
    TEST_CHECK(1U == intercore_transport_publish_gnss(&sender, &sent, 200U));
    active_index = shared.metadata.gnss_active_index;
    shared.gnss[active_index].header.type = (uint16)INTERCORE_RECORD_NAVIGATION;
    TEST_CHECK(INTERCORE_TRANSPORT_INVALID == intercore_transport_read_gnss(
        &receiver, &received, &source_ms, &record_sequence));

    fixture_init();
    receiver.last_navigation_sequence = 7U;
    receiver.last_gnss_sequence = 9U;
    shared.metadata.boot_epoch++;
    TEST_CHECK(INTERCORE_TRANSPORT_EPOCH_CHANGED == intercore_transport_read_gnss(
        &receiver, &received, &source_ms, &record_sequence));
    TEST_CHECK(0U == receiver.last_navigation_sequence);
    TEST_CHECK(0U == receiver.last_gnss_sequence);
}

int main(void)
{
    TEST_CHECK(48U == sizeof(intercore_gnss_payload_struct));
    TEST_CHECK(256U == sizeof(intercore_gnss_slot_struct));
    TEST_CHECK(8192U == sizeof(intercore_shared_layout_struct));
    TEST_CHECK(0x900U == offsetof(intercore_shared_layout_struct, gnss));
    TEST_CHECK(0xD00U == offsetof(intercore_shared_layout_struct, perception));
    test_round_trip_duplicate_and_crc();
    test_invalid_headers_and_epoch();

    if(0U != failures)
    {
        printf("gnss_intercore_test: %u failure(s)\n", (unsigned int)failures);
        return 1;
    }

    puts("gnss_intercore_test: PASS");
    return 0;
}

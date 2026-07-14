#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "intercore_camera.h"

static uint32 test_failure_count = 0U;
static _Alignas(32) intercore_shared_layout_struct shared;
static _Alignas(32) uint8 camera_data[INTERCORE_CAMERA_DATA_SIZE_BYTES];
static intercore_camera_transport_struct producer;
static intercore_camera_transport_struct consumer;

#define TEST_CHECK(condition)                                                   \
    do                                                                          \
    {                                                                           \
        if(!(condition))                                                        \
        {                                                                       \
            printf("FAIL:%s:%d: %s\n", __FILE__, __LINE__, #condition);         \
            test_failure_count++;                                               \
        }                                                                       \
    }while(0)

static void fixture_init(void)
{
    memset(&shared, 0, sizeof(shared));
    memset(camera_data, 0, sizeof(camera_data));
    shared.metadata.magic = INTERCORE_PROTOCOL_MAGIC;
    shared.metadata.version = INTERCORE_PROTOCOL_VERSION;
    shared.metadata.layout_size = (uint16)sizeof(shared);
    shared.metadata.boot_epoch = 1U;
    shared.metadata.cm7_0_ready = 1U;
    TEST_CHECK(1U == intercore_camera_producer_init(
                         &producer, &shared, camera_data, 1U));
    TEST_CHECK(1U == intercore_camera_consumer_attach(
                         &consumer, &shared, camera_data, 1U));
}

static void fixture_publish(uint8 expected_slot, uint32 sequence)
{
    uint8 slot_index = 0xFFU;
    volatile uint8 *pixels = NULL;

    shared.camera.capture_sequence = sequence - 1U;
    intercore_camera_producer_record_capture(&producer, sequence * 10U);
    TEST_CHECK(INTERCORE_CAMERA_OK ==
               intercore_camera_producer_claim(&producer, &slot_index, &pixels));
    TEST_CHECK(expected_slot == slot_index);
    pixels[0] = (uint8)sequence;
    TEST_CHECK(1U == intercore_camera_producer_publish(
                         &producer, slot_index, sequence * 10U, sequence * 10U + 1U));
}

static void test_layout(void)
{
    TEST_CHECK(32U == sizeof(intercore_camera_slot_struct));
    TEST_CHECK(256U == sizeof(intercore_camera_control_struct));
    TEST_CHECK(0xC00U == offsetof(intercore_shared_layout_struct, camera));
    TEST_CHECK(0xD00U == offsetof(intercore_shared_layout_struct, reserved));
    TEST_CHECK(8192U == sizeof(intercore_shared_layout_struct));
}

static void test_public_abi(void)
{
    intercore_camera_frame_view_struct view = {0};

    TEST_CHECK(0U == INTERCORE_CAMERA_NO_FRAME);
    TEST_CHECK(1U == INTERCORE_CAMERA_OK);
    TEST_CHECK(2U == INTERCORE_CAMERA_INVALID);
    TEST_CHECK(3U == INTERCORE_CAMERA_NO_FREE_SLOT);
    TEST_CHECK(4U == INTERCORE_CAMERA_EPOCH_CHANGED);

    fixture_init();
    TEST_CHECK(&shared.camera == producer.control);
    TEST_CHECK(&shared.camera == consumer.control);
    TEST_CHECK(0U == producer.last_consumed_sequence);
    TEST_CHECK(0U == consumer.last_consumed_sequence);
    view.width = INTERCORE_CAMERA_WIDTH;
    view.height = INTERCORE_CAMERA_HEIGHT;
    view.stride = INTERCORE_CAMERA_STRIDE;
    TEST_CHECK(INTERCORE_CAMERA_WIDTH == view.width);
    TEST_CHECK(INTERCORE_CAMERA_HEIGHT == view.height);
    TEST_CHECK(INTERCORE_CAMERA_STRIDE == view.stride);
}

static void test_normal_handoff(void)
{
    uint8 slot_index = 0xFFU;
    volatile uint8 *pixels = NULL;
    intercore_camera_frame_view_struct view;

    fixture_init();
    intercore_camera_producer_record_capture(&producer, 100U);
    TEST_CHECK(INTERCORE_CAMERA_OK ==
               intercore_camera_producer_claim(&producer, &slot_index, &pixels));
    TEST_CHECK(INTERCORE_CAMERA_SLOT_WRITING == shared.camera.slot[slot_index].state);
    pixels[0] = 0x12U;
    pixels[INTERCORE_CAMERA_SLOT_SIZE_BYTES - 1U] = 0x34U;
    TEST_CHECK(1U == intercore_camera_producer_publish(&producer, slot_index, 100U, 101U));
    TEST_CHECK(INTERCORE_CAMERA_SLOT_READY == shared.camera.slot[slot_index].state);

    TEST_CHECK(INTERCORE_CAMERA_OK ==
               intercore_camera_consumer_acquire_latest(&consumer, &view));
    TEST_CHECK(slot_index == view.slot_index);
    TEST_CHECK(INTERCORE_CAMERA_WIDTH == view.width);
    TEST_CHECK(INTERCORE_CAMERA_HEIGHT == view.height);
    TEST_CHECK(INTERCORE_CAMERA_STRIDE == view.stride);
    TEST_CHECK(0x12U == view.pixels[0]);
    TEST_CHECK(0x34U == view.pixels[INTERCORE_CAMERA_SLOT_SIZE_BYTES - 1U]);
    TEST_CHECK(INTERCORE_CAMERA_SLOT_READING == shared.camera.slot[slot_index].state);
    TEST_CHECK(1U == intercore_camera_consumer_release(&consumer, &view));
    TEST_CHECK(INTERCORE_CAMERA_SLOT_FREE == shared.camera.slot[slot_index].state);
    TEST_CHECK(view.sequence == consumer.last_consumed_sequence);
}

static void test_newest_ready_wins_without_fifo(void)
{
    intercore_camera_frame_view_struct view;

    fixture_init();
    fixture_publish(0U, 10U);
    fixture_publish(1U, 11U);
    TEST_CHECK(INTERCORE_CAMERA_OK ==
               intercore_camera_consumer_acquire_latest(&consumer, &view));
    TEST_CHECK(11U == view.sequence);
    TEST_CHECK(INTERCORE_CAMERA_SLOT_FREE == shared.camera.slot[0].state);
    TEST_CHECK(1U == shared.camera.stale_ready_drop_count);
}

static void test_release_records_consume_time_only_after_matching_read(void)
{
    intercore_camera_frame_view_struct view;
    uint32 sequence;

    fixture_init();
    fixture_publish(0U, 12U);
    TEST_CHECK(INTERCORE_CAMERA_OK ==
               intercore_camera_consumer_acquire_latest(&consumer, &view));
    sequence = view.sequence;
    shared.camera.consumer_heartbeat_ms = 222U;

    view.sequence++;
    TEST_CHECK(0U == intercore_camera_consumer_release(&consumer, &view));
    TEST_CHECK(0U == shared.camera.consumed_count);
    TEST_CHECK(0U == shared.camera.last_consume_ms);

    view.sequence = sequence;
    TEST_CHECK(1U == intercore_camera_consumer_release(&consumer, &view));
    TEST_CHECK(1U == shared.camera.consumed_count);
    TEST_CHECK(222U == shared.camera.last_consume_ms);
}

static void test_reading_slot_is_never_overwritten(void)
{
    intercore_camera_frame_view_struct view;
    uint8 slot_index;
    volatile uint8 *pixels;

    fixture_init();
    fixture_publish(0U, 20U);
    TEST_CHECK(INTERCORE_CAMERA_OK ==
               intercore_camera_consumer_acquire_latest(&consumer, &view));
    fixture_publish(1U, 21U);
    TEST_CHECK(INTERCORE_CAMERA_NO_FREE_SLOT ==
               intercore_camera_producer_claim(&producer, &slot_index, &pixels));
    TEST_CHECK(INTERCORE_CAMERA_SLOT_READING == shared.camera.slot[view.slot_index].state);
    TEST_CHECK(1U == shared.camera.no_free_drop_count);
}

static void test_epoch_change_is_rejected(void)
{
    uint8 slot_index = 0xFFU;
    volatile uint8 *pixels = NULL;
    intercore_camera_frame_view_struct view;

    fixture_init();
    shared.metadata.boot_epoch = 2U;
    TEST_CHECK(INTERCORE_CAMERA_EPOCH_CHANGED ==
               intercore_camera_producer_claim(&producer, &slot_index, &pixels));
    TEST_CHECK(INTERCORE_CAMERA_EPOCH_CHANGED ==
               intercore_camera_consumer_acquire_latest(&consumer, &view));
}

static void test_consumer_restart_releases_only_stale_reading(void)
{
    fixture_init();
    shared.camera.slot[0].state = INTERCORE_CAMERA_SLOT_WRITING;
    shared.camera.slot[1].state = INTERCORE_CAMERA_SLOT_READY;
    TEST_CHECK(1U == intercore_camera_consumer_attach(
                         &consumer, &shared, camera_data, 1U));
    TEST_CHECK(INTERCORE_CAMERA_SLOT_WRITING == shared.camera.slot[0].state);
    TEST_CHECK(INTERCORE_CAMERA_SLOT_READY == shared.camera.slot[1].state);

    shared.camera.slot[0].state = INTERCORE_CAMERA_SLOT_READING;
    TEST_CHECK(1U == intercore_camera_consumer_attach(
                         &consumer, &shared, camera_data, 1U));
    TEST_CHECK(INTERCORE_CAMERA_SLOT_FREE == shared.camera.slot[0].state);
    TEST_CHECK(INTERCORE_CAMERA_SLOT_READY == shared.camera.slot[1].state);
}

static void test_invalid_layout_fields_are_rejected(void)
{
    uint32 invalid_case;

    for(invalid_case = 0U; invalid_case < 10U; invalid_case++)
    {
        fixture_init();
        switch(invalid_case)
        {
            case 0U: shared.camera.magic = 0U; break;
            case 1U: shared.camera.version++; break;
            case 2U: shared.camera.format++; break;
            case 3U: shared.camera.width--; break;
            case 4U: shared.camera.height--; break;
            case 5U: shared.camera.stride--; break;
            case 6U: shared.camera.slot_count--; break;
            case 7U: shared.camera.frame_bytes--; break;
            case 8U: shared.camera.producer_boot_epoch++; break;
            default:
                shared.camera.slot[1].state = INTERCORE_CAMERA_SLOT_READING + 1U;
                break;
        }
        TEST_CHECK(0U == intercore_camera_consumer_attach(
                             &consumer, &shared, camera_data, 1U));
        TEST_CHECK(1U == shared.camera.invalid_layout_count);
    }
}

static void test_invalid_slot_state_rejects_every_public_transition(void)
{
    uint8 slot_index = 0xFFU;
    volatile uint8 *pixels = NULL;
    intercore_camera_frame_view_struct view = {0};
    uint32 captured_count;

    fixture_init();
    shared.camera.slot[1].state = INTERCORE_CAMERA_SLOT_READING + 1U;
    captured_count = shared.camera.captured_count;
    intercore_camera_producer_record_capture(&producer, 10U);
    TEST_CHECK(captured_count == shared.camera.captured_count);
    TEST_CHECK(1U == shared.camera.invalid_layout_count);

    fixture_init();
    shared.camera.slot[1].state = INTERCORE_CAMERA_SLOT_READING + 1U;
    TEST_CHECK(INTERCORE_CAMERA_INVALID ==
               intercore_camera_producer_claim(&producer, &slot_index, &pixels));
    TEST_CHECK(1U == shared.camera.invalid_layout_count);

    fixture_init();
    shared.camera.slot[0].state = INTERCORE_CAMERA_SLOT_WRITING;
    shared.camera.slot[1].state = INTERCORE_CAMERA_SLOT_READING + 1U;
    TEST_CHECK(0U == intercore_camera_producer_publish(&producer, 0U, 10U, 11U));
    TEST_CHECK(1U == shared.camera.invalid_layout_count);

    fixture_init();
    shared.camera.slot[0].state = INTERCORE_CAMERA_SLOT_READY;
    shared.camera.slot[0].sequence = 1U;
    shared.camera.slot[1].state = INTERCORE_CAMERA_SLOT_READING + 1U;
    TEST_CHECK(INTERCORE_CAMERA_INVALID ==
               intercore_camera_consumer_acquire_latest(&consumer, &view));
    TEST_CHECK(1U == shared.camera.invalid_layout_count);

    fixture_init();
    shared.camera.slot[0].state = INTERCORE_CAMERA_SLOT_READING;
    shared.camera.slot[0].sequence = 1U;
    shared.camera.slot[1].state = INTERCORE_CAMERA_SLOT_READING + 1U;
    view.slot_index = 0U;
    view.sequence = 1U;
    TEST_CHECK(0U == intercore_camera_consumer_release(&consumer, &view));
    TEST_CHECK(1U == shared.camera.invalid_layout_count);
}

int main(void)
{
    test_layout();
    test_public_abi();
    test_normal_handoff();
    test_newest_ready_wins_without_fifo();
    test_release_records_consume_time_only_after_matching_read();
    test_reading_slot_is_never_overwritten();
    test_epoch_change_is_rejected();
    test_consumer_restart_releases_only_stale_reading();
    test_invalid_layout_fields_are_rejected();
    test_invalid_slot_state_rejects_every_public_transition();

    if(0U != test_failure_count)
    {
        printf("intercore_camera_handoff_test: %u failure(s)\n",
               (unsigned int)test_failure_count);
        return 1;
    }

    printf("intercore_camera_handoff_test: PASS\n");
    return 0;
}

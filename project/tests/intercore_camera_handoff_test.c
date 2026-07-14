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
    TEST_CHECK(0x12U == view.pixels[0]);
    TEST_CHECK(0x34U == view.pixels[INTERCORE_CAMERA_SLOT_SIZE_BYTES - 1U]);
    TEST_CHECK(INTERCORE_CAMERA_SLOT_READING == shared.camera.slot[slot_index].state);
    TEST_CHECK(1U == intercore_camera_consumer_release(&consumer, &view));
    TEST_CHECK(INTERCORE_CAMERA_SLOT_FREE == shared.camera.slot[slot_index].state);
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
    shared.camera.slot[0].state = INTERCORE_CAMERA_SLOT_READING;
    shared.camera.slot[1].state = INTERCORE_CAMERA_SLOT_READY;
    TEST_CHECK(1U == intercore_camera_consumer_attach(
                         &consumer, &shared, camera_data, 1U));
    TEST_CHECK(INTERCORE_CAMERA_SLOT_FREE == shared.camera.slot[0].state);
    TEST_CHECK(INTERCORE_CAMERA_SLOT_READY == shared.camera.slot[1].state);
}

static void test_invalid_layout_is_rejected(void)
{
    fixture_init();
    shared.camera.frame_bytes = INTERCORE_CAMERA_SLOT_SIZE_BYTES - 1U;
    TEST_CHECK(0U == intercore_camera_consumer_attach(
                         &consumer, &shared, camera_data, 1U));
    TEST_CHECK(1U == shared.camera.invalid_layout_count);
}

int main(void)
{
    test_layout();
    test_normal_handoff();
    test_newest_ready_wins_without_fifo();
    test_reading_slot_is_never_overwritten();
    test_epoch_change_is_rejected();
    test_consumer_restart_releases_only_stale_reading();
    test_invalid_layout_is_rejected();

    if(0U != test_failure_count)
    {
        printf("intercore_camera_handoff_test: %u failure(s)\n",
               (unsigned int)test_failure_count);
        return 1;
    }

    printf("intercore_camera_handoff_test: PASS\n");
    return 0;
}

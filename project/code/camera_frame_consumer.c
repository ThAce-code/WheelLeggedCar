/*********************************************************************************************************************
* File: camera_frame_consumer.c
* Description: CM7_1 latest-frame vision boundary and WiFi-SPI Assistant display.
********************************************************************************************************************/

#include "camera_frame_consumer.h"
#include "camera_debug_config.h"
#include "camera_seekfree_transport.h"
#include "intercore_camera.h"
#include "intercore_memory.h"
#include "intercore_notify.h"
#include "intercore_transport.h"
#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"
#include "zf_device_mt9v03x.h"
#include "zf_device_wifi_spi.h"

#include <stdint.h>
#include <string.h>

static intercore_camera_transport_struct camera_transport;
static intercore_transport_struct control_transport;
static volatile intercore_shared_layout_struct *camera_shared;
static volatile uint8 *camera_data;
static volatile uint32 consumer_ms;
static uint8 consumer_notify_initialized;
static uint8 camera_information_initialized;
static uint32 socket_connected_ms;
static uint8 camera_gate_evidence_started;
static seekfree_assistant_camera_struct camera_information[INTERCORE_CAMERA_SLOT_COUNT];
camera_frame_consumer_diag_struct consumer_diag;
volatile camera_gate_snapshot_struct camera_gate_snapshot;
volatile camera_gate_evidence_struct camera_gate_evidence;

static void camera_frame_consumer_update_gate_snapshot(void)
{
    uint32 captured_before;
    uint32 captured_after;
    uint32 evidence_generation;
    uint32 generation;
    volatile intercore_camera_control_struct *control;

    if((0U == camera_transport.attached) || (NULL == camera_transport.control))
    {
        return;
    }

    control = camera_transport.control;
    generation = camera_gate_snapshot.generation + 1U;
    camera_gate_snapshot.generation = generation;
    do
    {
        captured_before = control->captured_count;
        camera_gate_snapshot.consumer_ms = camera_frame_consumer_now_ms();
        camera_gate_snapshot.producer_heartbeat_ms = control->producer_heartbeat_ms;
        camera_gate_snapshot.producer_last_publish_ms = control->last_publish_ms;
        camera_gate_snapshot.captured_count = captured_before;
        camera_gate_snapshot.latest_published_sequence = control->latest_published_sequence;
        camera_gate_snapshot.published_count = control->published_count;
        camera_gate_snapshot.notify_count = control->notify_count;
        camera_gate_snapshot.no_free_drop_count = control->no_free_drop_count;
        camera_gate_snapshot.period_drop_count = control->producer_period_drop_count;
        camera_gate_snapshot.slot_state[0] = control->slot[0].state;
        camera_gate_snapshot.slot_state[1] = control->slot[1].state;
        camera_gate_snapshot.slot_sequence[0] = control->slot[0].sequence;
        camera_gate_snapshot.slot_sequence[1] = control->slot[1].sequence;
        captured_after = control->captured_count;
    } while(captured_before != captured_after);

    camera_gate_snapshot.acquired_count = consumer_diag.acquired_count;
    camera_gate_snapshot.sent_count = consumer_diag.sent_count;
    camera_gate_snapshot.released_count = consumer_diag.released_count;
    camera_gate_snapshot.reconnect_count = consumer_diag.reconnect_count;
    camera_gate_snapshot.invalid_count = consumer_diag.invalid_count;
    camera_gate_snapshot.timeout_count = consumer_diag.timeout_count;
    camera_gate_snapshot.stale_count = consumer_diag.stale_count;
    camera_gate_snapshot.send_failure_count = consumer_diag.send_failure_count;
    camera_gate_snapshot.startup_send_histogram = consumer_diag.startup_send_histogram;
    camera_gate_snapshot.steady_send_histogram = consumer_diag.steady_send_histogram;
    camera_gate_snapshot.generation = generation + 1U;

    if(0U == camera_gate_evidence.complete_count)
    {
        if((0U == camera_gate_evidence_started) &&
           ((uint8)CAMERA_CONSUMER_LINK_CONNECTED == consumer_diag.socket_state) &&
           (APP_CAMERA_SEND_STARTUP_MS <=
            (camera_gate_snapshot.consumer_ms - socket_connected_ms)))
        {
            evidence_generation = camera_gate_evidence.generation + 1U;
            camera_gate_evidence.generation = evidence_generation;
            memcpy((void *)&camera_gate_evidence.start,
                   (const void *)&camera_gate_snapshot,
                   sizeof(camera_gate_snapshot));
            camera_gate_evidence.generation = evidence_generation + 1U;
            camera_gate_evidence_started = 1U;
        }
        else if((0U != camera_gate_evidence_started) &&
                (APP_CAMERA_GATE_WINDOW_MS <=
                 (camera_gate_snapshot.consumer_ms -
                  camera_gate_evidence.start.consumer_ms)))
        {
            evidence_generation = camera_gate_evidence.generation + 1U;
            camera_gate_evidence.generation = evidence_generation;
            memcpy((void *)&camera_gate_evidence.end,
                   (const void *)&camera_gate_snapshot,
                   sizeof(camera_gate_snapshot));
            camera_gate_evidence.complete_count++;
            camera_gate_evidence.generation = evidence_generation + 1U;
        }
    }
}

static uint8 camera_frame_consumer_try_attach(void)
{
    uint32 boot_epoch;

    if((NULL == camera_shared) || (NULL == camera_data))
    {
        camera_shared = intercore_memory_get_layout();
        camera_data = intercore_memory_get_camera_data();
    }
    if((NULL == camera_shared) || (NULL == camera_data))
    {
        consumer_diag.handoff_state = (uint8)CAMERA_CONSUMER_LINK_FAILED;
        return 1U;
    }

    boot_epoch = camera_shared->metadata.boot_epoch;
    if((0U == intercore_transport_cm7_1_attach(&control_transport, camera_shared)) ||
       (0U == intercore_camera_consumer_attach(&camera_transport,
                                               camera_shared,
                                               camera_data,
                                               boot_epoch)))
    {
        consumer_diag.handoff_state = (uint8)CAMERA_CONSUMER_LINK_FAILED;
        return 1U;
    }

    consumer_diag.handoff_state = (uint8)CAMERA_CONSUMER_LINK_CONNECTED;
    return 0U;
}

static void camera_frame_consumer_configure_assistant(void)
{
    if(0U != camera_information_initialized)
    {
        return;
    }

    camera_seekfree_transport_install();
    seekfree_assistant_camera_config(&camera_information[0],
                                     SEEKFREE_ASSISTANT_CAMERA_TYPE_MT9V03X,
                                     MT9V03X_W,
                                     MT9V03X_H,
                                     (uint8 *)(uintptr_t)INTERCORE_CAMERA_DATA_BASE_ADDRESS);
    seekfree_assistant_camera_config(&camera_information[1],
                                     SEEKFREE_ASSISTANT_CAMERA_TYPE_MT9V03X,
                                     MT9V03X_W,
                                     MT9V03X_H,
                                     (uint8 *)(uintptr_t)(INTERCORE_CAMERA_DATA_BASE_ADDRESS +
                                                         INTERCORE_CAMERA_SLOT_SIZE_BYTES));
    camera_information_initialized = 1U;
}

static void camera_frame_consumer_try_network(void)
{
#if APP_CAMERA_WIFI_ENABLE
    uint32 now_ms = camera_frame_consumer_now_ms();

    if((0U != consumer_diag.reconnect_count) &&
       (APP_CAMERA_WIFI_RETRY_MS > (now_ms - consumer_diag.last_reconnect_ms)))
    {
        return;
    }

    consumer_diag.last_reconnect_ms = now_ms;
    consumer_diag.reconnect_count++;
    if((uint8)CAMERA_CONSUMER_LINK_CONNECTED != consumer_diag.wifi_state)
    {
        consumer_diag.wifi_state = (uint8)CAMERA_CONSUMER_LINK_CONNECTING;
        consumer_diag.socket_state = (uint8)CAMERA_CONSUMER_LINK_NOT_RUN;
        if(0U != wifi_spi_init(APP_CAMERA_WIFI_SSID, APP_CAMERA_WIFI_PASSWORD))
        {
            consumer_diag.wifi_state = (uint8)CAMERA_CONSUMER_LINK_FAILED;
            consumer_diag.init_state = (uint8)CAMERA_CONSUMER_INIT_WIFI_FAILED;
            return;
        }
        consumer_diag.wifi_state = (uint8)CAMERA_CONSUMER_LINK_CONNECTED;
    }

    consumer_diag.socket_state = (uint8)CAMERA_CONSUMER_LINK_CONNECTING;
    if(0U != wifi_spi_socket_connect("TCP",
                                     APP_CAMERA_WIFI_TARGET_IP,
                                     APP_CAMERA_WIFI_TARGET_PORT,
                                     APP_CAMERA_WIFI_LOCAL_PORT))
    {
        consumer_diag.socket_state = (uint8)CAMERA_CONSUMER_LINK_FAILED;
        consumer_diag.wifi_state = (uint8)CAMERA_CONSUMER_LINK_FAILED;
        consumer_diag.init_state = (uint8)CAMERA_CONSUMER_INIT_SOCKET_FAILED;
        return;
    }

    consumer_diag.socket_state = (uint8)CAMERA_CONSUMER_LINK_CONNECTED;
    socket_connected_ms = camera_frame_consumer_now_ms();
    consumer_diag.init_state = (uint8)CAMERA_CONSUMER_INIT_OK;
#else
    consumer_diag.wifi_state = (uint8)CAMERA_CONSUMER_LINK_CONNECTED;
    consumer_diag.socket_state = (uint8)CAMERA_CONSUMER_LINK_CONNECTED;
    consumer_diag.init_state = (uint8)CAMERA_CONSUMER_INIT_OK;
#endif
}

uint8 camera_frame_consumer_init(void)
{
    memset(&consumer_diag, 0, sizeof(consumer_diag));
    memset(&camera_transport, 0, sizeof(camera_transport));
    memset(&control_transport, 0, sizeof(control_transport));
    memset(camera_information, 0, sizeof(camera_information));
    memset((void *)&camera_gate_snapshot, 0, sizeof(camera_gate_snapshot));
    memset((void *)&camera_gate_evidence, 0, sizeof(camera_gate_evidence));
    camera_shared = intercore_memory_get_layout();
    camera_data = intercore_memory_get_camera_data();
    consumer_ms = 0U;
    socket_connected_ms = 0U;
    camera_gate_evidence_started = 0U;
    camera_information_initialized = 0U;
    camera_frame_consumer_configure_assistant();
    consumer_notify_initialized = intercore_notify_init(INTERCORE_ROLE_CM7_1);
    if(0U == consumer_notify_initialized)
    {
        consumer_diag.init_state = (uint8)CAMERA_CONSUMER_INIT_HANDOFF_FAILED;
        return 1U;
    }
    if(0U != camera_frame_consumer_try_attach())
    {
        return 1U;
    }
    camera_frame_consumer_try_network();
    return ((uint8)CAMERA_CONSUMER_INIT_OK == consumer_diag.init_state) ? 0U : 1U;
}

void camera_frame_consumer_tick_1ms(void)
{
    consumer_ms++;
}

uint32 camera_frame_consumer_now_ms(void)
{
    return consumer_ms;
}

static void camera_frame_consumer_record_send_duration(
    camera_send_duration_histogram_struct *histogram,
    uint32 duration_ms)
{
    if(duration_ms <= 25U)        { histogram->le_25_ms++; }
    else if(duration_ms <= 50U)   { histogram->le_50_ms++; }
    else if(duration_ms <= 100U)  { histogram->le_100_ms++; }
    else if(duration_ms <= 200U)  { histogram->le_200_ms++; }
    else if(duration_ms <= 500U)  { histogram->le_500_ms++; }
    else if(duration_ms <= 1000U) { histogram->le_1000_ms++; }
    else                          { histogram->gt_1000_ms++; }
}

void camera_frame_consumer_service(void)
{
    uint8 release_ok;
    uint8 send_ok;
    uint32 producer_now_ms;
    uint32 send_duration_ms;
    uint32 send_start_ms;
    camera_vision_frame_view_struct vision_view;
    intercore_camera_frame_view_struct view;
    intercore_camera_result_enum result;

    camera_frame_consumer_update_gate_snapshot();
    if(0U == consumer_notify_initialized)
    {
        return;
    }
    if(0U == camera_transport.attached)
    {
        if(0U != camera_frame_consumer_try_attach())
        {
            return;
        }
    }
    if((uint8)CAMERA_CONSUMER_LINK_CONNECTED != consumer_diag.socket_state)
    {
        camera_frame_consumer_try_network();
        return;
    }

    camera_transport.control->consumer_heartbeat_ms = camera_frame_consumer_now_ms();
    (void)(intercore_notify_take_pending() & INTERCORE_NOTIFY_CAMERA_READY);
    result = intercore_camera_consumer_acquire_latest(&camera_transport, &view);
    consumer_diag.stale_ready_drop_count =
        camera_transport.control->stale_ready_drop_count;
    if(INTERCORE_CAMERA_NO_FRAME == result)
    {
        return;
    }
    if(INTERCORE_CAMERA_OK != result)
    {
        consumer_diag.invalid_count++;
        if(INTERCORE_CAMERA_EPOCH_CHANGED == result)
        {
            camera_transport.attached = 0U;
            consumer_diag.handoff_state = (uint8)CAMERA_CONSUMER_LINK_FAILED;
        }
        return;
    }

    consumer_diag.last_sequence = view.sequence;
    consumer_diag.last_capture_ms = view.capture_ms;
    producer_now_ms = camera_transport.control->producer_heartbeat_ms;
    consumer_diag.last_frame_age_ms =
        intercore_camera_frame_age_ms(producer_now_ms, view.capture_ms);
    consumer_diag.acquired_count++;

    vision_view.slot_index = view.slot_index;
    vision_view.sequence = view.sequence;
    vision_view.capture_ms = view.capture_ms;
    vision_view.frame_age_ms = consumer_diag.last_frame_age_ms;
    vision_view.width = view.width;
    vision_view.height = view.height;
    vision_view.stride = view.stride;
    vision_view.frame_bytes = view.frame_bytes;
    vision_view.pixels = view.pixels;
    consumer_diag.last_process_duration_us = 0U;
    consumer_diag.max_process_duration_us = 0U;
    camera_transport.control->last_process_duration_us = 0U;
    camera_transport.control->max_process_duration_us = 0U;

    if(APP_CAMERA_STALE_TIMEOUT_MS < consumer_diag.last_frame_age_ms)
    {
        consumer_diag.frame_valid = 0U;
        consumer_diag.timeout_count++;
        consumer_diag.stale_count++;
        camera_transport.control->timeout_count++;
    }
    else
    {
        consumer_diag.sample_0_0 = vision_view.pixels[0];
        consumer_diag.sample_center =
            vision_view.pixels[(60U * vision_view.stride) + 94U];
        consumer_diag.frame_valid = 1U;
        send_start_ms = camera_frame_consumer_now_ms();
        camera_seekfree_transport_begin(
            (uint32)sizeof(camera_information[vision_view.slot_index].config),
            vision_view.frame_bytes);
        seekfree_assistant_camera_send(&camera_information[vision_view.slot_index]);
        send_ok = camera_seekfree_transport_frame_complete();
        send_duration_ms = camera_frame_consumer_now_ms() - send_start_ms;
        consumer_diag.last_send_duration_ms = send_duration_ms;
        consumer_diag.max_send_duration_ms =
            (consumer_diag.max_send_duration_ms < send_duration_ms) ?
            send_duration_ms : consumer_diag.max_send_duration_ms;
        if((send_start_ms - socket_connected_ms) < APP_CAMERA_SEND_STARTUP_MS)
        {
            camera_frame_consumer_record_send_duration(
                &consumer_diag.startup_send_histogram, send_duration_ms);
        }
        else
        {
            camera_frame_consumer_record_send_duration(
                &consumer_diag.steady_send_histogram, send_duration_ms);
        }
        if(0U != send_ok)
        {
            consumer_diag.sent_count++;
            consumer_diag.last_sent_sequence = vision_view.sequence;
        }
        else
        {
            consumer_diag.send_failure_count++;
            consumer_diag.socket_state = (uint8)CAMERA_CONSUMER_LINK_FAILED;
            consumer_diag.wifi_state = (uint8)CAMERA_CONSUMER_LINK_FAILED;
            consumer_diag.init_state = (uint8)CAMERA_CONSUMER_INIT_SOCKET_FAILED;
            consumer_diag.last_reconnect_ms = camera_frame_consumer_now_ms();
        }
        camera_transport.control->last_send_duration_ms = send_duration_ms;
        camera_transport.control->max_send_duration_ms =
            (camera_transport.control->max_send_duration_ms < send_duration_ms) ?
            send_duration_ms : camera_transport.control->max_send_duration_ms;
    }

    release_ok = intercore_camera_consumer_release_at(
                     &camera_transport,
                     &view,
                     camera_frame_consumer_now_ms());
    if(0U != release_ok)
    {
        consumer_diag.released_count++;
        (void)intercore_camera_consumer_publish_observation(
                  &camera_transport,
                  consumer_diag.last_sequence,
                  consumer_diag.last_frame_age_ms,
                  consumer_diag.sample_0_0,
                  consumer_diag.sample_center,
                  consumer_diag.frame_valid);
    }
    else
    {
        consumer_diag.invalid_count++;
    }
}

const camera_frame_consumer_diag_struct *camera_frame_consumer_get_diag(void)
{
    return &consumer_diag;
}

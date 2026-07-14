/*********************************************************************************************************************
* File: camera_frame_consumer.c
* Description: CM7_1 latest-frame vision boundary and WiFi-SPI Assistant display.
********************************************************************************************************************/

#include "camera_frame_consumer.h"
#include "camera_debug_config.h"
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
static seekfree_assistant_camera_struct camera_information[INTERCORE_CAMERA_SLOT_COUNT];
camera_frame_consumer_diag_struct consumer_diag;

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
        consumer_diag.init_state = (uint8)CAMERA_CONSUMER_INIT_HANDOFF_FAILED;
        return 1U;
    }

    boot_epoch = camera_shared->metadata.boot_epoch;
    if((0U == intercore_transport_cm7_1_attach(&control_transport, camera_shared)) ||
       (0U == intercore_camera_consumer_attach(&camera_transport,
                                               camera_shared,
                                               camera_data,
                                               boot_epoch)))
    {
        consumer_diag.init_state = (uint8)CAMERA_CONSUMER_INIT_HANDOFF_FAILED;
        return 1U;
    }

#if APP_CAMERA_WIFI_ENABLE
    consumer_diag.init_state = (uint8)CAMERA_CONSUMER_INIT_NOT_RUN;
#else
    consumer_diag.init_state = (uint8)CAMERA_CONSUMER_INIT_OK;
#endif
    return 0U;
}

static void camera_frame_consumer_configure_assistant(void)
{
    if(0U != camera_information_initialized)
    {
        return;
    }

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
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
    consumer_diag.wifi_state = (uint8)CAMERA_CONSUMER_LINK_CONNECTING;
    consumer_diag.socket_state = (uint8)CAMERA_CONSUMER_LINK_NOT_RUN;
    if(0U != wifi_spi_init(APP_CAMERA_WIFI_SSID, APP_CAMERA_WIFI_PASSWORD))
    {
        consumer_diag.wifi_state = (uint8)CAMERA_CONSUMER_LINK_FAILED;
        consumer_diag.init_state = (uint8)CAMERA_CONSUMER_INIT_WIFI_FAILED;
        return;
    }

    consumer_diag.wifi_state = (uint8)CAMERA_CONSUMER_LINK_CONNECTED;
    consumer_diag.socket_state = (uint8)CAMERA_CONSUMER_LINK_CONNECTING;
    if(0U != wifi_spi_socket_connect("TCP",
                                     APP_CAMERA_WIFI_TARGET_IP,
                                     APP_CAMERA_WIFI_TARGET_PORT,
                                     APP_CAMERA_WIFI_LOCAL_PORT))
    {
        consumer_diag.socket_state = (uint8)CAMERA_CONSUMER_LINK_FAILED;
        consumer_diag.init_state = (uint8)CAMERA_CONSUMER_INIT_SOCKET_FAILED;
        return;
    }

    consumer_diag.socket_state = (uint8)CAMERA_CONSUMER_LINK_CONNECTED;
    camera_frame_consumer_configure_assistant();
    consumer_diag.init_state = (uint8)CAMERA_CONSUMER_INIT_OK;
#else
    consumer_diag.init_state = (uint8)CAMERA_CONSUMER_INIT_OK;
#endif
}

uint8 camera_frame_consumer_init(void)
{
    memset(&consumer_diag, 0, sizeof(consumer_diag));
    memset(&camera_transport, 0, sizeof(camera_transport));
    memset(&control_transport, 0, sizeof(control_transport));
    memset(camera_information, 0, sizeof(camera_information));
    camera_shared = intercore_memory_get_layout();
    camera_data = intercore_memory_get_camera_data();
    consumer_ms = 0U;
    camera_information_initialized = 0U;
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

void camera_frame_consumer_service(void)
{
    uint8 release_ok;
    uint32 producer_now_ms;
    uint32 send_duration_ms;
    uint32 send_start_ms;
    camera_vision_frame_view_struct vision_view;
    intercore_camera_frame_view_struct view;
    intercore_camera_result_enum result;

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
    if((uint8)CAMERA_CONSUMER_INIT_OK != consumer_diag.init_state)
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
            consumer_diag.init_state = (uint8)CAMERA_CONSUMER_INIT_HANDOFF_FAILED;
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
        seekfree_assistant_camera_send(&camera_information[vision_view.slot_index]);
        send_duration_ms = camera_frame_consumer_now_ms() - send_start_ms;
        consumer_diag.last_send_duration_ms = send_duration_ms;
        consumer_diag.max_send_duration_ms =
            (consumer_diag.max_send_duration_ms < send_duration_ms) ?
            send_duration_ms : consumer_diag.max_send_duration_ms;
        consumer_diag.sent_count++;
        consumer_diag.last_sent_sequence = vision_view.sequence;
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

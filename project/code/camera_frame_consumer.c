/*********************************************************************************************************************
* File: camera_frame_consumer.c
* Description: CM7_1 no-WiFi latest-frame consumer over the shared two-slot transport.
********************************************************************************************************************/

#include "camera_frame_consumer.h"
#include "camera_debug_config.h"
#include "intercore_camera.h"
#include "intercore_memory.h"
#include "intercore_notify.h"
#include "intercore_transport.h"

#include <string.h>

static intercore_camera_transport_struct camera_transport;
static intercore_transport_struct control_transport;
static volatile intercore_shared_layout_struct *camera_shared;
static volatile uint8 *camera_data;
static volatile uint32 consumer_ms;
static uint8 consumer_notify_initialized;
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

    consumer_diag.init_state = (uint8)CAMERA_CONSUMER_INIT_OK;
    return 0U;
}

uint8 camera_frame_consumer_init(void)
{
    memset(&consumer_diag, 0, sizeof(consumer_diag));
    memset(&camera_transport, 0, sizeof(camera_transport));
    memset(&control_transport, 0, sizeof(control_transport));
    camera_shared = intercore_memory_get_layout();
    camera_data = intercore_memory_get_camera_data();
    consumer_ms = 0U;
    consumer_notify_initialized = intercore_notify_init(INTERCORE_ROLE_CM7_1);
    if(0U == consumer_notify_initialized)
    {
        consumer_diag.init_state = (uint8)CAMERA_CONSUMER_INIT_HANDOFF_FAILED;
        return 1U;
    }
    return camera_frame_consumer_try_attach();
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
    uint32 now_ms;
    intercore_camera_frame_view_struct view;
    intercore_camera_result_enum result;

    if(0U == consumer_notify_initialized)
    {
        return;
    }
    if((uint8)CAMERA_CONSUMER_INIT_OK != consumer_diag.init_state)
    {
        (void)camera_frame_consumer_try_attach();
        return;
    }

    now_ms = camera_frame_consumer_now_ms();
    camera_transport.control->consumer_heartbeat_ms = now_ms;
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
            consumer_diag.init_state = (uint8)CAMERA_CONSUMER_INIT_HANDOFF_FAILED;
        }
        return;
    }

    consumer_diag.last_sequence = view.sequence;
    consumer_diag.last_capture_ms = view.capture_ms;
    consumer_diag.last_frame_age_ms = now_ms - view.capture_ms;
    consumer_diag.acquired_count++;
    if(APP_CAMERA_STALE_TIMEOUT_MS < consumer_diag.last_frame_age_ms)
    {
        consumer_diag.frame_valid = 0U;
        consumer_diag.timeout_count++;
        camera_transport.control->timeout_count++;
    }
    else
    {
        consumer_diag.sample_0_0 = view.pixels[0];
        consumer_diag.sample_center = view.pixels[(60U * view.stride) + 94U];
        consumer_diag.frame_valid = 1U;
    }

    release_ok = intercore_camera_consumer_release(&camera_transport, &view);
    if(0U != release_ok)
    {
        consumer_diag.released_count++;
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

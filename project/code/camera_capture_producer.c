/*********************************************************************************************************************
* File: camera_capture_producer.c
* Description: CM7_0 MT9V03X capture and two-slot latest-frame producer.
********************************************************************************************************************/

#include "camera_capture_producer.h"
#include "camera_debug_config.h"
#include "app.h"
#include "intercore_camera.h"
#include "intercore_memory.h"
#include "intercore_notify.h"
#include "zf_common_headfile.h"

#include <string.h>

static intercore_camera_transport_struct camera_transport;
camera_capture_producer_diag_struct producer_diag;

static uint32 producer_last_init_attempt_ms;
static uint8 producer_stale_latched;

static uint8 camera_capture_try_camera_init(void)
{
    producer_last_init_attempt_ms = app_get_ms();
    if(0U != mt9v03x_init())
    {
        producer_diag.init_state = (uint8)CAMERA_CAPTURE_INIT_CAMERA_FAILED;
        return 1U;
    }

    mt9v03x_finish_flag = 0U;
    producer_diag.init_state = (uint8)CAMERA_CAPTURE_INIT_OK;
    return 0U;
}

uint8 camera_capture_producer_init(void)
{
    volatile intercore_shared_layout_struct *shared;
    volatile uint8 *camera_data;

    memset(&producer_diag, 0, sizeof(producer_diag));
    memset(&camera_transport, 0, sizeof(camera_transport));
    producer_stale_latched = 0U;
    producer_last_init_attempt_ms = app_get_ms();
    producer_diag.last_publish_ms = producer_last_init_attempt_ms;

    shared = intercore_memory_get_layout();
    camera_data = intercore_memory_get_camera_data();
    if((NULL == shared) ||
       (0U == intercore_camera_producer_init(&camera_transport,
                                             shared,
                                             camera_data,
                                             shared->metadata.boot_epoch)))
    {
        producer_diag.init_state = (uint8)CAMERA_CAPTURE_INIT_HANDOFF_FAILED;
        return 1U;
    }

    timer_init(TC_TIME2_CH0, TIMER_US);
    timer_start(TC_TIME2_CH0);
    return camera_capture_try_camera_init();
}

void camera_capture_producer_service(void)
{
    uint8 notify_ok;
    uint8 publish_ok;
    uint8 slot_index;
    uint32 copy_duration_us;
    uint32 copy_start_us;
    uint32 now_ms;
    volatile uint8 *slot_pixels;
    intercore_camera_result_enum result;

    now_ms = app_get_ms();
    if(0U != camera_transport.attached)
    {
        camera_transport.control->producer_heartbeat_ms = now_ms;
    }

    if((uint8)CAMERA_CAPTURE_INIT_CAMERA_FAILED == producer_diag.init_state)
    {
        if(APP_CAMERA_RETRY_PERIOD_MS <= (now_ms - producer_last_init_attempt_ms))
        {
            (void)camera_capture_try_camera_init();
        }
        return;
    }
    if((uint8)CAMERA_CAPTURE_INIT_OK != producer_diag.init_state)
    {
        return;
    }

    if(0U != mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0U;
        now_ms = app_get_ms();
        intercore_camera_producer_record_capture(&camera_transport, now_ms);
        producer_diag.frame_count++;
        producer_diag.last_frame_ms = now_ms;
        producer_diag.frame_valid = 1U;
        producer_stale_latched = 0U;

        if(APP_CAMERA_DISPLAY_PERIOD_MS <= (now_ms - producer_diag.last_publish_ms))
        {
            result = intercore_camera_producer_claim(&camera_transport,
                                                     &slot_index,
                                                     &slot_pixels);
            if(INTERCORE_CAMERA_OK == result)
            {
                copy_start_us = timer_get(TC_TIME2_CH0);
                Cy_SysInt_DisableIRQ(tcpwm_0_interrupts_59_IRQn);
                memcpy((void *)slot_pixels, mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
                camera_transport.control->producer_heartbeat_ms = now_ms;
                publish_ok = intercore_camera_producer_publish(&camera_transport,
                                                               slot_index,
                                                               now_ms,
                                                               app_get_ms());
                copy_duration_us = timer_get(TC_TIME2_CH0) - copy_start_us;
                Cy_SysInt_EnableIRQ(tcpwm_0_interrupts_59_IRQn);
                if(0U == publish_ok)
                {
                    (void)intercore_camera_producer_abort(&camera_transport,
                                                          slot_index);
                    producer_diag.invalid_count++;
                }

                producer_diag.last_copy_duration_us = copy_duration_us;
                producer_diag.max_copy_duration_us =
                    (producer_diag.max_copy_duration_us < copy_duration_us) ?
                    copy_duration_us : producer_diag.max_copy_duration_us;
                camera_transport.control->last_copy_duration_us = copy_duration_us;
                camera_transport.control->max_copy_duration_us =
                    producer_diag.max_copy_duration_us;

                if(0U != publish_ok)
                {
                    producer_diag.last_publish_ms = now_ms;
                    producer_diag.publish_count++;
                    notify_ok = intercore_notify_try(INTERCORE_NOTIFY_CAMERA_READY);
                    if(0U != notify_ok)
                    {
                        camera_transport.control->notify_count++;
                    }
                }
            }
            else if(INTERCORE_CAMERA_NO_FREE_SLOT == result)
            {
                producer_diag.no_free_drop_count++;
            }
            else
            {
                producer_diag.invalid_count++;
            }
        }
        else
        {
            producer_diag.period_drop_count++;
        }
    }

    now_ms = app_get_ms();
    camera_transport.control->producer_heartbeat_ms = now_ms;
    if((0U != producer_diag.frame_valid) &&
       (APP_CAMERA_STALE_TIMEOUT_MS < (now_ms - producer_diag.last_frame_ms)))
    {
        producer_diag.frame_valid = 0U;
        if(0U == producer_stale_latched)
        {
            producer_stale_latched = 1U;
            producer_diag.timeout_count++;
            camera_transport.control->timeout_count++;
        }
    }
}

const camera_capture_producer_diag_struct *camera_capture_producer_get_diag(void)
{
    return &producer_diag;
}

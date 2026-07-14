/*********************************************************************************************************************
* File: camera_debug_app.c
* Description: Non-blocking CM7_1 camera-only portability application.
********************************************************************************************************************/

#include "camera_debug_app.h"
#include "camera_debug_config.h"

#include <string.h>

static uint8 image_copy[MT9V03X_H][MT9V03X_W];
static seekfree_assistant_camera_struct camera_information;
camera_debug_diag_struct camera_debug_diag;

static volatile uint32 camera_debug_ms = 0U;
static uint32 camera_debug_last_snapshot_ms = 0U;
static uint32 camera_debug_last_init_attempt_ms = 0U;
static uint8 camera_debug_stale_latched = 0U;

static uint8 camera_debug_try_camera_init(void)
{
    camera_debug_last_init_attempt_ms = camera_debug_app_now_ms();
    if(0U != mt9v03x_init())
    {
        camera_debug_diag.init_state = (uint8)CAMERA_DEBUG_INIT_CAMERA_FAILED;
        return 1U;
    }

    mt9v03x_finish_flag = 0U;
    camera_debug_diag.init_state = (uint8)CAMERA_DEBUG_INIT_OK;
    return 0U;
}

uint8 camera_debug_app_init(void)
{
    memset(&camera_debug_diag, 0, sizeof(camera_debug_diag));
    memset(image_copy[0], 0, MT9V03X_IMAGE_SIZE);
    (void)&camera_information;
    camera_debug_last_snapshot_ms = camera_debug_app_now_ms();
    camera_debug_stale_latched = 0U;
    return camera_debug_try_camera_init();
}

void camera_debug_app_tick_1ms(void)
{
    camera_debug_ms++;
}

uint32 camera_debug_app_now_ms(void)
{
    return camera_debug_ms;
}

void camera_debug_app_service(void)
{
    uint32 now_ms;

    now_ms = camera_debug_app_now_ms();
    if((uint8)CAMERA_DEBUG_INIT_OK != camera_debug_diag.init_state)
    {
        if(APP_CAMERA_RETRY_PERIOD_MS <= (now_ms - camera_debug_last_init_attempt_ms))
        {
            (void)camera_debug_try_camera_init();
        }
        return;
    }

    if(0U != mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0U;
        camera_debug_diag.frame_count++;
        camera_debug_diag.last_frame_ms = now_ms;
        camera_debug_diag.frame_valid = 1U;
        camera_debug_stale_latched = 0U;

        if(APP_CAMERA_DISPLAY_PERIOD_MS <= (now_ms - camera_debug_last_snapshot_ms))
        {
            memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
            camera_debug_last_snapshot_ms = now_ms;
            camera_debug_diag.snapshot_count++;
        }
        else
        {
            camera_debug_diag.dropped_count++;
        }
    }

    if((0U != camera_debug_diag.frame_valid) &&
       (APP_CAMERA_STALE_TIMEOUT_MS < (now_ms - camera_debug_diag.last_frame_ms)))
    {
        camera_debug_diag.frame_valid = 0U;
        if(0U == camera_debug_stale_latched)
        {
            camera_debug_stale_latched = 1U;
            camera_debug_diag.timeout_count++;
        }
    }
}

const camera_debug_diag_struct *camera_debug_app_get_diag(void)
{
    return &camera_debug_diag;
}

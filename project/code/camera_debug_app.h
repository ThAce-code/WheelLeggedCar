/*********************************************************************************************************************
* File: camera_debug_app.h
* Description: Camera-only CM7_1 portability diagnostics and service API.
********************************************************************************************************************/

#ifndef _camera_debug_app_h_
#define _camera_debug_app_h_

#include "zf_common_headfile.h"

typedef enum
{
    CAMERA_DEBUG_INIT_NOT_RUN = 0,
    CAMERA_DEBUG_INIT_CAMERA_FAILED,
    CAMERA_DEBUG_INIT_WIFI_FAILED,
    CAMERA_DEBUG_INIT_SOCKET_FAILED,
    CAMERA_DEBUG_INIT_OK
} camera_debug_init_state_enum;

typedef struct
{
    uint32 frame_count;
    uint32 snapshot_count;
    uint32 sent_count;
    uint32 dropped_count;
    uint32 timeout_count;
    uint32 last_frame_ms;
    uint32 last_send_ms;
    uint32 last_send_duration_ms;
    uint32 max_send_duration_ms;
    uint8 frame_valid;
    uint8 init_state;
} camera_debug_diag_struct;

extern camera_debug_diag_struct camera_debug_diag;

uint8 camera_debug_app_init(void);
void camera_debug_app_tick_1ms(void);
void camera_debug_app_service(void);
uint32 camera_debug_app_now_ms(void);
const camera_debug_diag_struct *camera_debug_app_get_diag(void);

#endif

/*********************************************************************************************************************
* File: camera_frame_consumer.h
* Description: CM7_1 no-WiFi camera frame consumer service.
********************************************************************************************************************/

#ifndef _camera_frame_consumer_h_
#define _camera_frame_consumer_h_

#include "zf_common_typedef.h"

typedef enum
{
    CAMERA_CONSUMER_INIT_NOT_RUN = 0,
    CAMERA_CONSUMER_INIT_HANDOFF_FAILED,
    CAMERA_CONSUMER_INIT_WIFI_FAILED,
    CAMERA_CONSUMER_INIT_SOCKET_FAILED,
    CAMERA_CONSUMER_INIT_OK
} camera_consumer_init_state_enum;

typedef struct
{
    uint32 acquired_count;
    uint32 released_count;
    uint32 stale_ready_drop_count;
    uint32 invalid_count;
    uint32 timeout_count;
    uint32 last_sequence;
    uint32 last_capture_ms;
    uint32 last_frame_age_ms;
    uint8 sample_0_0;
    uint8 sample_center;
    uint8 frame_valid;
    uint8 init_state;
} camera_frame_consumer_diag_struct;

uint8 camera_frame_consumer_init(void);
void camera_frame_consumer_tick_1ms(void);
void camera_frame_consumer_service(void);
uint32 camera_frame_consumer_now_ms(void);
const camera_frame_consumer_diag_struct *camera_frame_consumer_get_diag(void);

#endif

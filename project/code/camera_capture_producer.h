/*********************************************************************************************************************
* File: camera_capture_producer.h
* Description: CM7_0 MT9V03X capture and inter-core publication service.
********************************************************************************************************************/

#ifndef _camera_capture_producer_h_
#define _camera_capture_producer_h_

#include "zf_common_typedef.h"

typedef enum
{
    CAMERA_CAPTURE_INIT_NOT_RUN = 0,
    CAMERA_CAPTURE_INIT_HANDOFF_FAILED,
    CAMERA_CAPTURE_INIT_CAMERA_FAILED,
    CAMERA_CAPTURE_INIT_OK
} camera_capture_init_state_enum;

typedef struct
{
    uint32 frame_count;
    uint32 publish_count;
    uint32 period_drop_count;
    uint32 no_free_drop_count;
    uint32 invalid_count;
    uint32 timeout_count;
    uint32 last_frame_ms;
    uint32 last_publish_ms;
    uint32 last_copy_duration_us;
    uint32 max_copy_duration_us;
    uint8 init_state;
    uint8 frame_valid;
} camera_capture_producer_diag_struct;

uint8 camera_capture_producer_init(void);
void camera_capture_producer_service(void);
const camera_capture_producer_diag_struct *camera_capture_producer_get_diag(void);

#endif

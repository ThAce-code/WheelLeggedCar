/*********************************************************************************************************************
* File: camera_frame_consumer.h
* Description: CM7_1 camera vision boundary and WiFi-SPI display service.
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

typedef enum
{
    CAMERA_CONSUMER_LINK_NOT_RUN = 0,
    CAMERA_CONSUMER_LINK_CONNECTING,
    CAMERA_CONSUMER_LINK_CONNECTED,
    CAMERA_CONSUMER_LINK_FAILED
} camera_consumer_link_state_enum;

typedef struct
{
    uint8 slot_index;
    uint32 sequence;
    uint32 capture_ms;
    uint32 frame_age_ms;
    uint16 width;
    uint16 height;
    uint16 stride;
    uint32 frame_bytes;
    const volatile uint8 *pixels;
} camera_vision_frame_view_struct;

typedef struct
{
    uint32 le_25_ms;
    uint32 le_50_ms;
    uint32 le_100_ms;
    uint32 le_200_ms;
    uint32 le_500_ms;
    uint32 le_1000_ms;
    uint32 gt_1000_ms;
} camera_send_duration_histogram_struct;

typedef struct
{
    uint32 acquired_count;
    uint32 released_count;
    uint32 stale_ready_drop_count;
    uint32 invalid_count;
    uint32 timeout_count;
    uint32 sent_count;
    uint32 send_failure_count;
    uint32 reconnect_count;
    uint32 stale_count;
    uint32 last_sent_sequence;
    uint32 last_reconnect_ms;
    uint32 last_send_duration_ms;
    uint32 max_send_duration_ms;
    uint32 last_process_duration_us;
    uint32 max_process_duration_us;
    camera_send_duration_histogram_struct startup_send_histogram;
    camera_send_duration_histogram_struct steady_send_histogram;
    uint32 last_sequence;
    uint32 last_capture_ms;
    uint32 last_frame_age_ms;
    uint8 sample_0_0;
    uint8 sample_center;
    uint8 frame_valid;
    uint8 init_state;
    uint8 wifi_state;
    uint8 socket_state;
    uint8 handoff_state;
} camera_frame_consumer_diag_struct;

typedef struct
{
    uint32 generation;
    uint32 consumer_ms;
    uint32 producer_heartbeat_ms;
    uint32 producer_last_publish_ms;
    uint32 captured_count;
    uint32 latest_published_sequence;
    uint32 published_count;
    uint32 notify_count;
    uint32 no_free_drop_count;
    uint32 period_drop_count;
    uint32 acquired_count;
    uint32 sent_count;
    uint32 released_count;
    uint32 reconnect_count;
    uint32 invalid_count;
    uint32 timeout_count;
    uint32 stale_count;
    uint32 send_failure_count;
    uint32 slot_state[2];
    uint32 slot_sequence[2];
    camera_send_duration_histogram_struct startup_send_histogram;
    camera_send_duration_histogram_struct steady_send_histogram;
} camera_gate_snapshot_struct;

typedef struct
{
    uint32 generation;
    uint32 complete_count;
    camera_gate_snapshot_struct start;
    camera_gate_snapshot_struct end;
} camera_gate_evidence_struct;

extern volatile camera_gate_snapshot_struct camera_gate_snapshot;
extern volatile camera_gate_evidence_struct camera_gate_evidence;

uint8 camera_frame_consumer_init(void);
void camera_frame_consumer_tick_1ms(void);
void camera_frame_consumer_service(void);
uint32 camera_frame_consumer_now_ms(void);
const camera_frame_consumer_diag_struct *camera_frame_consumer_get_diag(void);

#endif

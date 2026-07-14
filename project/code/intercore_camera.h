#ifndef _intercore_camera_h_
#define _intercore_camera_h_

#include "intercore_transport.h"

typedef enum
{
    INTERCORE_CAMERA_SLOT_FREE = 0,
    INTERCORE_CAMERA_SLOT_WRITING = 1,
    INTERCORE_CAMERA_SLOT_READY = 2,
    INTERCORE_CAMERA_SLOT_READING = 3
}intercore_camera_slot_state_enum;

typedef enum
{
    INTERCORE_CAMERA_NO_FRAME = 0,
    INTERCORE_CAMERA_OK = 1,
    INTERCORE_CAMERA_INVALID = 2,
    INTERCORE_CAMERA_NO_FREE_SLOT = 3,
    INTERCORE_CAMERA_EPOCH_CHANGED = 4
}intercore_camera_result_enum;

typedef struct
{
    volatile intercore_camera_control_struct *control;
    volatile uint8 *data_plane;
    uint32 boot_epoch;
    uint32 last_consumed_sequence;
    uint8 role;
    uint8 attached;
    volatile intercore_shared_layout_struct *shared;
}intercore_camera_transport_struct;

typedef struct
{
    uint8 slot_index;
    uint32 sequence;
    uint32 capture_ms;
    uint32 publish_ms;
    uint16 width;
    uint16 height;
    uint16 stride;
    uint32 frame_bytes;
    volatile uint8 *pixels;
}intercore_camera_frame_view_struct;

uint8 intercore_camera_producer_init(intercore_camera_transport_struct *transport,
                                     volatile intercore_shared_layout_struct *shared,
                                     volatile uint8 *data_plane,
                                     uint32 boot_epoch);
uint8 intercore_camera_consumer_attach(intercore_camera_transport_struct *transport,
                                       volatile intercore_shared_layout_struct *shared,
                                       volatile uint8 *data_plane,
                                       uint32 boot_epoch);
void intercore_camera_producer_record_capture(intercore_camera_transport_struct *transport,
                                              uint32 capture_ms);
intercore_camera_result_enum intercore_camera_producer_claim(
    intercore_camera_transport_struct *transport,
    uint8 *slot_index,
    volatile uint8 **pixels);
uint8 intercore_camera_producer_publish(intercore_camera_transport_struct *transport,
                                        uint8 slot_index,
                                        uint32 capture_ms,
                                        uint32 publish_ms);
uint8 intercore_camera_producer_abort(intercore_camera_transport_struct *transport,
                                      uint8 slot_index);
intercore_camera_result_enum intercore_camera_consumer_acquire_latest(
    intercore_camera_transport_struct *transport,
    intercore_camera_frame_view_struct *view);
uint8 intercore_camera_consumer_release(intercore_camera_transport_struct *transport,
                                        const intercore_camera_frame_view_struct *view);
uint8 intercore_camera_consumer_release_at(
    intercore_camera_transport_struct *transport,
    const intercore_camera_frame_view_struct *view,
    uint32 consumer_ms);
uint32 intercore_camera_frame_age_ms(uint32 producer_ms, uint32 capture_ms);

#endif

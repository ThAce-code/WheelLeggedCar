#ifndef _intercore_protocol_h_
#define _intercore_protocol_h_

#include <stddef.h>
#include "zf_common_typedef.h"

#define INTERCORE_SHARED_BASE_ADDRESS      (0x28080000UL)
#define INTERCORE_SHARED_SIZE_BYTES        (8192U)
#define INTERCORE_PROTOCOL_MAGIC           (0x574C4349UL)
#define INTERCORE_PROTOCOL_VERSION         (2U)
#define INTERCORE_NAVIGATION_MAX_VALID_MS  (200U)
#define INTERCORE_CAMERA_DATA_BASE_ADDRESS (0x28060000UL)
#define INTERCORE_CAMERA_DATA_SIZE_BYTES   (65536U)
#define INTERCORE_CAMERA_SLOT_COUNT        (2U)
#define INTERCORE_CAMERA_SLOT_SIZE_BYTES   (22560U)
#define INTERCORE_CAMERA_MAGIC             (0x43414D52UL)
#define INTERCORE_CAMERA_VERSION           (2U)
#define INTERCORE_CAMERA_FORMAT_GRAY8      (1U)
#define INTERCORE_CAMERA_WIDTH             (188U)
#define INTERCORE_CAMERA_HEIGHT            (120U)
#define INTERCORE_CAMERA_STRIDE            (188U)

typedef enum
{
    INTERCORE_RECORD_NAVIGATION = 1,
    INTERCORE_RECORD_CONTROL_STATUS = 2
}intercore_record_type_enum;

typedef enum
{
    NAVIGATION_SOURCE_NONE = 0,
    NAVIGATION_SOURCE_WIRELESS_MANUAL = 1,
    NAVIGATION_SOURCE_VISION = 2,
    NAVIGATION_SOURCE_WAYPOINT = 3
}navigation_source_enum;

typedef enum
{
    NAVIGATION_MODE_DISARMED = 0,
    NAVIGATION_MODE_MANUAL = 1,
    NAVIGATION_MODE_VISION_ASSIST = 2,
    NAVIGATION_MODE_WAYPOINT = 3
}navigation_mode_enum;

typedef enum
{
    NAVIGATION_STOP_NONE = 0,
    NAVIGATION_STOP_DISABLED = 1,
    NAVIGATION_STOP_STALE = 2,
    NAVIGATION_STOP_EMERGENCY = 3,
    NAVIGATION_STOP_INVALID = 4
}navigation_stop_reason_enum;

typedef struct
{
    uint32 magic;
    uint16 version;
    uint16 type;
    uint32 size;
    uint32 sequence;
    uint32 source_ms;
    uint32 crc32;
}intercore_header_struct;

typedef struct
{
    float forward_rpm;
    float turn_rate_dps;
    float confidence;
    uint32 source_sequence;
    uint16 valid_for_ms;
    uint8 enable;
    uint8 source;
    uint8 mode;
    uint8 stop_reason;
    uint8 reserved[2];
}navigation_command_struct;

typedef struct
{
    uint32 magic;
    uint16 version;
    uint16 layout_size;
    uint32 boot_epoch;
    uint32 cm7_0_ready;
    uint32 cm7_1_ready;
    uint32 navigation_active_index;
    uint32 navigation_sequence;
    uint32 control_active_index;
    uint32 control_sequence;
    uint32 event_write_index;
    uint32 event_sequence;
    uint8 reserved[212];
}intercore_metadata_struct;

typedef struct
{
    uint32 timestamp_ms;
    float pitch_deg;
    float pitch_rate_dps;
    float left_wheel_rpm;
    float right_wheel_rpm;
    uint32 command_age_ms;
    uint32 scheduler_missed_tick_count;
    uint32 scheduler_max_gap_ms;
    uint8 app_state;
    uint8 safety_fault;
    uint8 active_motion_source;
    uint8 reserved0;
    uint8 reserved[28];
}control_status_struct;

typedef struct
{
    intercore_header_struct header;
    navigation_command_struct payload;
    uint8 reserved[208];
}intercore_navigation_slot_struct;

typedef struct
{
    intercore_header_struct header;
    control_status_struct payload;
    uint8 reserved[424];
}intercore_control_slot_struct;

typedef struct
{
    uint32 sequence;
    uint16 type;
    uint16 size;
    uint32 source_ms;
    uint8 data[48];
    uint32 crc32;
}intercore_event_struct;

typedef struct
{
    uint32 cm7_0_heartbeat_ms;
    uint32 cm7_1_heartbeat_ms;
    uint32 cm7_0_publish_count;
    uint32 cm7_1_publish_count;
    uint32 cm7_0_consume_count;
    uint32 cm7_1_consume_count;
    uint32 notify_success_count;
    uint32 notify_busy_count;
    uint32 crc_error_count;
    uint32 version_error_count;
    uint32 duplicate_count;
    uint32 boot_epoch_change_count;
    uint8 reserved[208];
}intercore_health_struct;

typedef struct
{
    uint32 state;
    uint32 sequence;
    uint32 capture_ms;
    uint32 publish_ms;
    uint32 frame_bytes;
    uint32 reserved[3];
}intercore_camera_slot_struct;

typedef struct
{
    uint32 magic;
    uint16 version;
    uint16 format;
    uint16 width;
    uint16 height;
    uint16 stride;
    uint16 slot_count;
    uint32 frame_bytes;
    uint32 producer_boot_epoch;
    uint32 capture_sequence;
    uint32 latest_published_sequence;
    intercore_camera_slot_struct slot[2];
    uint32 captured_count;
    uint32 published_count;
    uint32 no_free_drop_count;
    uint32 stale_ready_drop_count;
    uint32 consumed_count;
    uint32 invalid_layout_count;
    uint32 timeout_count;
    uint32 last_capture_ms;
    uint32 last_publish_ms;
    uint32 last_consume_ms;
    uint32 producer_heartbeat_ms;
    uint32 consumer_heartbeat_ms;
    uint32 last_copy_duration_us;
    uint32 max_copy_duration_us;
    uint32 last_send_duration_ms;
    uint32 max_send_duration_ms;
    uint32 notify_count;
    uint32 last_process_duration_us;
    uint32 max_process_duration_us;
    uint32 consumer_last_sequence;
    uint32 consumer_last_frame_age_ms;
    uint8 consumer_sample_0_0;
    uint8 consumer_sample_center;
    uint8 consumer_frame_valid;
    uint8 consumer_reserved;
    uint8 reserved[72];
}intercore_camera_control_struct;

typedef struct
{
    intercore_metadata_struct metadata;
    intercore_navigation_slot_struct navigation[2];
    intercore_control_slot_struct control[2];
    intercore_event_struct events[16];
    intercore_health_struct health;
    intercore_camera_control_struct camera;
    uint8 reserved[4864];
}intercore_shared_layout_struct;

#define INTERCORE_LAYOUT_CHECK(name, condition) \
    typedef char intercore_layout_check_##name[(condition) ? 1 : -1]

INTERCORE_LAYOUT_CHECK(header_size, sizeof(intercore_header_struct) == 24U);
INTERCORE_LAYOUT_CHECK(navigation_command_size, sizeof(navigation_command_struct) == 24U);
INTERCORE_LAYOUT_CHECK(metadata_size, sizeof(intercore_metadata_struct) == 256U);
INTERCORE_LAYOUT_CHECK(control_status_size, sizeof(control_status_struct) == 64U);
INTERCORE_LAYOUT_CHECK(navigation_slot_size, sizeof(intercore_navigation_slot_struct) == 256U);
INTERCORE_LAYOUT_CHECK(control_slot_size, sizeof(intercore_control_slot_struct) == 512U);
INTERCORE_LAYOUT_CHECK(event_size, sizeof(intercore_event_struct) == 64U);
INTERCORE_LAYOUT_CHECK(health_size, sizeof(intercore_health_struct) == 256U);
INTERCORE_LAYOUT_CHECK(camera_slot_size, sizeof(intercore_camera_slot_struct) == 32U);
INTERCORE_LAYOUT_CHECK(camera_control_size, sizeof(intercore_camera_control_struct) == 256U);
INTERCORE_LAYOUT_CHECK(camera_consumer_sequence_offset,
                       offsetof(intercore_camera_control_struct,
                                consumer_last_sequence) == 172U);
INTERCORE_LAYOUT_CHECK(camera_consumer_observation_offset,
                       offsetof(intercore_camera_control_struct,
                                consumer_last_frame_age_ms) == 176U);
INTERCORE_LAYOUT_CHECK(shared_size, sizeof(intercore_shared_layout_struct) == 8192U);
INTERCORE_LAYOUT_CHECK(metadata_offset, offsetof(intercore_shared_layout_struct, metadata) == 0x000U);
INTERCORE_LAYOUT_CHECK(navigation_offset, offsetof(intercore_shared_layout_struct, navigation) == 0x100U);
INTERCORE_LAYOUT_CHECK(control_offset, offsetof(intercore_shared_layout_struct, control) == 0x300U);
INTERCORE_LAYOUT_CHECK(events_offset, offsetof(intercore_shared_layout_struct, events) == 0x700U);
INTERCORE_LAYOUT_CHECK(health_offset, offsetof(intercore_shared_layout_struct, health) == 0xB00U);
INTERCORE_LAYOUT_CHECK(camera_offset, offsetof(intercore_shared_layout_struct, camera) == 0xC00U);
INTERCORE_LAYOUT_CHECK(reserved_offset, offsetof(intercore_shared_layout_struct, reserved) == 0xD00U);

uint32 intercore_crc32(const uint8 *data, uint32 size);
void intercore_record_prepare(intercore_header_struct *header,
                              intercore_record_type_enum type,
                              uint32 sequence,
                              uint32 source_ms,
                              const void *payload,
                              uint32 payload_size);
uint8 intercore_record_validate(const intercore_header_struct *header,
                                intercore_record_type_enum expected_type,
                                const void *payload,
                                uint32 expected_payload_size);
uint8 intercore_navigation_is_structurally_valid(
    const navigation_command_struct *command);

#endif

#ifndef _motion_command_router_h_
#define _motion_command_router_h_

#include "zf_common_typedef.h"

typedef enum
{
    MOTION_SOURCE_NONE = 0U,
    MOTION_SOURCE_AUTONOMOUS = 1U,
    MOTION_SOURCE_WIRELESS_MANUAL = 2U,
    MOTION_SOURCE_UART_LOCAL = 3U
}motion_source_enum;

typedef enum
{
    MOTION_STOP_NONE = 0U,
    MOTION_STOP_DISABLED = 1U,
    MOTION_STOP_STALE = 2U,
    MOTION_STOP_EMERGENCY = 3U,
    MOTION_STOP_MAINTENANCE = 4U,
    MOTION_STOP_SAFETY = 5U
}motion_stop_reason_enum;

typedef struct
{
    float forward_rpm;
    float turn_rate_dps;
    uint32 source_sequence;
    uint32 received_ms;
    uint16 valid_for_ms;
    uint8 enable;
}motion_command_request_struct;

typedef struct
{
    motion_source_enum active_source;
    motion_stop_reason_enum stop_reason;
    uint8 remote_armed;
    uint8 maintenance_mode;
    uint8 emergency_stop_latched;
    uint8 safety_fault;
    uint32 rejected_count;
    uint32 expired_count;
}motion_router_diag_struct;

void motion_command_router_init(void);
uint8 motion_command_router_arm_remote(uint32 now_ms, uint8 arm);
uint8 motion_command_router_submit(motion_source_enum source,
                                   const motion_command_request_struct *request);
void motion_command_router_update(uint32 now_ms, uint8 safety_fault);
uint8 motion_command_router_set_maintenance(uint8 enabled, uint32 now_ms);
uint8 motion_command_router_set_maintenance_mode(uint8 enabled, uint32 now_ms);
uint8 motion_command_router_latch_emergency_stop(uint32 now_ms);
uint8 motion_command_router_clear_emergency_stop(uint32 now_ms);
const motion_router_diag_struct *motion_command_router_get_diag(void);

#endif

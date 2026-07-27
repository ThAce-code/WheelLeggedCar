#include <string.h>
#include "motion_command_router.h"
#include "motion_command_router_port.h"

#define MOTION_ROUTER_SOURCE_COUNT       (4U)
#define MOTION_ROUTER_MAX_VALID_MS       (500U)
#define MOTION_ROUTER_MAX_REMOTE_VALID_MS (200U)

typedef struct
{
    motion_command_request_struct request;
    uint8 valid;
    uint32 last_sequence;
}motion_router_slot_struct;

static motion_router_slot_struct motion_slots[MOTION_ROUTER_SOURCE_COUNT];
static motion_router_diag_struct motion_router_diag;
static uint8 motion_router_safety_fault;
static uint8 motion_router_output_active;

static uint8 motion_request_is_finite(float value)
{
    return ((value == value) &&
            (3.402823466e+38F >= value) &&
            (-3.402823466e+38F <= value)) ? 1U : 0U;
}

static uint8 motion_request_is_valid(const motion_command_request_struct *request,
                                     motion_source_enum source)
{
    if(NULL == request)
    {
        return 0U;
    }
    if((0U == motion_request_is_finite(request->forward_rpm)) ||
       (0U == motion_request_is_finite(request->turn_rate_dps)) ||
       (0U == request->source_sequence) ||
       (1U > request->valid_for_ms) ||
       (MOTION_ROUTER_MAX_VALID_MS < request->valid_for_ms) ||
       ((0U != request->enable) && (1U != request->enable)))
    {
        return 0U;
    }
    if(((MOTION_SOURCE_AUTONOMOUS == source) ||
        (MOTION_SOURCE_WIRELESS_MANUAL == source)) &&
       (MOTION_ROUTER_MAX_REMOTE_VALID_MS < request->valid_for_ms))
    {
        return 0U;
    }
    return 1U;
}

static void motion_router_clear_slots(void)
{
    motion_slots[MOTION_SOURCE_AUTONOMOUS].valid = 0U;
    motion_slots[MOTION_SOURCE_WIRELESS_MANUAL].valid = 0U;
    motion_slots[MOTION_SOURCE_UART_LOCAL].valid = 0U;
}

static void motion_router_stop(uint32 now_ms, motion_stop_reason_enum reason)
{
    motion_command_router_port_stop(now_ms);
    motion_router_output_active = 0U;
    motion_router_diag.active_source = MOTION_SOURCE_NONE;
    motion_router_diag.stop_reason = reason;
}

void motion_command_router_init(void)
{
    memset(motion_slots, 0, sizeof(motion_slots));
    memset(&motion_router_diag, 0, sizeof(motion_router_diag));
    motion_router_safety_fault = 0U;
    motion_router_output_active = 0U;
    motion_router_diag.stop_reason = MOTION_STOP_DISABLED;
}

uint8 motion_command_router_arm_remote(uint32 now_ms, uint8 arm)
{
    (void)now_ms;
    if((0U != arm) ||
       (0U != motion_router_safety_fault) ||
       (0U != motion_router_diag.maintenance_mode) ||
       (0U != motion_router_diag.emergency_stop_latched))
    {
        return 0U;
    }
    motion_router_diag.remote_armed = 1U;
    return 1U;
}

uint8 motion_command_router_submit(motion_source_enum source,
                                   const motion_command_request_struct *request)
{
    motion_router_slot_struct *slot;

    if((MOTION_SOURCE_AUTONOMOUS > source) ||
       (MOTION_SOURCE_UART_LOCAL < source) ||
       (0U == motion_request_is_valid(request, source)))
    {
        motion_router_diag.rejected_count++;
        return 0U;
    }
    if(0U != motion_router_safety_fault)
    {
        motion_router_diag.rejected_count++;
        return 0U;
    }
    if(0U == request->enable)
    {
        motion_slots[source].valid = 0U;
        if((motion_router_diag.active_source == source) &&
           (0U != motion_router_output_active))
        {
            motion_router_stop(request->received_ms, MOTION_STOP_DISABLED);
        }
        return 1U;
    }
    if((MOTION_SOURCE_UART_LOCAL != source) &&
       ((0U == motion_router_diag.remote_armed) ||
        (0U != motion_router_diag.maintenance_mode) ||
        (0U != motion_router_diag.emergency_stop_latched)))
    {
        motion_router_diag.rejected_count++;
        return 0U;
    }
    if((MOTION_SOURCE_UART_LOCAL == source) &&
       ((0U != motion_router_diag.maintenance_mode) ||
        (0U != motion_router_diag.emergency_stop_latched)))
    {
        motion_router_diag.rejected_count++;
        return 0U;
    }
    slot = &motion_slots[source];
    if((0U != slot->last_sequence) &&
       (0 >= (int32)(request->source_sequence - slot->last_sequence)))
    {
        motion_router_diag.rejected_count++;
        return 0U;
    }
    slot->request = *request;
    slot->last_sequence = request->source_sequence;
    slot->valid = 1U;
    return 1U;
}

void motion_command_router_cancel_source(motion_source_enum source, uint32 now_ms)
{
    if((MOTION_SOURCE_AUTONOMOUS > source) ||
       (MOTION_SOURCE_UART_LOCAL < source))
    {
        return;
    }
    motion_slots[source].valid = 0U;
    if((motion_router_diag.active_source == source) &&
       (0U != motion_router_output_active))
    {
        motion_router_stop(now_ms, MOTION_STOP_DISABLED);
    }
}

void motion_command_router_update(uint32 now_ms, uint8 safety_fault)
{
    motion_source_enum selected_source = MOTION_SOURCE_NONE;
    uint32 age;
    uint32 source;

    motion_router_safety_fault = (0U != safety_fault) ? 1U : 0U;
    motion_router_diag.safety_fault = motion_router_safety_fault;
    if(0U != motion_router_safety_fault)
    {
        motion_router_diag.remote_armed = 0U;
        motion_router_clear_slots();
        motion_router_stop(now_ms, MOTION_STOP_SAFETY);
        return;
    }
    if(0U != motion_router_diag.emergency_stop_latched)
    {
        motion_router_stop(now_ms, MOTION_STOP_EMERGENCY);
        return;
    }
    for(source = MOTION_SOURCE_AUTONOMOUS;
        source <= MOTION_SOURCE_UART_LOCAL;
        source++)
    {
        if(0U != motion_slots[source].valid)
        {
            age = now_ms - motion_slots[source].request.received_ms;
            if(age >= motion_slots[source].request.valid_for_ms)
            {
                motion_slots[source].valid = 0U;
                motion_router_diag.expired_count++;
            }
        }
    }
    if(0U != motion_router_diag.maintenance_mode)
    {
        motion_router_clear_slots();
        motion_router_stop(now_ms, MOTION_STOP_MAINTENANCE);
        return;
    }
    if(1U == motion_slots[MOTION_SOURCE_UART_LOCAL].valid)
    {
        selected_source = MOTION_SOURCE_UART_LOCAL;
    }
    else if(1U == motion_slots[MOTION_SOURCE_WIRELESS_MANUAL].valid)
    {
        selected_source = MOTION_SOURCE_WIRELESS_MANUAL;
    }
    else if(1U == motion_slots[MOTION_SOURCE_AUTONOMOUS].valid)
    {
        selected_source = MOTION_SOURCE_AUTONOMOUS;
    }
    if(MOTION_SOURCE_NONE != selected_source)
    {
        motion_command_router_port_apply(
            motion_slots[selected_source].request.forward_rpm,
            motion_slots[selected_source].request.turn_rate_dps,
            motion_slots[selected_source].request.enable,
            now_ms);
        motion_router_output_active = 1U;
        motion_router_diag.active_source = selected_source;
        motion_router_diag.stop_reason = MOTION_STOP_NONE;
    }
    else if(0U != motion_router_output_active)
    {
        motion_router_stop(now_ms, MOTION_STOP_STALE);
    }
    else
    {
        motion_router_diag.active_source = MOTION_SOURCE_NONE;
        motion_router_diag.stop_reason = MOTION_STOP_STALE;
    }
}

uint8 motion_command_router_set_maintenance(uint8 enabled, uint32 now_ms)
{
    uint8 new_state = (0U != enabled) ? 1U : 0U;
    motion_router_diag.maintenance_mode = new_state;
    motion_router_diag.remote_armed = 0U;
    motion_router_clear_slots();
    motion_router_stop(now_ms, (0U != new_state) ?
                       MOTION_STOP_MAINTENANCE : MOTION_STOP_DISABLED);
    return 1U;
}

uint8 motion_command_router_set_maintenance_mode(uint8 enabled, uint32 now_ms)
{
    return motion_command_router_set_maintenance(enabled, now_ms);
}

uint8 motion_command_router_latch_emergency_stop(uint32 now_ms)
{
    motion_router_diag.emergency_stop_latched = 1U;
    motion_router_diag.remote_armed = 0U;
    motion_router_clear_slots();
    motion_router_stop(now_ms, MOTION_STOP_EMERGENCY);
    return 1U;
}

uint8 motion_command_router_clear_emergency_stop(uint32 now_ms)
{
    if(0U != motion_router_safety_fault)
    {
        return 0U;
    }
    motion_router_diag.emergency_stop_latched = 0U;
    motion_router_stop(now_ms, MOTION_STOP_DISABLED);
    return 1U;
}

const motion_router_diag_struct *motion_command_router_get_diag(void)
{
    return &motion_router_diag;
}

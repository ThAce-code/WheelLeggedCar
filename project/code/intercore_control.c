#include "intercore_control.h"
#include "intercore_memory.h"
#include "intercore_notify.h"
#include "intercore_transport.h"
#include "motion_command_router.h"

#if defined(INTERCORE_HOST_TEST)
#define APP_INTERCORE_COMMAND_TIMEOUT_MS  (200U)
#define APP_INTERCORE_FORWARD_LIMIT_RPM   (60.0f)
#define APP_INTERCORE_TURN_LIMIT_DPS      (60.0f)
#else
#include "app_config.h"
#endif

static intercore_transport_struct intercore_transport;
static volatile intercore_shared_layout_struct *intercore_shared;
static uint32 intercore_last_source_sequence[4U];
static uint32 intercore_rejected_count;

static uint8 intercore_is_finite(float value)
{
    return ((value == value) &&
            (3.402823466e+38F >= value) &&
            (-3.402823466e+38F <= value)) ? 1U : 0U;
}

static motion_source_enum intercore_map_source(uint8 source)
{
    if(NAVIGATION_SOURCE_WIRELESS_MANUAL == source)
    {
        return MOTION_SOURCE_WIRELESS_MANUAL;
    }
    if((NAVIGATION_SOURCE_VISION == source) ||
       (NAVIGATION_SOURCE_WAYPOINT == source))
    {
        return MOTION_SOURCE_AUTONOMOUS;
    }
    return MOTION_SOURCE_NONE;
}

uint8 intercore_control_accept_navigation(const navigation_command_struct *command,
                                          uint32 now_ms)
{
    motion_source_enum motion_source;
    motion_command_request_struct request;

    if((NULL == command) ||
       (0U == intercore_navigation_is_structurally_valid(command)) ||
       (0U == intercore_is_finite(command->forward_rpm)) ||
       (0U == intercore_is_finite(command->turn_rate_dps)) ||
       (APP_INTERCORE_FORWARD_LIMIT_RPM < command->forward_rpm) ||
       (-APP_INTERCORE_FORWARD_LIMIT_RPM > command->forward_rpm) ||
       (APP_INTERCORE_TURN_LIMIT_DPS < command->turn_rate_dps) ||
       (-APP_INTERCORE_TURN_LIMIT_DPS > command->turn_rate_dps) ||
       (NAVIGATION_SOURCE_WAYPOINT < command->source) ||
       (0U == command->source_sequence))
    {
        intercore_rejected_count++;
        return 0U;
    }
    if((0U != intercore_last_source_sequence[command->source]) &&
       (0 >= (int32)(command->source_sequence -
                     intercore_last_source_sequence[command->source])))
    {
        intercore_rejected_count++;
        return 0U;
    }

    motion_source = intercore_map_source(command->source);
    if(MOTION_SOURCE_NONE == motion_source)
    {
        intercore_rejected_count++;
        return 0U;
    }

    request.forward_rpm = command->forward_rpm;
    request.turn_rate_dps = command->turn_rate_dps;
    request.source_sequence = command->source_sequence;
    request.received_ms = now_ms;
    request.valid_for_ms = command->valid_for_ms;
    if(APP_INTERCORE_COMMAND_TIMEOUT_MS < request.valid_for_ms)
    {
        request.valid_for_ms = APP_INTERCORE_COMMAND_TIMEOUT_MS;
    }
    request.enable = command->enable;

    if(0U == request.enable)
    {
        motion_command_router_cancel_source(motion_source, now_ms);
        intercore_last_source_sequence[command->source] = command->source_sequence;
        return 1U;
    }
    if(0U == motion_command_router_submit(motion_source, &request))
    {
        intercore_rejected_count++;
        return 0U;
    }
    intercore_last_source_sequence[command->source] = command->source_sequence;
    return 1U;
}

uint8 intercore_control_init(void)
{
    intercore_shared = intercore_memory_get_layout();
    if((NULL == intercore_shared) ||
       (0U == intercore_transport_cm7_0_init(&intercore_transport,
                                             intercore_shared)) ||
       (0U == intercore_notify_init(INTERCORE_ROLE_CM7_0)))
    {
        return 0U;
    }
    intercore_last_source_sequence[0U] = 0U;
    intercore_last_source_sequence[1U] = 0U;
    intercore_last_source_sequence[2U] = 0U;
    intercore_last_source_sequence[3U] = 0U;
    intercore_rejected_count = 0U;
    return 1U;
}

void intercore_control_update(uint32 now_ms)
{
    navigation_command_struct command;
    uint32 record_sequence = 0U;
    intercore_transport_result_enum result;

    result = intercore_transport_read_navigation(&intercore_transport,
                                                 &command,
                                                 &record_sequence);
    (void)record_sequence;
    if(INTERCORE_TRANSPORT_OK == result)
    {
        (void)intercore_control_accept_navigation(&command, now_ms);
    }
    else if(INTERCORE_TRANSPORT_EPOCH_CHANGED == result)
    {
        motion_command_router_cancel_source(MOTION_SOURCE_AUTONOMOUS, now_ms);
        motion_command_router_cancel_source(MOTION_SOURCE_WIRELESS_MANUAL, now_ms);
        intercore_last_source_sequence[1U] = 0U;
        intercore_last_source_sequence[2U] = 0U;
        intercore_last_source_sequence[3U] = 0U;
    }
    else if(INTERCORE_TRANSPORT_INVALID == result)
    {
        intercore_rejected_count++;
    }
}

#ifndef _intercore_transport_h_
#define _intercore_transport_h_

#include "intercore_protocol.h"

typedef enum
{
    INTERCORE_ROLE_CM7_0 = 0,
    INTERCORE_ROLE_CM7_1 = 1
}intercore_role_enum;

typedef enum
{
    INTERCORE_TRANSPORT_NO_DATA = 0,
    INTERCORE_TRANSPORT_OK = 1,
    INTERCORE_TRANSPORT_INVALID = 2,
    INTERCORE_TRANSPORT_EPOCH_CHANGED = 3
}intercore_transport_result_enum;

typedef struct
{
    volatile intercore_shared_layout_struct *shared;
    uint32 boot_epoch;
    uint32 last_navigation_sequence;
    uint8 role;
    uint8 attached;
}intercore_transport_struct;

uint8 intercore_transport_cm7_0_init(intercore_transport_struct *transport,
                                     volatile intercore_shared_layout_struct *shared);
uint8 intercore_transport_cm7_1_attach(intercore_transport_struct *transport,
                                       volatile intercore_shared_layout_struct *shared);
uint8 intercore_transport_publish_navigation(intercore_transport_struct *transport,
                                             const navigation_command_struct *command,
                                             uint32 source_ms);
intercore_transport_result_enum intercore_transport_read_navigation(
    intercore_transport_struct *transport,
    navigation_command_struct *command,
    uint32 *record_sequence);

#endif

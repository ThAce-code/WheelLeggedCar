#ifndef _intercore_notify_h_
#define _intercore_notify_h_

#include "intercore_transport.h"

#define INTERCORE_NOTIFY_CLIENT_ID       (0x31UL)
#define INTERCORE_NOTIFY_NAVIGATION      (0x00000001UL)
#define INTERCORE_NOTIFY_CONTROL_STATUS  (0x00000002UL)
#define INTERCORE_NOTIFY_HEARTBEAT       (0x00000004UL)

typedef struct
{
    uint32 clientId;
    uint32 data;
}intercore_doorbell_struct;

typedef struct
{
    uint32 success_count;
    uint32 busy_count;
    uint32 received_count;
    uint32 pending_bits;
    uint8 initialized;
    uint8 in_flight;
}intercore_notify_diag_struct;

uint8 intercore_notify_init(intercore_role_enum role);
uint8 intercore_notify_try(uint32 bits);
uint32 intercore_notify_take_pending(void);
void intercore_notify_release_callback(void);
void intercore_notify_receive_callback(uint32 bits);
const intercore_notify_diag_struct *intercore_notify_get_diag(void);

#endif

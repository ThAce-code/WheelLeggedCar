#ifndef _intercore_notify_h_
#define _intercore_notify_h_

#include "intercore_transport.h"

/* cy_ipc_pipe indexes its callback table directly; this must be < CY_IPC_PIPE_MAX_CLIENTS (8). */
#define INTERCORE_NOTIFY_CLIENT_ID       (3UL)
#define INTERCORE_NOTIFY_NAVIGATION      (0x00000001UL)
#define INTERCORE_NOTIFY_CONTROL_STATUS  (0x00000002UL)
#define INTERCORE_NOTIFY_HEARTBEAT       (0x00000004UL)
#define INTERCORE_NOTIFY_CAMERA_READY    (0x00000008UL)

typedef struct
{
    uint32 clientId;
    uint32 data;
}intercore_doorbell_struct;

/* Reserve the final 8 bytes of the documented SRAM1 shared window for the doorbell. */
#define INTERCORE_NOTIFY_MESSAGE_ADDRESS \
    (INTERCORE_SHARED_BASE_ADDRESS + INTERCORE_SHARED_SIZE_BYTES - sizeof(intercore_doorbell_struct))

typedef struct
{
    uint32 success_count;
    uint32 busy_count;
    uint32 received_count;
    uint32 pending_bits;
    uint32 pending_out_bits;
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

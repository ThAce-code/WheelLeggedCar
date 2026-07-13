#include "intercore_notify.h"
#include "intercore_notify_port.h"

static intercore_doorbell_struct intercore_notify_message;
static volatile intercore_notify_diag_struct intercore_notify_diag;

uint8 intercore_notify_init(intercore_role_enum role)
{
    intercore_notify_diag.success_count = 0U;
    intercore_notify_diag.busy_count = 0U;
    intercore_notify_diag.received_count = 0U;
    intercore_notify_diag.pending_bits = 0U;
    intercore_notify_diag.in_flight = 0U;
    intercore_notify_diag.initialized = intercore_notify_port_init(role);
    return intercore_notify_diag.initialized;
}

uint8 intercore_notify_try(uint32 bits)
{
    if(0U == intercore_notify_diag.initialized)
    {
        return 0U;
    }

    if(0U != intercore_notify_diag.in_flight)
    {
        intercore_notify_diag.busy_count++;
        return 0U;
    }

    intercore_notify_message.clientId = INTERCORE_NOTIFY_CLIENT_ID;
    intercore_notify_message.data = bits;
    intercore_notify_diag.in_flight = 1U;
    if(0U == intercore_notify_port_send(&intercore_notify_message))
    {
        intercore_notify_diag.in_flight = 0U;
        intercore_notify_diag.busy_count++;
        return 0U;
    }

    intercore_notify_diag.success_count++;
    return 1U;
}

uint32 intercore_notify_take_pending(void)
{
    uint32 pending_bits = intercore_notify_diag.pending_bits;

    intercore_notify_diag.pending_bits = 0U;
    return pending_bits;
}

void intercore_notify_release_callback(void)
{
    intercore_notify_diag.in_flight = 0U;
}

void intercore_notify_receive_callback(uint32 bits)
{
    intercore_notify_diag.pending_bits |= bits;
    intercore_notify_diag.received_count++;
}

const intercore_notify_diag_struct *intercore_notify_get_diag(void)
{
    return (const intercore_notify_diag_struct *)&intercore_notify_diag;
}

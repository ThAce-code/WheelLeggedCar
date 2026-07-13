#include "intercore_notify.h"
#include "intercore_notify_port.h"

#if defined(INTERCORE_HOST_TEST)
#include <stdatomic.h>
#define INTERCORE_NOTIFY_DMB() atomic_signal_fence(memory_order_seq_cst)
#define INTERCORE_NOTIFY_ENTER_CRITICAL() ((uint32)0U)
#define INTERCORE_NOTIFY_EXIT_CRITICAL(state) ((void)(state))
static volatile intercore_doorbell_struct intercore_notify_message;
#else
#include <stdint.h>
#include "cy_project.h"
#define INTERCORE_NOTIFY_DMB() __DMB()
static uint32 intercore_notify_enter_critical(void)
{
    uint32 primask = (uint32)__get_PRIMASK();

    __disable_irq();
    return primask;
}

static void intercore_notify_exit_critical(uint32 primask)
{
    if(0U == primask)
    {
        __enable_irq();
    }
}
#define INTERCORE_NOTIFY_ENTER_CRITICAL() intercore_notify_enter_critical()
#define INTERCORE_NOTIFY_EXIT_CRITICAL(state) intercore_notify_exit_critical(state)
#define INTERCORE_NOTIFY_MESSAGE \
    ((volatile intercore_doorbell_struct *)(uintptr_t)INTERCORE_NOTIFY_MESSAGE_ADDRESS)
#endif

#if defined(INTERCORE_HOST_TEST)
#define INTERCORE_NOTIFY_MESSAGE (&intercore_notify_message)
#endif

static volatile intercore_notify_diag_struct intercore_notify_diag;
static volatile uint8 intercore_notify_release_handoff_active;

static uint8 intercore_notify_send(uint32 bits)
{
    uint32 send_bits;
    uint32 critical_state;

    critical_state = INTERCORE_NOTIFY_ENTER_CRITICAL();
    if(0U != intercore_notify_diag.in_flight)
    {
        intercore_notify_diag.pending_out_bits |= bits;
        intercore_notify_diag.busy_count++;
        INTERCORE_NOTIFY_EXIT_CRITICAL(critical_state);
        return 0U;
    }

    send_bits = bits | intercore_notify_diag.pending_out_bits;
    intercore_notify_diag.pending_out_bits = 0U;
    INTERCORE_NOTIFY_MESSAGE->clientId = INTERCORE_NOTIFY_CLIENT_ID;
    INTERCORE_NOTIFY_MESSAGE->data = send_bits;
    INTERCORE_NOTIFY_DMB();
    intercore_notify_diag.in_flight = 1U;
    INTERCORE_NOTIFY_EXIT_CRITICAL(critical_state);

    if(0U == intercore_notify_port_send(
                  (const intercore_doorbell_struct *)INTERCORE_NOTIFY_MESSAGE))
    {
        critical_state = INTERCORE_NOTIFY_ENTER_CRITICAL();
        intercore_notify_diag.in_flight = 0U;
        intercore_notify_diag.pending_out_bits |= send_bits;
        intercore_notify_diag.busy_count++;
        INTERCORE_NOTIFY_EXIT_CRITICAL(critical_state);
        return 0U;
    }

    critical_state = INTERCORE_NOTIFY_ENTER_CRITICAL();
    intercore_notify_diag.success_count++;
    INTERCORE_NOTIFY_EXIT_CRITICAL(critical_state);
    return 1U;
}

uint8 intercore_notify_init(intercore_role_enum role)
{
    if((INTERCORE_ROLE_CM7_0 != role) &&
       (INTERCORE_ROLE_CM7_1 != role))
    {
        intercore_notify_diag.initialized = 0U;
        return 0U;
    }

    intercore_notify_diag.success_count = 0U;
    intercore_notify_diag.busy_count = 0U;
    intercore_notify_diag.received_count = 0U;
    intercore_notify_diag.pending_bits = 0U;
    intercore_notify_diag.pending_out_bits = 0U;
    intercore_notify_diag.in_flight = 0U;
    intercore_notify_release_handoff_active = 0U;
    intercore_notify_diag.initialized = intercore_notify_port_init(role);
    return intercore_notify_diag.initialized;
}

uint8 intercore_notify_try(uint32 bits)
{
    if(0U == intercore_notify_diag.initialized)
    {
        return 0U;
    }

    return intercore_notify_send(bits);
}

uint32 intercore_notify_take_pending(void)
{
    uint32 pending_bits;
    uint32 critical_state = INTERCORE_NOTIFY_ENTER_CRITICAL();

    pending_bits = intercore_notify_diag.pending_bits;
    intercore_notify_diag.pending_bits = 0U;
    INTERCORE_NOTIFY_EXIT_CRITICAL(critical_state);
    return pending_bits;
}

void intercore_notify_release_callback(void)
{
    uint32 pending_bits;
    uint32 critical_state = INTERCORE_NOTIFY_ENTER_CRITICAL();

    intercore_notify_diag.in_flight = 0U;
    if(0U != intercore_notify_release_handoff_active)
    {
        INTERCORE_NOTIFY_EXIT_CRITICAL(critical_state);
        return;
    }

    pending_bits = intercore_notify_diag.pending_out_bits;
    intercore_notify_diag.pending_out_bits = 0U;
    intercore_notify_release_handoff_active = 1U;
    INTERCORE_NOTIFY_EXIT_CRITICAL(critical_state);

    if(0U != pending_bits)
    {
        (void)intercore_notify_send(pending_bits);
    }

    critical_state = INTERCORE_NOTIFY_ENTER_CRITICAL();
    intercore_notify_release_handoff_active = 0U;
    INTERCORE_NOTIFY_EXIT_CRITICAL(critical_state);
}

void intercore_notify_receive_callback(uint32 bits)
{
    uint32 critical_state = INTERCORE_NOTIFY_ENTER_CRITICAL();

    intercore_notify_diag.pending_bits |= bits;
    intercore_notify_diag.received_count++;
    INTERCORE_NOTIFY_EXIT_CRITICAL(critical_state);
}

const intercore_notify_diag_struct *intercore_notify_get_diag(void)
{
    return (const intercore_notify_diag_struct *)&intercore_notify_diag;
}

#include "intercore_transport.h"

#include <stdint.h>
#include <string.h>

#if defined(INTERCORE_HOST_TEST)
#include <stdatomic.h>
#define INTERCORE_DMB() atomic_signal_fence(memory_order_seq_cst)
#else
#include "cy_project.h"
#define INTERCORE_DMB() __DMB()
#endif

static uint8 intercore_shared_pointer_is_valid(
    const volatile intercore_shared_layout_struct *shared)
{
    return ((NULL != shared) &&
            (0U == ((uintptr_t)shared % 32U))) ? 1U : 0U;
}

static void intercore_copy_to_volatile(volatile uint8 *destination,
                                       const uint8 *source,
                                       uint32 size)
{
    uint32 index;

    for(index = 0U; index < size; index++)
    {
        destination[index] = source[index];
    }
}

static void intercore_copy_from_volatile(uint8 *destination,
                                         const volatile uint8 *source,
                                         uint32 size)
{
    uint32 index;

    for(index = 0U; index < size; index++)
    {
        destination[index] = source[index];
    }
}

static uint8 intercore_metadata_is_valid(
    const volatile intercore_shared_layout_struct *shared)
{
    return ((INTERCORE_PROTOCOL_MAGIC == shared->metadata.magic) &&
            (INTERCORE_PROTOCOL_VERSION == shared->metadata.version) &&
            (sizeof(intercore_shared_layout_struct) ==
             shared->metadata.layout_size) &&
            (1U == shared->metadata.cm7_0_ready)) ? 1U : 0U;
}

static uint32 intercore_next_sequence(uint32 sequence)
{
    sequence++;
    return (0U == sequence) ? 1U : sequence;
}

static void intercore_transport_accept_epoch(intercore_transport_struct *transport)
{
    transport->last_navigation_sequence = 0U;
    transport->last_gnss_sequence = 0U;
    transport->boot_epoch = transport->shared->metadata.boot_epoch;
}

uint8 intercore_transport_cm7_0_init(intercore_transport_struct *transport,
                                     volatile intercore_shared_layout_struct *shared)
{
    uint32 previous_epoch = 0U;
    uint32 index;
    volatile uint8 *bytes;

    if((NULL == transport) || (0U == intercore_shared_pointer_is_valid(shared)))
    {
        return 0U;
    }

    if((INTERCORE_PROTOCOL_MAGIC == shared->metadata.magic) &&
       (INTERCORE_PROTOCOL_VERSION == shared->metadata.version) &&
       (sizeof(intercore_shared_layout_struct) == shared->metadata.layout_size))
    {
        previous_epoch = shared->metadata.boot_epoch;
    }

    bytes = (volatile uint8 *)shared;
    for(index = 0U; index < sizeof(intercore_shared_layout_struct); index++)
    {
        bytes[index] = 0U;
    }

    previous_epoch = intercore_next_sequence(previous_epoch);
    shared->metadata.magic = INTERCORE_PROTOCOL_MAGIC;
    shared->metadata.version = INTERCORE_PROTOCOL_VERSION;
    shared->metadata.layout_size = (uint16)sizeof(intercore_shared_layout_struct);
    shared->metadata.boot_epoch = previous_epoch;
    shared->metadata.cm7_0_ready = 1U;
    INTERCORE_DMB();

    transport->shared = shared;
    transport->boot_epoch = previous_epoch;
    transport->last_navigation_sequence = 0U;
    transport->last_gnss_sequence = 0U;
    transport->role = INTERCORE_ROLE_CM7_0;
    transport->attached = 1U;
    return 1U;
}

uint8 intercore_transport_cm7_1_attach(intercore_transport_struct *transport,
                                       volatile intercore_shared_layout_struct *shared)
{
    if((NULL == transport) || (0U == intercore_shared_pointer_is_valid(shared)) ||
       (0U == intercore_metadata_is_valid(shared)))
    {
        return 0U;
    }

    transport->shared = shared;
    transport->boot_epoch = shared->metadata.boot_epoch;
    transport->last_navigation_sequence = 0U;
    transport->last_gnss_sequence = 0U;
    transport->role = INTERCORE_ROLE_CM7_1;
    transport->attached = 1U;
    INTERCORE_DMB();
    shared->metadata.cm7_1_ready = 1U;
    return 1U;
}

uint8 intercore_transport_publish_navigation(intercore_transport_struct *transport,
                                             const navigation_command_struct *command,
                                             uint32 source_ms)
{
    intercore_navigation_slot_struct local_slot = {0};
    uint32 inactive_index;

    if((NULL == transport) || (NULL == command) ||
       (INTERCORE_ROLE_CM7_1 != transport->role) ||
       (0U == transport->attached) || (NULL == transport->shared) ||
       (0U == intercore_metadata_is_valid(transport->shared)) ||
       (transport->boot_epoch != transport->shared->metadata.boot_epoch) ||
       (0U == intercore_navigation_is_structurally_valid(command)))
    {
        return 0U;
    }

    intercore_record_prepare(&local_slot.header,
                             INTERCORE_RECORD_NAVIGATION,
                             intercore_next_sequence(
                                 transport->shared->metadata.navigation_sequence),
                             source_ms,
                             command,
                             sizeof(*command));
    memcpy(&local_slot.payload, command, sizeof(*command));

    inactive_index = (1U == transport->shared->metadata.navigation_active_index) ? 0U : 1U;
    intercore_copy_to_volatile(
        (volatile uint8 *)&transport->shared->navigation[inactive_index],
        (const uint8 *)&local_slot,
        sizeof(local_slot));
    INTERCORE_DMB();
    transport->shared->metadata.navigation_active_index = inactive_index;
    transport->shared->metadata.navigation_sequence = local_slot.header.sequence;
    INTERCORE_DMB();
    transport->shared->health.cm7_1_publish_count++;
    return 1U;
}

intercore_transport_result_enum intercore_transport_read_navigation(
    intercore_transport_struct *transport,
    navigation_command_struct *command,
    uint32 *record_sequence)
{
    intercore_navigation_slot_struct local_slot;
    uint32 active_before;
    uint32 active_after;
    uint32 sequence_before;
    uint32 sequence_after;

    if((NULL == transport) || (NULL == command) || (NULL == record_sequence) ||
       (INTERCORE_ROLE_CM7_0 != transport->role) ||
       (0U == transport->attached) || (NULL == transport->shared))
    {
        return INTERCORE_TRANSPORT_INVALID;
    }
    if(0U == intercore_metadata_is_valid(transport->shared))
    {
        transport->shared->health.version_error_count++;
        return INTERCORE_TRANSPORT_INVALID;
    }
    if(transport->boot_epoch != transport->shared->metadata.boot_epoch)
    {
        transport->shared->health.boot_epoch_change_count++;
        intercore_transport_accept_epoch(transport);
        return INTERCORE_TRANSPORT_EPOCH_CHANGED;
    }

    active_before = transport->shared->metadata.navigation_active_index;
    sequence_before = transport->shared->metadata.navigation_sequence;
    if(1U < active_before)
    {
        return INTERCORE_TRANSPORT_INVALID;
    }
    if((0U == sequence_before) ||
       (sequence_before == transport->last_navigation_sequence))
    {
        return INTERCORE_TRANSPORT_NO_DATA;
    }

    intercore_copy_from_volatile((uint8 *)&local_slot,
                                 (const volatile uint8 *)&transport->shared->navigation[active_before],
                                 sizeof(local_slot));
    INTERCORE_DMB();
    active_after = transport->shared->metadata.navigation_active_index;
    sequence_after = transport->shared->metadata.navigation_sequence;
    if((active_before != active_after) || (sequence_before != sequence_after))
    {
        return INTERCORE_TRANSPORT_NO_DATA;
    }

    if(local_slot.header.sequence != sequence_before)
    {
        return INTERCORE_TRANSPORT_INVALID;
    }

    if((INTERCORE_PROTOCOL_MAGIC != local_slot.header.magic) ||
       (INTERCORE_PROTOCOL_VERSION != local_slot.header.version) ||
       ((uint16)INTERCORE_RECORD_NAVIGATION != local_slot.header.type) ||
       (sizeof(local_slot.payload) != local_slot.header.size))
    {
        transport->shared->health.version_error_count++;
        return INTERCORE_TRANSPORT_INVALID;
    }
    if(0U == intercore_record_validate(&local_slot.header,
                                       INTERCORE_RECORD_NAVIGATION,
                                       &local_slot.payload,
                                       sizeof(local_slot.payload)))
    {
        transport->shared->health.crc_error_count++;
        return INTERCORE_TRANSPORT_INVALID;
    }
    if(0U == intercore_navigation_is_structurally_valid(&local_slot.payload))
    {
        return INTERCORE_TRANSPORT_INVALID;
    }

    memcpy(command, &local_slot.payload, sizeof(*command));
    *record_sequence = local_slot.header.sequence;
    transport->last_navigation_sequence = local_slot.header.sequence;
    transport->shared->health.cm7_0_consume_count++;
    return INTERCORE_TRANSPORT_OK;
}

uint8 intercore_transport_publish_gnss(
    intercore_transport_struct *transport,
    const intercore_gnss_payload_struct *payload,
    uint32 source_ms)
{
    intercore_gnss_slot_struct local_slot = {0};
    uint32 inactive_index;

    if((NULL == transport) || (NULL == payload) ||
       (INTERCORE_ROLE_CM7_1 != transport->role) ||
       (0U == transport->attached) || (NULL == transport->shared) ||
       (0U == intercore_metadata_is_valid(transport->shared)) ||
       (transport->boot_epoch != transport->shared->metadata.boot_epoch))
    {
        return 0U;
    }

    intercore_record_prepare(&local_slot.header, INTERCORE_RECORD_GNSS,
                             intercore_next_sequence(transport->shared->metadata.gnss_sequence),
                             source_ms, payload, sizeof(*payload));
    memcpy(&local_slot.payload, payload, sizeof(*payload));
    inactive_index = (1U == transport->shared->metadata.gnss_active_index) ? 0U : 1U;
    intercore_copy_to_volatile((volatile uint8 *)&transport->shared->gnss[inactive_index],
                               (const uint8 *)&local_slot, sizeof(local_slot));
    INTERCORE_DMB();
    transport->shared->metadata.gnss_active_index = inactive_index;
    transport->shared->metadata.gnss_sequence = local_slot.header.sequence;
    INTERCORE_DMB();
    transport->shared->health.cm7_1_publish_count++;
    return 1U;
}

intercore_transport_result_enum intercore_transport_read_gnss(
    intercore_transport_struct *transport, intercore_gnss_payload_struct *payload,
    uint32 *source_ms, uint32 *record_sequence)
{
    intercore_gnss_slot_struct local_slot;
    uint32 active_before;
    uint32 active_after;
    uint32 sequence_before;
    uint32 sequence_after;

    if((NULL == transport) || (NULL == payload) || (NULL == source_ms) ||
       (NULL == record_sequence) || (INTERCORE_ROLE_CM7_0 != transport->role) ||
       (0U == transport->attached) || (NULL == transport->shared)) return INTERCORE_TRANSPORT_INVALID;
    if(0U == intercore_metadata_is_valid(transport->shared))
    {
        transport->shared->health.version_error_count++;
        return INTERCORE_TRANSPORT_INVALID;
    }
    if(transport->boot_epoch != transport->shared->metadata.boot_epoch)
    {
        transport->shared->health.boot_epoch_change_count++;
        intercore_transport_accept_epoch(transport);
        return INTERCORE_TRANSPORT_EPOCH_CHANGED;
    }
    active_before = transport->shared->metadata.gnss_active_index;
    sequence_before = transport->shared->metadata.gnss_sequence;
    if(1U < active_before) return INTERCORE_TRANSPORT_INVALID;
    if((0U == sequence_before) || (sequence_before == transport->last_gnss_sequence)) return INTERCORE_TRANSPORT_NO_DATA;
    intercore_copy_from_volatile((uint8 *)&local_slot,
                                 (const volatile uint8 *)&transport->shared->gnss[active_before],
                                 sizeof(local_slot));
    INTERCORE_DMB();
    active_after = transport->shared->metadata.gnss_active_index;
    sequence_after = transport->shared->metadata.gnss_sequence;
    if((active_before != active_after) || (sequence_before != sequence_after)) return INTERCORE_TRANSPORT_NO_DATA;
    if(local_slot.header.sequence != sequence_before) return INTERCORE_TRANSPORT_INVALID;
    if((INTERCORE_PROTOCOL_MAGIC != local_slot.header.magic) ||
       (INTERCORE_PROTOCOL_VERSION != local_slot.header.version) ||
       ((uint16)INTERCORE_RECORD_GNSS != local_slot.header.type) ||
       (sizeof(local_slot.payload) != local_slot.header.size))
    {
        transport->shared->health.version_error_count++;
        return INTERCORE_TRANSPORT_INVALID;
    }
    if(0U == intercore_record_validate(&local_slot.header, INTERCORE_RECORD_GNSS,
                                       &local_slot.payload, sizeof(local_slot.payload)))
    {
        transport->shared->health.crc_error_count++;
        return INTERCORE_TRANSPORT_INVALID;
    }
    memcpy(payload, &local_slot.payload, sizeof(*payload));
    *source_ms = local_slot.header.source_ms;
    *record_sequence = local_slot.header.sequence;
    transport->last_gnss_sequence = local_slot.header.sequence;
    transport->shared->health.cm7_0_consume_count++;
    return INTERCORE_TRANSPORT_OK;
}

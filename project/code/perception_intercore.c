#include "perception_intercore.h"

#if defined(__ICCARM__)
#include "cy_project.h"
#endif

static void perception_intercore_barrier(void)
{
#if defined(__ICCARM__)
    __DMB();
#elif defined(__GNUC__)
    __sync_synchronize();
#endif
}

static void perception_intercore_copy_from_volatile(uint8 *destination,
                                                     const volatile uint8 *source,
                                                     uint32 size)
{
    uint32 index;

    for(index = 0U; index < size; index++)
    {
        destination[index] = source[index];
    }
}

static void perception_intercore_copy_to_volatile(volatile uint8 *destination,
                                                   const uint8 *source,
                                                   uint32 size)
{
    uint32 index;

    for(index = 0U; index < size; index++)
    {
        destination[index] = source[index];
    }
}

static uint8 perception_intercore_shared_valid(
    const volatile intercore_shared_layout_struct *shared)
{
    return ((NULL != shared) &&
            (INTERCORE_PROTOCOL_MAGIC == shared->metadata.magic) &&
            (INTERCORE_PROTOCOL_VERSION == shared->metadata.version) &&
            ((uint16)sizeof(intercore_shared_layout_struct) ==
             shared->metadata.layout_size) &&
            (1U == shared->metadata.cm7_0_ready)) ? 1U : 0U;
}

static uint8 perception_intercore_control_valid(
    const volatile intercore_perception_control_struct *control,
    uint32 boot_epoch)
{
    return ((NULL != control) &&
            (INTERCORE_PERCEPTION_MAGIC == control->magic) &&
            (INTERCORE_PERCEPTION_VERSION == control->version) &&
            (INTERCORE_PERCEPTION_SLOT_COUNT == control->slot_count) &&
            (boot_epoch == control->producer_boot_epoch) &&
            (INTERCORE_PERCEPTION_SLOT_COUNT > control->pose_active_index) &&
            (INTERCORE_PERCEPTION_SLOT_COUNT > control->perception_active_index)) ? 1U : 0U;
}

static uint8 perception_intercore_transport_valid(
    const perception_intercore_transport_struct *transport)
{
    return ((NULL != transport) && (0U != transport->attached) &&
            (NULL != transport->shared) &&
            (1U == perception_intercore_shared_valid(transport->shared)) &&
            (transport->boot_epoch == transport->shared->metadata.boot_epoch) &&
            (1U == perception_intercore_control_valid(&transport->shared->perception,
                                                       transport->boot_epoch))) ? 1U : 0U;
}

static void perception_intercore_clear(volatile uint8 *data, uint32 size)
{
    uint32 index;

    for(index = 0U; index < size; index++)
    {
        data[index] = 0U;
    }
}

uint8 perception_intercore_cm7_0_init(perception_intercore_transport_struct *transport,
                                      volatile intercore_shared_layout_struct *shared)
{
    uint32 boot_epoch;

    if((NULL == transport) || (0U == perception_intercore_shared_valid(shared)))
    {
        return 0U;
    }

    boot_epoch = shared->metadata.boot_epoch;
    perception_intercore_clear((volatile uint8 *)&shared->perception,
                               sizeof(shared->perception));
    shared->perception.version = INTERCORE_PERCEPTION_VERSION;
    shared->perception.slot_count = INTERCORE_PERCEPTION_SLOT_COUNT;
    shared->perception.producer_boot_epoch = boot_epoch;
    perception_intercore_barrier();
    shared->perception.magic = INTERCORE_PERCEPTION_MAGIC;

    transport->shared = shared;
    transport->boot_epoch = boot_epoch;
    transport->last_pose_sequence = 0U;
    transport->last_perception_sequence = 0U;
    transport->role = (uint8)INTERCORE_ROLE_CM7_0;
    transport->attached = 1U;
    return 1U;
}

uint8 perception_intercore_cm7_1_attach(perception_intercore_transport_struct *transport,
                                        volatile intercore_shared_layout_struct *shared)
{
    uint32 boot_epoch;

    if((NULL == transport) || (0U == perception_intercore_shared_valid(shared)))
    {
        return 0U;
    }

    boot_epoch = shared->metadata.boot_epoch;
    if(0U == perception_intercore_control_valid(&shared->perception, boot_epoch))
    {
        return 0U;
    }

    transport->shared = shared;
    transport->boot_epoch = boot_epoch;
    transport->last_pose_sequence = 0U;
    transport->last_perception_sequence = 0U;
    transport->role = (uint8)INTERCORE_ROLE_CM7_1;
    transport->attached = 1U;
    return 1U;
}

uint8 perception_intercore_publish_pose(perception_intercore_transport_struct *transport,
                                        const perception_pose_snapshot_struct *snapshot)
{
    intercore_perception_pose_slot_struct local_slot;
    uint32 inactive_index;

    if((NULL == transport) || (NULL == snapshot) || (0U == snapshot->sequence) ||
       ((uint8)INTERCORE_ROLE_CM7_0 != transport->role) ||
       (0U == perception_intercore_transport_valid(transport)) ||
       (snapshot->sequence <= transport->shared->perception.pose_sequence))
    {
        return 0U;
    }

    inactive_index = (0U == transport->shared->perception.pose_active_index) ? 1U : 0U;
    local_slot.payload = *snapshot;
    local_slot.payload.crc32 = 0U;
    intercore_record_prepare(&local_slot.header,
                             INTERCORE_RECORD_PERCEPTION_POSE,
                             snapshot->sequence,
                             snapshot->timestamp_us / 1000U,
                             &local_slot.payload,
                             sizeof(local_slot.payload));
    perception_intercore_copy_to_volatile(
        (volatile uint8 *)&transport->shared->perception.pose[inactive_index],
        (const uint8 *)&local_slot,
        sizeof(local_slot));
    perception_intercore_barrier();
    transport->shared->perception.pose_sequence = snapshot->sequence;
    transport->shared->perception.pose_active_index = inactive_index;
    return 1U;
}

uint8 perception_intercore_read_pose(perception_intercore_transport_struct *transport,
                                     perception_pose_snapshot_struct *snapshot)
{
    intercore_perception_pose_slot_struct local_slot;
    uint32 active_before;
    uint32 active_after;
    uint32 sequence_before;
    uint32 sequence_after;

    if((NULL == transport) || (NULL == snapshot) ||
       ((uint8)INTERCORE_ROLE_CM7_1 != transport->role) ||
       (0U == perception_intercore_transport_valid(transport)))
    {
        return 0U;
    }

    active_before = transport->shared->perception.pose_active_index;
    sequence_before = transport->shared->perception.pose_sequence;
    if((INTERCORE_PERCEPTION_SLOT_COUNT <= active_before) ||
       (0U == sequence_before) ||
       (sequence_before <= transport->last_pose_sequence))
    {
        return 0U;
    }
    perception_intercore_copy_from_volatile(
        (uint8 *)&local_slot,
        (const volatile uint8 *)&transport->shared->perception.pose[active_before],
        sizeof(local_slot));
    perception_intercore_barrier();
    active_after = transport->shared->perception.pose_active_index;
    sequence_after = transport->shared->perception.pose_sequence;
    if((active_before != active_after) || (sequence_before != sequence_after) ||
       (sequence_before != local_slot.header.sequence) ||
       (sequence_before != local_slot.payload.sequence) ||
       (0U == intercore_record_validate(&local_slot.header,
                                        INTERCORE_RECORD_PERCEPTION_POSE,
                                        &local_slot.payload,
                                        sizeof(local_slot.payload))))
    {
        return 0U;
    }
    *snapshot = local_slot.payload;
    transport->last_pose_sequence = sequence_before;
    return 1U;
}

uint8 perception_intercore_publish_perception(
    perception_intercore_transport_struct *transport,
    const perception_snapshot_struct *snapshot)
{
    intercore_perception_snapshot_slot_struct local_slot;
    uint32 inactive_index;

    if((NULL == transport) || (NULL == snapshot) || (0U == snapshot->sequence) ||
       ((uint8)INTERCORE_ROLE_CM7_1 != transport->role) ||
       (0U == perception_intercore_transport_valid(transport)) ||
       (snapshot->sequence <= transport->shared->perception.perception_sequence))
    {
        return 0U;
    }

    inactive_index = (0U == transport->shared->perception.perception_active_index) ? 1U : 0U;
    local_slot.payload = *snapshot;
    local_slot.payload.crc32 = 0U;
    intercore_record_prepare(&local_slot.header,
                             INTERCORE_RECORD_PERCEPTION_SNAPSHOT,
                             snapshot->sequence,
                             snapshot->timestamp_us / 1000U,
                             &local_slot.payload,
                             sizeof(local_slot.payload));
    perception_intercore_copy_to_volatile(
        (volatile uint8 *)&transport->shared->perception.perception[inactive_index],
        (const uint8 *)&local_slot,
        sizeof(local_slot));
    perception_intercore_barrier();
    transport->shared->perception.perception_sequence = snapshot->sequence;
    transport->shared->perception.perception_active_index = inactive_index;
    return 1U;
}

uint8 perception_intercore_read_perception(
    perception_intercore_transport_struct *transport,
    perception_snapshot_struct *snapshot)
{
    intercore_perception_snapshot_slot_struct local_slot;
    uint32 active_before;
    uint32 active_after;
    uint32 sequence_before;
    uint32 sequence_after;

    if((NULL == transport) || (NULL == snapshot) ||
       ((uint8)INTERCORE_ROLE_CM7_0 != transport->role) ||
       (0U == perception_intercore_transport_valid(transport)))
    {
        return 0U;
    }

    active_before = transport->shared->perception.perception_active_index;
    sequence_before = transport->shared->perception.perception_sequence;
    if((INTERCORE_PERCEPTION_SLOT_COUNT <= active_before) ||
       (0U == sequence_before) ||
       (sequence_before <= transport->last_perception_sequence))
    {
        return 0U;
    }
    perception_intercore_copy_from_volatile(
        (uint8 *)&local_slot,
        (const volatile uint8 *)&transport->shared->perception.perception[active_before],
        sizeof(local_slot));
    perception_intercore_barrier();
    active_after = transport->shared->perception.perception_active_index;
    sequence_after = transport->shared->perception.perception_sequence;
    if((active_before != active_after) || (sequence_before != sequence_after) ||
       (sequence_before != local_slot.header.sequence) ||
       (sequence_before != local_slot.payload.sequence) ||
       (0U == intercore_record_validate(&local_slot.header,
                                        INTERCORE_RECORD_PERCEPTION_SNAPSHOT,
                                        &local_slot.payload,
                                        sizeof(local_slot.payload))))
    {
        return 0U;
    }
    *snapshot = local_slot.payload;
    transport->last_perception_sequence = sequence_before;
    return 1U;
}

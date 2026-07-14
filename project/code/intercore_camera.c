#include "intercore_camera.h"

#include <stdint.h>

#if defined(INTERCORE_HOST_TEST)
#include <stdatomic.h>
#define INTERCORE_CAMERA_DMB() atomic_signal_fence(memory_order_seq_cst)
#else
#include "cy_project.h"
#define INTERCORE_CAMERA_DMB() __DMB()
#endif

static uint8 intercore_camera_pointer_is_valid(const volatile void *pointer)
{
    return ((NULL != pointer) &&
            (0U == ((uintptr_t)pointer % 32U))) ? 1U : 0U;
}

static uint8 intercore_camera_metadata_is_valid(
    const volatile intercore_shared_layout_struct *shared,
    uint32 boot_epoch)
{
    return ((INTERCORE_PROTOCOL_MAGIC == shared->metadata.magic) &&
            (INTERCORE_PROTOCOL_VERSION == shared->metadata.version) &&
            (sizeof(intercore_shared_layout_struct) == shared->metadata.layout_size) &&
            (1U == shared->metadata.cm7_0_ready) &&
            (boot_epoch == shared->metadata.boot_epoch)) ? 1U : 0U;
}

static uint8 intercore_camera_control_is_valid(
    const volatile intercore_camera_control_struct *control,
    uint32 boot_epoch)
{
    return ((INTERCORE_CAMERA_MAGIC == control->magic) &&
            (INTERCORE_CAMERA_VERSION == control->version) &&
            (INTERCORE_CAMERA_FORMAT_GRAY8 == control->format) &&
            (INTERCORE_CAMERA_WIDTH == control->width) &&
            (INTERCORE_CAMERA_HEIGHT == control->height) &&
            (INTERCORE_CAMERA_STRIDE == control->stride) &&
            (INTERCORE_CAMERA_SLOT_COUNT == control->slot_count) &&
            (INTERCORE_CAMERA_SLOT_SIZE_BYTES == control->frame_bytes) &&
            (boot_epoch == control->producer_boot_epoch)) ? 1U : 0U;
}

static uint8 intercore_camera_transport_is_configured(
    intercore_camera_transport_struct *transport,
    intercore_role_enum expected_role)
{
    if((NULL == transport) || (0U == transport->attached) ||
       ((uint8)expected_role != transport->role) ||
       (0U == intercore_camera_pointer_is_valid(transport->shared)) ||
       (0U == intercore_camera_pointer_is_valid(transport->data_plane)))
    {
        return 0U;
    }
    if(0U == intercore_camera_metadata_is_valid(transport->shared,
                                                transport->boot_epoch))
    {
        return 0U;
    }
    if(0U == intercore_camera_control_is_valid(&transport->shared->camera,
                                               transport->boot_epoch))
    {
        transport->shared->camera.invalid_layout_count++;
        return 0U;
    }
    return 1U;
}

static intercore_camera_result_enum intercore_camera_transition_validate(
    intercore_camera_transport_struct *transport,
    intercore_role_enum expected_role)
{
    if((NULL != transport) && (0U != transport->attached) &&
       (NULL != transport->shared) &&
       (transport->boot_epoch != transport->shared->metadata.boot_epoch))
    {
        return INTERCORE_CAMERA_EPOCH_CHANGED;
    }
    return (0U != intercore_camera_transport_is_configured(transport,
                                                            expected_role)) ?
           INTERCORE_CAMERA_OK : INTERCORE_CAMERA_INVALID;
}

static uint32 intercore_camera_next_sequence(uint32 sequence)
{
    sequence++;
    return (0U == sequence) ? 1U : sequence;
}

static uint32 intercore_camera_find_free_slot(
    const volatile intercore_camera_control_struct *control)
{
    uint32 slot;

    for(slot = 0U; slot < INTERCORE_CAMERA_SLOT_COUNT; slot++)
    {
        if(INTERCORE_CAMERA_SLOT_FREE == control->slot[slot].state)
        {
            break;
        }
    }
    return slot;
}

uint8 intercore_camera_producer_init(intercore_camera_transport_struct *transport,
                                     volatile intercore_shared_layout_struct *shared,
                                     volatile uint8 *data_plane,
                                     uint32 boot_epoch)
{
    volatile uint8 *bytes;
    uint32 index;

    if((NULL == transport) ||
       (0U == intercore_camera_pointer_is_valid(shared)) ||
       (0U == intercore_camera_pointer_is_valid(data_plane)) ||
       (0U == intercore_camera_metadata_is_valid(shared, boot_epoch)))
    {
        return 0U;
    }

    transport->attached = 0U;
    bytes = (volatile uint8 *)&shared->camera;
    for(index = 0U; index < sizeof(shared->camera); index++)
    {
        bytes[index] = 0U;
    }

    shared->camera.version = INTERCORE_CAMERA_VERSION;
    shared->camera.format = INTERCORE_CAMERA_FORMAT_GRAY8;
    shared->camera.width = INTERCORE_CAMERA_WIDTH;
    shared->camera.height = INTERCORE_CAMERA_HEIGHT;
    shared->camera.stride = INTERCORE_CAMERA_STRIDE;
    shared->camera.slot_count = INTERCORE_CAMERA_SLOT_COUNT;
    shared->camera.frame_bytes = INTERCORE_CAMERA_SLOT_SIZE_BYTES;
    shared->camera.producer_boot_epoch = boot_epoch;
    shared->camera.slot[0].state = INTERCORE_CAMERA_SLOT_FREE;
    shared->camera.slot[1].state = INTERCORE_CAMERA_SLOT_FREE;
    INTERCORE_CAMERA_DMB();
    shared->camera.magic = INTERCORE_CAMERA_MAGIC;

    transport->shared = shared;
    transport->data_plane = data_plane;
    transport->boot_epoch = boot_epoch;
    transport->role = INTERCORE_ROLE_CM7_0;
    transport->attached = 1U;
    return 1U;
}

uint8 intercore_camera_consumer_attach(intercore_camera_transport_struct *transport,
                                       volatile intercore_shared_layout_struct *shared,
                                       volatile uint8 *data_plane,
                                       uint32 boot_epoch)
{
    uint32 slot;

    if(NULL != transport)
    {
        transport->attached = 0U;
    }
    if((NULL == transport) ||
       (0U == intercore_camera_pointer_is_valid(shared)) ||
       (0U == intercore_camera_pointer_is_valid(data_plane)) ||
       (0U == intercore_camera_metadata_is_valid(shared, boot_epoch)))
    {
        return 0U;
    }
    if(0U == intercore_camera_control_is_valid(&shared->camera, boot_epoch))
    {
        shared->camera.invalid_layout_count++;
        return 0U;
    }

    for(slot = 0U; slot < INTERCORE_CAMERA_SLOT_COUNT; slot++)
    {
        if(INTERCORE_CAMERA_SLOT_READING == shared->camera.slot[slot].state)
        {
            shared->camera.slot[slot].state = INTERCORE_CAMERA_SLOT_FREE;
        }
    }
    INTERCORE_CAMERA_DMB();

    transport->shared = shared;
    transport->data_plane = data_plane;
    transport->boot_epoch = boot_epoch;
    transport->role = INTERCORE_ROLE_CM7_1;
    transport->attached = 1U;
    return 1U;
}

void intercore_camera_producer_record_capture(intercore_camera_transport_struct *transport,
                                              uint32 capture_ms)
{
    volatile intercore_camera_control_struct *control;

    if(INTERCORE_CAMERA_OK !=
       intercore_camera_transition_validate(transport, INTERCORE_ROLE_CM7_0))
    {
        return;
    }
    control = &transport->shared->camera;
    control->captured_count++;
    control->capture_sequence = intercore_camera_next_sequence(control->capture_sequence);
    control->last_capture_ms = capture_ms;
}

intercore_camera_result_enum intercore_camera_producer_claim(
    intercore_camera_transport_struct *transport,
    uint8 *slot_index,
    volatile uint8 **pixels)
{
    intercore_camera_result_enum result;
    volatile intercore_camera_control_struct *control;
    uint32 slot;

    result = intercore_camera_transition_validate(transport, INTERCORE_ROLE_CM7_0);
    if(INTERCORE_CAMERA_OK != result)
    {
        return result;
    }
    if((NULL == slot_index) || (NULL == pixels))
    {
        return INTERCORE_CAMERA_INVALID;
    }

    control = &transport->shared->camera;
    slot = intercore_camera_find_free_slot(control);
    if(INTERCORE_CAMERA_SLOT_COUNT == slot)
    {
        control->no_free_drop_count++;
        return INTERCORE_CAMERA_NO_FREE_SLOT;
    }
    control->slot[slot].state = INTERCORE_CAMERA_SLOT_WRITING;
    INTERCORE_CAMERA_DMB();
    *slot_index = (uint8)slot;
    *pixels = transport->data_plane + (slot * INTERCORE_CAMERA_SLOT_SIZE_BYTES);
    return INTERCORE_CAMERA_OK;
}

uint8 intercore_camera_producer_publish(intercore_camera_transport_struct *transport,
                                        uint8 slot_index,
                                        uint32 capture_ms,
                                        uint32 publish_ms)
{
    volatile intercore_camera_control_struct *control;

    if((INTERCORE_CAMERA_OK !=
        intercore_camera_transition_validate(transport, INTERCORE_ROLE_CM7_0)) ||
       (INTERCORE_CAMERA_SLOT_COUNT <= slot_index))
    {
        return 0U;
    }
    control = &transport->shared->camera;
    if(INTERCORE_CAMERA_SLOT_WRITING != control->slot[slot_index].state)
    {
        return 0U;
    }

    control->slot[slot_index].sequence = control->capture_sequence;
    control->slot[slot_index].capture_ms = capture_ms;
    control->slot[slot_index].publish_ms = publish_ms;
    control->slot[slot_index].frame_bytes = INTERCORE_CAMERA_SLOT_SIZE_BYTES;
    control->latest_published_sequence = control->capture_sequence;
    control->published_count++;
    control->last_publish_ms = publish_ms;
    INTERCORE_CAMERA_DMB();
    control->slot[slot_index].state = INTERCORE_CAMERA_SLOT_READY;
    INTERCORE_CAMERA_DMB();
    return 1U;
}

intercore_camera_result_enum intercore_camera_consumer_acquire_latest(
    intercore_camera_transport_struct *transport,
    intercore_camera_frame_view_struct *view)
{
    intercore_camera_result_enum result;
    volatile intercore_camera_control_struct *control;
    uint32 slot;
    uint32 newest_slot = INTERCORE_CAMERA_SLOT_COUNT;
    uint32 newest_sequence = 0U;

    result = intercore_camera_transition_validate(transport, INTERCORE_ROLE_CM7_1);
    if(INTERCORE_CAMERA_OK != result)
    {
        return result;
    }
    if(NULL == view)
    {
        return INTERCORE_CAMERA_INVALID;
    }

    control = &transport->shared->camera;
    for(slot = 0U; slot < INTERCORE_CAMERA_SLOT_COUNT; slot++)
    {
        if((INTERCORE_CAMERA_SLOT_READY == control->slot[slot].state) &&
           ((INTERCORE_CAMERA_SLOT_COUNT == newest_slot) ||
            (newest_sequence < control->slot[slot].sequence)))
        {
            newest_slot = slot;
            newest_sequence = control->slot[slot].sequence;
        }
    }
    if(INTERCORE_CAMERA_SLOT_COUNT == newest_slot)
    {
        return INTERCORE_CAMERA_NO_READY_SLOT;
    }

    INTERCORE_CAMERA_DMB();
    control->slot[newest_slot].state = INTERCORE_CAMERA_SLOT_READING;
    INTERCORE_CAMERA_DMB();
    view->pixels = transport->data_plane +
                   (newest_slot * INTERCORE_CAMERA_SLOT_SIZE_BYTES);
    view->sequence = control->slot[newest_slot].sequence;
    view->capture_ms = control->slot[newest_slot].capture_ms;
    view->publish_ms = control->slot[newest_slot].publish_ms;
    view->frame_bytes = control->slot[newest_slot].frame_bytes;
    view->slot_index = (uint8)newest_slot;

    for(slot = 0U; slot < INTERCORE_CAMERA_SLOT_COUNT; slot++)
    {
        if((newest_slot != slot) &&
           (INTERCORE_CAMERA_SLOT_READY == control->slot[slot].state) &&
           (control->slot[slot].sequence < newest_sequence))
        {
            control->slot[slot].state = INTERCORE_CAMERA_SLOT_FREE;
            control->stale_ready_drop_count++;
        }
    }
    INTERCORE_CAMERA_DMB();
    return INTERCORE_CAMERA_OK;
}

uint8 intercore_camera_consumer_release(intercore_camera_transport_struct *transport,
                                        const intercore_camera_frame_view_struct *view)
{
    volatile intercore_camera_control_struct *control;

    if((INTERCORE_CAMERA_OK !=
        intercore_camera_transition_validate(transport, INTERCORE_ROLE_CM7_1)) ||
       (NULL == view) ||
       (INTERCORE_CAMERA_SLOT_COUNT <= view->slot_index))
    {
        return 0U;
    }
    control = &transport->shared->camera;
    if((INTERCORE_CAMERA_SLOT_READING != control->slot[view->slot_index].state) ||
       (view->sequence != control->slot[view->slot_index].sequence))
    {
        return 0U;
    }

    control->slot[view->slot_index].state = INTERCORE_CAMERA_SLOT_FREE;
    control->consumed_count++;
    INTERCORE_CAMERA_DMB();
    return 1U;
}

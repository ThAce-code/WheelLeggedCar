#include "intercore_protocol.h"

static uint32 intercore_crc32_update(uint32 crc, const uint8 *data, uint32 size)
{
    uint32 index;
    uint8 bit;

    if((NULL == data) && (0U != size))
    {
        return crc;
    }

    for(index = 0U; index < size; index++)
    {
        crc ^= (uint32)data[index];
        for(bit = 0U; bit < 8U; bit++)
        {
            if(0U != (crc & 1UL))
            {
                crc = (crc >> 1U) ^ 0xEDB88320UL;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

uint32 intercore_crc32(const uint8 *data, uint32 size)
{
    return intercore_crc32_update(0xFFFFFFFFUL, data, size) ^ 0xFFFFFFFFUL;
}

void intercore_record_prepare(intercore_header_struct *header,
                              intercore_record_type_enum type,
                              uint32 sequence,
                              uint32 source_ms,
                              const void *payload,
                              uint32 payload_size)
{
    uint32 crc;

    if(NULL == header)
    {
        return;
    }

    header->magic = INTERCORE_PROTOCOL_MAGIC;
    header->version = INTERCORE_PROTOCOL_VERSION;
    header->type = (uint16)type;
    header->size = payload_size;
    header->sequence = sequence;
    header->source_ms = source_ms;
    header->crc32 = 0U;

    crc = intercore_crc32((const uint8 *)header, 20U);
    if((NULL != payload) && (0U != payload_size))
    {
        crc = intercore_crc32_update(crc ^ 0xFFFFFFFFUL,
                                     (const uint8 *)payload,
                                     payload_size) ^ 0xFFFFFFFFUL;
    }
    header->crc32 = crc;
}

uint8 intercore_record_validate(const intercore_header_struct *header,
                                intercore_record_type_enum expected_type,
                                const void *payload,
                                uint32 expected_payload_size)
{
    uint32 crc;

    if(NULL == header)
    {
        return 0U;
    }
    if(INTERCORE_PROTOCOL_MAGIC != header->magic)
    {
        return 0U;
    }
    if(INTERCORE_PROTOCOL_VERSION != header->version)
    {
        return 0U;
    }
    if((uint16)expected_type != header->type)
    {
        return 0U;
    }
    if(expected_payload_size != header->size)
    {
        return 0U;
    }
    if((NULL == payload) && (0U != expected_payload_size))
    {
        return 0U;
    }

    crc = intercore_crc32((const uint8 *)header, 20U);
    if((0U != expected_payload_size) && (NULL != payload))
    {
        crc = intercore_crc32_update(crc ^ 0xFFFFFFFFUL,
                                     (const uint8 *)payload,
                                     expected_payload_size) ^ 0xFFFFFFFFUL;
    }
    return (crc == header->crc32) ? 1U : 0U;
}

static uint8 intercore_float_is_finite(float value)
{
    return ((value == value) &&
            (3.402823466e+38F >= value) &&
            (-3.402823466e+38F <= value)) ? 1U : 0U;
}

static uint8 intercore_reserved_is_zero(const uint8 *data, uint32 size)
{
    uint32 index;

    for(index = 0U; index < size; index++)
    {
        if(0U != data[index])
        {
            return 0U;
        }
    }
    return 1U;
}

uint8 intercore_navigation_is_structurally_valid(
    const navigation_command_struct *command)
{
    if(NULL == command)
    {
        return 0U;
    }
    if((0U == intercore_float_is_finite(command->forward_rpm)) ||
       (0U == intercore_float_is_finite(command->turn_rate_dps)) ||
       (0U == intercore_float_is_finite(command->confidence)))
    {
        return 0U;
    }
    if((0.0f > command->confidence) || (1.0f < command->confidence))
    {
        return 0U;
    }
    if(0U == command->source_sequence)
    {
        return 0U;
    }
    if((1U > command->valid_for_ms) ||
       (INTERCORE_NAVIGATION_MAX_VALID_MS < command->valid_for_ms))
    {
        return 0U;
    }
    if((0U != command->enable) && (1U != command->enable))
    {
        return 0U;
    }
    if(NAVIGATION_SOURCE_WAYPOINT < command->source)
    {
        return 0U;
    }
    if(NAVIGATION_MODE_WAYPOINT < command->mode)
    {
        return 0U;
    }
    if(NAVIGATION_STOP_INVALID < command->stop_reason)
    {
        return 0U;
    }
    if(0U == intercore_reserved_is_zero(command->reserved,
                                        sizeof(command->reserved)))
    {
        return 0U;
    }
    return 1U;
}

/*********************************************************************************************************************
* File: camera_seekfree_transport.c
* Description: Captures WiFi-SPI remaining-byte results hidden by the vendor void camera API.
********************************************************************************************************************/

#include "camera_seekfree_transport.h"
#ifdef CAMERA_SEEKFREE_TRANSPORT_HOST_TEST
typedef uint32 (*seekfree_assistant_transfer_callback_function)(const uint8 *buffer,
                                                                uint32 length);
enum { SEEKFREE_ASSISTANT_CUSTOM = 6 };
extern seekfree_assistant_transfer_callback_function seekfree_assistant_transfer_callback;
void seekfree_assistant_interface_init(int device);
uint32 wifi_spi_send_buffer(const uint8 *buffer, uint32 length);
#else
#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"
#include "zf_device_wifi_spi.h"
#endif

#include <string.h>

extern seekfree_assistant_transfer_callback_function seekfree_assistant_transfer_callback;

static camera_seekfree_transport_diag_struct transport_diag;

void camera_seekfree_transport_install(void)
{
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_CUSTOM);
    seekfree_assistant_transfer_callback = camera_seekfree_transport_transfer;
}

void camera_seekfree_transport_begin(uint32 header_bytes, uint32 payload_bytes)
{
    memset(&transport_diag, 0, sizeof(transport_diag));
    transport_diag.expected_header_bytes = header_bytes;
    transport_diag.expected_payload_bytes = payload_bytes;
}

uint32 camera_seekfree_transport_transfer(const uint8 *buffer, uint32 length)
{
    uint32 remaining;

    /* A payload without its complete header is not a valid Assistant frame. */
    if(2U <= transport_diag.segment_count)
    {
        remaining = length;
    }
    else if((1U == transport_diag.segment_count) &&
       (0U != transport_diag.header_remaining))
    {
        remaining = length;
    }
    else
    {
        remaining = wifi_spi_send_buffer(buffer, length);
    }
    if(0U == transport_diag.segment_count)
    {
        transport_diag.header_requested = length;
        transport_diag.header_remaining = remaining;
    }
    else if(1U == transport_diag.segment_count)
    {
        transport_diag.payload_requested = length;
        transport_diag.payload_remaining = remaining;
    }
    else
    {
        transport_diag.unexpected_segment = 1U;
    }
    transport_diag.segment_count++;
    return remaining;
}

uint8 camera_seekfree_transport_frame_complete(void)
{
    return ((transport_diag.segment_count == 2U) &&
            (0U == transport_diag.unexpected_segment) &&
            (transport_diag.expected_header_bytes == transport_diag.header_requested) &&
            (transport_diag.expected_payload_bytes == transport_diag.payload_requested) &&
            (transport_diag.header_remaining == 0U) &&
            (transport_diag.payload_remaining == 0U)) ? 1U : 0U;
}

const camera_seekfree_transport_diag_struct *camera_seekfree_transport_get_diag(void)
{
    return &transport_diag;
}

/*********************************************************************************************************************
* File: camera_seekfree_transport.h
* Description: Observable project adapter for one Assistant camera transfer.
********************************************************************************************************************/

#ifndef _camera_seekfree_transport_h_
#define _camera_seekfree_transport_h_

#ifdef CAMERA_SEEKFREE_TRANSPORT_HOST_TEST
#include <stdint.h>
typedef uint8_t uint8;
typedef uint32_t uint32;
#else
#include "zf_common_typedef.h"
#endif

typedef struct
{
    uint32 expected_header_bytes;
    uint32 expected_payload_bytes;
    uint32 header_requested;
    uint32 payload_requested;
    uint32 header_remaining;
    uint32 payload_remaining;
    uint8 segment_count;
    uint8 unexpected_segment;
} camera_seekfree_transport_diag_struct;

void camera_seekfree_transport_install(void);
void camera_seekfree_transport_begin(uint32 header_bytes, uint32 payload_bytes);
uint32 camera_seekfree_transport_transfer(const uint8 *buffer, uint32 length);
uint8 camera_seekfree_transport_frame_complete(void);
const camera_seekfree_transport_diag_struct *camera_seekfree_transport_get_diag(void);

#endif

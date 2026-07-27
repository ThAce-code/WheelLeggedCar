#ifndef _gnss_types_h_
#define _gnss_types_h_

#include "zf_common_typedef.h"

#define GNSS_SNAPSHOT_MAX_AGE_MS (300U)

typedef struct
{
    uint32 sequence;
    uint32 timestamp_ms;
    double latitude_deg;
    double longitude_deg;
    float local_x_m;
    float local_y_m;
    float speed_mps;
    float course_deg;
    float hdop;
    float position_sigma_m;
    uint32 checksum_error_count;
    uint32 timeout_count;
    uint16 satellite_count;
    uint8 fix_valid;
    uint8 fix_quality;
    uint8 origin_valid;
    uint8 reserved;
}gnss_snapshot_struct;

#endif

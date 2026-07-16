#ifndef _zf_device_gnss_h_
#define _zf_device_gnss_h_

#include "zf_common_typedef.h"

typedef enum
{
    TAU1201 = 1,
    GN42A = 1,
    GN43RFA = 2,
}gps_device_enum;

typedef struct
{
    uint16 year;
    uint8 month;
    uint8 day;
    uint8 hour;
    uint8 minute;
    uint8 second;
}gps_time_struct;

typedef struct
{
    gps_time_struct time;
    uint8 state;
    uint8 fix_quality;
    float hdop;
    uint32 rmc_sequence;
    uint32 gga_sequence;
    uint16 latitude_degree;
    uint16 latitude_cent;
    uint16 latitude_second;
    uint16 longitude_degree;
    uint16 longitude_cent;
    uint16 longitude_second;
    double latitude;
    double longitude;
    int8 ns;
    int8 ew;
    float speed;
    float direction;
    uint8 antenna_direction_state;
    float antenna_direction;
    uint8 satellite_used;
    float height;
}gnss_info_struct;

extern gnss_info_struct gnss;
extern volatile uint8 gnss_flag;

uint8 gnss_data_parse(void);
void gnss_init(gps_device_enum gps_device);

#endif

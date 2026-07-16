#include "local_position.h"

#include <math.h>
#include <stddef.h>

#define LOCAL_POSITION_PI             (3.14159265358979323846)
#define LOCAL_POSITION_EARTH_RADIUS_M (6378137.0)

static double origin_latitude_rad;
static double origin_longitude_rad;
static double origin_cos_latitude;
static uint8 origin_valid;

void local_position_reset(void)
{
    origin_latitude_rad = 0.0;
    origin_longitude_rad = 0.0;
    origin_cos_latitude = 1.0;
    origin_valid = 0U;
}

uint8 local_position_set_origin(double latitude_deg, double longitude_deg)
{
    if((!isfinite(latitude_deg)) || (!isfinite(longitude_deg)) ||
       (latitude_deg < -90.0) || (latitude_deg > 90.0) ||
       (longitude_deg < -180.0) || (longitude_deg > 180.0))
    {
        return 0U;
    }

    origin_latitude_rad = latitude_deg * LOCAL_POSITION_PI / 180.0;
    origin_longitude_rad = longitude_deg * LOCAL_POSITION_PI / 180.0;
    origin_cos_latitude = cos(origin_latitude_rad);
    origin_valid = 1U;
    return 1U;
}

uint8 local_position_has_origin(void)
{
    return origin_valid;
}

uint8 local_position_project(double latitude_deg, double longitude_deg,
                             float *east_m, float *north_m)
{
    double latitude_rad;
    double longitude_rad;

    if((0U == origin_valid) || (NULL == east_m) || (NULL == north_m) ||
       (!isfinite(latitude_deg)) || (!isfinite(longitude_deg)) ||
       (latitude_deg < -90.0) || (latitude_deg > 90.0) ||
       (longitude_deg < -180.0) || (longitude_deg > 180.0))
    {
        return 0U;
    }

    latitude_rad = latitude_deg * LOCAL_POSITION_PI / 180.0;
    longitude_rad = longitude_deg * LOCAL_POSITION_PI / 180.0;
    *east_m = (float)((longitude_rad - origin_longitude_rad) *
                      origin_cos_latitude * LOCAL_POSITION_EARTH_RADIUS_M);
    *north_m = (float)((latitude_rad - origin_latitude_rad) *
                       LOCAL_POSITION_EARTH_RADIUS_M);
    return 1U;
}

#include "sensor_gnss.h"

#include "local_position.h"
#include "zf_device_gnss.h"

#include <string.h>

static gnss_snapshot_struct latest;
static uint32 publish_sequence;
static uint32 checksum_error_count;
static uint32 timeout_count;
static uint32 last_sample_ms;
static uint8 new_snapshot;
static uint8 consecutive_usable;
static uint8 timeout_latched;
static uint16 origin_sample_count;
static double origin_latitude_sum;
static double origin_longitude_sum;

static uint8 sample_is_usable(void)
{
    return ((0U != gnss.state) && (0U != gnss.fix_quality) &&
            (SENSOR_GNSS_MIN_SATELLITES <= gnss.satellite_used) &&
            (0.0F < gnss.hdop) &&
            (SENSOR_GNSS_MAX_HDOP >= gnss.hdop)) ? 1U : 0U;
}

uint8 sensor_gnss_init(void)
{
    memset(&latest, 0, sizeof(latest));
    publish_sequence = 0U;
    checksum_error_count = 0U;
    timeout_count = 0U;
    last_sample_ms = 0U;
    new_snapshot = 0U;
    consecutive_usable = 0U;
    timeout_latched = 0U;
    origin_sample_count = 0U;
    origin_latitude_sum = 0.0;
    origin_longitude_sum = 0.0;
    local_position_reset();
    gnss_init(TAU1201);
    return 1U;
}

uint8 sensor_gnss_set_origin(double latitude_deg, double longitude_deg)
{
    origin_sample_count = 0U;
    origin_latitude_sum = 0.0;
    origin_longitude_sum = 0.0;
    return local_position_set_origin(latitude_deg, longitude_deg);
}

void sensor_gnss_service(uint32 now_ms)
{
    uint8 usable;

    if((0U != last_sample_ms) &&
       (GNSS_SNAPSHOT_MAX_AGE_MS < (now_ms - last_sample_ms)) &&
       (0U == timeout_latched))
    {
        timeout_count++;
        timeout_latched = 1U;
    }

    if(0U == gnss_flag)
    {
        return;
    }

    gnss_flag = 0U;
    if(0U != gnss_data_parse())
    {
        checksum_error_count++;
        return;
    }

    last_sample_ms = now_ms;
    timeout_latched = 0U;
    usable = sample_is_usable();
    consecutive_usable = usable ? (uint8)(consecutive_usable +
        ((consecutive_usable < SENSOR_GNSS_RECOVERY_FIX_COUNT) ? 1U : 0U)) : 0U;

    if((0U == local_position_has_origin()) && (0U != usable))
    {
        origin_latitude_sum += gnss.latitude;
        origin_longitude_sum += gnss.longitude;
        origin_sample_count++;
        if(SENSOR_GNSS_AUTO_ORIGIN_SAMPLE_COUNT <= origin_sample_count)
        {
            (void)local_position_set_origin(
                origin_latitude_sum / (double)origin_sample_count,
                origin_longitude_sum / (double)origin_sample_count);
        }
    }

    latest.sequence = ++publish_sequence;
    latest.timestamp_ms = now_ms;
    latest.latitude_deg = gnss.latitude;
    latest.longitude_deg = gnss.longitude;
    latest.speed_mps = gnss.speed / 3.6F;
    latest.course_deg = gnss.direction;
    latest.hdop = gnss.hdop;
    latest.position_sigma_m = -1.0F;
    latest.satellite_count = gnss.satellite_used;
    latest.fix_quality = gnss.fix_quality;
    latest.origin_valid = local_position_has_origin();
    latest.fix_valid = (SENSOR_GNSS_RECOVERY_FIX_COUNT <= consecutive_usable) ? 1U : 0U;
    latest.checksum_error_count = checksum_error_count;
    latest.timeout_count = timeout_count;
    if(0U == local_position_project(latest.latitude_deg, latest.longitude_deg,
                                    &latest.local_x_m, &latest.local_y_m))
    {
        latest.local_x_m = 0.0F;
        latest.local_y_m = 0.0F;
    }

    new_snapshot = 1U;
}

uint8 sensor_gnss_take_snapshot(gnss_snapshot_struct *snapshot)
{
    if((NULL == snapshot) || (0U == new_snapshot))
    {
        return 0U;
    }

    memcpy(snapshot, &latest, sizeof(*snapshot));
    new_snapshot = 0U;
    return 1U;
}

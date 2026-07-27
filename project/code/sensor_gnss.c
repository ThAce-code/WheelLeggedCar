#include "sensor_gnss.h"

#include "local_position.h"
#include "zf_device_gnss.h"

#include <math.h>
#include <string.h>

#define SENSOR_GNSS_UTC_DAY_MS      (86400000U)
#define SENSOR_GNSS_UTC_HALF_DAY_MS (43200000U)

static gnss_snapshot_struct latest;
static uint32 publish_sequence;
static uint32 checksum_error_count;
static uint32 timeout_count;
static uint32 last_sample_ms;
static uint8 new_snapshot;
static uint8 consecutive_usable;
static uint8 timeout_latched;
static uint8 pair_reset_pending;
static uint32 last_consumed_rmc_sequence;
static uint32 last_consumed_gga_sequence;
static uint16 origin_sample_count;
static double origin_latitude_sum;
static double origin_longitude_sum;

static uint8 coordinates_are_valid(double latitude, double longitude)
{
    return (isfinite(latitude) && isfinite(longitude) &&
            (-90.0 <= latitude) && (90.0 >= latitude) &&
            (-180.0 <= longitude) && (180.0 >= longitude)) ? 1U : 0U;
}

static uint8 rmc_epoch_is_older(uint32 rmc_utc_ms, uint32 gga_utc_ms)
{
    uint32 forward_ms = (gga_utc_ms + SENSOR_GNSS_UTC_DAY_MS - rmc_utc_ms) %
                        SENSOR_GNSS_UTC_DAY_MS;

    return ((0U != forward_ms) && (SENSOR_GNSS_UTC_HALF_DAY_MS > forward_ms)) ?
        1U : 0U;
}

static uint8 sample_is_usable(void)
{
    return ((0U != gnss.state) && (0U != gnss.fix_quality) &&
            (0U != coordinates_are_valid(gnss.latitude, gnss.longitude)) &&
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
    pair_reset_pending = 0U;
    last_consumed_rmc_sequence = 0U;
    last_consumed_gga_sequence = 0U;
    origin_sample_count = 0U;
    origin_latitude_sum = 0.0;
    origin_longitude_sum = 0.0;
    local_position_reset();
    gnss_init(TAU1201);
    last_consumed_rmc_sequence = gnss.rmc_sequence;
    last_consumed_gga_sequence = gnss.gga_sequence;
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
    uint8 parse_result;
    uint8 usable;

    if((0U != last_sample_ms) &&
       (GNSS_SNAPSHOT_MAX_AGE_MS < (now_ms - last_sample_ms)) &&
       (0U == timeout_latched))
    {
        timeout_count++;
        timeout_latched = 1U;
        consecutive_usable = 0U;
        latest.sequence = ++publish_sequence;
        latest.fix_valid = 0U;
        latest.timeout_count = timeout_count;
        new_snapshot = 1U;
        last_consumed_rmc_sequence = gnss.rmc_sequence;
        last_consumed_gga_sequence = gnss.gga_sequence;
        pair_reset_pending = 1U;
    }

    if(0U == gnss_flag)
    {
        return;
    }

    gnss_flag = 0U;
    parse_result = gnss_data_parse();
    if(0U != pair_reset_pending)
    {
        last_consumed_rmc_sequence = gnss.rmc_sequence;
        last_consumed_gga_sequence = gnss.gga_sequence;
        pair_reset_pending = 0U;
        if(0U != parse_result)
        {
            checksum_error_count++;
        }
        return;
    }

    if(0U != parse_result)
    {
        checksum_error_count++;
        return;
    }

    if((gnss.rmc_sequence == last_consumed_rmc_sequence) ||
       (gnss.gga_sequence == last_consumed_gga_sequence))
    {
        return;
    }

    if(gnss.rmc_utc_ms != gnss.gga_utc_ms)
    {
        if(0U != rmc_epoch_is_older(gnss.rmc_utc_ms, gnss.gga_utc_ms))
        {
            last_consumed_rmc_sequence = gnss.rmc_sequence;
        }
        else
        {
            last_consumed_gga_sequence = gnss.gga_sequence;
        }
        return;
    }

    last_consumed_rmc_sequence = gnss.rmc_sequence;
    last_consumed_gga_sequence = gnss.gga_sequence;

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
    if(0U != coordinates_are_valid(gnss.latitude, gnss.longitude))
    {
        latest.latitude_deg = gnss.latitude;
        latest.longitude_deg = gnss.longitude;
    }
    else
    {
        latest.latitude_deg = 0.0;
        latest.longitude_deg = 0.0;
    }
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

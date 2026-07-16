#ifndef _sensor_gnss_h_
#define _sensor_gnss_h_

#include "gnss_types.h"

#define SENSOR_GNSS_MIN_SATELLITES           (8U)
#define SENSOR_GNSS_MAX_HDOP                 (2.0F)
#define SENSOR_GNSS_RECOVERY_FIX_COUNT       (5U)
#define SENSOR_GNSS_AUTO_ORIGIN_SAMPLE_COUNT (50U)

uint8 sensor_gnss_init(void);
void sensor_gnss_service(uint32 now_ms);
uint8 sensor_gnss_take_snapshot(gnss_snapshot_struct *snapshot);
uint8 sensor_gnss_set_origin(double latitude_deg, double longitude_deg);

#endif

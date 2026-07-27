#ifndef _local_position_h_
#define _local_position_h_

#include "zf_common_typedef.h"

void local_position_reset(void);
uint8 local_position_set_origin(double latitude_deg, double longitude_deg);
uint8 local_position_has_origin(void);
uint8 local_position_project(double latitude_deg, double longitude_deg,
                             float *east_m, float *north_m);

#endif

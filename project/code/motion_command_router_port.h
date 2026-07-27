#ifndef _motion_command_router_port_h_
#define _motion_command_router_port_h_

#include "zf_common_typedef.h"

void motion_command_router_port_apply(float forward_rpm,
                                      float turn_rate_dps,
                                      uint8 enable,
                                      uint32 now_ms);
void motion_command_router_port_stop(uint32 now_ms);

#endif

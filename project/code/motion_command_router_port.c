#include "motion_command_router_port.h"
#include "control_chassis.h"

void motion_command_router_port_apply(float forward_rpm,
                                      float turn_rate_dps,
                                      uint8 enable,
                                      uint32 now_ms)
{
    control_chassis_set_cmd(forward_rpm, turn_rate_dps, enable, now_ms);
}

void motion_command_router_port_stop(uint32 now_ms)
{
    control_chassis_stop(now_ms);
}

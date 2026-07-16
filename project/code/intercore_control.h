#ifndef _intercore_control_h_
#define _intercore_control_h_

#include "intercore_protocol.h"

uint8 intercore_control_init(void);
void intercore_control_update(uint32 now_ms);
uint8 intercore_control_accept_navigation(const navigation_command_struct *command,
                                          uint32 now_ms);
uint8 intercore_control_get_latest_gnss(intercore_gnss_payload_struct *payload,
                                        uint32 *source_ms);

#endif

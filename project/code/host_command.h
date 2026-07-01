/*********************************************************************************************************************
* File: host_command.h
* Description: UART0/VOFA downlink command parser.
********************************************************************************************************************/

#ifndef _host_command_h_
#define _host_command_h_

#include "app_types.h"

void host_command_init(void);
void host_command_update(uint32 now_ms);

#endif

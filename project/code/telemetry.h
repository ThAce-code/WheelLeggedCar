/*********************************************************************************************************************
* File: telemetry.h
* Description: Debug telemetry interface.
********************************************************************************************************************/

#ifndef _telemetry_h_
#define _telemetry_h_

#include "zf_common_headfile.h"

void telemetry_init(void);
void telemetry_update(uint32 now_ms);

#endif

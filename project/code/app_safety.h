/*********************************************************************************************************************
* File: app_safety.h
* Description: Safety monitor interface.
********************************************************************************************************************/

#ifndef _app_safety_h_
#define _app_safety_h_

#include "app_types.h"

void app_safety_init(void);
void app_safety_update(uint32 now_ms);
uint8 app_safety_is_fault(void);
void app_safety_force_fault(void);

#endif

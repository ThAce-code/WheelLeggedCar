/*********************************************************************************************************************
* File: app_scheduler.h
* Description: Cooperative scheduler interface.
********************************************************************************************************************/

#ifndef _app_scheduler_h_
#define _app_scheduler_h_

#include "zf_common_headfile.h"

void app_scheduler_init(void);
void app_scheduler_tick_1ms(void);
void app_scheduler_run_pending(void);
uint32 app_scheduler_get_ms(void);

#endif

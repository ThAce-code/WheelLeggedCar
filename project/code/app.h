/*********************************************************************************************************************
* File: app.h
* Description: Top-level application interface.
********************************************************************************************************************/

#ifndef _app_h_
#define _app_h_

#include "zf_common_headfile.h"

uint8 app_init(void);
void app_run_once(void);
uint32 app_get_ms(void);

#endif

/*********************************************************************************************************************
* File: app_state.h
* Description: Top-level run state machine interface.
********************************************************************************************************************/

#ifndef _app_state_h_
#define _app_state_h_

#include "app_types.h"

void app_state_init(void);
void app_state_set(app_run_state_enum state);
app_run_state_enum app_state_get(void);
uint8 app_state_is_run_enabled(void);

#endif

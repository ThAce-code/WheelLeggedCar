/*********************************************************************************************************************
* File: app_state.c
* Description: Top-level run state machine.
********************************************************************************************************************/

#include "app_state.h"

static app_run_state_enum app_current_state = APP_STATE_BOOT;

void app_state_init(void)
{
    app_current_state = APP_STATE_BOOT;
}

void app_state_set(app_run_state_enum state)
{
    app_current_state = state;
}

app_run_state_enum app_state_get(void)
{
    return app_current_state;
}

uint8 app_state_is_run_enabled(void)
{
    return (APP_STATE_RUN == app_current_state) ? APP_TRUE : APP_FALSE;
}

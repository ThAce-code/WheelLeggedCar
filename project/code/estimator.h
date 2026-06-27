/*********************************************************************************************************************
* File: estimator.h
* Description: Vehicle state estimator interface.
********************************************************************************************************************/

#ifndef _estimator_h_
#define _estimator_h_

#include "app_types.h"

void estimator_init(void);
void estimator_update(uint32 now_ms);
const vehicle_state_struct *estimator_get_state(void);

#endif

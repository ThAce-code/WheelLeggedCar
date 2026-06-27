/*********************************************************************************************************************
* File: perception.h
* Description: Vision perception interface.
********************************************************************************************************************/

#ifndef _perception_h_
#define _perception_h_

#include "app_types.h"

void perception_init(void);
void perception_update(uint32 now_ms);
const vision_state_struct *perception_get_state(void);

#endif

/*********************************************************************************************************************
* File: dl1b_safety.h
* Description: Scheduled fail-closed DL1B snapshot service.
*********************************************************************************************************************/

#ifndef _dl1b_safety_h_
#define _dl1b_safety_h_

#include "single_gap_types.h"

uint8 dl1b_safety_init(uint32 now_ms);
void dl1b_safety_update(uint32 now_ms);
single_gap_tof_snapshot_struct dl1b_safety_get_snapshot(void);

#endif

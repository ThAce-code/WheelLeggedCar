/*********************************************************************************************************************
* File: dl1b_safety.c
* Description: Scheduled fail-closed DL1B snapshot service.
*********************************************************************************************************************/

#include "dl1b_safety.h"
#include "dl1b_safety_port.h"
#include "single_gap_config.h"

#include <string.h>

static single_gap_tof_snapshot_struct dl1b_snapshot;
static uint32 dl1b_last_poll_ms;

uint8 dl1b_safety_init(uint32 now_ms)
{
    memset(&dl1b_snapshot, 0, sizeof(dl1b_snapshot));
    dl1b_last_poll_ms = now_ms;
    if(0U == dl1b_safety_port_init())
    {
        return 0U;
    }
    dl1b_snapshot.initialized = 1U;
    return 1U;
}

void dl1b_safety_update(uint32 now_ms)
{
    uint16 distance_mm;

    if((0U == dl1b_snapshot.initialized) ||
       (SINGLE_GAP_TOF_PERIOD_MS > (now_ms - dl1b_last_poll_ms)))
    {
        return;
    }

    dl1b_last_poll_ms = now_ms;
    if(0U == dl1b_safety_port_read(&distance_mm))
    {
        dl1b_snapshot.valid = 0U;
        return;
    }

    dl1b_snapshot.distance_mm = distance_mm;
    dl1b_snapshot.sample_ms = now_ms;
    dl1b_snapshot.valid = 1U;
}

single_gap_tof_snapshot_struct dl1b_safety_get_snapshot(void)
{
    return dl1b_snapshot;
}

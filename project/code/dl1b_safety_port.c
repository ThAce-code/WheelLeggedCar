/*********************************************************************************************************************
* File: dl1b_safety_port.c
* Description: Hardware boundary for the vendor DL1B driver.
*********************************************************************************************************************/

#include "dl1b_safety_port.h"
#include "zf_device_dl1b.h"

uint8 dl1b_safety_port_init(void)
{
    return (0U == dl1b_init()) ? 1U : 0U;
}

uint8 dl1b_safety_port_read(uint16 *distance_mm)
{
    uint16 measured_mm;

    if(NULL == distance_mm)
    {
        return 0U;
    }

    dl1b_get_distance();
    measured_mm = dl1b_distance_mm;
    if((0U == dl1b_finsh_flag) || (1U > measured_mm) || (4000U < measured_mm))
    {
        return 0U;
    }

    *distance_mm = measured_mm;
    return 1U;
}

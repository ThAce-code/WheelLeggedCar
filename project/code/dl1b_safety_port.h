/*********************************************************************************************************************
* File: dl1b_safety_port.h
* Description: Hardware boundary for the vendor DL1B driver.
*********************************************************************************************************************/

#ifndef _dl1b_safety_port_h_
#define _dl1b_safety_port_h_

#include "zf_common_typedef.h"

uint8 dl1b_safety_port_init(void);
uint8 dl1b_safety_port_read(uint16 *distance_mm);

#endif

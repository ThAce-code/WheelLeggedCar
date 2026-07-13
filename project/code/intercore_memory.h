#ifndef _intercore_memory_h_
#define _intercore_memory_h_

#include "intercore_protocol.h"

uint8 intercore_memory_configure(void);
volatile intercore_shared_layout_struct *intercore_memory_get_layout(void);

#endif

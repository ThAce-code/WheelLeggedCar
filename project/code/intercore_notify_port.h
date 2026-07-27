#ifndef _intercore_notify_port_h_
#define _intercore_notify_port_h_

#include "intercore_notify.h"

uint8 intercore_notify_port_init(intercore_role_enum role);
uint8 intercore_notify_port_send(const intercore_doorbell_struct *message);

#endif

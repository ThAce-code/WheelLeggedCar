#ifndef _host_mock_actuator_motor_h_
#define _host_mock_actuator_motor_h_

#include "zf_common_typedef.h"

typedef struct
{
    uint32 age_ms;
    uint8 online;
    uint8 left_online;
    uint8 right_online;
} wheel_feedback_struct;

const wheel_feedback_struct *actuator_motor_get_feedback(void);

#endif

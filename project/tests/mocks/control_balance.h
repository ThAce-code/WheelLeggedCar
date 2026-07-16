#ifndef _host_mock_control_balance_h_
#define _host_mock_control_balance_h_

typedef struct
{
    float wheel_speed_rpm;
} balance_diag_struct;

const balance_diag_struct *control_balance_get_diag(void);

#endif

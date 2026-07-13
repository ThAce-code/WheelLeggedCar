#include "intercore_notify_port.h"

#ifndef INTERCORE_HOST_TEST
#include "ipc/cy_ipc_pipe.h"

static uint32 intercore_notify_destination;

static void intercore_notify_port_receive(uint32 *message)
{
    intercore_doorbell_struct *doorbell = (intercore_doorbell_struct *)message;

    intercore_notify_receive_callback(doorbell->data);
}

uint8 intercore_notify_port_init(intercore_role_enum role)
{
    cy_stc_ipc_pipe_config_t pipe_config =
    {
        .epIndexForThisCpu = (uint8)((INTERCORE_ROLE_CM7_0 == role) ? 0U : 1U),
        .epConfigData = CY_IPC_PIPE_ENDPOINTS_DEFAULT_CONFIG
    };

    Cy_IPC_Pipe_Init(&pipe_config);
    if(CY_IPC_PIPE_SUCCESS !=
       Cy_IPC_Pipe_RegisterCallback(intercore_notify_port_receive,
                                    INTERCORE_NOTIFY_CLIENT_ID))
    {
        return 0U;
    }

    NVIC_ClearPendingIRQ(pipe_config.epConfigData[pipe_config.epIndexForThisCpu].ipcCpuIntIdx);
    NVIC_EnableIRQ(pipe_config.epConfigData[pipe_config.epIndexForThisCpu].ipcCpuIntIdx);
    intercore_notify_destination =
        (INTERCORE_ROLE_CM7_0 == role) ? 1U : 0U;
    return 1U;
}

uint8 intercore_notify_port_send(const intercore_doorbell_struct *message)
{
    return (CY_IPC_PIPE_SUCCESS ==
            Cy_IPC_Pipe_SendMessage(intercore_notify_destination,
                                    (void *)message,
                                    intercore_notify_release_callback)) ? 1U : 0U;
}

#else

uint8 intercore_notify_port_init(intercore_role_enum role)
{
    (void)role;
    return 1U;
}

uint8 intercore_notify_port_send(const intercore_doorbell_struct *message)
{
    (void)message;
    return 1U;
}

#endif

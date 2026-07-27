#include "zf_common_headfile.h"
#include "intercore_memory.h"

/* These symbols are absolute linker symbols, not storage allocated by C. */
extern uint8 __intercore_shared_sram_base;
extern uint8 __intercore_shared_sram_size;
extern uint8 __camera_shared_sram_base;
extern uint8 __camera_shared_sram_size;

uint8 intercore_memory_configure(void)
{
    uint32 shared_base = (uint32)(uintptr_t)&__intercore_shared_sram_base;
    uint32 shared_size = (uint32)(uintptr_t)&__intercore_shared_sram_size;
    uint32 camera_base = (uint32)(uintptr_t)&__camera_shared_sram_base;
    uint32 camera_size = (uint32)(uintptr_t)&__camera_shared_sram_size;
    const cy_stc_mpu_region_cfg_t regions[2] =
    {
        {
            .addr = INTERCORE_SHARED_BASE_ADDRESS,
            .size = CY_MPU_SIZE_8KB,
            .permission = CY_MPU_ACCESS_P_FULL_ACCESS,
            .attribute = CY_MPU_ATTR_NORM_SHR_MEM_NC,
            .execute = CY_MPU_INST_ACCESS_DIS,
            .srd = 0U,
            .enable = CY_MPU_ENABLE
        },
        {
            .addr = INTERCORE_CAMERA_DATA_BASE_ADDRESS,
            .size = CY_MPU_SIZE_64KB,
            .permission = CY_MPU_ACCESS_P_FULL_ACCESS,
            .attribute = CY_MPU_ATTR_NORM_SHR_MEM_NC,
            .execute = CY_MPU_INST_ACCESS_DIS,
            .srd = 0U,
            .enable = CY_MPU_ENABLE
        }
    };

    if((INTERCORE_SHARED_BASE_ADDRESS != shared_base) ||
       (INTERCORE_SHARED_SIZE_BYTES != shared_size) ||
       (0U != (shared_base & (INTERCORE_SHARED_SIZE_BYTES - 1U))) ||
       (INTERCORE_CAMERA_DATA_BASE_ADDRESS != camera_base) ||
       (INTERCORE_CAMERA_DATA_SIZE_BYTES != camera_size) ||
       (0U != (camera_base & (INTERCORE_CAMERA_DATA_SIZE_BYTES - 1U))))
    {
        return 0U;
    }

    SCB_CleanInvalidateDCache_by_Addr((volatile void *)shared_base,
                                     (int32_t)shared_size);
    SCB_CleanInvalidateDCache_by_Addr((volatile void *)camera_base,
                                     (int32_t)camera_size);
    __DSB();
    __ISB();

    if(CY_MPU_FAILURE == Cy_MPU_Setup(regions,
                                      2U,
                                      CY_MPU_USE_DEFAULT_MAP_AS_BG,
                                      CY_MPU_DISABLED_DURING_FAULT_NMI))
    {
        return 0U;
    }

    __DSB();
    __ISB();
    return 1U;
}

volatile intercore_shared_layout_struct *intercore_memory_get_layout(void)
{
    return (volatile intercore_shared_layout_struct *)(uintptr_t)INTERCORE_SHARED_BASE_ADDRESS;
}

volatile uint8 *intercore_memory_get_camera_data(void)
{
    return (volatile uint8 *)(uintptr_t)INTERCORE_CAMERA_DATA_BASE_ADDRESS;
}

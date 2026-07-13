#include "zf_common_headfile.h"
#include "intercore_memory.h"

/* These symbols are absolute linker symbols, not storage allocated by C. */
extern uint8 __intercore_shared_sram_base;
extern uint8 __intercore_shared_sram_size;

uint8 intercore_memory_configure(void)
{
    uint32 base = (uint32)(uintptr_t)&__intercore_shared_sram_base;
    uint32 size = (uint32)(uintptr_t)&__intercore_shared_sram_size;
    const cy_stc_mpu_region_cfg_t shared_region =
    {
        .addr = INTERCORE_SHARED_BASE_ADDRESS,
        .size = CY_MPU_SIZE_8KB,
        .permission = CY_MPU_ACCESS_P_FULL_ACCESS,
        .attribute = CY_MPU_ATTR_NORM_SHR_MEM_NC,
        .execute = CY_MPU_INST_ACCESS_DIS,
        .srd = 0U,
        .enable = CY_MPU_ENABLE
    };

    if((INTERCORE_SHARED_BASE_ADDRESS != base) ||
       (INTERCORE_SHARED_SIZE_BYTES != size) ||
       (0U != (base & (INTERCORE_SHARED_SIZE_BYTES - 1U))))
    {
        return 0U;
    }

    SCB_CleanInvalidateDCache_by_Addr((volatile void *)base, (int32_t)size);
    __DSB();
    __ISB();

    if(CY_MPU_FAILURE == Cy_MPU_Setup(&shared_region,
                                      1U,
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

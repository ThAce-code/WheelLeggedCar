/*********************************************************************************************************************
* File: cm7_0_isr.c
* Description: Interrupt handlers for the CM7_0 application core.
********************************************************************************************************************/


#include "zf_common_headfile.h"
#include "app_scheduler.h"
#include "bldc_foc_uart.h"
// **************************** PIT interrupt handlers ****************************
void pit0_ch0_isr()
{
    pit_isr_flag_clear(PIT_CH0);
    app_scheduler_tick_1ms();
  

	
	
	
	
}

void pit0_ch1_isr()
{
    pit_isr_flag_clear(PIT_CH1);
	
	
	
	
}


void pit0_ch2_isr()
{
    pit_isr_flag_clear(PIT_CH2);
	
	
	
	
}

void pit0_ch10_isr()
{
    pit_isr_flag_clear(PIT_CH10);
}

void pit0_ch11_isr()
{
    pit_isr_flag_clear(PIT_CH11);
}

void pit0_ch12_isr()
{
    pit_isr_flag_clear(PIT_CH12);
}

void pit0_ch13_isr()
{
    pit_isr_flag_clear(PIT_CH13);
}

void pit0_ch14_isr()
{
    pit_isr_flag_clear(PIT_CH14);
}

void pit0_ch15_isr()
{
    pit_isr_flag_clear(PIT_CH15);
}

void pit0_ch16_isr()
{
    pit_isr_flag_clear(PIT_CH16);
}

void pit0_ch17_isr()
{
    pit_isr_flag_clear(PIT_CH17);
}

void pit0_ch18_isr()
{
    pit_isr_flag_clear(PIT_CH18);
}

void pit0_ch19_isr()
{
    pit_isr_flag_clear(PIT_CH19);
}

void pit0_ch20_isr()
{
    pit_isr_flag_clear(PIT_CH20);
}

void pit0_ch21_isr()
{
    pit_isr_flag_clear(PIT_CH21);
}
// **************************** PIT interrupt handlers ****************************


// **************************** GPIO external interrupt handlers ****************************
void gpio_0_exti_isr()
{
    
  
  
}

void gpio_1_exti_isr()
{
    if(exti_flag_get(P01_0))
    {

      
      
            
    }
    if(exti_flag_get(P01_1))
    {

            
            
    }
}

void gpio_2_exti_isr()
{
    if(exti_flag_get(P02_0))
    {
            
            
    }
    if(exti_flag_get(P02_4))
    {
            
            
    }

}

void gpio_3_exti_isr()
{



}

void gpio_4_exti_isr()
{



}

void gpio_5_exti_isr()
{



}

void gpio_6_exti_isr()
{
	


}

void gpio_7_exti_isr()
{



}

void gpio_8_exti_isr()
{



}

void gpio_9_exti_isr()
{



}

void gpio_10_exti_isr()
{



}

void gpio_11_exti_isr()
{



}

void gpio_12_exti_isr()
{



}

void gpio_13_exti_isr()
{



}

void gpio_14_exti_isr()
{



}

void gpio_15_exti_isr()
{



}

void gpio_16_exti_isr()
{



}

void gpio_17_exti_isr()
{



}

void gpio_18_exti_isr()
{



}

void gpio_19_exti_isr()
{



}

void gpio_20_exti_isr()
{



}

void gpio_21_exti_isr()
{



}

void gpio_22_exti_isr()
{



}

void gpio_23_exti_isr()
{



}
// **************************** GPIO external interrupt handlers ****************************

//// **************************** DMA interrupt handler ****************************
//void dma_event_callback(void* callback_arg, cyhal_dma_event_t event)
//{
//    CY_UNUSED_PARAMETER(event);
//	
//
//	
//	
//}
// **************************** DMA interrupt handler ****************************

// **************************** UART interrupt handlers ****************************
// UART0 is the default debug UART.
void uart0_isr (void)
{
    if(uart_isr_mask(UART_0))
    {
#if DEBUG_UART_USE_INTERRUPT
        debug_interrupr_handler();
#endif
    }
}

void uart1_isr (void)
{
    if(uart_isr_mask(UART_1))
    {
        bldc_foc_uart_rx_isr();
    }
}

void uart2_isr (void)
{
    if(uart_isr_mask(UART_2))
    {
        gnss_uart_callback();
    }
}

void uart3_isr (void)
{
    (void)uart_isr_mask(UART_3);
}

void uart4_isr (void)
{
    if(uart_isr_mask(UART_4))
    {
        uart_receiver_handler();
    }
}

void uart5_isr (void)
{
    (void)uart_isr_mask(UART_5);
}

void uart6_isr (void)
{
    (void)uart_isr_mask(UART_6);
}
// **************************** UART interrupt handlers ****************************

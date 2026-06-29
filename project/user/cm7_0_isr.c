/*********************************************************************************************************************
* CYT4BB Opensourec Library ���� CYT4BB ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� CYT4BB ��Դ���һ����
*
* CYT4BB ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          isr_arm_7_0
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT4BB
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-1-9      pudding            first version
********************************************************************************************************************/


#include "zf_common_headfile.h"
#include "app_scheduler.h"
#include "bldc_foc_uart.h"
// **************************** PIT�жϺ��� ****************************
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
// **************************** PIT�жϺ��� ****************************


// **************************** �ⲿ�жϺ��� ****************************
void gpio_0_exti_isr()
{
    
  
  
}

void gpio_1_exti_isr()
{
    if(exti_flag_get(P01_0))				// ʾ��P1_0�˿��ⲿ�ж��ж�
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
// **************************** �ⲿ�жϺ��� ****************************

//// **************************** DMA�жϺ��� ****************************
//void dma_event_callback(void* callback_arg, cyhal_dma_event_t event)
//{
//    CY_UNUSED_PARAMETER(event);
//	
//
//	
//	
//}
// **************************** DMA�жϺ��� ****************************

// **************************** �����жϺ��� ****************************
// ����0Ĭ����Ϊ���Դ���
void uart0_isr (void)
{
    if(Cy_SCB_GetRxInterruptMask(get_scb_module(UART_0)) & CY_SCB_UART_RX_NOT_EMPTY)            // ����0�����ж�
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_0), CY_SCB_UART_RX_NOT_EMPTY);              // ��������жϱ�־λ
        
#if DEBUG_UART_USE_INTERRUPT                        				                // ������� debug �����ж�
        debug_interrupr_handler();                  				                // ���� debug ���ڽ��մ������� ���ݻᱻ debug ���λ�������ȡ
#endif                                              				                // ����޸��� DEBUG_UART_INDEX ����δ�����Ҫ�ŵ���Ӧ�Ĵ����ж�ȥ
      
        
        
    }
    else if(Cy_SCB_GetTxInterruptMask(get_scb_module(UART_0)) & CY_SCB_UART_TX_DONE)            // ����0�����ж�
    {           
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_0), CY_SCB_UART_TX_DONE);                   // ��������жϱ�־λ
        
        
        
    }
}

void uart1_isr (void)
{
    if(Cy_SCB_GetRxInterruptMask(get_scb_module(UART_1)) & CY_SCB_UART_RX_NOT_EMPTY)            // ����1�����ж�
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_1), CY_SCB_UART_RX_NOT_EMPTY);              // ��������жϱ�־λ

        bldc_foc_uart_rx_isr();
        
        
    }
    else if(Cy_SCB_GetTxInterruptMask(get_scb_module(UART_1)) & CY_SCB_UART_TX_DONE)            // ����1�����ж�
    {
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_1), CY_SCB_UART_TX_DONE);                   // ��������жϱ�־λ
        
        
        
    }
}

void uart2_isr (void)
{
    if(Cy_SCB_GetRxInterruptMask(get_scb_module(UART_2)) & CY_SCB_UART_RX_NOT_EMPTY)            // ����2�����ж�
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_2), CY_SCB_UART_RX_NOT_EMPTY);              // ��������жϱ�־λ

        gnss_uart_callback();
        
        
    }
    else if(Cy_SCB_GetTxInterruptMask(get_scb_module(UART_2)) & CY_SCB_UART_TX_DONE)            // ����2�����ж�
    {
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_2), CY_SCB_UART_TX_DONE);                   // ��������жϱ�־λ
        
        
        
    }
}

void uart3_isr (void)
{
    if(Cy_SCB_GetRxInterruptMask(get_scb_module(UART_3)) & CY_SCB_UART_RX_NOT_EMPTY)            // ����3�����ж�
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_3), CY_SCB_UART_RX_NOT_EMPTY);              // ��������жϱ�־λ

        
        
        
    }
    else if(Cy_SCB_GetTxInterruptMask(get_scb_module(UART_3)) & CY_SCB_UART_TX_DONE)            // ����3�����ж�
    {
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_3), CY_SCB_UART_TX_DONE);                   // ��������жϱ�־λ
        
        
        
    }
}

void uart4_isr (void)
{
    
    if(Cy_SCB_GetRxInterruptMask(get_scb_module(UART_4)) & CY_SCB_UART_RX_NOT_EMPTY)            // ����4�����ж�
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_4), CY_SCB_UART_RX_NOT_EMPTY);              // ��������жϱ�־λ

        
        uart_receiver_handler();                                                                // ���ڽ��ջ��ص�����
        
        
    }
    else if(Cy_SCB_GetTxInterruptMask(get_scb_module(UART_4)) & CY_SCB_UART_TX_DONE)            // ����4�����ж�
    {
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_4), CY_SCB_UART_TX_DONE);                   // ��������жϱ�־λ
        
        
        
    }
}
// **************************** �����жϺ��� ****************************
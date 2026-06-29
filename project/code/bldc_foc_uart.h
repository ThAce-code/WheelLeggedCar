/*********************************************************************************************************************
* File: bldc_foc_uart.h
* Description: UART protocol adapter for dual BLDC FOC driver.
********************************************************************************************************************/

#ifndef _bldc_foc_uart_h_
#define _bldc_foc_uart_h_

#include "app_types.h"

#define BLDC_FOC_ASCII_LINE_MAX         (64U)

typedef struct
{
    int16 left_speed;
    int16 right_speed;
    int16 left_angle;
    int16 right_angle;
    int16 left_reduced_angle;
    int16 right_reduced_angle;
    char  last_unknown_ascii[BLDC_FOC_ASCII_LINE_MAX];
    uint32 last_rx_ms;
    uint32 checksum_error_count;
    uint32 unknown_frame_count;
    uint8 online;
}bldc_foc_feedback_struct;

void bldc_foc_uart_init(void);
void bldc_foc_uart_set_duty(int16 left_duty, int16 right_duty);
void bldc_foc_uart_stop(void);
void bldc_foc_uart_start_feedback(void);
void bldc_foc_uart_stop_feedback(void);
void bldc_foc_uart_zero_calibrate(void);
void bldc_foc_uart_set_angle_zero(void);
void bldc_foc_uart_rx_isr(void);
const bldc_foc_feedback_struct *bldc_foc_uart_get_feedback(void);

#endif

/*********************************************************************************************************************
* File: bldc_foc_uart.c
* Description: UART protocol adapter for dual BLDC FOC driver.
********************************************************************************************************************/

#include "bldc_foc_uart.h"
#include "app_config.h"
#include "app_scheduler.h"
#include "zf_common_interrupt.h"
#include "zf_driver_gpio.h"

#define BLDC_FOC_FRAME_HEAD             (0xA5U)
#define BLDC_FOC_FRAME_LEN              (7U)
#define BLDC_FOC_FRAME_PAYLOAD_LEN      (6U)

#define BLDC_FOC_FUNC_SET_DUTY          (0x01U)
#define BLDC_FOC_FUNC_UPLOAD_SPEED      (0x02U)
#define BLDC_FOC_FUNC_ZERO_CALIBRATE    (0x03U)
#define BLDC_FOC_FUNC_UPLOAD_ANGLE      (0x04U)
#define BLDC_FOC_FUNC_UPLOAD_RDT_ANGLE  (0x05U)
#define BLDC_FOC_FUNC_SET_ANGLE_ZERO    (0x06U)

static bldc_foc_feedback_struct bldc_foc_feedback;
static uint8 bldc_foc_initialized = APP_FALSE;
static uint8 bldc_foc_packet[BLDC_FOC_FRAME_LEN];
static uint8 bldc_foc_packet_index = 0;
static char bldc_foc_ascii_line[BLDC_FOC_ASCII_LINE_MAX];
static uint8 bldc_foc_ascii_index = 0;

#if APP_BLDC_USE_ASCII_COMMANDS
static void bldc_foc_send_string(const char *str);
#endif

static void bldc_foc_clear_feedback(void)
{
    uint8 i;

    bldc_foc_feedback.left_motor_rpm = 0;
    bldc_foc_feedback.right_motor_rpm = 0;
    bldc_foc_feedback.left_angle = 0;
    bldc_foc_feedback.right_angle = 0;
    bldc_foc_feedback.left_reduced_angle = 0;
    bldc_foc_feedback.right_reduced_angle = 0;
    bldc_foc_feedback.last_tx_left = 0;
    bldc_foc_feedback.last_tx_right = 0;
    bldc_foc_feedback.last_rx_ms = 0;
    bldc_foc_feedback.checksum_error_count = 0;
    bldc_foc_feedback.feedback_range_error_count = 0;
    bldc_foc_feedback.unknown_frame_count = 0;
    bldc_foc_feedback.tx_frame_count = 0;
    bldc_foc_feedback.last_tx_func = 0;
    bldc_foc_feedback.online = APP_FALSE;

    for(i = 0; i < BLDC_FOC_ASCII_LINE_MAX; i++)
    {
        bldc_foc_feedback.last_unknown_ascii[i] = '\0';
        bldc_foc_ascii_line[i] = '\0';
    }
}

static uint8 bldc_foc_checksum(const uint8 *frame)
{
    uint8 i;
    uint16 sum = 0;

    for(i = 0; i < BLDC_FOC_FRAME_PAYLOAD_LEN; i++)
    {
        sum += frame[i];
    }
    return (uint8)(sum & 0xFFU);
}

static int16 bldc_foc_unpack_int16(uint8 high, uint8 low)
{
    return (int16)(((uint16)high << 8) | (uint16)low);
}

static void bldc_foc_pack_int16(uint8 *high, uint8 *low, int16 value)
{
    uint16 raw;

    raw = (uint16)value;
    *high = (uint8)((raw >> 8) & 0xFFU);
    *low = (uint8)(raw & 0xFFU);
}

#if APP_BLDC_USE_ASCII_COMMANDS
static void bldc_foc_append_text(char *buffer, uint8 *index, const char *text)
{
    while('\0' != *text)
    {
        buffer[*index] = *text;
        (*index)++;
        text++;
    }
}

static void bldc_foc_append_int16(char *buffer, uint8 *index, int16 value)
{
    char digits[6];
    uint8 digit_count = 0;
    int32 temp;

    temp = (int32)value;
    if(0 > temp)
    {
        buffer[*index] = '-';
        (*index)++;
        temp = -temp;
    }

    do
    {
        digits[digit_count] = (char)('0' + (temp % 10));
        digit_count++;
        temp /= 10;
    }while(0 != temp);

    while(0U < digit_count)
    {
        digit_count--;
        buffer[*index] = digits[digit_count];
        (*index)++;
    }
}

static void bldc_foc_send_ascii_duty(int16 left_value, int16 right_value)
{
    char command[40];
    uint8 index = 0;

    bldc_foc_append_text(command, &index, "SET-DUTY,");
    bldc_foc_append_int16(command, &index, left_value);
    command[index] = ',';
    index++;
    bldc_foc_append_int16(command, &index, right_value);
    command[index] = '\r';
    index++;
    command[index] = '\0';

    bldc_foc_send_string(command);
}

static uint8 bldc_foc_send_ascii_command(uint8 func, int16 left_value, int16 right_value)
{
    switch(func)
    {
        case BLDC_FOC_FUNC_SET_DUTY:
            bldc_foc_send_ascii_duty(left_value, right_value);
            break;

        case BLDC_FOC_FUNC_UPLOAD_SPEED:
            bldc_foc_send_string("GET-SPEED\r");
            break;

        case BLDC_FOC_FUNC_ZERO_CALIBRATE:
            bldc_foc_send_string("SET-ZERO\r");
            break;

        case BLDC_FOC_FUNC_UPLOAD_ANGLE:
            bldc_foc_send_string("GET-ANGLE\r");
            break;

        case BLDC_FOC_FUNC_UPLOAD_RDT_ANGLE:
            bldc_foc_send_string("GET-RDT-ANGLE\r");
            break;

        case BLDC_FOC_FUNC_SET_ANGLE_ZERO:
            bldc_foc_send_string("SET-ANGLE-ZERO\r");
            break;

        default:
            return APP_FALSE;
    }

    return APP_TRUE;
}
#endif

static void bldc_foc_send_frame(uint8 func, int16 left_value, int16 right_value)
{
    uint8 frame[BLDC_FOC_FRAME_LEN];

    if(APP_FALSE == bldc_foc_initialized)
    {
        return;
    }

    frame[0] = BLDC_FOC_FRAME_HEAD;
    frame[1] = func;
    bldc_foc_pack_int16(&frame[2], &frame[3], left_value);
    bldc_foc_pack_int16(&frame[4], &frame[5], right_value);
    frame[6] = bldc_foc_checksum(frame);

    bldc_foc_feedback.last_tx_func = func;
    bldc_foc_feedback.last_tx_left = left_value;
    bldc_foc_feedback.last_tx_right = right_value;
    bldc_foc_feedback.tx_frame_count++;

#if APP_BLDC_TX_GPIO_PROBE_ENABLE
    gpio_toggle_level(P04_1);
    return;
#endif

#if APP_BLDC_USE_ASCII_COMMANDS
    if(APP_TRUE == bldc_foc_send_ascii_command(func, left_value, right_value))
    {
        return;
    }
#endif

    uart_write_buffer(APP_BLDC_UART_INDEX, frame, BLDC_FOC_FRAME_LEN);
}

#if APP_BLDC_USE_ASCII_COMMANDS
static void bldc_foc_send_string(const char *str)
{
    if(APP_FALSE == bldc_foc_initialized)
    {
        return;
    }
    uart_write_string(APP_BLDC_UART_INDEX, str);
}
#endif

static void bldc_foc_mark_rx(void)
{
    bldc_foc_feedback.last_rx_ms = app_scheduler_get_ms();
    bldc_foc_feedback.online = APP_TRUE;
}

static void bldc_foc_save_ascii_line(void)
{
    uint8 i;

    if(0U == bldc_foc_ascii_index)
    {
        return;
    }

    bldc_foc_ascii_line[bldc_foc_ascii_index] = '\0';
    for(i = 0; i < BLDC_FOC_ASCII_LINE_MAX; i++)
    {
        bldc_foc_feedback.last_unknown_ascii[i] = bldc_foc_ascii_line[i];
        if('\0' == bldc_foc_ascii_line[i])
        {
            break;
        }
    }
    bldc_foc_ascii_index = 0;
    bldc_foc_feedback.unknown_frame_count++;
}

static void bldc_foc_parse_ascii_byte(uint8 dat)
{
    if(('\r' == dat) || ('\n' == dat))
    {
        bldc_foc_save_ascii_line();
        return;
    }

    if((0x20U <= dat) && (BLDC_FOC_ASCII_LINE_MAX - 1U > bldc_foc_ascii_index))
    {
        bldc_foc_ascii_line[bldc_foc_ascii_index] = (char)dat;
        bldc_foc_ascii_index++;
    }
}

static void bldc_foc_resync_after_bad_packet(void)
{
    uint8 i;

    for(i = 1U; i < BLDC_FOC_FRAME_LEN; i++)
    {
        if(BLDC_FOC_FRAME_HEAD == bldc_foc_packet[i])
        {
            bldc_foc_packet[0] = BLDC_FOC_FRAME_HEAD;
            bldc_foc_packet_index = 1U;
            return;
        }
    }

    bldc_foc_packet_index = 0U;
}

static void bldc_foc_process_packet(void)
{
    int16 left_value;
    int16 right_value;

    if(bldc_foc_packet[BLDC_FOC_FRAME_LEN - 1U] != bldc_foc_checksum(bldc_foc_packet))
    {
        bldc_foc_feedback.checksum_error_count++;
        bldc_foc_resync_after_bad_packet();
        return;
    }

    left_value = bldc_foc_unpack_int16(bldc_foc_packet[2], bldc_foc_packet[3]);
    right_value = bldc_foc_unpack_int16(bldc_foc_packet[4], bldc_foc_packet[5]);

    switch(bldc_foc_packet[1])
    {
        case BLDC_FOC_FUNC_UPLOAD_SPEED:
            if(((int16)APP_BLDC_FEEDBACK_RPM_ABS_MAX < left_value) ||
               ((int16)-APP_BLDC_FEEDBACK_RPM_ABS_MAX > left_value) ||
               ((int16)APP_BLDC_FEEDBACK_RPM_ABS_MAX < right_value) ||
               ((int16)-APP_BLDC_FEEDBACK_RPM_ABS_MAX > right_value))
            {
                bldc_foc_feedback.feedback_range_error_count++;
            }
            else
            {
                bldc_foc_feedback.left_motor_rpm = left_value;
                bldc_foc_feedback.right_motor_rpm = right_value;
                bldc_foc_mark_rx();
            }
            break;

        case BLDC_FOC_FUNC_UPLOAD_ANGLE:
            bldc_foc_feedback.left_angle = left_value;
            bldc_foc_feedback.right_angle = right_value;
            break;

        case BLDC_FOC_FUNC_UPLOAD_RDT_ANGLE:
            bldc_foc_feedback.left_reduced_angle = left_value;
            bldc_foc_feedback.right_reduced_angle = right_value;
            bldc_foc_mark_rx();
            break;

        default:
            bldc_foc_feedback.unknown_frame_count++;
            break;
    }

    bldc_foc_packet_index = 0U;
}

static void bldc_foc_parse_byte(uint8 dat)
{
    if(0U == bldc_foc_packet_index)
    {
        if(BLDC_FOC_FRAME_HEAD == dat)
        {
            bldc_foc_packet[0] = dat;
            bldc_foc_packet_index = 1;
        }
        else
        {
            bldc_foc_parse_ascii_byte(dat);
        }
        return;
    }

    bldc_foc_packet[bldc_foc_packet_index] = dat;
    bldc_foc_packet_index++;
    if(BLDC_FOC_FRAME_LEN <= bldc_foc_packet_index)
    {
        bldc_foc_process_packet();
        if(BLDC_FOC_FRAME_LEN <= bldc_foc_packet_index)
        {
            bldc_foc_packet_index = 0;
        }
    }
}

void bldc_foc_uart_init(void)
{
    bldc_foc_clear_feedback();
    bldc_foc_packet_index = 0;
    bldc_foc_ascii_index = 0;

#if APP_BLDC_TX_GPIO_PROBE_ENABLE
    gpio_init(P04_1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    bldc_foc_initialized = APP_TRUE;
    return;
#endif

    uart_init(APP_BLDC_UART_INDEX,
              APP_BLDC_UART_BAUDRATE,
              APP_BLDC_UART_TX_PIN,
              APP_BLDC_UART_RX_PIN);
    uart_rx_interrupt(APP_BLDC_UART_INDEX, 0);
    bldc_foc_initialized = APP_TRUE;
}

void bldc_foc_uart_set_duty(int16 left_duty, int16 right_duty)
{
    bldc_foc_send_frame(BLDC_FOC_FUNC_SET_DUTY, left_duty, right_duty);
}

void bldc_foc_uart_stop(void)
{
    bldc_foc_uart_set_duty(0, 0);
}

void bldc_foc_uart_start_feedback(void)
{
    uart_rx_interrupt(APP_BLDC_UART_INDEX, 1);
    bldc_foc_send_frame(BLDC_FOC_FUNC_UPLOAD_SPEED, 0, 0);
}

void bldc_foc_uart_stop_feedback(void)
{
    uart_rx_interrupt(APP_BLDC_UART_INDEX, 0);
#if APP_BLDC_USE_ASCII_COMMANDS
    bldc_foc_send_string("STOP-SEND\r");
#endif
}

void bldc_foc_uart_zero_calibrate(void)
{
    bldc_foc_send_frame(BLDC_FOC_FUNC_ZERO_CALIBRATE, 0, 0);
}

void bldc_foc_uart_set_angle_zero(void)
{
    bldc_foc_send_frame(BLDC_FOC_FUNC_SET_ANGLE_ZERO, 0, 0);
}

void bldc_foc_uart_rx_isr(void)
{
    uint8 dat;

    while(uart_query_byte(APP_BLDC_UART_INDEX, &dat))
    {
        bldc_foc_parse_byte(dat);
    }
}

void bldc_foc_uart_copy_feedback(bldc_foc_feedback_struct *snapshot)
{
    uint32 primask;

    primask = interrupt_global_disable();
    *snapshot = bldc_foc_feedback;
    interrupt_global_enable(primask);
}

const bldc_foc_feedback_struct *bldc_foc_uart_get_feedback(void)
{
    return &bldc_foc_feedback;
}

/*********************************************************************************************************************
* File: host_command.c
* Description: UART0/VOFA downlink command parser.
********************************************************************************************************************/

#include "host_command.h"
#include "actuator_motor.h"
#include "zf_common_debug.h"

#define HOST_COMMAND_RX_BUFFER_LEN       (32U)
#define HOST_COMMAND_LINE_MAX            (32U)

static char host_command_line[HOST_COMMAND_LINE_MAX];
static uint8 host_command_index = 0;

static uint8 host_command_is_space(uint8 ch)
{
    return ((' ' == ch) || ('\t' == ch)) ? APP_TRUE : APP_FALSE;
}

static uint8 host_command_match_stop(const char *line)
{
    return (('S' == line[0]) &&
            ('T' == line[1]) &&
            ('O' == line[2]) &&
            ('P' == line[3]) &&
            ('\0' == line[4])) ? APP_TRUE : APP_FALSE;
}

static uint8 host_command_parse_number(const char *text, float *value)
{
    uint8 index = 0;
    uint8 digit_found = APP_FALSE;
    float sign = 1.0f;
    float result = 0.0f;
    float fraction_scale = 0.1f;

    if('-' == text[index])
    {
        sign = -1.0f;
        index++;
    }
    else if('+' == text[index])
    {
        index++;
    }

    while(('0' <= text[index]) && ('9' >= text[index]))
    {
        digit_found = APP_TRUE;
        result = (result * 10.0f) + (float)(text[index] - '0');
        index++;
    }

    if('.' == text[index])
    {
        index++;
        while(('0' <= text[index]) && ('9' >= text[index]))
        {
            digit_found = APP_TRUE;
            result += ((float)(text[index] - '0') * fraction_scale);
            fraction_scale *= 0.1f;
            index++;
        }
    }

    if('\0' != text[index])
    {
        return APP_FALSE;
    }
    if(APP_FALSE == digit_found)
    {
        return APP_FALSE;
    }

    *value = sign * result;
    return APP_TRUE;
}

static uint8 host_command_parse_pid_gain(const char *text, float *kp, float *ki, float *kd)
{
    char number_text[16];
    float values[3];
    uint8 value_index = 0;
    uint8 number_index = 0;
    uint8 read_index = 0;

    while('\0' != text[read_index])
    {
        if(',' == text[read_index])
        {
            if((0U == number_index) || (3U <= value_index))
            {
                return APP_FALSE;
            }
            number_text[number_index] = '\0';
            if(APP_FALSE == host_command_parse_number(number_text, &values[value_index]))
            {
                return APP_FALSE;
            }
            value_index++;
            number_index = 0;
        }
        else
        {
            if((sizeof(number_text) - 1U) <= number_index)
            {
                return APP_FALSE;
            }
            number_text[number_index] = text[read_index];
            number_index++;
        }
        read_index++;
    }

    if((0U == number_index) || (2U != value_index))
    {
        return APP_FALSE;
    }
    number_text[number_index] = '\0';
    if(APP_FALSE == host_command_parse_number(number_text, &values[value_index]))
    {
        return APP_FALSE;
    }

    if((0.0f > values[0]) || (0.0f > values[1]) || (0.0f > values[2]))
    {
        return APP_FALSE;
    }

    *kp = values[0];
    *ki = values[1];
    *kd = values[2];
    return APP_TRUE;
}

static void host_command_process_line(char *line, uint32 now_ms)
{
    float value;
    float kp;
    float ki;
    float kd;
    uint8 read_index = 0;
    uint8 write_index = 0;

    while('\0' != line[read_index])
    {
        if(APP_FALSE == host_command_is_space((uint8)line[read_index]))
        {
            line[write_index] = line[read_index];
            write_index++;
        }
        read_index++;
    }
    line[write_index] = '\0';

    if(APP_TRUE == host_command_match_stop(line))
    {
        actuator_motor_set_mode_stop();
        actuator_motor_record_command_error(APP_FALSE);
        return;
    }

    if(('M' == line[0]) && (',' == line[1]) &&
       (APP_TRUE == host_command_parse_number(&line[2], &value)))
    {
        actuator_motor_set_mode_motor_rpm(value, value);
        actuator_motor_record_host_motion(now_ms);
        actuator_motor_record_command_error(APP_FALSE);
        return;
    }

    if(('D' == line[0]) && (',' == line[1]) &&
       (APP_TRUE == host_command_parse_number(&line[2], &value)))
    {
        actuator_motor_set_mode_open_duty(value, value);
        actuator_motor_record_host_motion(now_ms);
        actuator_motor_record_command_error(APP_FALSE);
        return;
    }

    if(('P' == line[0]) && (',' == line[1]) &&
       (APP_TRUE == host_command_parse_pid_gain(&line[2], &kp, &ki, &kd)))
    {
        actuator_motor_set_rpm_pid_gain(APP_TRUE, APP_TRUE, kp, ki, kd);
        actuator_motor_record_command_error(APP_FALSE);
        return;
    }

    if(('P' == line[0]) && ('L' == line[1]) && (',' == line[2]) &&
       (APP_TRUE == host_command_parse_pid_gain(&line[3], &kp, &ki, &kd)))
    {
        actuator_motor_set_rpm_pid_gain(APP_TRUE, APP_FALSE, kp, ki, kd);
        actuator_motor_record_command_error(APP_FALSE);
        return;
    }

    if(('P' == line[0]) && ('R' == line[1]) && (',' == line[2]) &&
       (APP_TRUE == host_command_parse_pid_gain(&line[3], &kp, &ki, &kd)))
    {
        actuator_motor_set_rpm_pid_gain(APP_FALSE, APP_TRUE, kp, ki, kd);
        actuator_motor_record_command_error(APP_FALSE);
        return;
    }

    actuator_motor_record_command_error(APP_TRUE);
}

static void host_command_push_byte(uint8 ch, uint32 now_ms)
{
    if(('\r' == ch) || ('\n' == ch))
    {
        if(0U < host_command_index)
        {
            host_command_line[host_command_index] = '\0';
            host_command_process_line(host_command_line, now_ms);
            host_command_index = 0;
        }
        return;
    }

    if((0x20U <= ch) && ((HOST_COMMAND_LINE_MAX - 1U) > host_command_index))
    {
        host_command_line[host_command_index] = (char)ch;
        host_command_index++;
    }
    else if((HOST_COMMAND_LINE_MAX - 1U) <= host_command_index)
    {
        host_command_index = 0;
        actuator_motor_record_command_error(APP_TRUE);
    }
}

void host_command_init(void)
{
    host_command_index = 0;
    host_command_line[0] = '\0';
}

void host_command_update(uint32 now_ms)
{
    uint8 buffer[HOST_COMMAND_RX_BUFFER_LEN];
    uint32 count;
    uint32 index;

    (void)now_ms;

    count = debug_read_ring_buffer(buffer, HOST_COMMAND_RX_BUFFER_LEN);
    for(index = 0; index < count; index++)
    {
        host_command_push_byte(buffer[index], now_ms);
    }
}

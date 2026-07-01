/*********************************************************************************************************************
* File: host_command.c
* Description: UART0/VOFA downlink command parser.
********************************************************************************************************************/

#include "host_command.h"
#include "actuator_motor.h"
#include "control_chassis.h"
#include "control_balance.h"
#include "lsm6dsv16x_driver.h"
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

static uint8 host_command_parse_two_numbers(const char *text, float *first, float *second)
{
    char number_text[16];
    float values[2];
    uint8 value_index = 0;
    uint8 number_index = 0;
    uint8 read_index = 0;

    while('\0' != text[read_index])
    {
        if(',' == text[read_index])
        {
            if((0U == number_index) || (1U <= value_index))
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

    if((0U == number_index) || (1U != value_index))
    {
        return APP_FALSE;
    }
    number_text[number_index] = '\0';
    if(APP_FALSE == host_command_parse_number(number_text, &values[value_index]))
    {
        return APP_FALSE;
    }

    *first = values[0];
    *second = values[1];
    return APP_TRUE;
}

static uint8 host_command_parse_four_numbers(const char *text, float *first, float *second, float *third, float *fourth)
{
    char number_text[16];
    float values[4];
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

    if((0U == number_index) || (3U != value_index))
    {
        return APP_FALSE;
    }
    number_text[number_index] = '\0';
    if(APP_FALSE == host_command_parse_number(number_text, &values[value_index]))
    {
        return APP_FALSE;
    }

    *first = values[0];
    *second = values[1];
    *third = values[2];
    *fourth = values[3];
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
    float ks;
    float pos_kp;
    float period_ms_f;
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
        control_balance_set_ident_excitation(0.0f, 0U, now_ms);
        control_chassis_stop(now_ms);
        control_balance_set_mode(BALANCE_MODE_OFF);
        actuator_motor_set_mode_stop();
        actuator_motor_record_command_error(APP_FALSE);
        return;
    }

    if(('I' == line[0]) && ('M' == line[1]) && ('U' == line[2]) &&
       ('_' == line[3]) && ('Z' == line[4]) && ('E' == line[5]) &&
       ('R' == line[6]) && ('O' == line[7]) && ('\0' == line[8]))
    {
        lsm6dsv16x_gyro_offset_init();
        actuator_motor_record_command_error(APP_FALSE);
        return;
    }

    if(('B' == line[0]) && ('Z' == line[1]) && ('\0' == line[2]))
    {
        control_balance_reset_motion_state_public();
        actuator_motor_record_command_error(APP_FALSE);
        return;
    }

    if(('B' == line[0]) && ('I' == line[1]) && (',' == line[2]) &&
       (APP_TRUE == host_command_parse_two_numbers(&line[3], &value, &period_ms_f)))
    {
        if(APP_TRUE == control_balance_set_ident_excitation(0.0f == value ? 0.0f : value,
                                                             (uint32)period_ms_f, now_ms))
        {
            actuator_motor_record_command_error(APP_FALSE);
            return;
        }
    }

    if(('B' == line[0]) && ('L' == line[1]) && (',' == line[2]) &&
       (APP_TRUE == host_command_parse_four_numbers(&line[3], &kp, &ki, &ks, &pos_kp)))
    {
        if(APP_TRUE == control_balance_set_full_gain(kp, ki, ks, pos_kp))
        {
            actuator_motor_record_command_error(APP_FALSE);
            return;
        }
    }

    if(('B' == line[0]) && ('P' == line[1]) && (',' == line[2]) &&
       (APP_TRUE == host_command_parse_two_numbers(&line[3], &kp, &ki)))
    {
        if(APP_TRUE == control_balance_set_gain(kp, ki))
        {
            actuator_motor_record_command_error(APP_FALSE);
            return;
        }
    }

    if(('B' == line[0]) && (',' == line[1]) &&
       (APP_TRUE == host_command_parse_number(&line[2], &value)))
    {
        if(0.0f == value)
        {
            control_balance_set_ident_excitation(0.0f, 0U, now_ms);
            control_chassis_stop(now_ms);
            control_balance_set_mode(BALANCE_MODE_OFF);
            actuator_motor_record_command_error(APP_FALSE);
            return;
        }
        if(1.0f == value)
        {
            control_balance_set_mode(BALANCE_MODE_STANDBY);
            actuator_motor_record_command_error(APP_FALSE);
            return;
        }
        if(2.0f == value)
        {
            control_balance_set_mode(BALANCE_MODE_BALANCE_TEST);
            actuator_motor_record_command_error(APP_FALSE);
            return;
        }
    }

    if(('C' == line[0]) && (',' == line[1]) &&
       (APP_TRUE == host_command_parse_two_numbers(&line[2], &kp, &ki)))
    {
        control_chassis_set_cmd(kp, ki, APP_TRUE, now_ms);
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

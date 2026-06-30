/*********************************************************************************************************************
* File: actuator_motor.c
* Description: Brushless motor actuator.
********************************************************************************************************************/

#include "actuator_motor.h"
#include "app_config.h"
#include "bldc_foc_uart.h"

static motor_cmd_struct actuator_motor_cmd;
static wheel_feedback_struct actuator_motor_feedback;
static motor_diag_struct actuator_motor_diag;
static uint8 actuator_motor_output_active = APP_FALSE;
static uint32 actuator_motor_last_send_ms = 0;

static void actuator_motor_send_duty(int16 left_duty, int16 right_duty);
static void actuator_motor_send_duty_periodic(uint32 now_ms, int16 left_duty, int16 right_duty);

#if APP_BLDC_TEST_ENABLE
#include "app_state.h"

static uint8  actuator_motor_test_active = APP_FALSE;
static uint8  actuator_motor_test_step = 0;
static uint32 actuator_motor_test_step_start_ms = 0;

static void actuator_motor_test_update(uint32 now_ms)
{
    int16 left_duty;
    int16 right_duty;
    int16 test_duty;

    if(APP_STATE_FAULT == app_state_get())
    {
        actuator_motor_send_duty_periodic(now_ms, 0, 0);
        return;
    }

    if(now_ms < APP_BLDC_TEST_START_DELAY_MS)
    {
        actuator_motor_send_duty_periodic(now_ms, 0, 0);
        actuator_motor_test_active = APP_TRUE;
        actuator_motor_test_step = 0;
        actuator_motor_test_step_start_ms = APP_BLDC_TEST_START_DELAY_MS;
        return;
    }

    if(APP_FALSE == actuator_motor_test_active)
    {
        actuator_motor_test_active = APP_TRUE;
        actuator_motor_test_step = 0;
        actuator_motor_test_step_start_ms = now_ms;
    }

    if(APP_BLDC_TEST_STEP_MS <= (now_ms - actuator_motor_test_step_start_ms))
    {
        actuator_motor_test_step++;
        actuator_motor_test_step_start_ms = now_ms;

        if(5U < actuator_motor_test_step)
        {
            if(APP_BLDC_TEST_REPEAT)
            {
                actuator_motor_test_step = 0;
            }
            else
            {
                actuator_motor_test_step = 5;
            }
        }
    }

    test_duty = (int16)APP_BLDC_TEST_DUTY;

    if((int16)APP_BLDC_DUTY_LIMIT < test_duty)
    {
        test_duty = (int16)APP_BLDC_DUTY_LIMIT;
    }
    if((-(int16)APP_BLDC_DUTY_LIMIT) > test_duty)
    {
        test_duty = -(int16)APP_BLDC_DUTY_LIMIT;
    }
#if APP_BLDC_SAFE_START_ENABLE
    if((int16)APP_BLDC_SAFE_START_LIMIT < test_duty)
    {
        test_duty = (int16)APP_BLDC_SAFE_START_LIMIT;
    }
    if((-(int16)APP_BLDC_SAFE_START_LIMIT) > test_duty)
    {
        test_duty = -(int16)APP_BLDC_SAFE_START_LIMIT;
    }
#endif

    left_duty = 0;
    right_duty = 0;

    switch(actuator_motor_test_step)
    {
        case 0:  left_duty = test_duty;   right_duty = 0;          break;
        case 1:  left_duty = -test_duty;  right_duty = 0;          break;
        case 2:  left_duty = 0;           right_duty = test_duty;  break;
        case 3:  left_duty = 0;           right_duty = -test_duty; break;
        case 4:  left_duty = test_duty;   right_duty = test_duty;  break;
        case 5:  left_duty = 0;           right_duty = 0;          break;
        default: left_duty = 0;           right_duty = 0;          break;
    }

    actuator_motor_send_duty_periodic(now_ms, left_duty, right_duty);
}
#endif

static void actuator_motor_clear_snapshot(void)
{
    uint8 i;

    actuator_motor_feedback.left_speed = 0;
    actuator_motor_feedback.right_speed = 0;
    actuator_motor_feedback.left_reduced_angle = 0;
    actuator_motor_feedback.right_reduced_angle = 0;
    actuator_motor_feedback.last_rx_ms = 0;
    actuator_motor_feedback.age_ms = 0;
    actuator_motor_feedback.online = APP_FALSE;

    actuator_motor_diag.left_raw_angle = 0;
    actuator_motor_diag.right_raw_angle = 0;
    actuator_motor_diag.last_tx_left = 0;
    actuator_motor_diag.last_tx_right = 0;
    actuator_motor_diag.checksum_error_count = 0;
    actuator_motor_diag.unknown_frame_count = 0;
    actuator_motor_diag.tx_frame_count = 0;
    actuator_motor_diag.last_tx_func = 0;
    for(i = 0; i < MOTOR_DIAG_ASCII_LINE_MAX; i++)
    {
        actuator_motor_diag.last_unknown_ascii[i] = '\0';
    }
}

static void actuator_motor_copy_ascii_diag(const char *src)
{
    uint8 i;

    for(i = 0; i < (MOTOR_DIAG_ASCII_LINE_MAX - 1U); i++)
    {
        actuator_motor_diag.last_unknown_ascii[i] = src[i];
        if('\0' == src[i])
        {
            return;
        }
    }
    actuator_motor_diag.last_unknown_ascii[MOTOR_DIAG_ASCII_LINE_MAX - 1U] = '\0';
}

static void actuator_motor_refresh_feedback(uint32 now_ms)
{
    const bldc_foc_feedback_struct *raw;

    raw = bldc_foc_uart_get_feedback();
    actuator_motor_feedback.left_speed = raw->left_speed;
    actuator_motor_feedback.right_speed = raw->right_speed;
    actuator_motor_feedback.left_reduced_angle = raw->left_reduced_angle;
    actuator_motor_feedback.right_reduced_angle = raw->right_reduced_angle;
    actuator_motor_feedback.last_rx_ms = raw->last_rx_ms;

    if((APP_TRUE == raw->online) && (now_ms >= raw->last_rx_ms))
    {
        actuator_motor_feedback.age_ms = now_ms - raw->last_rx_ms;
        actuator_motor_feedback.online = (APP_BLDC_FEEDBACK_TIMEOUT_MS >= actuator_motor_feedback.age_ms) ? APP_TRUE : APP_FALSE;
    }
    else
    {
        actuator_motor_feedback.age_ms = 0;
        actuator_motor_feedback.online = APP_FALSE;
    }

    actuator_motor_diag.left_raw_angle = raw->left_angle;
    actuator_motor_diag.right_raw_angle = raw->right_angle;
    actuator_motor_diag.last_tx_left = raw->last_tx_left;
    actuator_motor_diag.last_tx_right = raw->last_tx_right;
    actuator_motor_diag.checksum_error_count = raw->checksum_error_count;
    actuator_motor_diag.unknown_frame_count = raw->unknown_frame_count;
    actuator_motor_diag.tx_frame_count = raw->tx_frame_count;
    actuator_motor_diag.last_tx_func = raw->last_tx_func;
    actuator_motor_copy_ascii_diag(raw->last_unknown_ascii);
}

static float actuator_motor_limit(float value)
{
    float limit;

    limit = APP_MOTOR_CMD_LIMIT;
    if((float)APP_BLDC_DUTY_LIMIT < limit)
    {
        limit = (float)APP_BLDC_DUTY_LIMIT;
    }
#if APP_BLDC_SAFE_START_ENABLE
    if(APP_BLDC_SAFE_START_LIMIT < limit)
    {
        limit = APP_BLDC_SAFE_START_LIMIT;
    }
#endif

    if(limit < value)
    {
        return limit;
    }
    if((-limit) > value)
    {
        return -limit;
    }
    return value;
}

static int16 actuator_motor_float_to_duty(float value)
{
    if(0.0f <= value)
    {
        return (int16)(value + 0.5f);
    }
    return (int16)(value - 0.5f);
}

static void actuator_motor_send_duty(int16 left_duty, int16 right_duty)
{
    actuator_motor_cmd.left_target = (float)left_duty;
    actuator_motor_cmd.right_target = (float)right_duty;
    actuator_motor_cmd.enable = ((0 != left_duty) || (0 != right_duty)) ? APP_TRUE : APP_FALSE;
    bldc_foc_uart_set_duty(left_duty, right_duty);
    actuator_motor_output_active = actuator_motor_cmd.enable;
}

static void actuator_motor_send_duty_periodic(uint32 now_ms, int16 left_duty, int16 right_duty)
{
    if(((float)left_duty != actuator_motor_cmd.left_target) ||
       ((float)right_duty != actuator_motor_cmd.right_target) ||
       (APP_BLDC_SEND_PERIOD_MS <= (now_ms - actuator_motor_last_send_ms)))
    {
        actuator_motor_last_send_ms = now_ms;
        actuator_motor_send_duty(left_duty, right_duty);
    }
}

static void actuator_motor_send_current(void)
{
    bldc_foc_uart_set_duty(actuator_motor_float_to_duty(actuator_motor_cmd.left_target),
                           actuator_motor_float_to_duty(actuator_motor_cmd.right_target));
    actuator_motor_output_active = APP_TRUE;
}

void actuator_motor_init(void)
{
    actuator_motor_clear_snapshot();
    bldc_foc_uart_init();
    actuator_motor_stop();
#if APP_BLDC_START_FEEDBACK
    bldc_foc_uart_start_feedback();
#endif
}

void actuator_motor_set_cmd(const motor_cmd_struct *cmd)
{
    actuator_motor_cmd.left_target = actuator_motor_limit(cmd->left_target);
    actuator_motor_cmd.right_target = actuator_motor_limit(cmd->right_target);
    actuator_motor_cmd.enable = cmd->enable;
}

void actuator_motor_update(uint32 now_ms)
{
    actuator_motor_refresh_feedback(now_ms);

#if APP_BLDC_TEST_ENABLE
    actuator_motor_test_update(now_ms);
    return;
#endif

    if(APP_FALSE == actuator_motor_cmd.enable)
    {
        if(APP_TRUE == actuator_motor_output_active)
        {
            actuator_motor_stop();
        }
        return;
    }

    if(APP_BLDC_SEND_PERIOD_MS <= (now_ms - actuator_motor_last_send_ms))
    {
        actuator_motor_last_send_ms = now_ms;
        actuator_motor_send_current();
    }
}

void actuator_motor_stop(void)
{
    actuator_motor_cmd.left_target = 0.0f;
    actuator_motor_cmd.right_target = 0.0f;
    actuator_motor_cmd.enable = APP_FALSE;
    bldc_foc_uart_stop();
    actuator_motor_output_active = APP_FALSE;
    actuator_motor_last_send_ms = 0;
}

const motor_cmd_struct *actuator_motor_get_cmd(void)
{
    return &actuator_motor_cmd;
}

const wheel_feedback_struct *actuator_motor_get_feedback(void)
{
    return &actuator_motor_feedback;
}

const motor_diag_struct *actuator_motor_get_diag(void)
{
    return &actuator_motor_diag;
}

/*********************************************************************************************************************
* File: actuator_motor.c
* Description: Brushless motor actuator.
********************************************************************************************************************/

#include "actuator_motor.h"
#include "app_config.h"
#include "app_state.h"
#include "bldc_foc_uart.h"
#include "control_pid.h"

static motor_cmd_struct actuator_motor_cmd;
static wheel_feedback_struct actuator_motor_feedback;
static motor_diag_struct actuator_motor_diag;
static motor_rpm_loop_diag_struct actuator_motor_rpm_diag;
static pid_controller_struct actuator_motor_left_pid;
static pid_controller_struct actuator_motor_right_pid;
static uint8 actuator_motor_open_loop_enable = APP_FALSE;
static float actuator_motor_open_loop_left_duty = 0.0f;
static float actuator_motor_open_loop_right_duty = 0.0f;
static uint8 actuator_motor_output_active = APP_FALSE;
static uint32 actuator_motor_last_send_ms = 0;
static uint32 actuator_motor_last_loop_ms = 0;
static uint32 actuator_motor_last_feedback_request_ms = 0;
static motor_actuator_cmd_struct actuator_motor_actuator_cmd;
static uint32 actuator_motor_last_host_motion_ms = 0;
static uint32 actuator_motor_left_rpm_zero_since_ms = 0;
static uint32 actuator_motor_right_rpm_zero_since_ms = 0;

static void actuator_motor_send_duty(int16 left_duty, int16 right_duty);
static void actuator_motor_send_duty_periodic(uint32 now_ms, int16 left_duty, int16 right_duty);

#if APP_BLDC_TEST_ENABLE
#include "app_state.h"


static void actuator_motor_test_update(uint32 now_ms)
{
    int16 test_duty;

    if(APP_STATE_FAULT == app_state_get())
    {
        actuator_motor_send_duty_periodic(now_ms, 0, 0);
        return;
    }

    if(now_ms < APP_BLDC_TEST_START_DELAY_MS)
    {
        actuator_motor_send_duty_periodic(now_ms, 0, 0);
        return;
    }

    test_duty = (int16)APP_BLDC_TEST_DUTY;
    if((int16)APP_BLDC_DUTY_LIMIT < test_duty)
    {
        test_duty = (int16)APP_BLDC_DUTY_LIMIT;
    }
#if APP_BLDC_SAFE_START_ENABLE
    if((int16)APP_BLDC_SAFE_START_LIMIT < test_duty)
    {
        test_duty = (int16)APP_BLDC_SAFE_START_LIMIT;
    }
#endif

    actuator_motor_send_duty_periodic(now_ms, -test_duty, -test_duty);
}
#endif

static float actuator_motor_absf(float value)
{
    return (0.0f > value) ? -value : value;
}

static float actuator_motor_limit_abs(float value, float limit)
{
    float abs_limit;

    abs_limit = actuator_motor_absf(limit);
    if(abs_limit < value)
    {
        return abs_limit;
    }
    if((-abs_limit) > value)
    {
        return -abs_limit;
    }
    return value;
}

static void actuator_motor_reset_rpm_loop(void)
{
    control_pid_reset(&actuator_motor_left_pid);
    control_pid_reset(&actuator_motor_right_pid);
    actuator_motor_rpm_diag.left_rpm_error = 0.0f;
    actuator_motor_rpm_diag.right_rpm_error = 0.0f;
    actuator_motor_rpm_diag.left_duty = 0.0f;
    actuator_motor_rpm_diag.right_duty = 0.0f;
    actuator_motor_rpm_diag.left_integral = 0.0f;
    actuator_motor_rpm_diag.right_integral = 0.0f;
    actuator_motor_last_loop_ms = 0;
}

static void actuator_motor_clear_snapshot(void)
{
    uint8 i;

    actuator_motor_feedback.left_motor_rpm = 0;
    actuator_motor_feedback.right_motor_rpm = 0;
    actuator_motor_feedback.left_reduced_angle = 0;
    actuator_motor_feedback.right_reduced_angle = 0;
    actuator_motor_feedback.last_rx_ms = 0;
    actuator_motor_feedback.age_ms = 0;
    actuator_motor_feedback.online = APP_FALSE;
    actuator_motor_feedback.left_online = APP_FALSE;
    actuator_motor_feedback.right_online = APP_FALSE;

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

    actuator_motor_rpm_diag.enable = APP_FALSE;
    actuator_motor_rpm_diag.target_motor_rpm = 0.0f;
    actuator_motor_rpm_diag.left_target_motor_rpm = 0.0f;
    actuator_motor_rpm_diag.right_target_motor_rpm = 0.0f;
    actuator_motor_rpm_diag.left_motor_rpm = 0.0f;
    actuator_motor_rpm_diag.right_motor_rpm = 0.0f;
    actuator_motor_rpm_diag.left_rpm_error = 0.0f;
    actuator_motor_rpm_diag.right_rpm_error = 0.0f;
    actuator_motor_rpm_diag.left_duty = 0.0f;
    actuator_motor_rpm_diag.right_duty = 0.0f;
    actuator_motor_rpm_diag.left_integral = 0.0f;
    actuator_motor_rpm_diag.right_integral = 0.0f;
    actuator_motor_rpm_diag.left_kp = APP_MOTOR_LEFT_RPM_KP;
    actuator_motor_rpm_diag.left_ki = APP_MOTOR_LEFT_RPM_KI;
    actuator_motor_rpm_diag.left_kd = APP_MOTOR_LEFT_RPM_KD;
    actuator_motor_rpm_diag.right_kp = APP_MOTOR_RIGHT_RPM_KP;
    actuator_motor_rpm_diag.right_ki = APP_MOTOR_RIGHT_RPM_KI;
    actuator_motor_rpm_diag.right_kd = APP_MOTOR_RIGHT_RPM_KD;
    actuator_motor_rpm_diag.command_error_count = 0;
    actuator_motor_actuator_cmd.mode = MOTOR_MODE_STOP;
    actuator_motor_actuator_cmd.left_motor_rpm = 0.0f;
    actuator_motor_actuator_cmd.right_motor_rpm = 0.0f;
    actuator_motor_actuator_cmd.left_open_duty = 0.0f;
    actuator_motor_actuator_cmd.right_open_duty = 0.0f;
    actuator_motor_actuator_cmd.enable = APP_FALSE;
    actuator_motor_last_host_motion_ms = 0;
    actuator_motor_rpm_diag.mode = MOTOR_MODE_STOP;
    actuator_motor_rpm_diag.host_motion_active = APP_FALSE;
    actuator_motor_rpm_diag.host_motion_age_ms = 0;
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
    bldc_foc_feedback_struct raw_snapshot;
    const bldc_foc_feedback_struct *raw;

    bldc_foc_uart_copy_feedback(&raw_snapshot);
    raw = &raw_snapshot;
    actuator_motor_feedback.left_motor_rpm = raw->left_motor_rpm;
    actuator_motor_feedback.right_motor_rpm = raw->right_motor_rpm;
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

    actuator_motor_rpm_diag.left_motor_rpm = APP_MOTOR_LEFT_RPM_SIGN * (float)actuator_motor_feedback.left_motor_rpm;
    actuator_motor_rpm_diag.right_motor_rpm = APP_MOTOR_RIGHT_RPM_SIGN * (float)actuator_motor_feedback.right_motor_rpm;

    /* Per-side encoder health: detect asymmetric zero-RPM when output is active.
       Both RPMs arrive in the same UART frame, so overall online only means
       the driver board is communicating — a disconnected encoder still shows 0. */
    if(APP_TRUE == actuator_motor_feedback.online)
    {
        actuator_motor_feedback.left_online = APP_TRUE;
        actuator_motor_feedback.right_online = APP_TRUE;

        if(APP_TRUE == actuator_motor_output_active)
        {
            int16 abs_left_rpm;
            int16 abs_right_rpm;

            abs_left_rpm = (raw->left_motor_rpm >= 0)
                         ? raw->left_motor_rpm
                         : (int16)(-raw->left_motor_rpm);
            abs_right_rpm = (raw->right_motor_rpm >= 0)
                          ? raw->right_motor_rpm
                          : (int16)(-raw->right_motor_rpm);

            /* Left encoder fault: zero RPM while right side shows >= 100 RPM */
            if((0 == raw->left_motor_rpm) && (abs_right_rpm >= 100))
            {
                if(0U == actuator_motor_left_rpm_zero_since_ms)
                {
                    actuator_motor_left_rpm_zero_since_ms = now_ms;
                }
                if((now_ms - actuator_motor_left_rpm_zero_since_ms) > APP_BLDC_PER_SIDE_RPM_TIMEOUT_MS)
                {
                    actuator_motor_feedback.left_online = APP_FALSE;
                }
            }
            else
            {
                actuator_motor_left_rpm_zero_since_ms = 0U;
            }

            /* Right encoder fault: zero RPM while left side shows >= 100 RPM */
            if((0 == raw->right_motor_rpm) && (abs_left_rpm >= 100))
            {
                if(0U == actuator_motor_right_rpm_zero_since_ms)
                {
                    actuator_motor_right_rpm_zero_since_ms = now_ms;
                }
                if((now_ms - actuator_motor_right_rpm_zero_since_ms) > APP_BLDC_PER_SIDE_RPM_TIMEOUT_MS)
                {
                    actuator_motor_feedback.right_online = APP_FALSE;
                }
            }
            else
            {
                actuator_motor_right_rpm_zero_since_ms = 0U;
            }
        }
    }
    else
    {
        actuator_motor_feedback.left_online = APP_FALSE;
        actuator_motor_feedback.right_online = APP_FALSE;
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

static int16 actuator_motor_float_to_duty(float value)
{
    if(0.0f <= value)
    {
        return (int16)(value + 0.5f);
    }
    return (int16)(value - 0.5f);
}

static int16 actuator_motor_last_left_duty = 0;
static int16 actuator_motor_last_right_duty = 0;

static void actuator_motor_send_duty(int16 left_duty, int16 right_duty)
{
    actuator_motor_last_left_duty = left_duty;
    actuator_motor_last_right_duty = right_duty;
    bldc_foc_uart_set_duty(left_duty, right_duty);
    actuator_motor_output_active = ((0 != left_duty) || (0 != right_duty)) ? APP_TRUE : APP_FALSE;
}

static void actuator_motor_send_duty_periodic(uint32 now_ms, int16 left_duty, int16 right_duty)
{
    if((left_duty != actuator_motor_last_left_duty) ||
       (right_duty != actuator_motor_last_right_duty) ||
       (APP_BLDC_SEND_PERIOD_MS <= (now_ms - actuator_motor_last_send_ms)))
    {
        actuator_motor_last_send_ms = now_ms;
        actuator_motor_send_duty(left_duty, right_duty);
    }
}

void actuator_motor_init(void)
{
    actuator_motor_clear_snapshot();
    control_pid_init(&actuator_motor_left_pid);
    control_pid_init(&actuator_motor_right_pid);
    control_pid_set_gain(&actuator_motor_left_pid,
                         APP_MOTOR_LEFT_RPM_KP,
                         APP_MOTOR_LEFT_RPM_KI,
                         APP_MOTOR_LEFT_RPM_KD);
    control_pid_set_gain(&actuator_motor_right_pid,
                         APP_MOTOR_RIGHT_RPM_KP,
                         APP_MOTOR_RIGHT_RPM_KI,
                         APP_MOTOR_RIGHT_RPM_KD);
    control_pid_set_limit(&actuator_motor_left_pid, APP_MOTOR_RPM_INTEGRAL_LIMIT, APP_MOTOR_RPM_DUTY_LIMIT);
    control_pid_set_limit(&actuator_motor_right_pid, APP_MOTOR_RPM_INTEGRAL_LIMIT, APP_MOTOR_RPM_DUTY_LIMIT);
    bldc_foc_uart_init();
    actuator_motor_stop();
#if APP_BLDC_START_FEEDBACK
    bldc_foc_uart_start_feedback();
#endif
}

void actuator_motor_set_actuator_cmd(const motor_actuator_cmd_struct *cmd)
{
    if(cmd->mode != actuator_motor_actuator_cmd.mode)
    {
        actuator_motor_reset_rpm_loop();
    }

    actuator_motor_actuator_cmd = *cmd;
    actuator_motor_rpm_diag.mode = cmd->mode;

    if(MOTOR_MODE_STOP == cmd->mode)
    {
        actuator_motor_stop();
    }
    else if(MOTOR_MODE_RPM_CLOSED_LOOP == cmd->mode)
    {
        actuator_motor_set_motor_rpm_target(cmd->left_motor_rpm, cmd->right_motor_rpm, cmd->enable);
    }
    else if(MOTOR_MODE_OPEN_DUTY == cmd->mode)
    {
        actuator_motor_set_open_loop_duty(cmd->left_open_duty, cmd->right_open_duty, cmd->enable);
    }
}

void actuator_motor_set_mode_stop(void)
{
    motor_actuator_cmd_struct cmd;

    cmd.mode = MOTOR_MODE_STOP;
    cmd.left_motor_rpm = 0.0f;
    cmd.right_motor_rpm = 0.0f;
    cmd.left_open_duty = 0.0f;
    cmd.right_open_duty = 0.0f;
    cmd.enable = APP_FALSE;
    actuator_motor_set_actuator_cmd(&cmd);
}

void actuator_motor_set_mode_open_duty(float left_duty, float right_duty)
{
    motor_actuator_cmd_struct cmd;

    cmd.mode = MOTOR_MODE_OPEN_DUTY;
    cmd.left_motor_rpm = 0.0f;
    cmd.right_motor_rpm = 0.0f;
    cmd.left_open_duty = left_duty;
    cmd.right_open_duty = right_duty;
    cmd.enable = APP_TRUE;
    actuator_motor_set_actuator_cmd(&cmd);
}

void actuator_motor_set_mode_motor_rpm(float left_motor_rpm, float right_motor_rpm)
{
    motor_actuator_cmd_struct cmd;

    cmd.mode = MOTOR_MODE_RPM_CLOSED_LOOP;
    cmd.left_motor_rpm = left_motor_rpm;
    cmd.right_motor_rpm = right_motor_rpm;
    cmd.left_open_duty = 0.0f;
    cmd.right_open_duty = 0.0f;
    cmd.enable = APP_TRUE;
    actuator_motor_set_actuator_cmd(&cmd);
}

void actuator_motor_record_host_motion(uint32 now_ms)
{
    actuator_motor_last_host_motion_ms = now_ms;
    actuator_motor_rpm_diag.host_motion_active = APP_TRUE;
    actuator_motor_rpm_diag.host_motion_age_ms = 0;
}

void actuator_motor_set_cmd(const motor_cmd_struct *cmd)
{
    actuator_motor_set_motor_rpm_target(cmd->left_target_motor_rpm, cmd->right_target_motor_rpm, cmd->enable);
}

void actuator_motor_set_motor_rpm_target(float left_motor_rpm, float right_motor_rpm, uint8 enable)
{
    float limited_left;
    float limited_right;

    actuator_motor_open_loop_enable = APP_FALSE;
    actuator_motor_open_loop_left_duty = 0.0f;
    actuator_motor_open_loop_right_duty = 0.0f;

    limited_left = actuator_motor_limit_abs(left_motor_rpm, APP_MOTOR_RPM_TARGET_LIMIT);
    limited_right = actuator_motor_limit_abs(right_motor_rpm, APP_MOTOR_RPM_TARGET_LIMIT);

    actuator_motor_cmd.left_target_motor_rpm = limited_left;
    actuator_motor_cmd.right_target_motor_rpm = limited_right;
    actuator_motor_cmd.enable = enable;

    actuator_motor_rpm_diag.enable = enable;
    actuator_motor_rpm_diag.left_target_motor_rpm = limited_left;
    actuator_motor_rpm_diag.right_target_motor_rpm = limited_right;
    if(limited_left == limited_right)
    {
        actuator_motor_rpm_diag.target_motor_rpm = limited_left;
    }
    else
    {
        actuator_motor_rpm_diag.target_motor_rpm = 0.5f * (limited_left + limited_right);
    }

    actuator_motor_actuator_cmd.mode = MOTOR_MODE_RPM_CLOSED_LOOP;
    actuator_motor_rpm_diag.mode = MOTOR_MODE_RPM_CLOSED_LOOP;
}

void actuator_motor_set_open_loop_duty(float left_duty, float right_duty, uint8 enable)
{
    float limited_left;
    float limited_right;

    limited_left = actuator_motor_limit_abs(left_duty, APP_MOTOR_RPM_DUTY_LIMIT);
    limited_right = actuator_motor_limit_abs(right_duty, APP_MOTOR_RPM_DUTY_LIMIT);

    actuator_motor_open_loop_left_duty = limited_left;
    actuator_motor_open_loop_right_duty = limited_right;
    actuator_motor_open_loop_enable = enable;
    actuator_motor_cmd.left_target_motor_rpm = 0.0f;
    actuator_motor_cmd.right_target_motor_rpm = 0.0f;
    actuator_motor_cmd.enable = enable;

    actuator_motor_rpm_diag.enable = (APP_TRUE == enable) ? 2U : APP_FALSE;
    actuator_motor_rpm_diag.target_motor_rpm = 0.0f;
    actuator_motor_rpm_diag.left_target_motor_rpm = 0.0f;
    actuator_motor_rpm_diag.right_target_motor_rpm = 0.0f;
    actuator_motor_reset_rpm_loop();

    actuator_motor_actuator_cmd.mode = MOTOR_MODE_OPEN_DUTY;
    actuator_motor_rpm_diag.mode = MOTOR_MODE_OPEN_DUTY;
}

void actuator_motor_set_rpm_pid_gain(uint8 left_enable, uint8 right_enable, float kp, float ki, float kd)
{
    if(APP_TRUE == left_enable)
    {
        control_pid_set_gain(&actuator_motor_left_pid, kp, ki, kd);
        actuator_motor_rpm_diag.left_kp = kp;
        actuator_motor_rpm_diag.left_ki = ki;
        actuator_motor_rpm_diag.left_kd = kd;
        control_pid_reset(&actuator_motor_left_pid);
    }

    if(APP_TRUE == right_enable)
    {
        control_pid_set_gain(&actuator_motor_right_pid, kp, ki, kd);
        actuator_motor_rpm_diag.right_kp = kp;
        actuator_motor_rpm_diag.right_ki = ki;
        actuator_motor_rpm_diag.right_kd = kd;
        control_pid_reset(&actuator_motor_right_pid);
    }
}

void actuator_motor_record_command_error(uint8 is_error)
{
    if(APP_TRUE == is_error)
    {
        actuator_motor_rpm_diag.command_error_count++;
    }
}

static void actuator_motor_update_rpm_loop(uint32 now_ms)
{
    float dt_s;
    float left_motor_rpm;
    float right_motor_rpm;
    float left_duty;
    float right_duty;
    int16 left_duty_i;
    int16 right_duty_i;

    if((APP_STATE_FAULT == app_state_get()) ||
       (APP_FALSE == actuator_motor_cmd.enable) ||
       (APP_FALSE == actuator_motor_feedback.online) ||
       (APP_FALSE == actuator_motor_feedback.left_online) ||
       (APP_FALSE == actuator_motor_feedback.right_online))
    {
        actuator_motor_send_duty_periodic(now_ms, 0, 0);
        actuator_motor_stop();
        return;
    }

    if(0U == actuator_motor_last_loop_ms)
    {
        dt_s = (float)APP_MOTOR_PERIOD_MS / 1000.0f;
    }
    else
    {
        dt_s = (float)(now_ms - actuator_motor_last_loop_ms) / 1000.0f;
    }
    actuator_motor_last_loop_ms = now_ms;

    left_motor_rpm = APP_MOTOR_LEFT_RPM_SIGN * (float)actuator_motor_feedback.left_motor_rpm;
    right_motor_rpm = APP_MOTOR_RIGHT_RPM_SIGN * (float)actuator_motor_feedback.right_motor_rpm;

    left_duty = control_pid_update(&actuator_motor_left_pid,
                                   actuator_motor_cmd.left_target_motor_rpm,
                                   left_motor_rpm,
                                   dt_s);
    right_duty = control_pid_update(&actuator_motor_right_pid,
                                    actuator_motor_cmd.right_target_motor_rpm,
                                    right_motor_rpm,
                                    dt_s);

    left_duty = APP_MOTOR_LEFT_DUTY_SIGN * left_duty;
    right_duty = APP_MOTOR_RIGHT_DUTY_SIGN * right_duty;
    left_duty = actuator_motor_limit_abs(left_duty, APP_MOTOR_RPM_DUTY_LIMIT);
    right_duty = actuator_motor_limit_abs(right_duty, APP_MOTOR_RPM_DUTY_LIMIT);

    left_duty_i = actuator_motor_float_to_duty(left_duty);
    right_duty_i = actuator_motor_float_to_duty(right_duty);

    actuator_motor_rpm_diag.left_motor_rpm = left_motor_rpm;
    actuator_motor_rpm_diag.right_motor_rpm = right_motor_rpm;
    actuator_motor_rpm_diag.left_rpm_error = actuator_motor_cmd.left_target_motor_rpm - left_motor_rpm;
    actuator_motor_rpm_diag.right_rpm_error = actuator_motor_cmd.right_target_motor_rpm - right_motor_rpm;
    actuator_motor_rpm_diag.left_duty = left_duty;
    actuator_motor_rpm_diag.right_duty = right_duty;
    actuator_motor_rpm_diag.left_integral = actuator_motor_left_pid.integral;
    actuator_motor_rpm_diag.right_integral = actuator_motor_right_pid.integral;

    actuator_motor_send_duty_periodic(now_ms, left_duty_i, right_duty_i);
}

static void actuator_motor_update_open_loop(uint32 now_ms)
{
    float left_motor_rpm;
    float right_motor_rpm;
    float left_duty;
    float right_duty;
    int16 left_duty_i;
    int16 right_duty_i;

    if(APP_STATE_FAULT == app_state_get())
    {
        actuator_motor_send_duty_periodic(now_ms, 0, 0);
        actuator_motor_stop();
        return;
    }

    if(APP_FALSE == actuator_motor_open_loop_enable)
    {
        actuator_motor_send_duty_periodic(now_ms, 0, 0);
        actuator_motor_rpm_diag.left_duty = 0.0f;
        actuator_motor_rpm_diag.right_duty = 0.0f;
        actuator_motor_rpm_diag.mode = MOTOR_MODE_STOP;
        return;
    }

#if APP_MOTOR_OPEN_DUTY_REQUIRE_FEEDBACK
    if(APP_FALSE == actuator_motor_feedback.online)
    {
        actuator_motor_send_duty_periodic(now_ms, 0, 0);
        actuator_motor_rpm_diag.left_duty = 0.0f;
        actuator_motor_rpm_diag.right_duty = 0.0f;
        return;
    }
#endif

    left_motor_rpm = APP_MOTOR_LEFT_RPM_SIGN * (float)actuator_motor_feedback.left_motor_rpm;
    right_motor_rpm = APP_MOTOR_RIGHT_RPM_SIGN * (float)actuator_motor_feedback.right_motor_rpm;

    left_duty = APP_MOTOR_LEFT_DUTY_SIGN * actuator_motor_open_loop_left_duty;
    right_duty = APP_MOTOR_RIGHT_DUTY_SIGN * actuator_motor_open_loop_right_duty;
    left_duty = actuator_motor_limit_abs(left_duty, APP_MOTOR_RPM_DUTY_LIMIT);
    right_duty = actuator_motor_limit_abs(right_duty, APP_MOTOR_RPM_DUTY_LIMIT);

    left_duty_i = actuator_motor_float_to_duty(left_duty);
    right_duty_i = actuator_motor_float_to_duty(right_duty);

    actuator_motor_rpm_diag.enable = 2U;
    actuator_motor_rpm_diag.left_motor_rpm = left_motor_rpm;
    actuator_motor_rpm_diag.right_motor_rpm = right_motor_rpm;
    actuator_motor_rpm_diag.left_rpm_error = 0.0f;
    actuator_motor_rpm_diag.right_rpm_error = 0.0f;
    actuator_motor_rpm_diag.left_duty = left_duty;
    actuator_motor_rpm_diag.right_duty = right_duty;
    actuator_motor_rpm_diag.left_integral = 0.0f;
    actuator_motor_rpm_diag.right_integral = 0.0f;

    actuator_motor_send_duty_periodic(now_ms, left_duty_i, right_duty_i);
}

static uint8 actuator_motor_host_motion_timed_out(uint32 now_ms)
{
    if(0U == actuator_motor_last_host_motion_ms)
    {
        actuator_motor_rpm_diag.host_motion_active = APP_FALSE;
        actuator_motor_rpm_diag.host_motion_age_ms = 0;
        return APP_FALSE;
    }

    actuator_motor_rpm_diag.host_motion_active = APP_TRUE;
    actuator_motor_rpm_diag.host_motion_age_ms = now_ms - actuator_motor_last_host_motion_ms;

#if (0U == APP_HOST_COMMAND_TIMEOUT_MS)
    return APP_FALSE;
#else
    return (APP_HOST_COMMAND_TIMEOUT_MS < actuator_motor_rpm_diag.host_motion_age_ms) ? APP_TRUE : APP_FALSE;
#endif
}

void actuator_motor_update(uint32 now_ms)
{
    actuator_motor_refresh_feedback(now_ms);

    if((MOTOR_MODE_STOP != actuator_motor_actuator_cmd.mode) &&
       (APP_TRUE == actuator_motor_host_motion_timed_out(now_ms)))
    {
        actuator_motor_set_mode_stop();
        return;
    }

    if((APP_FALSE == actuator_motor_feedback.online) &&
       (APP_BLDC_FEEDBACK_REQUEST_MS <= (now_ms - actuator_motor_last_feedback_request_ms)))
    {
        actuator_motor_last_feedback_request_ms = now_ms;
        bldc_foc_uart_start_feedback();
    }

#if APP_MOTOR_RPM_LOOP_ENABLE
    if(APP_TRUE == actuator_motor_open_loop_enable)
    {
        actuator_motor_update_open_loop(now_ms);
        return;
    }
    actuator_motor_update_rpm_loop(now_ms);
    return;
#endif

#if !APP_MOTOR_RPM_LOOP_ENABLE
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
        actuator_motor_send_duty(actuator_motor_float_to_duty(actuator_motor_cmd.left_target_motor_rpm),
                                 actuator_motor_float_to_duty(actuator_motor_cmd.right_target_motor_rpm));
    }
#endif
}

void actuator_motor_stop(void)
{
    actuator_motor_cmd.left_target_motor_rpm = 0.0f;
    actuator_motor_cmd.right_target_motor_rpm = 0.0f;
    actuator_motor_cmd.enable = APP_FALSE;
    actuator_motor_open_loop_enable = APP_FALSE;
    actuator_motor_open_loop_left_duty = 0.0f;
    actuator_motor_open_loop_right_duty = 0.0f;
    actuator_motor_rpm_diag.enable = APP_FALSE;
    actuator_motor_rpm_diag.target_motor_rpm = 0.0f;
    actuator_motor_rpm_diag.left_target_motor_rpm = 0.0f;
    actuator_motor_rpm_diag.right_target_motor_rpm = 0.0f;
    bldc_foc_uart_stop();
    actuator_motor_output_active = APP_FALSE;
    actuator_motor_last_send_ms = 0;
    actuator_motor_left_rpm_zero_since_ms = 0U;
    actuator_motor_right_rpm_zero_since_ms = 0U;
    actuator_motor_reset_rpm_loop();
    actuator_motor_actuator_cmd.mode = MOTOR_MODE_STOP;
    actuator_motor_actuator_cmd.enable = APP_FALSE;
    actuator_motor_last_host_motion_ms = 0;
    actuator_motor_rpm_diag.mode = MOTOR_MODE_STOP;
    actuator_motor_rpm_diag.host_motion_active = APP_FALSE;
    actuator_motor_rpm_diag.host_motion_age_ms = 0;
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

const motor_rpm_loop_diag_struct *actuator_motor_get_motor_rpm_loop_diag(void)
{
    return &actuator_motor_rpm_diag;
}

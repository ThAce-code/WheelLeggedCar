/*********************************************************************************************************************
* File: app_config.h
* Description: Project-level timing and configuration.
********************************************************************************************************************/

#ifndef _app_config_h_
#define _app_config_h_

#include "zf_common_headfile.h"

#define APP_TICK_PERIOD_MS              (1U)
#define APP_SCHEDULER_IMU_ONLY          (0U)
#define APP_HEARTBEAT_PERIOD_MS         (250U)

#define APP_IMU_PERIOD_MS               (5U)
#define APP_IMU_USE_INT1                (0U)
#define APP_IMU_STALE_TIMEOUT_MS        (100U)

#define APP_TELEMETRY_PERIOD_MS         (5U)
#define APP_TELEMETRY_ENABLE            (1U)

#define APP_ROLL_LIMIT_DEG              (45.0f)
#define APP_PITCH_LIMIT_DEG             (45.0f)
#define APP_SAFETY_ARM_DELAY_MS         (1000U)

/* --- Servo actuator configuration --- */
#define APP_SERVO_COUNT                 (4U)
#define APP_SERVO_ACTIVE_MASK           (0x0FU)
#define APP_SERVO_PWM_FREQ_HZ           (50U)
#define APP_SERVO_PWM_PERIOD_US         (20000U)
#define APP_SERVO_MIN_PULSE_US          (500U)
#define APP_SERVO_MID_PULSE_US          (1500U)
#define APP_SERVO_MAX_PULSE_US          (2500U)
#define APP_SERVO_MIN_DEG               (0.0f)
#define APP_SERVO_MID_DEG               (90.0f)
#define APP_SERVO_MAX_DEG               (180.0f)
#define APP_SERVO_MAX_SPEED_DPS         (450.0f)
#define APP_SERVO_MAX_UPDATE_GAP_MS     (20U)
#define APP_SERVO_PERIOD_MS             (10U)
#define APP_SERVO_TEST_ENABLE           (0U)

#define APP_SERVO0_PWM_CH               TCPWM_CH13_P00_3
#define APP_SERVO1_PWM_CH               TCPWM_CH12_P01_0
#define APP_SERVO2_PWM_CH               TCPWM_CH11_P01_1
#define APP_SERVO3_PWM_CH               TCPWM_CH09_P05_0

/* --- Motor actuator configuration --- */
#define APP_MOTOR_PERIOD_MS             (1U)
#define APP_MOTOR_CMD_LIMIT             (10000.0f)

#define APP_BLDC_UART_INDEX             (UART_1)
#define APP_BLDC_UART_BAUDRATE          (460800)
#define APP_BLDC_UART_TX_PIN            (UART1_TX_P04_1)
#define APP_BLDC_UART_RX_PIN            (UART1_RX_P04_0)
#define APP_BLDC_TX_GPIO_PROBE_ENABLE   (0U)
#define APP_BLDC_USE_ASCII_COMMANDS     (0U)

#define APP_BLDC_DUTY_LIMIT             (10000)
#define APP_BLDC_SAFE_START_ENABLE      (0U)
#define APP_BLDC_SAFE_START_LIMIT       (1000.0f)
#define APP_BLDC_SEND_PERIOD_MS         (20U)
#define APP_BLDC_FEEDBACK_TIMEOUT_MS    (100U)
#define APP_BLDC_FEEDBACK_REQUEST_MS    (200U)
#define APP_BLDC_START_FEEDBACK         (1U)

#define APP_BLDC_TEST_ENABLE            (0U)
#define APP_BLDC_TEST_START_DELAY_MS    (2000U)
#define APP_BLDC_TEST_DUTY              (1000)
#define APP_BLDC_TEST_STEP_MS           (3000U)
#define APP_BLDC_TEST_REPEAT            (1U)

#define APP_HOST_COMMAND_PERIOD_MS       (5U)
#define APP_HOST_COMMAND_TIMEOUT_MS      (0U)

#define APP_MOTOR_RPM_LOOP_ENABLE       (1U)
#define APP_MOTOR_OPEN_DUTY_REQUIRE_FEEDBACK   (0U)
#define APP_MOTOR_RPM_TARGET_LIMIT      (1000.0f)
#define APP_MOTOR_RPM_DUTY_LIMIT        (2000.0f)
#define APP_MOTOR_RPM_INTEGRAL_LIMIT    (1000.0f)
#define APP_MOTOR_LEFT_RPM_KP           (2.23353f)
#define APP_MOTOR_LEFT_RPM_KI           (45.6807f)
#define APP_MOTOR_LEFT_RPM_KD           (0.0f)
#define APP_MOTOR_RIGHT_RPM_KP          (2.14218f)
#define APP_MOTOR_RIGHT_RPM_KI          (42.2684f)
#define APP_MOTOR_RIGHT_RPM_KD          (0.0f)

#define APP_MOTOR_LEFT_DUTY_SIGN         (-1.0f)
#define APP_MOTOR_RIGHT_DUTY_SIGN        (-1.0f)
#define APP_MOTOR_LEFT_RPM_SIGN         (1.0f)
#define APP_MOTOR_RIGHT_RPM_SIGN        (-1.0f)

#define APP_SAFETY_PERIOD_MS            (1U)
#define APP_LEG_CONTROL_PERIOD_MS       (10U)

#define APP_LEG_CALIB_ENABLE            (0U)
#define APP_LEG_CALIB_SERVO_ID          LEG_SERVO_FL
#define APP_LEG_CALIB_OFFSET_DEG        (0.0f)

#define APP_LEG_VERIFY_ENABLE           (0U)
#define APP_LEG_VERIFY_DELAY_MS         (2000U)
#define APP_LEG_VERIFY_HEIGHT_CMD       (0.0f)
#define APP_LEG_VERIFY_PITCH_CMD        (25.0f)
#define APP_LEG_VERIFY_ROLL_CMD         (0.0f)

#if (APP_LEG_CALIB_ENABLE && APP_LEG_VERIFY_ENABLE)
#error "APP_LEG_CALIB_ENABLE and APP_LEG_VERIFY_ENABLE cannot both be enabled"
#endif

#endif

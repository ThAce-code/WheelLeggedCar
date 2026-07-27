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
#define APP_CHASSIS_PERIOD_MS           (5U)
#define APP_BALANCE_PERIOD_MS           (5U)

#define APP_IMU_PERIOD_MS               (5U)
#define APP_IMU_USE_INT1                (1U)
#define APP_IMU_INT1_PIN                (P19_3)
#define APP_IMU_STALE_TIMEOUT_MS        (30U)
#define APP_BALANCE_IMU_MAX_AGE_MS      (15U)
#define APP_IMU_PITCH_RATE_LPF_ALPHA    (0.50f)
#define APP_IMU_GYRO_CAL_RETRY_COUNT    (3U)
#define APP_IMU_GYRO_CAL_MAX_ABS_MEAN_DPS (5.0f)
#define APP_IMU_GYRO_CAL_MAX_VARIANCE_DPS2 (4.0f)

#define APP_TELEMETRY_PERIOD_MS         (10U)
#define APP_TELEMETRY_ENABLE            (1U)
#define APP_TELEMETRY_BALANCE_ENABLE    (1U)

#define APP_ROLL_LIMIT_DEG              (45.0f)
#define APP_PITCH_LIMIT_DEG             (45.0f)
#define APP_SAFETY_ARM_DELAY_MS         (1000U)

/* --- Servo actuator configuration --- */
#define APP_SERVO_COUNT                 (4U)
#define APP_SERVO_ACTIVE_MASK           (0x0FU)
#define APP_SERVO_PWM_FREQ_HZ           (300U)
#define APP_SERVO_CONTROL_PERIOD_US     (1000000U / APP_SERVO_PWM_FREQ_HZ)
#define APP_SERVO_MIN_PULSE_US          (500U)
#define APP_SERVO_MID_PULSE_US          (1500U)
#define APP_SERVO_MAX_PULSE_US          (2500U)
#define APP_SERVO_MIN_DEG               (0.0f)
#define APP_SERVO_MID_DEG               (90.0f)
#define APP_SERVO_MAX_DEG               (180.0f)
#define APP_SERVO_MAX_SPEED_DPS         (90.0f)
#define APP_LEG_FAST_SERVO_MAX_SPEED_DPS (180.0f)
#define APP_SERVO_LPF_ALPHA             (0.05f)
#define APP_SERVO_SETTLE_ERROR_DEG      (0.2f)
#define APP_SERVO_SETTLE_MS             (100U)
#define APP_SERVO_SETTLE_TICKS          ((APP_SERVO_SETTLE_MS * APP_SERVO_PWM_FREQ_HZ + 999U) / 1000U)
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
#define APP_BLDC_FEEDBACK_RPM_ABS_MAX   (5000)
#define APP_BLDC_PER_SIDE_RPM_TIMEOUT_MS (200U)
#define APP_BLDC_FEEDBACK_REQUEST_MS    (200U)
#define APP_BLDC_START_FEEDBACK         (1U)

#define APP_BLDC_TEST_ENABLE            (0U)
#define APP_BLDC_TEST_START_DELAY_MS    (2000U)
#define APP_BLDC_TEST_DUTY              (1000)
#define APP_BLDC_TEST_STEP_MS           (3000U)
#define APP_BLDC_TEST_REPEAT            (1U)

#define APP_HOST_COMMAND_PERIOD_MS       (1U)
#define APP_HOST_COMMAND_TIMEOUT_MS      (500U)
#define APP_CHASSIS_CMD_TIMEOUT_MS       (500U)

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

#define APP_CHASSIS_RPM_LIMIT                (200.0f)
#define APP_CHASSIS_FORWARD_RPM_LIMIT        (60.0f)
#define APP_CHASSIS_TURN_RATE_LIMIT_DPS      (60.0f)
#define APP_CHASSIS_FORWARD_RAMP_RPM_S       (60.0f)
#define APP_CHASSIS_TURN_RATE_RAMP_DPS_S     (120.0f)
#define APP_CHASSIS_SPEED_KP                 (0.30f)
#define APP_CHASSIS_SPEED_KI                 (0.0f)
#define APP_CHASSIS_SPEED_INTEGRAL_LIMIT     (100.0f)
#define APP_CHASSIS_SPEED_PITCH_LIMIT_DEG    (8.0f)
#define APP_CHASSIS_FAST_FORWARD_RPM_LIMIT       (220.0f)
#define APP_CHASSIS_FAST_BLEND_START_RPM         (40.0f)
#define APP_CHASSIS_FAST_BLEND_FULL_RPM          (90.0f)
#define APP_CHASSIS_FAST_BLEND_RAMP_S            (1.0f)
#define APP_CHASSIS_FAST_SPEED_PITCH_LIMIT_DEG   (12.0f)
#define APP_CHASSIS_TURN_KP                  (0.0f)
#define APP_CHASSIS_TURN_KI                  (0.05f)
#define APP_CHASSIS_TURN_INTEGRAL_LIMIT       (300.0f)
#define APP_CHASSIS_TURN_RPM_LIMIT           (60.0f)
#define APP_CHASSIS_TURN_GYRO_LPF_ALPHA        (0.27f)
#define APP_CHASSIS_TURN_GYRO_DEADBAND_DPS     (1.5f)
#define APP_CHASSIS_TURN_GYRO_STEP_LIMIT_DPS   (30.0f)
#define APP_CHASSIS_IMU_MAX_AGE_MS             (15U)
#define APP_CHASSIS_WHEEL_MAX_AGE_MS           (30U)
#define APP_CHASSIS_TURN_ZERO_TARGET_DPS       (0.5f)
#define APP_CHASSIS_FORWARD_ZERO_TARGET_RPM    (0.5f)
#define APP_CHASSIS_TURN_INTEGRAL_DECAY        (0.98f)
#define APP_CHASSIS_DRIVE_GAIN_ABS_LIMIT     (100.0f)
#define APP_BALANCE_RPM_LIMIT           (300.0f)
#define APP_BALANCE_TEST_PITCH_LIMIT_DEG (45.0f)
#define APP_BALANCE_PITCH_KP            (18.0f)
#define APP_BALANCE_PITCH_RATE_KD       (8.0f)
#define APP_BALANCE_WHEEL_SPEED_KS      (3.0f)
#define APP_BALANCE_WHEEL_POS_KP        (0.0f)
#define APP_BALANCE_FAST_WHEEL_SPEED_KS          (0.50f)
#define APP_BALANCE_FAST_SPEED_FF_GAIN           (0.0f)
#define APP_BALANCE_PITCH_SETPOINT_DEG  (-1.35f)
#define APP_BALANCE_WHEEL_POS_LIMIT_REV (2.0f)
#define APP_BALANCE_WHEEL_POS_DECAY     (0.999f)

#define APP_BALANCE_FINITE_ABS_LIMIT     (100000.0f)
#define APP_BALANCE_GAIN_ABS_LIMIT       (1000.0f)
#define APP_BALANCE_IDENT_RPM_LIMIT      (20.0f)
#define APP_BALANCE_IDENT_MIN_PERIOD_MS  (100U)
#define APP_BALANCE_IDENT_MAX_PERIOD_MS  (2000U)

#define APP_LEG_CONTROL_PERIOD_MS       (10U)

#define APP_LEG_CALIB_ENABLE            (0U)
#define APP_LEG_CALIB_SERVO_ID          LEG_SERVO_FL
#define APP_LEG_CALIB_OFFSET_DEG        (0.0f)
#define APP_LEG_DIRECT_STEP_TEST_ENABLE (0U)

#define APP_LEG_VERIFY_ENABLE           (0U)
#define APP_LEG_VERIFY_DELAY_MS         (2000U)
#define APP_LEG_VERIFY_HEIGHT_CMD       (0.0f)
#define APP_LEG_VERIFY_PITCH_CMD        (25.0f)
#define APP_LEG_VERIFY_ROLL_CMD         (0.0f)

#if (APP_LEG_CALIB_ENABLE && APP_LEG_VERIFY_ENABLE)
#error "APP_LEG_CALIB_ENABLE and APP_LEG_VERIFY_ENABLE cannot both be enabled"
#endif

#endif

/*********************************************************************************************************************
* File: app_config.h
* Description: Project-level timing and limit configuration.
********************************************************************************************************************/

#ifndef _app_config_h_
#define _app_config_h_

#include "zf_common_headfile.h"

#define APP_TICK_PERIOD_MS              (1U)
#define APP_SCHEDULER_IMU_ONLY          (1U)
#define APP_HEARTBEAT_PERIOD_MS         (250U)

#define APP_SAFETY_PERIOD_MS            (1U)
#define APP_ENCODER_PERIOD_MS           (1U)
#define APP_MOTOR_PERIOD_MS             (1U)
#define APP_IMU_PERIOD_MS               (5U)
#define APP_IMU_USE_INT1                (1U)
#define APP_IMU_INT1_PIN                P11_0
#define APP_IMU_STALE_TIMEOUT_MS        (50U)
#define APP_ESTIMATOR_PERIOD_MS         (5U)
#define APP_CHASSIS_CONTROL_PERIOD_MS   (5U)
#define APP_LEG_CONTROL_PERIOD_MS       (10U)
#define APP_SERVO_PERIOD_MS             (10U)
#define APP_CAMERA_PERIOD_MS            (20U)
#define APP_PERCEPTION_PERIOD_MS        (20U)
#define APP_TELEMETRY_PERIOD_MS         (10U)

#define APP_ROLL_LIMIT_DEG              (45.0f)
#define APP_PITCH_LIMIT_DEG             (45.0f)
#define APP_MOTOR_CMD_LIMIT             (1000.0f)

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
#define APP_SERVO_TEST_ENABLE           (0U)
#define APP_SERVO_TEST_INDEX            (0U)
#define APP_SERVO_TEST_MIN_DEG          (60.0f)
#define APP_SERVO_TEST_MAX_DEG          (120.0f)
#define APP_SERVO_TEST_PERIOD_MS        (1000U)

#define APP_SERVO0_PWM_CH               TCPWM_CH13_P00_3
#define APP_SERVO1_PWM_CH               TCPWM_CH14_P00_2
#define APP_SERVO2_PWM_CH               TCPWM_CH06_P02_1
#define APP_SERVO3_PWM_CH               TCPWM_CH07_P02_0

#endif

/*********************************************************************************************************************
* File: app_config.h
* Description: Project-level timing and configuration — IMU polling test.
********************************************************************************************************************/

#ifndef _app_config_h_
#define _app_config_h_

#include "zf_common_headfile.h"

#define APP_TICK_PERIOD_MS              (1U)
#define APP_SCHEDULER_IMU_ONLY          (1U)
#define APP_HEARTBEAT_PERIOD_MS         (250U)

#define APP_IMU_PERIOD_MS               (5U)
#define APP_IMU_USE_INT1                (0U)
#define APP_IMU_STALE_TIMEOUT_MS        (100U)

#define APP_TELEMETRY_PERIOD_MS         (10U)

#define APP_ROLL_LIMIT_DEG              (45.0f)
#define APP_PITCH_LIMIT_DEG             (45.0f)

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

#endif

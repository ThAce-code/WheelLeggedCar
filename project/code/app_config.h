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

#endif

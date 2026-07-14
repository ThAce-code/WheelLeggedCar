/*********************************************************************************************************************
* File: camera_debug_config.h
* Description: Camera-only portability-gate configuration.
********************************************************************************************************************/

#ifndef _camera_debug_config_h_
#define _camera_debug_config_h_

#ifndef APP_CAMERA_WIFI_ENABLE
#define APP_CAMERA_WIFI_ENABLE          (1U)
#endif

#ifndef APP_CAMERA_CAPTURE_ENABLE
#define APP_CAMERA_CAPTURE_ENABLE       (1U)
#endif

#define APP_CAMERA_DISPLAY_PERIOD_MS    (100U)
#define APP_CAMERA_STALE_TIMEOUT_MS     (200U)
#define APP_CAMERA_RETRY_PERIOD_MS      (1000U)
#define APP_CAMERA_WIFI_RETRY_MS        (5000U)
#define APP_CAMERA_SEND_STARTUP_MS      (5000U)
#define APP_CAMERA_GATE_WINDOW_MS       (60000U)

#ifndef APP_CAMERA_WIFI_SSID
#define APP_CAMERA_WIFI_SSID            "SEEKFREE"
#endif

#ifndef APP_CAMERA_WIFI_PASSWORD
#define APP_CAMERA_WIFI_PASSWORD        "SEEKFREE123"
#endif

#ifndef APP_CAMERA_WIFI_TARGET_IP
#define APP_CAMERA_WIFI_TARGET_IP       "192.168.137.1"
#endif

#ifndef APP_CAMERA_WIFI_TARGET_PORT
#define APP_CAMERA_WIFI_TARGET_PORT     "8086"
#endif

#ifndef APP_CAMERA_WIFI_LOCAL_PORT
#define APP_CAMERA_WIFI_LOCAL_PORT      "6666"
#endif

#endif

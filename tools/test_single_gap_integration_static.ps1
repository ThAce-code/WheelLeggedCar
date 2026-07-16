param(
    [string]$Root = (Split-Path -Parent $PSScriptRoot)
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Require-Pattern
{
    param(
        [string]$RelativePath,
        [string]$Pattern
    )

    $path = Join-Path $Root $RelativePath
    if(-not (Test-Path -LiteralPath $path))
    {
        throw "Missing $RelativePath"
    }
    if(-not (Select-String -Path $path -Pattern $Pattern -Quiet))
    {
        throw "Missing '$Pattern' in $RelativePath"
    }
}

function Require-TextPattern
{
    param(
        [string]$RelativePath,
        [string]$Pattern
    )

    $path = Join-Path $Root $RelativePath
    if(-not (Test-Path -LiteralPath $path))
    {
        throw "Missing $RelativePath"
    }
    $text = [System.IO.File]::ReadAllText($path)
    if(-not [Regex]::IsMatch($text, $Pattern))
    {
        throw "Missing text pattern '$Pattern' in $RelativePath"
    }
}

function Reject-Pattern
{
    param(
        [string]$RelativePath,
        [string]$Pattern
    )

    $path = Join-Path $Root $RelativePath
    $files = Get-ChildItem -Path $path -File -ErrorAction SilentlyContinue
    foreach($file in $files)
    {
        if(Select-String -Path $file.FullName -Pattern $Pattern -Quiet)
        {
            throw "Forbidden '$Pattern' in $($file.FullName)"
        }
    }
}

Require-Pattern 'project/code/single_gap_config.h' 'SINGLE_GAP_CONTROL_PERIOD_MS\s+\(40U\)'
Require-Pattern 'project/code/single_gap_config.h' 'SINGLE_GAP_POSE_PERIOD_MS\s+\(50U\)'
Require-Pattern 'project/code/single_gap_config.h' 'SINGLE_GAP_TOF_STOP_MM\s+\(350U\)'
Require-Pattern 'project/code/single_gap_config.h' 'SINGLE_GAP_SENSOR_STALE_MS\s+\(100U\)'
Require-Pattern 'project/code/single_gap_config.h' 'SINGLE_GAP_MOTION_ENABLE\s+\(0U\)'
Require-Pattern 'project/code/single_gap_config.h' 'SINGLE_GAP_WHEEL_CIRCUMFERENCE_MM\s+\(0U\)'
Require-Pattern 'project/code/single_gap_config.h' 'SINGLE_GAP_MAX_RUNS\s+\(96U\)'
Require-Pattern 'project/code/single_gap_config.h' 'SINGLE_GAP_MAX_COMPONENTS\s+\(24U\)'
Require-Pattern 'project/code/single_gap_types.h' 'SINGLE_GAP_STATE_PASS_CANDIDATE'
Require-Pattern 'project/code/single_gap_types.h' 'SINGLE_GAP_STOP_TOF_NEAR'
Require-Pattern 'project/code/single_gap_types.h' 'single_gap_output_struct'
Require-Pattern 'project/code/dl1b_safety.h' 'uint8\s+dl1b_safety_init\s*\(uint32\s+now_ms\)'
Require-Pattern 'project/code/dl1b_safety.h' 'void\s+dl1b_safety_update\s*\(uint32\s+now_ms\)'
Require-Pattern 'project/code/dl1b_safety_port.h' 'uint8\s+dl1b_safety_port_read\s*\(uint16\s*\*distance_mm\)'
Require-Pattern 'libraries/zf_device/zf_device_dl1b.h' 'DL1B_SCL_PIN\s+\(P19_0\)'
Require-Pattern 'libraries/zf_device/zf_device_dl1b.h' 'DL1B_SDA_PIN\s+\(P19_1\)'
Require-Pattern 'libraries/zf_device/zf_device_dl1b.h' 'DL1B_XS_PIN\s+\(\s*P07_2\s*\)'
Require-Pattern 'project/user/main_cm0plus.c' '#include\s+"single_gap_config\.h"'
Require-Pattern 'project/user/main_cm7_0.c' '#include\s+"single_gap_config\.h"'
Require-TextPattern 'project/user/main_cm0plus.c' '(?s)#if\s*\(SINGLE_GAP_ENABLE\s*==\s*0U\).*?gpio_init\s*\(P19_0.*?#endif'
Require-TextPattern 'project/user/main_cm7_0.c' '(?s)#if\s*\(SINGLE_GAP_ENABLE\s*==\s*0U\).*?gpio_(?:low|high|init|toggle_level)\s*\(P19_0.*?#endif'
Require-TextPattern 'project/code/single_gap_detector.h' '(?s)uint8\s+single_gap_detector_process\s*\(\s*const\s+uint8\s*\*pixels\s*,\s*uint16\s+width\s*,\s*uint16\s+height\s*,\s*uint16\s+stride\s*,\s*uint32\s+sequence\s*,\s*uint32\s+capture_ms\s*,\s*single_gap_observation_struct\s*\*observation\s*\)'
Require-Pattern 'project/code/single_gap_types.h' 'uint32\s+last_observation_sequence\s*;'
Require-TextPattern 'project/code/single_gap_controller.h' '(?s)void\s+single_gap_controller_update\s*\(\s*single_gap_controller_struct\s*\*controller\s*,\s*const\s+single_gap_observation_struct\s*\*observation\s*,\s*const\s+single_gap_tof_snapshot_struct\s*\*tof\s*,\s*float\s+odometry_m\s*,\s*uint8\s+odometry_valid\s*,\s*float\s+forward_rpm\s*,\s*uint32\s+now_ms\s*,\s*single_gap_output_struct\s*\*output\s*\)'
Require-Pattern 'project/code/single_gap_pose_source.h' 'uint8\s+single_gap_pose_source_init\s*\(void\)'
Require-Pattern 'project/code/single_gap_pose_source.h' 'void\s+single_gap_pose_source_update\s*\(uint32\s+now_ms\)'
Require-Pattern 'project/code/app.c' 'single_gap_pose_source_init\s*\(\)'
Require-Pattern 'project/code/app_scheduler.c' 'single_gap_pose_source_update\s*\(now_ms\)'
Require-Pattern 'project/iar/project_config/cyt4bb7_cm_7_0.ewp' 'single_gap_pose_source\.c'
Require-Pattern 'project/iar/project_config/cyt4bb7_cm_7_0.ewp' 'single_gap_pose_source\.h'
Require-TextPattern 'project/code/single_gap_app.h' '(?s)uint8\s+single_gap_app_init\s*\(void\).*?void\s+single_gap_app_on_frame\s*\(const\s+camera_vision_frame_view_struct\s*\*frame\).*?void\s+single_gap_app_service\s*\(uint32\s+now_ms\).*?float\s+single_gap_speed_mps_to_rpm\s*\(float\s+speed_mps\s*,\s*float\s+circumference_m\)'
Require-Pattern 'project/code/camera_frame_consumer.h' 'typedef\s+void\s*\(\*camera_frame_handler_fn\)\(const\s+camera_vision_frame_view_struct\s*\*frame\)'
Require-Pattern 'project/code/camera_frame_consumer.h' 'void\s+camera_frame_consumer_set_handler\s*\(camera_frame_handler_fn\s+handler\)'
Require-TextPattern 'project/code/camera_frame_consumer.c' '(?s)CAMERA_CONSUMER_LINK_CONNECTED\s*!=\s*consumer_diag\.socket_state\s*\)\s*\{\s*camera_frame_consumer_try_network\s*\(\s*\)\s*;\s*\}\s*.*?intercore_camera_consumer_acquire_latest'
Require-TextPattern 'project/code/camera_frame_consumer.c' '(?s)camera_frame_handler\s*\(\s*&vision_view\s*\).*?seekfree_assistant_camera_send'
Require-TextPattern 'project/code/camera_debug_config.h' '(?s)#if\s+SINGLE_GAP_MOTION_ENABLE\s*#undef\s+APP_CAMERA_WIFI_ENABLE\s*#define\s+APP_CAMERA_WIFI_ENABLE\s+\(0U\)\s*#endif'
Require-Pattern 'project/user/main_cm7_1.c' 'single_gap_app_init\s*\(\)'
Require-TextPattern 'project/user/main_cm7_1.c' '(?s)single_gap_app_service\s*\(\s*camera_frame_consumer_now_ms\s*\(\s*\)\s*\)\s*;.*?camera_frame_consumer_service\s*\(\s*\)'
Require-Pattern 'project/iar/project_config/cyt4bb7_cm_7_1.ewp' 'single_gap_app\.c'
Require-Pattern 'project/iar/project_config/cyt4bb7_cm_7_1.ewp' 'single_gap_app\.h'
Require-Pattern 'project/iar/project_config/cyt4bb7_cm_7_1.ewp' 'single_gap_detector\.c'
Require-Pattern 'project/iar/project_config/cyt4bb7_cm_7_1.ewp' 'single_gap_controller\.c'
Require-Pattern 'project/iar/project_config/cyt4bb7_cm_7_1.ewp' 'dl1b_safety\.c'
Require-Pattern 'project/code/app_config.h' '#include\s+"single_gap_config\.h"'
Require-TextPattern 'project/code/app_config.h' '(?s)#if\s+SINGLE_GAP_MOTION_ENABLE\s*#define\s+APP_CAMERA_DEBUG_ONLY\s+\(0U\)\s*#else\s*#define\s+APP_CAMERA_DEBUG_ONLY\s+\(1U\)\s*#endif'
Require-TextPattern 'project/code/intercore_control.c' '(?s)NAVIGATION_SOURCE_VISION\s*==\s*command->source.*?NAVIGATION_MODE_VISION_ASSIST\s*!=\s*command->mode.*?SINGLE_GAP_NAV_VALID_MS\s*<\s*command->valid_for_ms.*?SINGLE_GAP_TURN_LIMIT_DPS\s*<\s*command->turn_rate_dps.*?-SINGLE_GAP_TURN_LIMIT_DPS\s*>\s*command->turn_rate_dps'
Reject-Pattern 'project/code/single_gap_*.c' '\b(malloc|calloc|realloc|free)\s*\('

$mingwBin = 'C:\msys64\ucrt64\bin'
$gccPath = Join-Path $mingwBin 'gcc.exe'
if(Test-Path -LiteralPath $gccPath)
{
    $env:PATH = "$mingwBin;$env:PATH"
    $macroLines = & $gccPath -dM -E -x c `
        -D_zf_common_headfile_h_ `
        -DSINGLE_GAP_ENABLE=1U `
        -DSINGLE_GAP_MOTION_ENABLE=1U `
        -DSINGLE_GAP_WHEEL_CIRCUMFERENCE_MM=200U `
        -I (Join-Path $Root 'project\code') `
        -I (Join-Path $Root 'libraries\zf_common') `
        -include app_config.h `
        (Join-Path $Root 'project\code\single_gap_config.h') 2>&1
    if(0 -ne $LASTEXITCODE)
    {
        throw "Failed to preprocess motion-enabled app_config.h: $($macroLines -join ' ')"
    }
    $macroText = $macroLines -join "`n"
    if(($macroText -match '(?m)^#define\s+SINGLE_GAP_MOTION_ENABLE\s+\(?1U\)?\s*$') -and
       ($macroText -match '(?m)^#define\s+APP_CAMERA_DEBUG_ONLY\s+\(?1U\)?\s*$'))
    {
        throw 'Motion-enabled preprocessing left APP_CAMERA_DEBUG_ONLY enabled'
    }
}

Write-Output 'single-gap static contracts: PASS'

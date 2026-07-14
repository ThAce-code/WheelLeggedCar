$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$failures = New-Object System.Collections.Generic.List[string]

function Add-Failure([string]$message)
{
    $script:failures.Add($message)
}

function Read-RepoFile([string]$relativePath)
{
    $path = Join-Path $repoRoot $relativePath
    if (-not (Test-Path -LiteralPath $path -PathType Leaf))
    {
        Add-Failure "missing file: $relativePath"
        return $null
    }

    return [System.IO.File]::ReadAllText($path)
}

function Assert-Hash([string]$relativePath, [string]$expectedHash)
{
    $path = Join-Path $repoRoot $relativePath
    if (-not (Test-Path -LiteralPath $path -PathType Leaf))
    {
        Add-Failure "missing provenance file: $relativePath"
        return
    }

    $actualHash = (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash
    if ($actualHash -ne $expectedHash)
    {
        Add-Failure "SHA-256 mismatch: $relativePath (actual $actualHash)"
    }
}

function Assert-Match([string]$text, [string]$pattern, [string]$message)
{
    if (($null -eq $text) -or ($text -notmatch $pattern))
    {
        Add-Failure $message
    }
}

function Assert-NoMatch([string]$text, [string]$pattern, [string]$message)
{
    if (($null -ne $text) -and ($text -match $pattern))
    {
        Add-Failure $message
    }
}

Assert-Hash 'libraries\zf_device\zf_device_mt9v03x.c' '33C9B8C1D5641CB48933B4E6796788BB247C813909B4A3F16D963A6B40D7E7E0'
Assert-Hash 'libraries\zf_device\zf_device_mt9v03x.h' '0C489FABA1851C861E33B44E222D7D72B331EEC851B09F93DC694A20FA17DED2'
Assert-Hash 'libraries\zf_components\seekfree_assistant.c' '6C6BABD379FAFCCBB64F4DE27A8E837EC3C73EF4E7ECD169656CF80B0C13A28B'
Assert-Hash 'libraries\zf_components\seekfree_assistant.h' 'FA6FC8DD75323AF03A49FB48BBA63029AD2CB36C1C8484F4B9596835EADEBBB5'

$gitAttributes = Read-RepoFile '.gitattributes'
$lockedSources = @(
    'libraries/zf_device/zf_device_mt9v03x.c',
    'libraries/zf_device/zf_device_mt9v03x.h',
    'libraries/zf_components/seekfree_assistant.c',
    'libraries/zf_components/seekfree_assistant.h'
)
foreach ($lockedSource in $lockedSources)
{
    $escapedSource = [Regex]::Escape($lockedSource)
    Assert-Match $gitAttributes "(?m)^$escapedSource\s+-text\s+-whitespace\s*$" "hash-locked source lacks exact checkout/whitespace attributes: $lockedSource"
}

$headfile = Read-RepoFile 'libraries\zf_common\zf_common_headfile.h'
Assert-Match $headfile '(?m)^\s*#include\s+"zf_device_mt9v03x\.h"' 'zf_common_headfile.h does not enable zf_device_mt9v03x.h'

$cm7Project = Read-RepoFile 'project\iar\project_config\cyt4bb7_cm_7_1.ewp'
Assert-Match $cm7Project '\\zf_device\\zf_device_mt9v03x\.c</name>' 'CM7_1 project does not contain zf_device_mt9v03x.c'
Assert-Match $cm7Project '\\zf_device\\zf_device_mt9v03x\.h</name>' 'CM7_1 project does not contain zf_device_mt9v03x.h'
Assert-Match $cm7Project '(?s)<name>IlinkKeepSymbols</name>\s*<state>mt9v03x_h_num</state>\s*<state>mt9v03x_w_num</state>\s*<state>mt9v03x_image_temp</state>' 'CM7_1 linker does not retain all three fixed MT9V03X objects for map verification'

$assistantInterface = Read-RepoFile 'libraries\zf_components\seekfree_assistant_interface.c'
Assert-Match $assistantInterface '(?s)case\s+SEEKFREE_ASSISTANT_WIFI_SPI\s*:.*?seekfree_assistant_transfer_callback\s*=\s*wifi_spi_send_buffer\s*;.*?seekfree_assistant_receive_callback\s*=\s*wifi_spi_read_buffer\s*;' 'Assistant WIFI_SPI interface no longer maps to wifi_spi_send_buffer/read_buffer'

$wifiHeader = Read-RepoFile 'libraries\zf_device\zf_device_wifi_spi.h'
$wifiPinPatterns = @(
    '(?m)^\s*#define\s+WIFI_SPI_MISO_PIN\s+\(\s*SPI0_MISO_P02_0\s*\)',
    '(?m)^\s*#define\s+WIFI_SPI_MOSI_PIN\s+\(\s*SPI0_MOSI_P02_1\s*\)',
    '(?m)^\s*#define\s+WIFI_SPI_SCK_PIN\s+\(\s*SPI0_CLK_P02_2\s*\)',
    '(?m)^\s*#define\s+WIFI_SPI_CS_PIN\s+\(\s*P02_3\s*\)',
    '(?m)^\s*#define\s+WIFI_SPI_INT_PIN\s+\(\s*P02_4\s*\)',
    '(?m)^\s*#define\s+WIFI_SPI_RST_PIN\s+\(\s*P23_0\s*\)'
)
foreach ($pattern in $wifiPinPatterns)
{
    Assert-Match $wifiHeader $pattern "WiFi-SPI confirmed pin contract is missing: $pattern"
}

$cameraHeader = Read-RepoFile 'libraries\zf_device\zf_device_mt9v03x.h'
Assert-Match $cameraHeader '(?m)^\s*#define\s+MT9V03X_PCLK_PIN\s+\(\s*P06_5\s*\)' 'MT9V03X PCLK is not P06_5'
Assert-Match $cameraHeader '(?m)^\s*#define\s+MT9V03X_VSYNC_PIN\s+\(\s*P06_6\s*\)' 'MT9V03X VSYNC is not P06_6'
Assert-Match $cameraHeader '(?m)^\s*#define\s+MT9V03X_DATA_PIN\s+\(\s*P18_0\s*\).*P18_0\s*~\s*P18_7' 'MT9V03X data bus is not declared as P18_0 through P18_7'
Assert-Match $cameraHeader '(?m)^\s*#define\s+MT9V03X_W\s+\(\s*188\s*\)' 'MT9V03X width is not 188'
Assert-Match $cameraHeader '(?m)^\s*#define\s+MT9V03X_H\s+\(\s*120\s*\)' 'MT9V03X height is not 120'

$cameraSource = Read-RepoFile 'libraries\zf_device\zf_device_mt9v03x.c'
Assert-Match $cameraSource '(?m)^\s*#pragma\s+location\s*=\s*0x28026024[^\r\n]*\r?\n\s*__no_init\s+uint8\s+mt9v03x_image_temp\s*\[' 'mt9v03x_image_temp fixed placement 0x28026024 is missing'
Assert-Match $cameraSource '(?s)#pragma\s+location\s*=\s*0x28006bf0\s+__no_init\s+uint16\s+mt9v03x_h_num\s*;' 'mt9v03x_h_num fixed placement 0x28006BF0 is missing'
Assert-Match $cameraSource '(?s)#pragma\s+location\s*=\s*0x28006bf2\s+__no_init\s+uint16\s+mt9v03x_w_num\s*;' 'mt9v03x_w_num fixed placement 0x28006BF2 is missing'

$assistantHeader = Read-RepoFile 'libraries\zf_components\seekfree_assistant.h'
Assert-Match $assistantHeader '(?s)void\s+seekfree_assistant_camera_config\s*\(\s*seekfree_assistant_camera_struct\s*\*\s*camera_obj\s*,' 'E9 object-style seekfree_assistant_camera_config API is missing'
Assert-Match $assistantHeader '(?s)void\s+seekfree_assistant_camera_send\s*\(\s*seekfree_assistant_camera_struct\s*\*\s*camera_obj\s*\)\s*;' 'E9 object-style seekfree_assistant_camera_send API is missing'

$linkerDirectives = Read-RepoFile 'project\iar\icf\linker_directives_tviibh.icf'
Assert-Match $linkerDirectives '(?s)mem:\[from\s+_base_SRAM\s+to\s+0x28006BEF\s*\]\s*\|\s*mem:\[from\s+0x28006BF4\s+to\s+\(_base_SRAM\s*\+\s*_size_SRAM\s*-\s*1\)\s*\]' 'CM0+ SRAM union does not reserve 0x28006BF0-0x28006BF3'
Assert-Match $linkerDirectives '(?s)mem:\[from\s+_base_SRAM\s+to\s+0x28026023\s*\]\s*\|\s*mem:\[from\s+0x2802B844\s+to\s+\(_base_SRAM\s*\+\s*_size_SRAM\s*-\s*1\)\s*\]' 'CM7_0 SRAM union does not reserve 0x28026024-0x2802B843'
Assert-Match $linkerDirectives '(?s)define\s+region\s+SRAM_HEAP_STACK\s*=\s*mem:\[from\s+0x28006BF4\s+to\s+\(_base_SRAM\s*\+\s*_size_SRAM\s*-\s*1\)\s*\]' 'CM0+ heap/stack region is not a contiguous post-camera range'
Assert-Match $linkerDirectives '(?s)define\s+region\s+SRAM_HEAP_STACK\s*=\s*mem:\[from\s+0x2802B844\s+to\s+\(_base_SRAM\s*\+\s*_size_SRAM\s*-\s*1\)\s*\]' 'CM7_0 heap/stack region is not a contiguous post-camera range'
Assert-Match $linkerDirectives '(?m)^\s*place\s+at\s+end\s+of\s+SRAM_HEAP_STACK\s*\{\s*block\s+HEAP_STACK\s*\}\s*;' 'HEAP_STACK is not placed at the end of its contiguous SRAM range'
Assert-Match $linkerDirectives '(?m)^\s*define\s+symbol\s+intercore_shared_sram_size\s*=\s*8K\s*;' 'shared SRAM size is no longer 8 KiB'
Assert-Match $linkerDirectives '(?m)^\s*define\s+symbol\s+_base_SRAM_CM7_1\s*=\s*_base_SRAM_CM7_SHARED\s*\+\s*intercore_shared_sram_size\s*;' 'CM7_1 ordinary SRAM no longer begins after the 8 KiB shared range'

$applicationFiles = Get-ChildItem -Path (Join-Path $repoRoot 'project\code'), (Join-Path $repoRoot 'project\user') -File -Recurse -Include '*.c','*.h'
foreach ($file in $applicationFiles)
{
    $applicationText = [System.IO.File]::ReadAllText($file.FullName)
    if ($applicationText -match '(?i)wifi_spi_send_buffer\s*\(|udp[_a-z0-9]*packet|packetiz[^\r\n]*udp|custom[^\r\n]*udp')
    {
        Add-Failure "application source introduces a direct/custom UDP packet path: $($file.FullName.Substring($repoRoot.Length + 1))"
    }
}

# Task 4: CM7_1 camera-only portability and hard motor lock.
$appConfig = Read-RepoFile 'project\code\app_config.h'
Assert-Match $appConfig '(?m)^\s*#define\s+APP_CAMERA_DEBUG_ONLY\s+\(\s*1U\s*\)' 'APP_CAMERA_DEBUG_ONLY is not locked to 1U'

$cameraDebugConfig = Read-RepoFile 'project\code\camera_debug_config.h'
Assert-Match $cameraDebugConfig '(?m)^\s*#define\s+APP_CAMERA_WIFI_ENABLE\s+\(\s*0U\s*\)' 'APP_CAMERA_WIFI_ENABLE is not disabled for the portability gate'
Assert-Match $cameraDebugConfig '(?m)^\s*#define\s+APP_CAMERA_DISPLAY_PERIOD_MS\s+\(\s*100U\s*\)' 'camera snapshot period is not 100 ms'
Assert-Match $cameraDebugConfig '(?m)^\s*#define\s+APP_CAMERA_STALE_TIMEOUT_MS\s+\(\s*200U\s*\)' 'camera stale timeout is not 200 ms'

$cameraDebugHeader = Read-RepoFile 'project\code\camera_debug_app.h'
Assert-Match $cameraDebugHeader '(?s)typedef\s+enum\s*\{\s*CAMERA_DEBUG_INIT_NOT_RUN\s*=\s*0\s*,\s*CAMERA_DEBUG_INIT_CAMERA_FAILED\s*,\s*CAMERA_DEBUG_INIT_WIFI_FAILED\s*,\s*CAMERA_DEBUG_INIT_SOCKET_FAILED\s*,\s*CAMERA_DEBUG_INIT_OK\s*\}\s*camera_debug_init_state_enum\s*;' 'camera debug init-state public enum does not match the approved contract'
Assert-Match $cameraDebugHeader '(?s)typedef\s+struct\s*\{.*?uint32\s+frame_count\s*;.*?uint32\s+snapshot_count\s*;.*?uint32\s+sent_count\s*;.*?uint32\s+dropped_count\s*;.*?uint32\s+timeout_count\s*;.*?uint32\s+last_frame_ms\s*;.*?uint32\s+last_send_ms\s*;.*?uint32\s+last_send_duration_ms\s*;.*?uint32\s+max_send_duration_ms\s*;.*?uint8\s+frame_valid\s*;.*?uint8\s+init_state\s*;.*?\}\s*camera_debug_diag_struct\s*;' 'camera debug diagnostics struct does not match the approved public contract'
foreach ($apiPattern in @(
    'uint8\s+camera_debug_app_init\s*\(\s*void\s*\)\s*;',
    'void\s+camera_debug_app_tick_1ms\s*\(\s*void\s*\)\s*;',
    'void\s+camera_debug_app_service\s*\(\s*void\s*\)\s*;',
    'uint32\s+camera_debug_app_now_ms\s*\(\s*void\s*\)\s*;',
    'const\s+camera_debug_diag_struct\s*\*\s*camera_debug_app_get_diag\s*\(\s*void\s*\)\s*;'
))
{
    Assert-Match $cameraDebugHeader $apiPattern "camera debug public API is missing: $apiPattern"
}

$cameraDebugSource = Read-RepoFile 'project\code\camera_debug_app.c'
Assert-Match $cameraDebugSource '(?m)^\s*static\s+uint8\s+image_copy\s*\[\s*MT9V03X_H\s*\]\s*\[\s*MT9V03X_W\s*\]\s*;' 'camera debug app does not use one fixed image_copy buffer'
Assert-Match $cameraDebugSource '(?m)^\s*static\s+seekfree_assistant_camera_struct\s+camera_information\s*;' 'camera debug app does not reserve the approved Assistant camera object'
Assert-Match $cameraDebugSource 'memcpy\s*\(\s*image_copy\s*\[\s*0\s*\]\s*,\s*mt9v03x_image\s*\[\s*0\s*\]\s*,\s*MT9V03X_IMAGE_SIZE\s*\)' 'camera snapshot does not copy exactly MT9V03X_IMAGE_SIZE bytes'
Assert-NoMatch $cameraDebugSource '(?i)\b(?:malloc|calloc|realloc|free)\s*\(' 'camera debug app introduces dynamic allocation'
Assert-NoMatch $cameraDebugSource '(?i)\b(?:wifi_spi_init|wifi_spi_socket_connect|seekfree_assistant_interface_init|seekfree_assistant_camera_config|seekfree_assistant_camera_send)\s*\(' 'WiFi or Assistant transport is reachable in the disabled portability implementation'

$actuatorMotor = Read-RepoFile 'project\code\actuator_motor.c'
Assert-Match $actuatorMotor '(?s)static\s+void\s+actuator_motor_send_duty\s*\(\s*int16\s+left_duty\s*,\s*int16\s+right_duty\s*\)\s*\{\s*#if\s+APP_CAMERA_DEBUG_ONLY\s+left_duty\s*=\s*0\s*;\s*right_duty\s*=\s*0\s*;\s*#endif.*?bldc_foc_uart_set_duty\s*\(\s*left_duty\s*,\s*right_duty\s*\)' 'actuator_motor_send_duty does not clamp both duties to zero before the BLDC choke point'

$boardAdc = Read-RepoFile 'project\code\board_adc.c'
Assert-Match $boardAdc '(?m)^\s*#include\s+"app_config\.h"' 'board_adc.c does not include app_config.h'
Assert-Match $boardAdc '(?s)#if\s+!APP_CAMERA_DEBUG_ONLY\s+adc_init\s*\(\s*BUS_PHASE_PORT.*?adc_mean_filter_convert\s*\(\s*BUS_PHASE_PORT.*?#endif' 'board_adc.c does not compile out BUS_PHASE_PORT initialization and warm-up reads'
Assert-Match $boardAdc '(?s)#if\s+!APP_CAMERA_DEBUG_ONLY\s+uint16\s+adc_value_bus_phase\s*=\s*0\s*;\s*#endif.*?#if\s+!APP_CAMERA_DEBUG_ONLY\s+adc_value_bus_phase\s*=\s*adc_convert\s*\(\s*BUS_PHASE_PORT.*?#endif' 'board_adc.c does not compile out the BUS_PHASE_PORT raw variable and conversion'
Assert-Match $boardAdc '(?s)#if\s+APP_CAMERA_DEBUG_ONLY\s+adc_information\.voltage_bus\s*=\s*0\.0f\s*;\s*adc_information\.voltage_bus_filter\s*=\s*0\.0f\s*;\s*adc_information\.voltage_bus_filter_offset\s*=\s*0\.0f\s*;\s*#else.*?#endif' 'camera-debug bus current and filter are not forced to benign zero'

$cm7Main = Read-RepoFile 'project\user\main_cm7_1.c'
Assert-Match $cm7Main 'pit_ms_init\s*\(\s*PIT_CH2\s*,\s*1\s*\)' 'CM7_1 main does not initialize PIT_CH2 at 1 ms'
Assert-Match $cm7Main 'camera_debug_app_init\s*\(\s*\)' 'CM7_1 main does not initialize the camera debug app'
Assert-Match $cm7Main '(?s)while\s*\(\s*true\s*\)\s*\{\s*camera_debug_app_service\s*\(\s*\)\s*;\s*\}' 'CM7_1 main loop does not exclusively service the camera debug app'
Assert-NoMatch $cm7Main '(?i)uart_init\s*\(\s*UART_0' 'CM7_1 main introduces UART0 initialization'

$cm7Isr = Read-RepoFile 'project\user\cm7_1_isr.c'
Assert-Match $cm7Isr '(?s)void\s+pit0_ch2_isr\s*\(\s*\)\s*\{\s*pit_isr_flag_clear\s*\(\s*PIT_CH2\s*\)\s*;\s*camera_debug_app_tick_1ms\s*\(\s*\)\s*;\s*\}' 'PIT_CH2 ISR is not limited to flag clear plus 1 ms camera tick'
Assert-NoMatch $cm7Isr '(?i)\b(?:mt9v03x_init|memcpy|wifi_spi_|seekfree_assistant_|camera_debug_app_init|camera_debug_app_service)\s*\(' 'camera initialization, copying, service, or transport appears in CM7_1 ISR context'
foreach ($pitChannel in 10..21)
{
    Assert-Match $cm7Isr "(?s)void\s+pit0_ch${pitChannel}_isr\s*\(\s*\)\s*\{\s*pit_isr_flag_clear\s*\(\s*PIT_CH${pitChannel}\s*\)\s*;\s*\}" "CM7_1 PIT_CH$pitChannel linker stub is missing or does more than clear its own flag"
}

foreach ($cameraProjectFile in @('camera_debug_app.c', 'camera_debug_app.h', 'camera_debug_config.h'))
{
    Assert-Match $cm7Project ([Regex]::Escape("\code\$cameraProjectFile</name>")) "CM7_1 project does not contain $cameraProjectFile"
}

if ($failures.Count -ne 0)
{
    Write-Host "FAIL: camera/Seekfree API static contract ($($failures.Count) issue(s))" -ForegroundColor Red
    foreach ($failure in $failures)
    {
        Write-Host " - $failure" -ForegroundColor Red
    }
    exit 1
}

Write-Host 'PASS: camera/Seekfree API static contract' -ForegroundColor Green

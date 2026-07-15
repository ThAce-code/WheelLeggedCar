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

function Remove-CComments([string]$text)
{
    if ($null -eq $text)
    {
        return $null
    }

    return [Regex]::Replace($text, '(?s)/\*.*?\*/|//[^\r\n]*', '')
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
Assert-Match $linkerDirectives '(?s)mem:\[from\s+_base_SRAM\s+to\s+0x28026023\s*\]\s*\|\s*mem:\[from\s+0x2802B844\s+to\s+0x2805FFFF\s*\]\s*\|\s*mem:\[from\s+0x28070000\s+to\s+0x2807FFFF\s*\]' 'CM7_0 SRAM union does not reserve the fixed image and 0x28060000-0x2806FFFF camera data plane'
Assert-Match $linkerDirectives '(?s)define\s+region\s+SRAM_HEAP_STACK\s*=\s*mem:\[from\s+0x28006BF4\s+to\s+\(_base_SRAM\s*\+\s*_size_SRAM\s*-\s*1\)\s*\]' 'CM0+ heap/stack region is not a contiguous post-camera range'
Assert-Match $linkerDirectives '(?s)define\s+region\s+SRAM_HEAP_STACK\s*=\s*mem:\[from\s+0x28070000\s+to\s+0x2807FFFF\s*\]' 'CM7_0 heap/stack region is not 0x28070000-0x2807FFFF'
Assert-Match $linkerDirectives '(?m)^\s*place\s+at\s+end\s+of\s+SRAM_HEAP_STACK\s*\{\s*block\s+HEAP_STACK\s*\}\s*;' 'HEAP_STACK is not placed at the end of its contiguous SRAM range'
Assert-Match $linkerDirectives '(?m)^\s*define\s+symbol\s+intercore_shared_sram_size\s*=\s*8K\s*;' 'shared SRAM size is no longer 8 KiB'
Assert-Match $linkerDirectives '(?m)^\s*define\s+symbol\s+_base_SRAM_CM7_1\s*=\s*_base_SRAM_CM7_SHARED\s*\+\s*intercore_shared_sram_size\s*;' 'CM7_1 ordinary SRAM no longer begins after the 8 KiB shared range'
Assert-Match $linkerDirectives '(?s)define\s+symbol\s+camera_shared_sram_base\s*=\s*0x28060000\s*;\s*define\s+symbol\s+camera_shared_sram_size\s*=\s*64K\s*;.*?define\s+exported\s+symbol\s+__camera_shared_sram_base\s*=\s*camera_shared_sram_base\s*;\s*define\s+exported\s+symbol\s+__camera_shared_sram_size\s*=\s*camera_shared_sram_size\s*;' 'camera linker symbols are not exported as 0x28060000/64K'
Assert-Match $linkerDirectives '(?s)(?=.*?define\s+symbol\s+_base_SRAM_CM7_SHARED\s*=\s*sram_base_address\s*\+\s*cm0plus_sram_reserve\s*\+\s*cm7_0_sram_reserve\s*;)(?=.*?define\s+symbol\s+intercore_shared_sram_size\s*=\s*8K\s*;)(?=.*?define\s+symbol\s+_base_SRAM_CM7_1\s*=\s*_base_SRAM_CM7_SHARED\s*\+\s*intercore_shared_sram_size\s*;)' 'shared control 0x28080000/8K or CM7_1 base 0x28082000 contract changed'

$intercoreMemory = Read-RepoFile 'project\code\intercore_memory.c'
Assert-Match $intercoreMemory '(?s)extern\s+uint8\s+__intercore_shared_sram_base\s*;\s*extern\s+uint8\s+__intercore_shared_sram_size\s*;\s*extern\s+uint8\s+__camera_shared_sram_base\s*;\s*extern\s+uint8\s+__camera_shared_sram_size\s*;' 'intercore_memory_configure does not consume both linker regions'
Assert-Match $intercoreMemory '(?s)INTERCORE_SHARED_BASE_ADDRESS\s*!=\s*shared_base.*?INTERCORE_SHARED_SIZE_BYTES\s*!=\s*shared_size.*?INTERCORE_CAMERA_DATA_BASE_ADDRESS\s*!=\s*camera_base.*?INTERCORE_CAMERA_DATA_SIZE_BYTES\s*!=\s*camera_size' 'intercore_memory_configure does not validate both linker base/size pairs'
Assert-Match $intercoreMemory '(?s)const\s+cy_stc_mpu_region_cfg_t\s+regions\s*\[\s*2\s*\]\s*=\s*\{.*?\.addr\s*=\s*INTERCORE_SHARED_BASE_ADDRESS\s*,.*?\.size\s*=\s*CY_MPU_SIZE_8KB\s*,.*?\.attribute\s*=\s*CY_MPU_ATTR_NORM_SHR_MEM_NC\s*,.*?\.execute\s*=\s*CY_MPU_INST_ACCESS_DIS\s*,.*?\.addr\s*=\s*INTERCORE_CAMERA_DATA_BASE_ADDRESS\s*,.*?\.size\s*=\s*CY_MPU_SIZE_64KB\s*,.*?\.attribute\s*=\s*CY_MPU_ATTR_NORM_SHR_MEM_NC\s*,.*?\.execute\s*=\s*CY_MPU_INST_ACCESS_DIS\s*,' 'MPU region array is not two non-cacheable, execute-never shared/control regions'
Assert-Match $intercoreMemory 'Cy_MPU_Setup\s*\(\s*regions\s*,\s*2U\s*,' 'Cy_MPU_Setup does not receive the two-element region array'
if (([Regex]::Matches($intercoreMemory, 'Cy_MPU_Setup\s*\(')).Count -ne 1)
{
    Add-Failure 'intercore_memory_configure must call Cy_MPU_Setup exactly once'
}

$intercoreMemoryHeader = Read-RepoFile 'project\code\intercore_memory.h'
Assert-Match $intercoreMemoryHeader 'volatile\s+uint8\s*\*\s*intercore_memory_get_camera_data\s*\(\s*void\s*\)\s*;' 'camera data-plane getter is not exposed'

$intercoreCameraHeader = Read-RepoFile 'project\code\intercore_camera.h'
Assert-Match $intercoreCameraHeader '(?s)typedef\s+enum\s*\{\s*INTERCORE_CAMERA_NO_FRAME\s*=\s*0\s*,\s*INTERCORE_CAMERA_OK\s*=\s*1\s*,\s*INTERCORE_CAMERA_INVALID\s*=\s*2\s*,\s*INTERCORE_CAMERA_NO_FREE_SLOT\s*=\s*3\s*,\s*INTERCORE_CAMERA_EPOCH_CHANGED\s*=\s*4\s*\}\s*intercore_camera_result_enum\s*;' 'camera result enum does not match the approved fixed ABI'
Assert-Match $intercoreCameraHeader '(?s)typedef\s+struct\s*\{\s*uint8\s+slot_index\s*;\s*uint32\s+sequence\s*;\s*uint32\s+capture_ms\s*;\s*uint32\s+publish_ms\s*;\s*uint16\s+width\s*;\s*uint16\s+height\s*;\s*uint16\s+stride\s*;\s*uint32\s+frame_bytes\s*;\s*volatile\s+uint8\s*\*\s*pixels\s*;\s*\}\s*intercore_camera_frame_view_struct\s*;' 'camera frame view does not match the approved fixed ABI'
Assert-Match $intercoreCameraHeader '(?s)typedef\s+struct\s*\{\s*volatile\s+intercore_camera_control_struct\s*\*\s*control\s*;\s*volatile\s+uint8\s*\*\s*data_plane\s*;\s*uint32\s+boot_epoch\s*;\s*uint32\s+last_consumed_sequence\s*;\s*uint8\s+role\s*;\s*uint8\s+attached\s*;' 'camera transport prefix does not match the approved fixed ABI'
Assert-Match $intercoreCameraHeader '(?s)uint8\s+intercore_camera_consumer_publish_observation\s*\(\s*intercore_camera_transport_struct\s*\*\s*transport\s*,\s*uint32\s+sequence\s*,\s*uint32\s+frame_age_ms\s*,\s*uint8\s+sample_0_0\s*,\s*uint8\s+sample_center\s*,\s*uint8\s+frame_valid\s*\)\s*;' 'camera consumer shared-observation API is missing'

$intercoreProtocol = Read-RepoFile 'project\code\intercore_protocol.h'
Assert-Match $intercoreProtocol '(?m)^\s*#define\s+INTERCORE_CAMERA_VERSION\s+\(\s*2U\s*\)' 'camera protocol version is not 2 after adding the shared consumer observation'
Assert-Match $intercoreProtocol '(?s)uint32\s+max_process_duration_us\s*;\s*uint32\s+consumer_last_sequence\s*;\s*uint32\s+consumer_last_frame_age_ms\s*;\s*uint8\s+consumer_sample_0_0\s*;\s*uint8\s+consumer_sample_center\s*;\s*uint8\s+consumer_frame_valid\s*;\s*uint8\s+consumer_reserved\s*;\s*uint32\s+producer_period_drop_count\s*;\s*uint8\s+reserved\s*\[\s*68\s*\]\s*;' 'camera control does not expose the approved consumer mirror and producer period-drop diagnostic inside the existing 256-byte ABI'
Assert-Match $intercoreProtocol 'INTERCORE_LAYOUT_CHECK\s*\(\s*camera_consumer_sequence_offset\s*,\s*offsetof\s*\(\s*intercore_camera_control_struct\s*,\s*consumer_last_sequence\s*\)\s*==\s*172U\s*\)' 'consumer sequence offset check is missing'
Assert-Match $intercoreProtocol 'INTERCORE_LAYOUT_CHECK\s*\(\s*camera_consumer_observation_offset\s*,\s*offsetof\s*\(\s*intercore_camera_control_struct\s*,\s*consumer_last_frame_age_ms\s*\)\s*==\s*176U\s*\)' 'consumer observation offset check is missing'
Assert-Match $intercoreProtocol 'INTERCORE_LAYOUT_CHECK\s*\(\s*camera_period_drop_offset\s*,\s*offsetof\s*\(\s*intercore_camera_control_struct\s*,\s*producer_period_drop_count\s*\)\s*==\s*184U\s*\)' 'producer period-drop offset check is missing'

$handoffSpec = Read-RepoFile 'docs\superpowers\specs\2026-07-14-mt9v03x-cross-core-handoff-design.md'
$handoffPlan = Read-RepoFile 'docs\superpowers\plans\2026-07-14-mt9v03x-cross-core-handoff.md'
foreach($approvedDocument in @(
    @{ Name = 'approved handoff spec'; Text = $handoffSpec },
    @{ Name = 'approved handoff plan'; Text = $handoffPlan }
))
{
    Assert-Match $approvedDocument.Text '(?i)camera-control version (?:is|equals) 2\b' "$($approvedDocument.Name) does not identify camera-control version 2"
    Assert-Match $approvedDocument.Text '(?s)uint32\s+consumer_last_sequence\s*;\s*uint32\s+consumer_last_frame_age_ms\s*;\s*uint8\s+consumer_sample_0_0\s*;\s*uint8\s+consumer_sample_center\s*;\s*uint8\s+consumer_frame_valid\s*;\s*uint8\s+consumer_reserved\s*;\s*uint32\s+producer_period_drop_count\s*;\s*uint8\s+reserved\s*\[\s*68\s*\]\s*;' "$($approvedDocument.Name) does not contain the approved version-2 diagnostics layout"
    Assert-Match $approvedDocument.Text '(?s)consumer_last_sequence.*?172.*?consumer_last_frame_age_ms.*?176' "$($approvedDocument.Name) does not lock the consumer mirror offsets to 172/176"
    Assert-Match $approvedDocument.Text '(?i)only after (?:a )?successful release' "$($approvedDocument.Name) does not constrain mirror publication to successful release"
    Assert-Match $approvedDocument.Text '(?s)consumer_last_frame_age_ms.*?consumer_sample_0_0.*?consumer_sample_center.*?consumer_frame_valid.*?DMB.*?consumer_last_sequence.*?DMB' "$($approvedDocument.Name) does not specify the sequence-last DMB commit order"
    Assert-Match $approvedDocument.Text '(?i)change note' "$($approvedDocument.Name) omits the version-2 change note"
}

$intercoreCameraSource = Read-RepoFile 'project\code\intercore_camera.c'
Assert-Match $intercoreCameraSource '(?s)if\s*\(\s*INTERCORE_CAMERA_MAGIC\s*!=\s*control->magic\s*\)\s*\{\s*return\s+0U\s*;\s*\}\s*INTERCORE_CAMERA_DMB\s*\(\s*\)\s*;' 'camera control validation does not acquire after observing magic'
Assert-Match $intercoreCameraSource '(?s)control->slot\[slot_index\]\.sequence\s*=\s*control->capture_sequence\s*;.*?INTERCORE_CAMERA_DMB\s*\(\s*\)\s*;\s*control->slot\[slot_index\]\.state\s*=\s*INTERCORE_CAMERA_SLOT_READY\s*;\s*INTERCORE_CAMERA_DMB\s*\(\s*\)\s*;' 'camera publish does not order fields before READY with DMB barriers'
Assert-Match $intercoreCameraSource '(?s)INTERCORE_CAMERA_DMB\s*\(\s*\)\s*;\s*control->slot\[view->slot_index\]\.state\s*=\s*INTERCORE_CAMERA_SLOT_FREE\s*;\s*INTERCORE_CAMERA_DMB\s*\(\s*\)\s*;' 'camera release does not fence before and after FREE ownership transfer'
Assert-Match $intercoreCameraSource '(?s)for\s*\(\s*slot\s*=\s*0U\s*;\s*slot\s*<\s*INTERCORE_CAMERA_SLOT_COUNT\s*;\s*slot\+\+\s*\).*?INTERCORE_CAMERA_SLOT_READING\s*<\s*control->slot\[slot\]\.state' 'camera layout validation does not reject illegal slot states'
Assert-Match $intercoreCameraSource '(?s)intercore_camera_consumer_publish_observation\s*\([^)]*\)\s*\{.*?last_consumed_sequence\s*!=\s*sequence.*?return\s+0U\s*;.*?control->consumer_last_frame_age_ms\s*=\s*frame_age_ms\s*;.*?control->consumer_sample_0_0\s*=\s*sample_0_0\s*;.*?control->consumer_sample_center\s*=\s*sample_center\s*;.*?control->consumer_frame_valid\s*=\s*frame_valid\s*;.*?INTERCORE_CAMERA_DMB\s*\(\s*\)\s*;\s*control->consumer_last_sequence\s*=\s*sequence\s*;\s*INTERCORE_CAMERA_DMB\s*\(\s*\)\s*;' 'consumer observation is not fenced with sequence as the final commit marker'

$cm7_0Project = Read-RepoFile 'project\iar\project_config\cyt4bb7_cm_7_0.ewp'
$cm7_0ApplicationMappings = [Regex]::Matches(
    $cm7_0Project,
    '\$PROJ_DIR\$\\\.\.\\\.\.\\(code|user)\\([^<]+\.c)</name>')
foreach($mapping in $cm7_0ApplicationMappings)
{
    $relativePath = 'project\' + $mapping.Groups[1].Value + '\' + $mapping.Groups[2].Value
    $mappedText = Remove-CComments (Read-RepoFile $relativePath)
    Assert-NoMatch $mappedText '(?i)\b(?:wifi_spi_|seekfree_assistant_)\w*\s*\(' "WiFi or Assistant work is reachable from CM7_0 mapped application source: $relativePath"
}
foreach ($cameraProjectFile in @('intercore_camera.c', 'intercore_camera.h'))
{
    $cameraProjectPattern = [Regex]::Escape("\code\$cameraProjectFile</name>")
    Assert-Match $cm7_0Project $cameraProjectPattern "CM7_0 project does not contain $cameraProjectFile"
    Assert-Match $cm7Project $cameraProjectPattern "CM7_1 project does not contain $cameraProjectFile"
}

$intercoreNotify = Read-RepoFile 'project\code\intercore_notify.h'
Assert-Match $intercoreNotify '(?m)^\s*#define\s+INTERCORE_NOTIFY_CAMERA_READY\s+\(\s*0x00000008UL\s*\)' 'INTERCORE_NOTIFY_CAMERA_READY is not 0x00000008UL'

$applicationFiles = Get-ChildItem -Path (Join-Path $repoRoot 'project\code'), (Join-Path $repoRoot 'project\user') -File -Recurse -Include '*.c','*.h'
foreach ($file in $applicationFiles)
{
    $applicationText = Remove-CComments ([System.IO.File]::ReadAllText($file.FullName))
    $isTransportAdapter = ($file.Name -eq 'camera_seekfree_transport.c')
    if (($applicationText -match '(?i)wifi_spi_send_buffer\s*\(') -and (-not $isTransportAdapter))
    {
        Add-Failure "application source bypasses the camera transport adapter: $($file.FullName.Substring($repoRoot.Length + 1))"
    }
    if ($applicationText -match '(?i)wifi_spi_read_buffer\s*\(')
    {
        Add-Failure "application source directly reads WiFi-SPI: $($file.FullName.Substring($repoRoot.Length + 1))"
    }
    if ($applicationText -match '(?i)udp[_a-z0-9]*packet|packetiz[^\r\n]*udp|custom[^\r\n]*udp')
    {
        Add-Failure "application source introduces a direct/custom UDP packet path: $($file.FullName.Substring($repoRoot.Length + 1))"
    }
}

# Task 3: CM7_0 capture producer and CM7_1 WiFi-SPI frame consumer.
$appConfig = Read-RepoFile 'project\code\app_config.h'
Assert-Match $appConfig '(?m)^\s*#define\s+APP_CAMERA_DEBUG_ONLY\s+\(\s*1U\s*\)' 'APP_CAMERA_DEBUG_ONLY is not locked to 1U'

$cameraDebugConfig = Read-RepoFile 'project\code\camera_debug_config.h'
Assert-Match $cameraDebugConfig '(?m)^\s*#define\s+APP_CAMERA_WIFI_ENABLE\s+\(\s*1U\s*\)' 'APP_CAMERA_WIFI_ENABLE is not enabled for the CM7_1 display gate'
Assert-Match $cameraDebugConfig '(?m)^\s*#define\s+APP_CAMERA_DISPLAY_PERIOD_MS\s+\(\s*40U\s*\)' 'camera snapshot period is not 40 ms'
Assert-Match $cameraDebugConfig '(?m)^\s*#define\s+APP_CAMERA_STALE_TIMEOUT_MS\s+\(\s*200U\s*\)' 'camera stale timeout is not 200 ms'
$retryMatch = [Regex]::Match($cameraDebugConfig, '(?m)^\s*#define\s+APP_CAMERA_WIFI_RETRY_MS\s+\(\s*(\d+)U\s*\)')
if((-not $retryMatch.Success) -or ([uint32]$retryMatch.Groups[1].Value -lt 5000))
{
    Add-Failure 'camera WiFi reconnect interval is less than 5000 CM7_1 ticks'
}
Assert-Match $cameraDebugConfig '(?m)^\s*#define\s+APP_CAMERA_CAPTURE_ENABLE\s+\(\s*1U\s*\)' 'camera capture compile switch is not enabled in the delivered configuration'
Assert-Match $cameraDebugConfig '(?m)^\s*#define\s+APP_CAMERA_GATE_WINDOW_MS\s+\(\s*60000U\s*\)' 'automatic camera gate evidence window is not 60 seconds in the CM7_1 clock domain'

$cameraProducer = Remove-CComments (Read-RepoFile 'project\code\camera_capture_producer.c')
Assert-Match $cameraProducer 'Cy_SysInt_DisableIRQ\s*\(\s*tcpwm_0_interrupts_59_IRQn\s*\)' 'camera producer does not disable only the TCPWM59 system interrupt source'
Assert-Match $cameraProducer 'Cy_SysInt_EnableIRQ\s*\(\s*tcpwm_0_interrupts_59_IRQn\s*\)' 'camera producer does not re-enable the TCPWM59 system interrupt source'
Assert-Match $cameraProducer 'memcpy\s*\(\s*\(\s*void\s*\*\s*\)\s*slot_pixels\s*,\s*mt9v03x_image\s*\[\s*0\s*\]\s*,\s*MT9V03X_IMAGE_SIZE\s*\)' 'camera producer does not copy exactly MT9V03X_IMAGE_SIZE bytes into the claimed slot'
Assert-Match $cameraProducer '(?s)camera_capture_producer_service\s*\([^)]*\)\s*\{.*?Cy_SysInt_DisableIRQ\s*\(\s*tcpwm_0_interrupts_59_IRQn\s*\)\s*;.*?memcpy\s*\(\s*\(\s*void\s*\*\s*\)\s*slot_pixels\s*,\s*mt9v03x_image\s*\[\s*0\s*\]\s*,\s*MT9V03X_IMAGE_SIZE\s*\)\s*;\s*camera_transport\.control->producer_heartbeat_ms\s*=\s*now_ms\s*;\s*publish_ok\s*=\s*intercore_camera_producer_publish\s*\(.*?\)\s*;.*?Cy_SysInt_EnableIRQ\s*\(\s*tcpwm_0_interrupts_59_IRQn\s*\)\s*;.*?if\s*\(\s*0U\s*==\s*publish_ok\s*\).*?intercore_camera_producer_abort\s*\(\s*&camera_transport\s*,\s*slot_index\s*\)' 'producer critical path is not ordered Disable -> memcpy -> producer clock catch-up -> publish -> Enable -> conditional abort'
Assert-Match $cameraProducer 'APP_CAMERA_DISPLAY_PERIOD_MS\s*<=\s*\(\s*now_ms\s*-\s*producer_diag\.last_publish_ms\s*\)' 'camera producer does not enforce the configured latest-frame publication period'
Assert-NoMatch $cameraProducer '(?i)\bNVIC_DisableIRQ\s*\(\s*CPUIntIdx3_IRQn\s*\)|\b(?:__disable_irq|Cy_SysLib_EnterCriticalSection)\s*\(' 'camera producer masks the aggregate CPU IRQ or globally disables interrupts'
Assert-NoMatch $cameraProducer '(?i)\b(?:Cy_TCPWM_ClearInterrupt|Cy_TCPWM_Counter_ClearInterrupt|NVIC_ClearPendingIRQ)\s*\(' 'camera producer clears pending camera or aggregate interrupt state'
Assert-NoMatch $cameraProducer '(?i)\b(?:wifi_spi_|seekfree_assistant_)\w*\s*\(' 'WiFi or Assistant work is reachable from the CM7_0 producer'
Assert-Match $cameraProducer '(?s)#if\s+!APP_CAMERA_CAPTURE_ENABLE.*?return\s+0U\s*;.*?#endif' 'producer init lacks a true capture-disabled baseline switch'
Assert-Match $cameraProducer '(?s)camera_capture_producer_service\s*\([^)]*\)\s*\{\s*#if\s+!APP_CAMERA_CAPTURE_ENABLE\s+return\s*;\s*#else.*?#endif\s*\}' 'producer service does not compile out capture work for the baseline'

$cameraConsumer = Remove-CComments (Read-RepoFile 'project\code\camera_frame_consumer.c')
$cameraAdapter = Remove-CComments (Read-RepoFile 'project\code\camera_seekfree_transport.c')
$cameraAdapterHeader = Remove-CComments (Read-RepoFile 'project\code\camera_seekfree_transport.h')
Assert-NoMatch $cameraConsumer '(?i)\b(?:mt9v03x_init|mt9v03x_finish_flag)\w*\s*\(' 'camera init or driver flag is reachable from the CM7_1 consumer'
Assert-NoMatch $cameraConsumer '(?is)memcpy\s*\([^;]*(?:view\.pixels|camera_data)' 'camera frame pixels are copied by the CM7_1 consumer'
Assert-NoMatch $cameraConsumer '(?i)\bmt9v03x_finish_flag\b' 'camera driver finish flag is referenced by the CM7_1 consumer'
Assert-Match $cameraConsumer '(?s)camera_frame_consumer_try_network\s*\([^)]*\)\s*\{.*?if\s*\(\s*\(uint8\)CAMERA_CONSUMER_LINK_CONNECTED\s*!=\s*consumer_diag\.wifi_state\s*\).*?wifi_spi_init\s*\(\s*APP_CAMERA_WIFI_SSID\s*,\s*APP_CAMERA_WIFI_PASSWORD\s*\).*?wifi_spi_socket_connect\s*\(\s*"TCP"\s*,\s*APP_CAMERA_WIFI_TARGET_IP\s*,\s*APP_CAMERA_WIFI_TARGET_PORT\s*,\s*APP_CAMERA_WIFI_LOCAL_PORT\s*\)' 'network retry does not preserve an already-connected WiFi association while reconnecting TCP'
Assert-Match $cameraConsumer '(?s)wifi_spi_socket_connect\s*\(\s*"TCP"\s*,\s*APP_CAMERA_WIFI_TARGET_IP\s*,\s*APP_CAMERA_WIFI_TARGET_PORT\s*,\s*APP_CAMERA_WIFI_LOCAL_PORT\s*\).*?consumer_diag\.socket_state\s*=\s*\(uint8\)CAMERA_CONSUMER_LINK_CONNECTED\s*;\s*socket_connected_ms\s*=\s*camera_frame_consumer_now_ms\s*\(\s*\)\s*;' 'socket stability timestamp is not sampled after the blocking connect succeeds'
Assert-NoMatch $cameraConsumer 'socket_connected_ms\s*=\s*now_ms\s*;' 'socket stability timestamp incorrectly reuses the pre-connect attempt timestamp'
Assert-Match $cameraAdapter 'seekfree_assistant_interface_init\s*\(\s*SEEKFREE_ASSISTANT_CUSTOM\s*\)' 'CM7_1 consumer does not select the observable custom Assistant adapter'
Assert-Match $cameraConsumer '(?s)seekfree_assistant_camera_config\s*\(\s*&camera_information\[0\]\s*,\s*SEEKFREE_ASSISTANT_CAMERA_TYPE_MT9V03X\s*,\s*MT9V03X_W\s*,\s*MT9V03X_H\s*,\s*\(\s*uint8\s*\*\s*\)\s*\(\s*uintptr_t\s*\)\s*INTERCORE_CAMERA_DATA_BASE_ADDRESS\s*\).*?seekfree_assistant_camera_config\s*\(\s*&camera_information\[1\]\s*,\s*SEEKFREE_ASSISTANT_CAMERA_TYPE_MT9V03X\s*,\s*MT9V03X_W\s*,\s*MT9V03X_H\s*,\s*\(\s*uint8\s*\*\s*\)\s*\(\s*uintptr_t\s*\)\s*\(\s*INTERCORE_CAMERA_DATA_BASE_ADDRESS\s*\+\s*INTERCORE_CAMERA_SLOT_SIZE_BYTES\s*\)\s*\)' 'CM7_1 consumer does not configure one Assistant camera object for each fixed shared slot'
Assert-Match $cameraConsumer '(?s)if\s*\(\s*\(\s*0U\s*!=\s*consumer_diag\.reconnect_count\s*\)\s*&&\s*\(\s*APP_CAMERA_WIFI_RETRY_MS\s*>\s*\(\s*now_ms\s*-\s*consumer_diag\.last_reconnect_ms\s*\)\s*\)\s*\)\s*\{\s*return\s*;\s*\}.*?consumer_diag\.last_reconnect_ms\s*=\s*now_ms\s*;.*?consumer_diag\.reconnect_count\+\+' 'CM7_1 reconnect attempts are not gated by APP_CAMERA_WIFI_RETRY_MS'
Assert-NoMatch $cameraConsumer '\bwifi_spi_socket_close\s*\(' 'failed socket path attempts a close command even though the module interrupt can remain low after a transfer timeout'
Assert-Match $cameraConsumer '(?s)if\s*\(\s*0U\s*!=\s*wifi_spi_socket_connect.*?\{\s*consumer_diag\.socket_state\s*=\s*\(uint8\)CAMERA_CONSUMER_LINK_FAILED\s*;\s*consumer_diag\.wifi_state\s*=\s*\(uint8\)CAMERA_CONSUMER_LINK_FAILED\s*;\s*consumer_diag\.init_state\s*=\s*\(uint8\)CAMERA_CONSUMER_INIT_SOCKET_FAILED\s*;\s*return\s*;\s*\}' 'socket-connect failure does not force the next retry through a WiFi-SPI module reset'
Assert-Match $cameraConsumer '(?s)if\s*\(\s*\(uint8\)CAMERA_CONSUMER_LINK_CONNECTED\s*!=\s*consumer_diag\.socket_state\s*\)\s*\{.*?camera_frame_consumer_try_network\s*\(\s*\)\s*;\s*return\s*;\s*\}.*?intercore_camera_consumer_acquire_latest' 'CM7_1 can acquire a frame while a blocking network connection attempt is in progress'
Assert-Match $cameraConsumer '(?s)producer_now_ms\s*=\s*camera_transport\.control->producer_heartbeat_ms\s*;.*?consumer_diag\.last_frame_age_ms\s*=\s*intercore_camera_frame_age_ms\s*\(\s*producer_now_ms\s*,\s*view\.capture_ms\s*\)' 'consumer frame age is not calculated from a snapshot of the CM7_0 producer clock domain'
Assert-NoMatch $cameraConsumer '\bnow_ms\s*-\s*view\.capture_ms\b' 'consumer subtracts producer capture time from the unrelated CM7_1 clock domain'
Assert-Match $cameraConsumer '(?s)vision_view\.slot_index\s*=\s*view\.slot_index\s*;.*?vision_view\.sequence\s*=\s*view\.sequence\s*;.*?vision_view\.capture_ms\s*=\s*view\.capture_ms\s*;.*?vision_view\.frame_age_ms\s*=\s*consumer_diag\.last_frame_age_ms\s*;.*?vision_view\.width\s*=\s*view\.width\s*;.*?vision_view\.height\s*=\s*view\.height\s*;.*?vision_view\.stride\s*=\s*view\.stride\s*;.*?vision_view\.frame_bytes\s*=\s*view\.frame_bytes\s*;.*?vision_view\.pixels\s*=\s*view\.pixels\s*;' 'consumer does not expose the complete read-only vision frame boundary'
Assert-Match $cameraConsumer '(?s)consumer_diag\.last_process_duration_us\s*=\s*0U\s*;\s*consumer_diag\.max_process_duration_us\s*=\s*0U\s*;\s*camera_transport\.control->last_process_duration_us\s*=\s*0U\s*;\s*camera_transport\.control->max_process_duration_us\s*=\s*0U\s*;' 'current vision boundary does not publish honest zero processing duration locally and in shared v2 diagnostics'
Assert-Match $cameraConsumer '(?s)camera_seekfree_transport_begin\s*\(.*?seekfree_assistant_camera_send\s*\(\s*&camera_information\[vision_view\.slot_index\]\s*\)\s*;.*?send_ok\s*=\s*camera_seekfree_transport_frame_complete\s*\(\s*\)\s*;.*?if\s*\(\s*0U\s*!=\s*send_ok\s*\).*?consumer_diag\.sent_count\+\+.*?else.*?consumer_diag\.send_failure_count\+\+.*?CAMERA_CONSUMER_LINK_FAILED.*?intercore_camera_consumer_release_at' 'consumer does not count only complete header+payload sends, fail the socket on partial transfer, and release the READING slot'
Assert-NoMatch $cameraConsumer '\b\w+\s*=\s*seekfree_assistant_camera_send\s*\(' 'consumer fabricates a success result from the void Assistant send API'
Assert-Match $cameraConsumer 'intercore_camera_consumer_release_at\s*\(\s*&camera_transport\s*,\s*&view\s*,\s*camera_frame_consumer_now_ms\s*\(\s*\)\s*\)' 'consumer does not refresh the heartbeat with the current tick immediately at release'
Assert-Match $cameraConsumer '(?s)if\s*\(\s*0U\s*!=\s*release_ok\s*\)\s*\{\s*consumer_diag\.released_count\+\+\s*;\s*\(void\)intercore_camera_consumer_publish_observation\s*\(\s*&camera_transport\s*,\s*consumer_diag\.last_sequence\s*,\s*consumer_diag\.last_frame_age_ms\s*,\s*consumer_diag\.sample_0_0\s*,\s*consumer_diag\.sample_center\s*,\s*consumer_diag\.frame_valid\s*\)\s*;' 'consumer does not publish the shared observation immediately after a successful release'

$cameraConsumerHeader = Remove-CComments (Read-RepoFile 'project\code\camera_frame_consumer.h')
Assert-Match $cameraConsumerHeader '(?s)typedef\s+struct\s*\{\s*uint8\s+slot_index\s*;\s*uint32\s+sequence\s*;\s*uint32\s+capture_ms\s*;\s*uint32\s+frame_age_ms\s*;\s*uint16\s+width\s*;\s*uint16\s+height\s*;\s*uint16\s+stride\s*;\s*uint32\s+frame_bytes\s*;\s*const\s+volatile\s+uint8\s*\*\s*pixels\s*;\s*\}\s*camera_vision_frame_view_struct\s*;' 'camera vision frame view boundary is not const volatile'
Assert-Match $cameraConsumerHeader '(?s)uint32\s+send_failure_count\s*;.*?camera_send_duration_histogram_struct\s+startup_send_histogram\s*;.*?camera_send_duration_histogram_struct\s+steady_send_histogram\s*;' 'consumer diagnostics lack send failures and startup/steady duration histograms'
Assert-Match $cameraConsumer '(?s)consumer_diag\.send_failure_count\+\+\s*;\s*consumer_diag\.socket_state\s*=\s*\(uint8\)CAMERA_CONSUMER_LINK_FAILED\s*;\s*consumer_diag\.wifi_state\s*=\s*\(uint8\)CAMERA_CONSUMER_LINK_FAILED\s*;\s*consumer_diag\.init_state\s*=\s*\(uint8\)CAMERA_CONSUMER_INIT_SOCKET_FAILED\s*;' 'frame-send failure does not force the next retry through a WiFi-SPI module reset'
Assert-Match $cameraConsumerHeader '(?s)typedef\s+struct\s*\{\s*uint32\s+generation\s*;\s*uint32\s+complete_count\s*;\s*camera_gate_snapshot_struct\s+start\s*;\s*camera_gate_snapshot_struct\s+end\s*;\s*\}\s*camera_gate_evidence_struct\s*;.*?extern\s+volatile\s+camera_gate_evidence_struct\s+camera_gate_evidence\s*;' 'camera gate evidence does not expose generation, completion count, and complete start/end snapshots'
Assert-Match $cameraConsumer '(?s)camera_frame_consumer_update_gate_snapshot\s*\([^)]*\)\s*\{.*?camera_gate_snapshot\.generation\s*=\s*generation\s*\+\s*1U\s*;.*?if\s*\(\s*0U\s*==\s*camera_gate_evidence\.complete_count\s*\).*?CAMERA_CONSUMER_LINK_CONNECTED\s*==\s*consumer_diag\.socket_state.*?APP_CAMERA_SEND_STARTUP_MS\s*<=\s*\(\s*camera_gate_snapshot\.consumer_ms\s*-\s*socket_connected_ms\s*\).*?memcpy\s*\(\s*\(\s*void\s*\*\s*\)\s*&camera_gate_evidence\.start\s*,\s*\(\s*const\s+void\s*\*\s*\)\s*&camera_gate_snapshot\s*,\s*sizeof\s*\(\s*camera_gate_snapshot\s*\)\s*\).*?APP_CAMERA_GATE_WINDOW_MS\s*<=\s*\(\s*camera_gate_snapshot\.consumer_ms\s*-\s*camera_gate_evidence\.start\.consumer_ms\s*\).*?camera_gate_evidence\.generation\s*=\s*evidence_generation\s*;.*?memcpy\s*\(\s*\(\s*void\s*\*\s*\)\s*&camera_gate_evidence\.end\s*,\s*\(\s*const\s+void\s*\*\s*\)\s*&camera_gate_snapshot\s*,\s*sizeof\s*\(\s*camera_gate_snapshot\s*\)\s*\).*?camera_gate_evidence\.complete_count\+\+\s*;.*?camera_gate_evidence\.generation\s*=\s*evidence_generation\s*\+\s*1U\s*;' 'camera gate evidence is not latched from the first stable 60-second CM7_1 snapshot window with odd/even publication'

Assert-Match $cameraAdapterHeader 'camera_seekfree_transport_begin\s*\(' 'camera transport begin API is missing'
Assert-Match $cameraAdapterHeader 'camera_seekfree_transport_frame_complete\s*\(' 'camera transport completion API is missing'
Assert-Match $cameraAdapter '(?s)camera_seekfree_transport_transfer\s*\([^)]*\).*?wifi_spi_send_buffer\s*\(.*?remaining' 'camera adapter does not retain the WiFi-SPI remaining-byte result'
Assert-Match $cameraAdapter '(?s)if\s*\(\s*\(1U\s*==\s*transport_diag\.segment_count\).*?transport_diag\.header_remaining.*?remaining\s*=\s*length\s*;\s*\}\s*else\s*\{\s*remaining\s*=\s*wifi_spi_send_buffer' 'camera adapter does not suppress the payload transfer after a partial header'
if(([Regex]::Matches($cameraAdapter, 'remaining\s*=\s*wifi_spi_send_buffer\s*\(')).Count -ne 1)
{
    Add-Failure 'camera adapter must have exactly one WiFi-SPI send choke point'
}
Assert-Match $cameraAdapter '(?s)segment_count\s*==\s*2U.*?header_remaining\s*==\s*0U.*?payload_remaining\s*==\s*0U' 'camera adapter completion does not require two complete Assistant segments'

$tryAttachMatch = [Regex]::Match($cameraConsumer, '(?s)static\s+uint8\s+camera_frame_consumer_try_attach\s*\([^)]*\)\s*\{(.*?)\n\}')
if((-not $tryAttachMatch.Success) -or ($tryAttachMatch.Groups[1].Value -match '(?i)wifi_spi_|wifi_state|socket_state|init_state'))
{
    Add-Failure 'camera epoch attach mutates WiFi/socket/init state or calls network code'
}
Assert-Match $cameraConsumerHeader '(?s)uint32\s+sent_count\s*;.*?uint32\s+reconnect_count\s*;.*?uint32\s+stale_count\s*;.*?uint32\s+last_sent_sequence\s*;.*?uint32\s+last_reconnect_ms\s*;.*?uint32\s+last_send_duration_ms\s*;.*?uint32\s+max_send_duration_ms\s*;.*?uint32\s+last_process_duration_us\s*;.*?uint32\s+max_process_duration_us\s*;.*?uint8\s+wifi_state\s*;.*?uint8\s+socket_state\s*;' 'consumer diagnostics do not expose WiFi/socket, reconnect, stale, send, and process state'

$actuatorMotor = Read-RepoFile 'project\code\actuator_motor.c'
Assert-Match $actuatorMotor '(?s)static\s+void\s+actuator_motor_send_duty\s*\(\s*int16\s+left_duty\s*,\s*int16\s+right_duty\s*\)\s*\{\s*#if\s+APP_CAMERA_DEBUG_ONLY\s+left_duty\s*=\s*0\s*;\s*right_duty\s*=\s*0\s*;\s*#endif.*?bldc_foc_uart_set_duty\s*\(\s*left_duty\s*,\s*right_duty\s*\)' 'actuator_motor_send_duty does not clamp both duties to zero before the BLDC choke point'

$boardAdc = Read-RepoFile 'project\code\board_adc.c'
Assert-Match $boardAdc '(?m)^\s*#include\s+"app_config\.h"' 'board_adc.c does not include app_config.h'
Assert-Match $boardAdc '(?s)#if\s+!APP_CAMERA_DEBUG_ONLY\s+adc_init\s*\(\s*BUS_PHASE_PORT.*?adc_mean_filter_convert\s*\(\s*BUS_PHASE_PORT.*?#endif' 'board_adc.c does not compile out BUS_PHASE_PORT initialization and warm-up reads'
Assert-Match $boardAdc '(?s)#if\s+!APP_CAMERA_DEBUG_ONLY\s+uint16\s+adc_value_bus_phase\s*=\s*0\s*;\s*#endif.*?#if\s+!APP_CAMERA_DEBUG_ONLY\s+adc_value_bus_phase\s*=\s*adc_convert\s*\(\s*BUS_PHASE_PORT.*?#endif' 'board_adc.c does not compile out the BUS_PHASE_PORT raw variable and conversion'
Assert-Match $boardAdc '(?s)adc_value_c_phase\s*=\s*adc_convert\s*\(\s*C_PHASE_PORT\s*\)\s*-\s*adc_information\.current_c_offset\s*;.*?#if\s+!APP_CAMERA_DEBUG_ONLY\s+adc_value_bus_phase\s*=\s*adc_convert\s*\(\s*BUS_PHASE_PORT\s*\)\s*-\s*adc_information\.voltage_bus_offset\s*;\s*.*?#endif\s+adc_value_v_reference\s*=\s*adc_mean_filter_convert\s*\(\s*V_REFERENCE\s*,\s*5\s*\)\s*;' 'non-camera BUS_PHASE conversion no longer precedes the V_REFERENCE mean sample'
Assert-Match $boardAdc '(?s)#if\s+APP_CAMERA_DEBUG_ONLY\s+adc_information\.voltage_bus\s*=\s*0\.0f\s*;\s*adc_information\.voltage_bus_filter\s*=\s*0\.0f\s*;\s*adc_information\.voltage_bus_filter_offset\s*=\s*0\.0f\s*;\s*#else.*?#endif' 'camera-debug bus current and filter are not forced to benign zero'

$cm7_0Main = Read-RepoFile 'project\user\main_cm7_0.c'
Assert-Match $cm7_0Main '(?s)app_result\s*=\s*app_init\s*\(\s*\)\s*;.*?if\s*\(\s*0U\s*!=\s*app_result\s*\).*?\}.*?camera_capture_producer_init\s*\(\s*\)' 'CM7_0 main does not initialize the camera producer after successful app_init'
Assert-Match $cm7_0Main '(?s)app_run_once\s*\(\s*\)\s*;\s*camera_capture_producer_service\s*\(\s*\)\s*;' 'CM7_0 main does not service the camera producer immediately after app_run_once'

$cm7Main = Read-RepoFile 'project\user\main_cm7_1.c'
Assert-Match $cm7Main 'pit_ms_init\s*\(\s*PIT_CH2\s*,\s*1\s*\)' 'CM7_1 main does not initialize PIT_CH2 at 1 ms'
Assert-Match $cm7Main 'camera_frame_consumer_init\s*\(\s*\)' 'CM7_1 main does not initialize the frame consumer'
Assert-Match $cm7Main '(?s)while\s*\(\s*true\s*\)\s*\{\s*camera_frame_consumer_service\s*\(\s*\)\s*;\s*\}' 'CM7_1 main loop does not exclusively service the frame consumer'
Assert-NoMatch $cm7Main '(?i)\b(?:mt9v03x_init|mt9v03x_finish_flag|wifi_spi_|seekfree_assistant_)\w*\s*\(' 'CM7_1 main owns camera capture, WiFi, or Assistant work'
Assert-NoMatch $cm7Main '(?i)uart_init\s*\(\s*UART_0' 'CM7_1 main introduces UART0 initialization'

$cm7Isr = Read-RepoFile 'project\user\cm7_1_isr.c'
Assert-Match $cm7Isr '(?s)void\s+pit0_ch2_isr\s*\(\s*\)\s*\{\s*pit_isr_flag_clear\s*\(\s*PIT_CH2\s*\)\s*;\s*camera_frame_consumer_tick_1ms\s*\(\s*\)\s*;\s*\}' 'PIT_CH2 ISR is not limited to flag clear plus the consumer 1 ms tick'
foreach ($pitChannel in 10..21)
{
    Assert-Match $cm7Isr "(?s)void\s+pit0_ch${pitChannel}_isr\s*\(\s*\)\s*\{\s*pit_isr_flag_clear\s*\(\s*PIT_CH${pitChannel}\s*\)\s*;\s*\}" "CM7_1 PIT_CH$pitChannel linker stub is missing or does more than clear its own flag"
}

$isrFiles = Get-ChildItem -Path (Join-Path $repoRoot 'project\user') -File -Filter '*_isr.c'
foreach ($isrFile in $isrFiles)
{
    $isrText = [System.IO.File]::ReadAllText($isrFile.FullName)
    Assert-NoMatch $isrText '(?i)\b(?:mt9v03x_init|memcpy|wifi_spi_|seekfree_assistant_|camera_capture_producer_service|camera_frame_consumer_service)\w*\s*\(' "camera copy, WiFi, Assistant, or deferred camera service appears in ISR context: $($isrFile.Name)"
}

$keepSymbolsPattern = '(?s)<name>IlinkKeepSymbols</name>\s*<state>mt9v03x_h_num</state>\s*<state>mt9v03x_w_num</state>\s*<state>mt9v03x_image_temp</state>'
Assert-Match $cm7_0Project '\\zf_device\\zf_device_mt9v03x\.c</name>' 'CM7_0 project does not contain zf_device_mt9v03x.c'
Assert-Match $cm7_0Project $keepSymbolsPattern 'CM7_0 linker does not retain all three fixed MT9V03X objects'
Assert-NoMatch $cm7Project '\\zf_device\\zf_device_mt9v03x\.c</name>' 'CM7_1 project still contains zf_device_mt9v03x.c'
Assert-NoMatch $cm7Project $keepSymbolsPattern 'CM7_1 linker still retains fixed MT9V03X objects'
Assert-Match $cm7Project '\\zf_device\\zf_device_mt9v03x\.h</name>' 'CM7_1 project does not retain the MT9V03X dimension header'

foreach ($cameraProjectFile in @('camera_capture_producer.c', 'camera_capture_producer.h', 'camera_debug_config.h'))
{
    Assert-Match $cm7_0Project ([Regex]::Escape("\code\$cameraProjectFile</name>")) "CM7_0 project does not contain $cameraProjectFile"
}
foreach ($cameraProjectFile in @('camera_frame_consumer.c', 'camera_frame_consumer.h', 'camera_debug_config.h'))
{
    Assert-Match $cm7Project ([Regex]::Escape("\code\$cameraProjectFile</name>")) "CM7_1 project does not contain $cameraProjectFile"
}
Assert-NoMatch $cm7_0Project '\\code\\camera_debug_app\.[ch]</name>' 'CM7_0 project still contains the deleted camera_debug_app'
Assert-NoMatch $cm7Project '\\code\\camera_debug_app\.[ch]</name>' 'CM7_1 project still contains the deleted camera_debug_app'

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

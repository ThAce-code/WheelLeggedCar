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

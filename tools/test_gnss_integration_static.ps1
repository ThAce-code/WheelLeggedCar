$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$sensorPath = Join-Path $repoRoot 'project\code\sensor_gnss.c'
$isrPath = Join-Path $repoRoot 'project\user\cm7_1_isr.c'
$failures = [System.Collections.Generic.List[string]]::new()

function Assert-True([bool]$Condition, [string]$Message)
{
    if(-not $Condition)
    {
        $script:failures.Add($Message)
    }
}

Assert-True (Test-Path -LiteralPath $sensorPath) 'sensor_gnss.c is missing'
if(Test-Path -LiteralPath $sensorPath)
{
    $sensor = Get-Content -Raw -LiteralPath $sensorPath
    Assert-True ($sensor -match 'gnss_init\(TAU1201\)') 'TAU1201 is not initialized'
    Assert-True ($sensor -match 'gnss_data_parse\(\)') 'GNSS parsing is not serviced'
    Assert-True ($sensor -notmatch 'UART_0|debug_info_init') 'sensor_gnss claims UART0'
}

$isr = Get-Content -Raw -LiteralPath $isrPath
Assert-True ($isr -match 'gnss_uart_callback\(\)') 'UART2 ISR no longer feeds GNSS bytes'
Assert-True ($isr -notmatch 'gnss_data_parse\(\)') 'GNSS parsing incorrectly runs in ISR'

$main = Get-Content -Raw -LiteralPath (Join-Path $repoRoot 'project\user\main_cm7_1.c')
$ewp = Get-Content -Raw -LiteralPath (Join-Path $repoRoot 'project\iar\project_config\cyt4bb7_cm_7_1.ewp')
Assert-True ($main -match 'sensor_gnss_service') 'CM7_1 main loop does not service GNSS'
Assert-True ($main -match 'intercore_transport_cm7_1_attach') 'CM7_1 never attaches the GNSS publisher'
Assert-True ($main -match 'gnss_transport_attached\s*=\s*0U') 'GNSS publisher has no retry state'
Assert-True ($ewp -match 'code\\sensor_gnss\.c') 'CM7_1 IAR project omits sensor_gnss.c'
Assert-True ($ewp -match 'code\\local_position\.c') 'CM7_1 IAR project omits local_position.c'

$telemetry = Get-Content -Raw -LiteralPath (Join-Path $repoRoot 'project\code\telemetry.c')
$config = Get-Content -Raw -LiteralPath (Join-Path $repoRoot 'project\code\app_config.h')
Assert-True ($config -match 'APP_TELEMETRY_PROFILE_GNSS') 'GNSS telemetry profile is undefined'
Assert-True ($config -match 'APP_TELEMETRY_PERIOD_MS\s+\(20U\)') 'GNSS telemetry is not 50 Hz'
Assert-True ($telemetry -match 'static\s+float\s+vofa_data\[20\]') 'GNSS profile is not fixed at 20 floats'
Assert-True ($telemetry -match 'gps_age_ms') 'GNSS profile omits data age'
Assert-True ($telemetry -notmatch 'GNGGA|GNRMC|\$GN') 'raw NMEA appears in JustFloat telemetry'

if(0 -ne $failures.Count)
{
    $failures | ForEach-Object { "FAIL: $_" }
    exit 1
}

'PASS: GNSS integration contract'

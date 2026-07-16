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

if(0 -ne $failures.Count)
{
    $failures | ForEach-Object { "FAIL: $_" }
    exit 1
}

'PASS: GNSS integration contract'

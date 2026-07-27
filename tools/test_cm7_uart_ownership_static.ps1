$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$cm71Path = Join-Path $repoRoot 'project\user\cm7_1_isr.c'
$cm70Path = Join-Path $repoRoot 'project\user\cm7_0_isr.c'
$ewpPath = Join-Path $repoRoot 'project\iar\project_config\cyt4bb7_cm_7_1.ewp'
$uartDriverPath = Join-Path $repoRoot 'libraries\zf_driver\zf_driver_uart.c'

$cm71 = Get-Content -Raw -LiteralPath $cm71Path
$cm70 = Get-Content -Raw -LiteralPath $cm70Path
$uartDriver = Get-Content -Raw -LiteralPath $uartDriverPath
$failures = @()

if ($cm71 -match '(?m)^\s*void\s+uart0_isr\s*\(') {
    $failures += 'CM7_1 ISR source still defines uart0_isr.'
}
if ($cm71 -match '\buart0_isr\b|\bUART_0\b') {
    $failures += 'CM7_1 ISR source still references UART0.'
}
foreach ($handler in @('uart1_isr', 'uart2_isr', 'uart3_isr', 'uart4_isr')) {
    if ($cm71 -notmatch "(?m)^\s*void\s+$handler\s*\(") {
        $failures += "CM7_1 ISR source lost required $handler."
    }
}
if ($cm70 -notmatch '(?m)^\s*void\s+uart0_isr\s*\(' -or $cm70 -notmatch '\bUART_0\b') {
    $failures += 'CM7_0 ISR source no longer owns UART0.'
}

[xml]$ewp = Get-Content -Raw -LiteralPath $ewpPath
$projectFiles = @($ewp.SelectNodes('//file/name') | ForEach-Object { $_.InnerText })
if (-not ($projectFiles | Where-Object { $_ -match 'cm7_1_isr\.c$' })) {
    $failures += 'CM7_1 EWP no longer includes cm7_1_isr.c.'
}
if ($projectFiles | Where-Object { $_ -match 'uart0_isr|UART_0' }) {
    $failures += 'CM7_1 EWP contains an explicit UART0 handler/vector reference.'
}

$cm71DispatchPattern = '(?s)#if\s+CY_CORE_CM7_1\s*void\s*\(\*uart_isr_func\[7\]\)\(\)\s*=\s*\{\s*0\s*,\s*uart1_isr\s*,\s*uart2_isr\s*,\s*uart3_isr\s*,\s*uart4_isr\s*,\s*0\s*,\s*0\s*\}\s*;'
if ($uartDriver -notmatch $cm71DispatchPattern) {
    $failures += 'UART driver lacks the exact CM7_1-owned ISR dispatch table.'
}

$nullGuardPattern = 'if\s*\(\s*0\s*==\s*uart_isr_func\[uart_n\]\s*\)\s*\{\s*zf_assert\(0U\);\s*return;\s*\}'
if ([regex]::Matches($uartDriver, $nullGuardPattern, 'Singleline').Count -ne 2) {
    $failures += 'UART driver must guard both interrupt registrations against null handlers.'
}
if ($uartDriver -notmatch "(?s)void\s+uart_sbus_init\s*\([^)]*\).*?$nullGuardPattern.*?Cy_SysInt_InitIRQ") {
    $failures += 'uart_sbus_init does not reject an unsupported core UART before registration.'
}
if ($uartDriver -notmatch "(?s)void\s+uart_init\s*\([^)]*\).*?$nullGuardPattern.*?interrupt_init") {
    $failures += 'uart_init does not reject an unsupported core UART before registration.'
}

$sbusStart = $uartDriver.IndexOf('void uart_sbus_init')
$uartInitStart = $uartDriver.IndexOf('void uart_init')
$sbusBody = $uartDriver.Substring($sbusStart, $uartInitStart - $sbusStart)
$uartInitBody = $uartDriver.Substring($uartInitStart)
foreach ($functionBody in @($sbusBody, $uartInitBody)) {
    $guardMatch = [regex]::Match($functionBody, $nullGuardPattern, 'Singleline')
    $configIndex = $functionBody.IndexOf('get_uart_config')
    if (-not $guardMatch.Success -or $configIndex -lt 0 -or $guardMatch.Index -gt $configIndex) {
        $failures += 'Unsupported UART guard must run before UART hardware setup.'
    }
}

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Output 'CM7 UART ownership static checks passed.'

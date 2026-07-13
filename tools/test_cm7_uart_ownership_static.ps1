$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$cm71Path = Join-Path $repoRoot 'project\user\cm7_1_isr.c'
$cm70Path = Join-Path $repoRoot 'project\user\cm7_0_isr.c'
$ewpPath = Join-Path $repoRoot 'project\iar\project_config\cyt4bb7_cm_7_1.ewp'

$cm71 = Get-Content -Raw -LiteralPath $cm71Path
$cm70 = Get-Content -Raw -LiteralPath $cm70Path
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

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Output 'CM7 UART ownership static checks passed.'

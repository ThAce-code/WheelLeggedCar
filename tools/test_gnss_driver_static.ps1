$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$header = Get-Content -Raw -LiteralPath (Join-Path $repoRoot 'libraries\zf_device\zf_device_gnss.h')
$source = Get-Content -Raw -LiteralPath (Join-Path $repoRoot 'libraries\zf_device\zf_device_gnss.c')
$failures = [System.Collections.Generic.List[string]]::new()

function Assert-Match([string]$Text, [string]$Pattern, [string]$Message)
{
    if($Text -notmatch $Pattern)
    {
        $script:failures.Add($Message)
    }
}

Assert-Match $header 'uint8\s+fix_quality\s*;' 'gnss_info_struct lacks GGA fix_quality'
Assert-Match $header 'float\s+hdop\s*;' 'gnss_info_struct lacks GGA hdop'
Assert-Match $header 'extern\s+volatile\s+uint8\s+gnss_flag\s*;' 'gnss_flag is not volatile across ISR/main'
Assert-Match $source 'get_parameter_index\(6,\s*buf\)' 'GGA field 6 Fix Quality is not parsed'
Assert-Match $source 'get_parameter_index\(8,\s*buf\)' 'GGA field 8 HDOP is not parsed'
Assert-Match $source 'memset\(&gnss,\s*0,\s*sizeof\(gnss\)\)' 'gnss public state is not reset at init'
Assert-Match $source 'return_state\s*=\s*1U?\s*;\s*\}\s*else\s*\{\s*gps_gnrmc_parse' 'bad RMC checksum can leave parsing stuck'
Assert-Match $source 'return_state\s*=\s*1U?\s*;\s*\}\s*else\s*\{\s*gps_gngga_parse' 'bad GGA checksum can leave parsing stuck'

if(0 -ne $failures.Count)
{
    $failures | ForEach-Object { Write-Host "FAIL: $_" -ForegroundColor Red }
    exit 1
}

Write-Host 'PASS: TAU1201 GGA quality parser contract'

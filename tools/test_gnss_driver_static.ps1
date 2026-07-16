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

function Assert-NotMatch([string]$Text, [string]$Pattern, [string]$Message)
{
    if($Text -match $Pattern)
    {
        $script:failures.Add($Message)
    }
}

function Get-CFunction([string]$Text, [string]$Name)
{
    $signature = [regex]::Match($Text, "(?m)^\s*(?:static\s+)?(?:uint8|uint32|void)\s+$Name\s*\([^)]*\)\s*\{")
    if(-not $signature.Success)
    {
        $script:failures.Add("source lacks $Name function")
        return ''
    }

    $depth = 0
    for($index = $signature.Index; $index -lt $Text.Length; $index++)
    {
        if('{' -eq $Text[$index])
        {
            $depth++
        }
        elseif('}' -eq $Text[$index])
        {
            $depth--
            if(0 -eq $depth)
            {
                return $Text.Substring($signature.Index, $index - $signature.Index + 1)
            }
        }
    }

    $script:failures.Add("source has unterminated $Name function")
    return ''
}

$ggaParser = Get-CFunction $source 'gps_gngga_parse'
$dataParser = Get-CFunction $source 'gnss_data_parse'
$sequenceHelper = Get-CFunction $source 'gnss_next_sequence'
$checksumHelper = Get-CFunction $source 'gnss_sentence_checksum_valid'
$uartCallback = Get-CFunction $source 'gnss_uart_callback'

Assert-Match $header 'uint8\s+fix_quality\s*;' 'gnss_info_struct lacks GGA fix_quality'
Assert-Match $header 'float\s+hdop\s*;' 'gnss_info_struct lacks GGA hdop'
Assert-Match $header 'uint32\s+rmc_sequence\s*;' 'gnss_info_struct lacks the RMC parse sequence'
Assert-Match $header 'uint32\s+gga_sequence\s*;' 'gnss_info_struct lacks the GGA parse sequence'
Assert-Match $header 'uint32\s+rmc_utc_ms\s*;' 'gnss_info_struct lacks the RMC UTC epoch key'
Assert-Match $header 'uint32\s+gga_utc_ms\s*;' 'gnss_info_struct lacks the GGA UTC epoch key'
Assert-Match $header 'extern\s+volatile\s+uint8\s+gnss_flag\s*;' 'gnss_flag is not volatile across ISR/main'
Assert-Match $ggaParser 'fix_quality\s*=\s*\(uint8\)get_int_number\(&buf\[get_parameter_index\(6,\s*buf\)\]\)\s*;' 'GGA quality fields are not parsed unconditionally before latitude-dependent logic'
Assert-Match $ggaParser 'gnss->fix_quality\s*=\s*fix_quality\s*;' 'GGA field 6 Fix Quality is not assigned by gps_gngga_parse'
Assert-Match $ggaParser 'gnss->satellite_used\s*=\s*\(uint8\)get_int_number\(\s*&buf\[get_parameter_index\(7,\s*buf\)\]\)\s*;' 'GGA field 7 satellite count is not assigned by gps_gngga_parse'
Assert-Match $ggaParser 'gnss->hdop\s*=\s*get_float_number\(&buf\[get_parameter_index\(8,\s*buf\)\]\)\s*;' 'GGA field 8 HDOP is not assigned by gps_gngga_parse'
Assert-NotMatch $ggaParser 'get_parameter_index\(2,\s*buf\)' 'GGA latitude gate can retain stale quality fields on an empty-latitude no-fix sentence'
Assert-Match $ggaParser 'gnss->gga_utc_ms\s*=\s*utc_ms\s*;' 'GGA parser does not retain its UTC epoch key'
Assert-Match $ggaParser 'return\s+1U\s*;' 'checksum-valid no-fix GGA is incorrectly treated as a framing error'
Assert-Match $source 'memset\(&gnss,\s*0,\s*sizeof\(gnss\)\)' 'gnss public state is not reset at init'
Assert-Match $checksumHelper 'for\s*\([^;]+;\s*index\s*<\s*length\s*;' 'checksum validation is not length bounded'
Assert-Match $checksumHelper 'star_index\s*\+\s*2U\s*>=\s*length' 'checksum validation does not reject short checksum fields'
Assert-Match $dataParser 'gnss_sentence_checksum_valid\(gps_rmc_buffer,\s*gps_rmc_length,\s*gps_rmc_truncated\)' 'RMC checksum does not use captured length/truncation state'
Assert-Match $dataParser 'else\s+if\(0U\s*==\s*gps_gngga_parse\(\(char\s*\*\)gps_gga_buffer,\s*&gnss\)\)' 'checksum-valid GGA is not wired to gps_gngga_parse with the GGA buffer and public state'
Assert-Match $sequenceHelper 'sequence\+\+\s*;\s*return\s+\(0U\s*!=\s*sequence\)\s*\?\s*sequence\s*:\s*1U\s*;' 'GNSS parse sequence does not skip zero on wrap'
Assert-Match $dataParser 'gps_gnrmc_parse[\s\S]+?else\s*\{\s*gnss\.rmc_sequence\s*=\s*gnss_next_sequence\(gnss\.rmc_sequence\)' 'format-valid RMC does not advance its parse sequence'
Assert-Match $dataParser 'gps_gngga_parse[\s\S]+?else\s*\{\s*gnss\.gga_sequence\s*=\s*gnss_next_sequence\(gnss\.gga_sequence\)' 'format-valid GGA does not advance its parse sequence'
Assert-Match $uartCallback 'gps_rmc_length\s*=\s*temp_length\s*;[\s\S]+?gps_rmc_buffer\[gps_rmc_length\]\s*=\s*0U\s*;' 'RMC capture is not length tracked and NUL terminated'
Assert-Match $uartCallback 'gps_gga_length\s*=\s*temp_length\s*;[\s\S]+?gps_gga_buffer\[gps_gga_length\]\s*=\s*0U\s*;' 'GGA capture is not length tracked and NUL terminated'
Assert-Match $uartCallback 'gps_ths_length\s*=\s*temp_length\s*;[\s\S]+?gps_ths_buffer\[gps_ths_length\]\s*=\s*0U\s*;' 'THS capture is not length tracked and NUL terminated'
Assert-NotMatch $dataParser 'strchr\([^;]+\x27\*\x27\)' 'checksum framing still dereferences an unbounded strchr result'

if(0 -ne $failures.Count)
{
    $failures | ForEach-Object { Write-Host "FAIL: $_" -ForegroundColor Red }
    exit 1
}

Write-Host 'PASS: TAU1201 GGA quality parser contract'

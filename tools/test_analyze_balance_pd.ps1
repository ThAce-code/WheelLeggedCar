$ErrorActionPreference = "Stop"

$scriptPath = Join-Path $PSScriptRoot "analyze_balance_pd.m"
if(-not (Test-Path -LiteralPath $scriptPath)) {
    throw "Missing tools/analyze_balance_pd.m"
}

$text = Get-Content -LiteralPath $scriptPath -Raw

function Assert-Contains {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )

    if($Text -notmatch $Pattern) {
        throw $Message
    }
}

function Assert-NotContains {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )

    if($Text -match $Pattern) {
        throw $Message
    }
}

Assert-NotContains $text '[A-Za-z]:\\' "script must not contain Windows absolute paths"
Assert-Contains $text 'fullfile\("data"' "script should use relative data/ paths"
Assert-Contains $text 'balance_capture_\*\.csv' "script should read balance capture CSV files"
Assert-Contains $text 'unique\(notes' "script should group by note"
Assert-Contains $text 'max\(\[files\(idx\)\.datenum\]\)' "script should keep the newest capture per note"
Assert-NotContains $text '\.folder' "script should not use dir().folder absolute paths for data files"
Assert-Contains $text 'balance_pd_summary\.csv' "script should write a summary CSV"
Assert-Contains $text 'balance_pd_timeseries\.png' "script should save a time-series figure"
Assert-Contains $text 'balance_pd_score\.png' "script should save a score figure"
Assert-Contains $text 'balance_mode' "script should filter balance test mode"
Assert-Contains $text 'feedback_online' "script should check feedback quality"
Assert-Contains $text 'pitch_abs_p95' "script should compute pitch quality metric"
Assert-Contains $text 'balance_rpm_abs_p95' "script should compute output effort metric"

Write-Host "analyze_balance_pd static checks passed"

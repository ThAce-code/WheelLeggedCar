$ErrorActionPreference = "Stop"

$scriptPath = Join-Path $PSScriptRoot "identify_balance_model.m"
if(-not (Test-Path -LiteralPath $scriptPath)) {
    throw "Missing tools/identify_balance_model.m"
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
Assert-Contains $text 'max\(\[files\(idx\)\.datenum\]\)' "script should keep newest capture per note"
Assert-Contains $text 'pinv\(phi\)' "script should identify model with least squares"
Assert-Contains $text 'local_dlqr' "script should solve LQR locally"
Assert-NotContains $text '\bdlqr\(' "script should not require Control System Toolbox dlqr"
Assert-Contains $text 'balance_model_summary\.csv' "script should save model summary"
Assert-Contains $text 'balance_model_fit\.png' "script should save fit figure"
Assert-Contains $text 'Best candidate: BL' "script should print BL command"
Assert-Contains $text 'LQR candidate sweep' "script should sweep Q matrices"
Assert-Contains $text 'qRateList' "script should try multiple rate penalties"
Assert-Contains $text 'isfinite' "script should filter non-finite rows"

Write-Host "identify_balance_model static checks passed"

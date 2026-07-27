$ErrorActionPreference = "Stop"

$csv = Join-Path $env:TEMP ("ik-fit-physical-" + [Guid]::NewGuid().ToString() + ".csv")
@'
sample_id,label,cmd_a0_deg,cmd_a1_deg,cmd_a2_deg,cmd_a3_deg,measured_x_mm,measured_y_mm,note
0,ref_start,90,90,90,90,-21.45,48.09,""
1,differential_80_100,80,90,100,90,-20.72,38.79,""
2,differential_70_110,70,90,110,90,-20.38,32.17,""
3,ref_mid,90,90,90,90,-20.41,46.59,""
4,differential_100_80,100,90,80,90,-21.03,59.07,""
5,differential_110_70,110,90,70,90,-21.40,73.27,""
6,differential_120_60,120,90,60,90,-22.03,88.49,""
7,asymmetric_120_80,120,90,80,90,-31.42,74.12,""
8,asymmetric_120_100,120,90,100,90,-37.94,59.34,""
9,asymmetric_120_110,120,90,110,90,-39.58,53.01,""
10,common_120,120,90,120,90,-40.62,47.37,""
11,asymmetric_110_120,110,90,120,90,-35.96,43.96,""
12,asymmetric_100_120,100,90,120,90,-30.91,39.63,""
13,asymmetric_100_110,100,90,110,90,-29.63,43.06,""
14,common_110,110,90,110,90,-33.08,46.70,""
15,asymmetric_110_100,110,90,100,90,-31.53,52.01,""
16,common_100,100,90,100,90,-26.82,47.21,""
17,asymmetric_90_100,90,90,100,90,-23.86,43.26,""
18,common_80,80,90,80,90,-15.04,47.60,""
19,ref_end,90,90,90,90,-20.44,47.39,""
'@ | Set-Content $csv -Encoding UTF8

try {
    $output = python tools\fit_leg_ik_calibration.py --input $csv --kfold 5 2>&1
    if(0 -ne $LASTEXITCODE) {
        $output | Write-Host
        throw "physical-coordinate fit script failed"
    }
    $text = $output -join "`n"
    foreach($required in @(
        "reference=(-20.766667,47.356667) mm",
        "directions=(-1,-1)",
        "determinant=-1",
        "candidate_leg_physical_calibration",
        ".physical_reference_x_mm = -20.766667f",
        ".physical_workspace_vertex_count = 8U"
    )) {
        if($text -notmatch [regex]::Escape($required)) {
            throw "fit output missing: $required"
        }
    }
    if($text -match "offset-only") {
        throw "fit script must not emit the obsolete offset-only candidate"
    }

    $negativeCsv = Join-Path $env:TEMP ("ik-fit-negative-axis-" + [Guid]::NewGuid().ToString() + ".csv")
    (Get-Content $csv -Raw) -replace ',([0-9]+\.[0-9]+),""', ',-$1,""' |
        Set-Content $negativeCsv -Encoding UTF8
    $negativeOutput = python tools\fit_leg_ik_calibration.py --input $negativeCsv --no-split 2>&1
    if(0 -eq $LASTEXITCODE -or ($negativeOutput -join "`n") -notmatch "coordinate convention") {
        $negativeOutput | Write-Host
        throw "fit script must reject reversed Y coordinates"
    }

    $missingRefCsv = Join-Path $env:TEMP ("ik-fit-missing-ref-" + [Guid]::NewGuid().ToString() + ".csv")
    (Get-Content $csv -Raw).Replace('ref_mid', 'not_a_reference') |
        Set-Content $missingRefCsv -Encoding UTF8
    $missingRefOutput = python tools\fit_leg_ik_calibration.py --input $missingRefCsv --no-split 2>&1
    if(0 -eq $LASTEXITCODE -or ($missingRefOutput -join "`n") -notmatch "ref_start, ref_mid, and ref_end") {
        $missingRefOutput | Write-Host
        throw "fit script must require all three named references"
    }
}
finally {
    Remove-Item $csv -Force -ErrorAction SilentlyContinue
    Remove-Item $negativeCsv -Force -ErrorAction SilentlyContinue
    Remove-Item $missingRefCsv -Force -ErrorAction SilentlyContinue
}

Write-Host "physical-coordinate fit calibration smoke test passed"

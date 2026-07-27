$ErrorActionPreference = "Stop"

$csv = Join-Path $env:TEMP ("ik-fit-physical-" + [Guid]::NewGuid().ToString() + ".csv")
$negativeCsv = $null
$missingRefCsv = $null
@'
sample_id,label,cmd_a0_deg,cmd_a1_deg,cmd_a2_deg,cmd_a3_deg,servo0_output_deg,servo1_output_deg,servo2_output_deg,servo3_output_deg,ik_valid,leg_mode,legacy_stance_ref_units,legacy_stance_rate_units_s,ik_margin,drive_forward_limit_rpm,motion_state,fault_reason,drive_allowed,measured_x_mm,measured_y_mm,note
0,ref_start,90,90,90,90,90,90,90,90,0,0,0,0,0,0,0,0,0,-21.45,48.09,""
1,differential_80_100,80,90,100,90,80,90,100,90,0,0,0,0,0,0,0,0,0,-20.72,38.79,""
2,differential_70_110,70,90,110,90,70,90,110,90,0,0,0,0,0,0,0,0,0,-20.38,32.17,""
3,ref_mid,90,90,90,90,90,90,90,90,0,0,0,0,0,0,0,0,0,-20.41,46.59,""
4,differential_100_80,100,90,80,90,100,90,80,90,0,0,0,0,0,0,0,0,0,-21.03,59.07,""
5,differential_110_70,110,90,70,90,110,90,70,90,0,0,0,0,0,0,0,0,0,-21.40,73.27,""
6,differential_120_60,120,90,60,90,120,90,60,90,0,0,0,0,0,0,0,0,0,-22.03,88.49,""
7,asymmetric_120_80,120,90,80,90,120,90,80,90,0,0,0,0,0,0,0,0,0,-31.42,74.12,""
8,asymmetric_120_100,120,90,100,90,120,90,100,90,0,0,0,0,0,0,0,0,0,-37.94,59.34,""
9,asymmetric_120_110,120,90,110,90,120,90,110,90,0,0,0,0,0,0,0,0,0,-39.58,53.01,""
10,common_120,120,90,120,90,120,90,120,90,0,0,0,0,0,0,0,0,0,-40.62,47.37,""
11,asymmetric_110_120,110,90,120,90,110,90,120,90,0,0,0,0,0,0,0,0,0,-35.96,43.96,""
12,asymmetric_100_120,100,90,120,90,100,90,120,90,0,0,0,0,0,0,0,0,0,-30.91,39.63,""
13,asymmetric_100_110,100,90,110,90,100,90,110,90,0,0,0,0,0,0,0,0,0,-29.63,43.06,""
14,common_110,110,90,110,90,110,90,110,90,0,0,0,0,0,0,0,0,0,-33.08,46.70,""
15,asymmetric_110_100,110,90,100,90,110,90,100,90,0,0,0,0,0,0,0,0,0,-31.53,52.01,""
16,common_100,100,90,100,90,100,90,100,90,0,0,0,0,0,0,0,0,0,-26.82,47.21,""
17,asymmetric_90_100,90,90,100,90,90,90,100,90,0,0,0,0,0,0,0,0,0,-23.86,43.26,""
18,common_80,80,90,80,90,80,90,80,90,0,0,0,0,0,0,0,0,0,-15.04,47.60,""
19,ref_end,90,90,90,90,90,90,90,90,0,0,0,0,0,0,0,0,0,-20.44,47.39,""
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
        "servo[0]: neutral_deg=90.000000 ik_offset_deg=0.000000 direction=-1.000000",
        "servo[2]: neutral_deg=90.000000 ik_offset_deg=0.000000 direction=-1.000000",
        "candidate_leg_physical_calibration",
        ".physical_reference_x_mm = -20.766667f",
        ".physical_reference_y_mm = 47.356667f",
        ".alpha_reference_deg = 170.536799f",
        ".beta_reference_deg = -4.081158f",
        ".model_reference_x_mm = 22.830129f",
        ".model_reference_y_mm = 46.929213f",
        ".model_to_physical_scale = 0.955219899f",
        ".model_to_physical_m00 = -0.996313812f",
        ".model_to_physical_m01 = 0.085783378f",
        ".model_to_physical_m10 = 0.085783378f",
        ".model_to_physical_m11 = 0.996313812f",
        "physical_workspace vertex_count=8 (LEG_PHYSICAL_WORKSPACE_VERTEX_COUNT)",
        ".physical_workspace = {",
        "{-40.620f, 47.370f}",
        "{-20.380f, 32.170f}",
        "{-22.030f, 88.490f}",
        "{-39.580f, 53.010f}",
        ".physical_workspace_inset_mm = 2.000f"
    )) {
        if(-not $text.Contains($required)) {
            throw "fit output missing: $required"
        }
    }
    foreach($obsolete in @(
        "x_offset_mm",
        "y_offset_mm",
        ".x_min_mm",
        ".x_max_mm",
        ".y_min_mm",
        ".y_max_mm",
        ".command_direction_a",
        ".command_direction_b",
        ".physical_workspace_vertex_count"
    )) {
        if($text.Contains($obsolete)) {
            throw "fit output contains obsolete or nonexistent candidate field: $obsolete"
        }
    }

    @'
import importlib.util
import sys
from pathlib import Path

path = Path("tools/fit_leg_ik_calibration.py")
spec = importlib.util.spec_from_file_location("fit_leg_ik_calibration", path)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)

servo = module.ServoInputConfig(
    servo_index=0,
    neutral_deg=92.0,
    min_deg=10.0,
    max_deg=175.0,
    direction=-1.0,
    ik_offset_deg=2.0,
)
assert abs(module.logical_command_delta_deg(94.0, servo)) < 1e-12
assert abs(module.logical_command_delta_deg(84.0, servo) - 10.0) < 1e-12
'@ | python -
    if(0 -ne $LASTEXITCODE) {
        throw "logical command reconstruction must use neutral, offset, and direction"
    }

    $source = Get-Content tools\fit_leg_ik_calibration.py -Raw
    foreach($obsolete in @(
        "x_offset_mm",
        "y_offset_mm",
        "command_direction_a",
        "command_direction_b"
    )) {
        if($source.Contains($obsolete)) {
            throw "fit source retains obsolete calibration contract: $obsolete"
        }
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
    if($null -ne $negativeCsv) {
        Remove-Item $negativeCsv -Force -ErrorAction SilentlyContinue
    }
    if($null -ne $missingRefCsv) {
        Remove-Item $missingRefCsv -Force -ErrorAction SilentlyContinue
    }
}

Write-Host "physical-coordinate fit calibration smoke test passed"

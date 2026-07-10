$ErrorActionPreference = "Stop"

$csv = Join-Path $env:TEMP ("ik-fit-sample-" + [Guid]::NewGuid().ToString() + ".csv")
@'
sample_id,label,cmd_a0_deg,cmd_a1_deg,cmd_a2_deg,cmd_a3_deg,servo0_output_deg,servo1_output_deg,servo2_output_deg,servo3_output_deg,ik_valid,leg_mode,leg_height_ref_mm,leg_height_rate_mm_s,ik_margin,drive_forward_limit_rpm,motion_state,fault_reason,drive_allowed,measured_x_mm,measured_y_mm,note
0,center_90,90,90,90,90,90,90,90,90,0,0,0,0,0,0,0,0,0,19,55,""
1,all_85,85,85,85,85,85,85,85,85,0,0,0,0,0,0,0,0,0,16,50,""
2,all_95,95,95,95,95,95,95,95,95,0,0,0,0,0,0,0,0,0,22,60,""
'@ | Set-Content $csv -Encoding UTF8

$output = python tools\fit_leg_ik_calibration.py --input $csv --max-iter 10 2>&1
if(0 -ne $LASTEXITCODE) {
    $output | Write-Host
    throw "fit script failed"
}
if(($output -join "`n") -notmatch "usable_rows=3") {
    throw "fit script must report usable row count"
}
if(($output -join "`n") -notmatch "rmse_y_mm=") {
    throw "fit script must report y RMSE"
}
if(($output -join "`n") -notmatch "candidate_leg_config") {
    throw "fit script must print candidate config block"
}
Remove-Item $csv -Force
Write-Host "fit calibration smoke test passed"

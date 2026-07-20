$ErrorActionPreference = "Stop"

function Assert-True {
    param([bool]$Condition, [string]$Message)
    if(-not $Condition) {
        throw $Message
    }
}

$scriptPath = "tools/fit_balance_lqr_model.m"
Assert-True (Test-Path $scriptPath) "Missing balance LQR MATLAB fitter."
$source = Get-Content $scriptPath -Raw

Assert-True ($source -match 'balance_lqr_id_\*\.csv') "Wrong balance input glob."
Assert-True ($source -notmatch 'balance_capture_\*\.csv') "Legacy balance captures must not be read."
foreach($column in @('elapsed_s','time_ms','balance_mode','feedback_online','pitch_deg','pitch_rate_dps','balance_rpm','left_motor_rpm','right_motor_rpm','firmware_frame_sequence','imu_age_ms','balance_output_limit_rpm','last_command')) {
    Assert-True ($source -match [regex]::Escape($column)) "Missing required column: $column"
}
Assert-True ($source -match 'balance_mode\s*==\s*2') "Active balance filter missing."
Assert-True ($source -match 'feedback_online\s*>=\s*0\.5') "Feedback health filter missing."
Assert-True ($source -match 'imu_age_ms\s*<=\s*15') "IMU age filter missing."
Assert-True ($source -match 'abs\(.*pitch_deg.*\).*<=\s*15') "Pitch safety filter missing."
Assert-True ($source -match 'firmware_frame_sequence') "Sequence-gap boundary check missing."
Assert-True ($source -match 'wheel_position_rev') "Wheel-position state missing."
Assert-True ($source -match 'cumsum') "Wheel-position integration missing."
Assert-True ($source -match 'rank\(phi\).*5|rankPhi.*5') "Rank-5 gate missing."
Assert-True ($source -match 'cond\(') "Condition diagnostic missing."
Assert-True ($source -match 'validation_rmse') "Validation RMSE missing."
Assert-True ($source -match 'multi_step') "Multi-step validation missing."
Assert-True ($source -match 'eig\(') "Eigenvalue diagnostic missing."
Assert-True ($source -match 'dlqr') "dlqr cross-check missing."
Assert-True ($source -match 'local_dare') "Local DARE fallback missing."
foreach($output in @('balance_lqr_ab\.mat','balance_lqr_model_quality\.csv','balance_lqr_candidates\.csv','balance_lqr_fit\.png')) {
    Assert-True ($source -match $output) "Missing output: $output"
}
Assert-True ($source -notmatch 'app_config\.h') "Fitter must not edit firmware."
Assert-True ($source -notmatch 'Set-Content') "Fitter must not edit source files."

Write-Host "balance LQR MATLAB contract checks passed"

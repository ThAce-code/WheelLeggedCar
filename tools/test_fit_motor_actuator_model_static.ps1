$ErrorActionPreference = "Stop"

function Assert-True {
    param([bool]$Condition, [string]$Message)
    if(-not $Condition) {
        throw $Message
    }
}

$scriptPath = "tools/fit_motor_actuator_model.m"
Assert-True (Test-Path $scriptPath) "Missing motor actuator MATLAB fitter."
$source = Get-Content $scriptPath -Raw

Assert-True ($source -match 'motor_ident_\*\.csv') "Wrong input glob."
foreach($column in @('elapsed_s','expected_mode','expected_open_duty','feedback_online','left_motor_rpm','right_motor_rpm','left_duty','right_duty')) {
    Assert-True ($source -match [regex]::Escape($column)) "Missing required column: $column"
}
Assert-True ($source -match 'phi\s*=\s*\[rpm\(1:end-1\).*duty\(1:end-1\).*sign\(duty\(1:end-1\)\)') "Signed regressor missing."
Assert-True ($source -match 'rank\(phi\)') "Rank diagnostic missing."
Assert-True ($source -match 'cond\(') "Condition diagnostic missing."
Assert-True ($source -match 'validation') "Validation split missing."
Assert-True ($source -match 'positive.*residual|residual.*positive') "Positive residual diagnostic missing."
Assert-True ($source -match 'negative.*residual|residual.*negative') "Negative residual diagnostic missing."
Assert-True ($source -match 'iddata') "iddata cross-check missing."
Assert-True ($source -match 'tfest') "tfest cross-check missing."
Assert-True ($source -match 'motor_actuator_model\.csv') "CSV output missing."
Assert-True ($source -match 'motor_actuator_model\.mat') "MAT output missing."
Assert-True ($source -match 'motor_actuator_model_fit\.png') "Plot output missing."
Assert-True ($source -notmatch 'app_config\.h') "Fitter must not edit firmware."
Assert-True ($source -notmatch 'Set-Content') "Fitter must not edit source files."

Write-Host "motor actuator MATLAB contract checks passed"

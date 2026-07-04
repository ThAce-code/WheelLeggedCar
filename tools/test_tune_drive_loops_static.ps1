$ErrorActionPreference = "Stop"

function Assert-Contains {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )
    $text = Get-Content $Path -Raw
    if($text -notmatch $Pattern) {
        throw $Message
    }
}

Assert-Contains "tools/tune_drive_loops.m" "gyro_z_raw_dps" "Script must read raw gyro_z for quality checks."
Assert-Contains "tools/tune_drive_loops.m" "turn_integral" "Script must read turn integral."
Assert-Contains "tools/tune_drive_loops.m" "zeroGyroStd" "Script must compute zero-turn gyro noise."
Assert-Contains "tools/tune_drive_loops.m" "zeroGyroMean" "Script must compute zero-turn gyro bias."
Assert-Contains "tools/tune_drive_loops.m" "turnSameSignPct" "Script must compute turn sign agreement."
Assert-Contains "tools/tune_drive_loops.m" "turnSatPct" "Script must compute turn output saturation."
Assert-Contains "tools/tune_drive_loops.m" "rejectReason" "Script must record rejection reasons."
Assert-Contains "tools/tune_drive_loops.m" "BT,%.3f,%.3f" "Script must print BT gain recommendations."
Assert-Contains "tools/tune_drive_loops_fast.m" "fast_blend" "Fast tune script must handle fast blend."
Assert-Contains "tools/tune_drive_loops_fast.m" "theo_ceiling" "Fast tune script must compute speed ceiling."
Assert-Contains "tools/tune_drive_loops_fast.m" "budget_used_pct" "Fast tune script must compute RPM budget."
Assert-Contains "tools/tune_drive_loops_fast.m" "fast_term_budget" "Fast tune script must output term budget CSV."
Assert-Contains "tools/tune_drive_loops_fast.m" "fast_ceiling_analysis" "Fast tune script must output ceiling analysis CSV."
Assert-Contains "tools/tune_drive_loops_fast.m" "fast_blend_transition" "Fast tune script must plot blend transition."

Write-Host "tune_drive_loops static checks passed"

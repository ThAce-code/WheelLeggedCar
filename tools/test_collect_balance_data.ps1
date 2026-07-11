$ErrorActionPreference = "Stop"

$scriptPath = Join-Path $PSScriptRoot "collect_balance_data.ps1"
. $scriptPath -LoadOnly

function Assert-True {
    param(
        [bool]$Condition,
        [string]$Message
    )

    if(-not $Condition) {
        throw $Message
    }
}

function Assert-Near {
    param(
        [double]$Actual,
        [double]$Expected,
        [double]$Tolerance,
        [string]$Message
    )

    if([Math]::Abs($Actual - $Expected) -gt $Tolerance) {
        throw ("{0}: actual={1}, expected={2}" -f $Message, $Actual, $Expected)
    }
}

$schedule = Parse-CommandSchedule -Text "0:STOP;1.5:B,2;2:C,0,0"
Assert-True ($schedule.Count -eq 3) "expected three scheduled commands"
Assert-Near $schedule[1].AtSeconds 1.5 0.0001 "second command time"
Assert-True ($schedule[2].Command -eq "C,0,0") "third command text"
Assert-True ((Convert-CsvField "C,0,0") -eq '"C,0,0"') "CSV fields with commas must be quoted"
Assert-True ((Convert-CsvField 'note "quoted"') -eq '"note ""quoted"""') "CSV quotes must be escaped"
Assert-True (($Fields.Split(",").Count -eq 85)) "CSV header must contain metadata, 80 telemetry fields, and note"

$values = [single[]](1234.0, 2.0, 1.5, 4.5, 90.0, -12.25, 9.75, 1.0, 48.0, 47.0, -120.0, -118.0, 4.0, 0.2, 20.0, 18.5, 0.9, 5.0, 0.0, 0.0, 0.0, 2.5, 3.5, 4.5, 0.6, 0.05, 5.0, 10.0, 0.25, 12.0, 9.0, 1.5, 2.4, -8.0, 3.0, 20.0, 0.0, 1.5, 3.0, 100.0, 95.0, 0.3, 0.0, 95.0, 0.0, 95.0, 1.0, 1.0, 88.0, 92.0, 89.0, 91.0, 19.2, 8.4, 2.7, -1.35, 64.0, 190.0, 96.5, 4.0, 0.42, 30.0, 2.0, 1.0, 1.0, 91.0, 89.0, 89.0, 91.0, 90.5, 89.5, 89.5, 90.5, 0.5, 0.0, 0.5, 0.0, 1.0, 2.0, 250.0)
$buffer = New-Object System.Collections.Generic.List[byte]
$buffer.Add(0x55)
foreach($value in $values) {
    foreach($byte in [BitConverter]::GetBytes($value)) {
        $buffer.Add($byte)
    }
}
foreach($byte in [byte[]](0x00, 0x00, 0x80, 0x7F)) {
    $buffer.Add($byte)
}

$frames = @(Pop-BalanceFrames -Buffer $buffer)
Assert-True ($frames.Count -eq 1) "expected one parsed frame"
Assert-Near $frames[0].time_ms 1234.0 0.001 "time_ms"
Assert-Near $frames[0].balance_mode 2.0 0.001 "balance_mode"
Assert-Near $frames[0].roll_deg 1.5 0.001 "roll_deg"
Assert-Near $frames[0].pitch_deg 4.5 0.001 "pitch_deg"
Assert-Near $frames[0].yaw_deg 90.0 0.001 "yaw_deg"
Assert-Near $frames[0].pitch_rate_dps -12.25 0.001 "pitch_rate_dps"
Assert-Near $frames[0].balance_rpm 9.75 0.001 "balance_rpm"
Assert-Near $frames[0].feedback_online 1.0 0.001 "feedback_online"
Assert-Near $frames[0].left_motor_rpm 48.0 0.001 "left_motor_rpm"
Assert-Near $frames[0].right_motor_rpm 47.0 0.001 "right_motor_rpm"
Assert-Near $frames[0].left_duty -120.0 0.001 "left_duty"
Assert-Near $frames[0].right_duty -118.0 0.001 "right_duty"
Assert-Near $frames[0].balance_kp 4.0 0.001 "balance_kp"
Assert-Near $frames[0].balance_kd 0.2 0.001 "balance_kd"
Assert-Near $frames[0].forward_target_rpm 20.0 0.001 "forward_target_rpm"
Assert-Near $frames[0].forward_actual_rpm 18.5 0.001 "forward_actual_rpm"
Assert-Near $frames[0].speed_pitch_offset_deg 0.9 0.001 "speed_pitch_offset_deg"
Assert-Near $frames[0].pitch_setpoint_deg 5.0 0.001 "pitch_setpoint_deg"
Assert-Near $frames[0].turn_target_dps 0.0 0.001 "turn_target_dps"
Assert-Near $frames[0].gyro_z_dps 0.0 0.001 "gyro_z_dps"
Assert-Near $frames[0].turn_rpm 0.0 0.001 "turn_rpm"
Assert-Near $frames[0].gyro_z_raw_dps 2.5 0.001 "gyro_z_raw_dps"
Assert-Near $frames[0].turn_error_dps 3.5 0.001 "turn_error_dps"
Assert-Near $frames[0].turn_integral 4.5 0.001 "turn_integral"
Assert-Near $frames[0].turn_kp 0.6 0.001 "turn_kp"
Assert-Near $frames[0].turn_ki 0.05 0.001 "turn_ki"
Assert-Near $frames[0].imu_age_ms 5.0 0.001 "imu_age_ms"
Assert-Near $frames[0].wheel_age_ms 10.0 0.001 "wheel_age_ms"
Assert-Near $frames[0].fast_blend 0.25 0.001 "fast_blend"
Assert-Near $frames[0].speed_integral 12.0 0.001 "speed_integral"
Assert-Near $frames[0].speed_pitch_limit_deg 9.0 0.001 "speed_pitch_limit_deg"
Assert-Near $frames[0].speed_ff_rpm 1.5 0.001 "speed_ff_rpm"
Assert-Near $frames[0].wheel_speed_ks 2.4 0.001 "wheel_speed_ks"
Assert-Near $frames[0].pitch_term_rpm -8.0 0.001 "pitch_term_rpm"
Assert-Near $frames[0].rate_term_rpm 3.0 0.001 "rate_term_rpm"
Assert-Near $frames[0].speed_term_rpm 20.0 0.001 "speed_term_rpm"
Assert-Near $frames[0].pos_term_rpm 0.0 0.001 "pos_term_rpm"
Assert-Near $frames[0].ff_term_rpm 1.5 0.001 "ff_term_rpm"
Assert-Near $frames[0].leg_mode 3.0 0.001 "leg_mode"
Assert-Near $frames[0].leg_target_height_mm 100.0 0.001 "leg_target_height_mm"
Assert-Near $frames[0].leg_height_cmd_est_mm 95.0 0.001 "leg_height_cmd_est_mm"
Assert-Near $frames[0].leg_height_norm 0.3 0.001 "leg_height_norm"
Assert-Near $frames[0].leg_ik_valid 1.0 0.001 "leg_ik_valid"
Assert-Near $frames[0].leg_output_enable 1.0 0.001 "leg_output_enable"
Assert-Near $frames[0].servo0_output_deg 88.0 0.001 "servo0_output_deg"
Assert-Near $frames[0].balance_pitch_kp_eff 19.2 0.001 "balance_pitch_kp_eff"
Assert-Near $frames[0].chassis_forward_limit_eff_rpm 64.0 0.001 "chassis_forward_limit_eff_rpm"
Assert-Near $frames[0].chassis_fast_forward_limit_eff_rpm 190.0 0.001 "chassis_fast_forward_limit_eff_rpm"
Assert-Near $frames[0].leg_height_ref_mm 96.5 0.001 "leg_height_ref_mm"
Assert-Near $frames[0].leg_height_rate_mm_s 4.0 0.001 "leg_height_rate_mm_s"
Assert-Near $frames[0].leg_ik_margin 0.42 0.001 "leg_ik_margin"
Assert-Near $frames[0].leg_drive_forward_limit_rpm 30.0 0.001 "leg_drive_forward_limit_rpm"
Assert-Near $frames[0].leg_motion_state 2.0 0.001 "leg_motion_state"
Assert-Near $frames[0].leg_fault_reason 1.0 0.001 "leg_fault_reason"
Assert-Near $frames[0].leg_drive_allowed 1.0 0.001 "leg_drive_allowed"
Assert-Near $frames[0].servo0_target_deg 91.0 0.001 "servo0_target_deg"
Assert-Near $frames[0].servo0_filtered_deg 90.5 0.001 "servo0_filtered_deg"
Assert-Near $frames[0].servo_max_error_deg 0.5 0.001 "servo_max_error_deg"
Assert-Near $frames[0].servo_settled 0.0 0.001 "servo_settled"
Assert-Near $frames[0].servo_s7_progress 0.5 0.001 "servo_s7_progress"
Assert-Near $frames[0].servo_trajectory_mode 2.0 0.001 "servo_trajectory_mode"
Assert-Near $frames[0].servo_s7_remaining_ms 250.0 0.001 "servo_s7_remaining_ms"
Assert-True ($Fields -match "leg_height_ref_mm") "CSV header must include leg_height_ref_mm"
Assert-True ($Fields -match "servo0_output_deg") "CSV header must include servo output labels"
Assert-True ($Fields -notmatch "leg_actual_height_mm") "CSV header must not imply measured height"
Assert-True ($buffer.Count -eq 0) "buffer should be consumed after frame"

Write-Host "collect_balance_data tests passed"

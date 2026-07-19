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
# One packed status float expands to five decoded status/provenance columns.
# 4 metadata + 72 wire values + 5 decoded fields + note = 82 fields.
Assert-True (($Fields.Split(",").Count -eq 82)) "CSV header and decoded telemetry column count must remain synchronized"
Assert-True ($FloatCount -eq 72) "collector must decode the 72-float wire contract"

# 72-float test frame: indices 0-45 control data, 46-54 timing diagnostics,
# 55-71 race-assist diagnostics.
# Pose status 31 sets IK valid, both pose-valid bits, measured-left source,
# and mirror-assumption right source.
$values = [single[]](
    1234.0, 2.0, 1.5, 4.5, 90.0, -12.25, 9.75, 1.0, 48.0, 47.0, -120.0, -118.0,
    3.0, 100.0, 96.5, 0.3, 31.0, 1.0,
    88.0, 92.0, 89.0, 91.0,
    91.0, 89.0, 89.0, 91.0,
    90.5, 89.5, 89.5, 90.5,
    0.5, 0.0, 0.5,
    -20.75, 47.25, -20.50, 47.00, 0.42, 2.0, 1.0,
    35.0, 1.0, 1.0, 0.0, 2.0, 250.0,
    42.0, 3.0, 7.0, 4.0, 12345.0, 987.0, 2.0, 6.0, -1.75,
    1.0, 4.0, 2.0, 3.0, -0.70, -0.60, 125.0, 150.0, 148.0, 140.0,
    10.0, -1.35, 300.0, 0.80, 0.25, 0.50, 9.0
)
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
# 0-11: core motor/balance/IMU
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
# 12-17: legacy leg stance/IK
Assert-Near $frames[0].leg_mode 3.0 0.001 "leg_mode"
Assert-Near $frames[0].leg_legacy_stance_target_units 100.0 0.001 "leg_legacy_stance_target_units"
Assert-Near $frames[0].leg_legacy_stance_ref_units 96.5 0.001 "leg_legacy_stance_ref_units"
Assert-Near $frames[0].leg_legacy_stance_norm 0.3 0.001 "leg_legacy_stance_norm"
Assert-Near $frames[0].leg_pose_status_flags 31.0 0.001 "leg_pose_status_flags"
Assert-Near $frames[0].leg_ik_valid 1.0 0.001 "leg_ik_valid"
Assert-Near $frames[0].leg_left_pose_valid 1.0 0.001 "leg_left_pose_valid"
Assert-Near $frames[0].leg_right_pose_valid 1.0 0.001 "leg_right_pose_valid"
Assert-True ($frames[0].leg_left_pose_source -eq "measured_calibration") "left pose source"
Assert-True ($frames[0].leg_right_pose_source -eq "mirror_assumption") "right pose source"
Assert-Near $frames[0].leg_output_enable 1.0 0.001 "leg_output_enable"
# 18-21: servo output
Assert-Near $frames[0].servo0_output_deg 88.0 0.001 "servo0_output_deg"
# 22-25: servo targets
Assert-Near $frames[0].servo0_target_deg 91.0 0.001 "servo0_target_deg"
# 26-29: servo filtered
Assert-Near $frames[0].servo0_filtered_deg 90.5 0.001 "servo0_filtered_deg"
# 30-32: settle diagnostics
Assert-Near $frames[0].servo_max_error_deg 0.5 0.001 "servo_max_error_deg"
Assert-Near $frames[0].servo_settled 0.0 0.001 "servo_settled"
Assert-Near $frames[0].servo_s7_progress 0.5 0.001 "servo_s7_progress"
# 33-37: physical command poses and IK margin
Assert-Near $frames[0].leg_left_command_x_mm -20.75 0.001 "leg_left_command_x_mm"
Assert-Near $frames[0].leg_left_command_y_mm 47.25 0.001 "leg_left_command_y_mm"
Assert-Near $frames[0].leg_right_command_x_mm -20.50 0.001 "leg_right_command_x_mm"
Assert-Near $frames[0].leg_right_command_y_mm 47.00 0.001 "leg_right_command_y_mm"
Assert-Near $frames[0].leg_ik_margin 0.42 0.001 "leg_ik_margin"
# 38-39: leg motion state
Assert-Near $frames[0].leg_motion_state 2.0 0.001 "leg_motion_state"
Assert-Near $frames[0].leg_fault_reason 1.0 0.001 "leg_fault_reason"
# 40-45: safety and trajectory mode
Assert-Near $frames[0].leg_drive_forward_limit_rpm 35.0 0.001 "leg_drive_forward_limit_rpm"
Assert-Near $frames[0].leg_drive_allowed 1.0 0.001 "leg_drive_allowed"
Assert-Near $frames[0].servo_fast_mode 1.0 0.001 "servo_fast_mode"
Assert-Near $frames[0].servo_direct_bypass 0.0 0.001 "servo_direct_bypass"
Assert-Near $frames[0].servo_trajectory_mode 2.0 0.001 "servo_trajectory_mode"
Assert-Near $frames[0].servo_s7_remaining_ms 250.0 0.001 "servo_s7_remaining_ms"
# 46-54: timing and sample-integrity diagnostics
Assert-Near $frames[0].firmware_frame_sequence 42.0 0.001 "firmware_frame_sequence"
Assert-Near $frames[0].telemetry_drop_count 3.0 0.001 "telemetry_drop_count"
Assert-Near $frames[0].scheduler_missed_tick_count 7.0 0.001 "scheduler_missed_tick_count"
Assert-Near $frames[0].scheduler_max_gap_ms 4.0 0.001 "scheduler_max_gap_ms"
Assert-Near $frames[0].servo_tick_count 12345.0 0.001 "servo_tick_count"
Assert-Near $frames[0].imu_int_count 987.0 0.001 "imu_int_count"
Assert-Near $frames[0].imu_invalid_count 2.0 0.001 "imu_invalid_count"
Assert-Near $frames[0].imu_age_ms 6.0 0.001 "imu_age_ms"
Assert-Near $frames[0].gyro_y_raw_dps -1.75 0.001 "gyro_y_raw_dps"

# 55-71: race-assist diagnostics
Assert-Near $frames[0].race_assist_enable 1.0 0.001 "race_assist_enable"
Assert-Near $frames[0].race_assist_level 4.0 0.001 "race_assist_level"
Assert-Near $frames[0].race_assist_state 2.0 0.001 "race_assist_state"
Assert-Near $frames[0].race_assist_fault_reason 3.0 0.001 "race_assist_fault_reason"
Assert-Near $frames[0].race_u_request -0.70 0.001 "race_u_request"
Assert-Near $frames[0].race_u_actual -0.60 0.001 "race_u_actual"
Assert-Near $frames[0].requested_accel_rpm_s 125.0 0.001 "requested_accel_rpm_s"
Assert-Near $frames[0].forward_target_rpm 150.0 0.001 "forward_target_rpm"
Assert-Near $frames[0].forward_ramped_rpm 148.0 0.001 "forward_ramped_rpm"
Assert-Near $frames[0].wheel_speed_measured_rpm 140.0 0.001 "wheel_speed_measured_rpm"
Assert-Near $frames[0].speed_error_rpm 10.0 0.001 "speed_error_rpm"
Assert-Near $frames[0].pitch_setpoint_deg -1.35 0.001 "pitch_setpoint_deg"
Assert-Near $frames[0].balance_output_limit_rpm 300.0 0.001 "balance_output_limit_rpm"
Assert-Near $frames[0].race_turn_scale 0.80 0.001 "race_turn_scale"
Assert-Near $frames[0].left_ik_margin 0.25 0.001 "left_ik_margin"
Assert-Near $frames[0].right_ik_margin 0.50 0.001 "right_ik_margin"
Assert-Near $frames[0].ik_branch_flags 9.0 0.001 "ik_branch_flags"

Assert-True ($Fields -match "servo0_target_deg") "CSV header must include servo target"
Assert-True ($Fields -match "servo0_filtered_deg") "CSV header must include servo filtered"
Assert-True ($Fields -match "servo_max_error_deg") "CSV header must include max error"
Assert-True ($Fields -match "servo_settled") "CSV header must include settled"
Assert-True ($Fields -match "servo_s7_progress") "CSV header must include S7 progress"
Assert-True ($Fields -match "leg_drive_allowed") "CSV header must include drive permission"
Assert-True ($Fields -match "servo_trajectory_mode") "CSV header must include trajectory mode"
Assert-True ($Fields -match "leg_left_command_x_mm") "CSV header must include left physical X"
Assert-True ($Fields -match "leg_right_command_y_mm") "CSV header must include right physical Y"
Assert-True ($Fields -match "leg_left_pose_source") "CSV header must include left pose source"
Assert-True ($Fields -match "leg_right_pose_source") "CSV header must include right pose source"
Assert-True ($Fields -match "leg_legacy_stance_target_units") "CSV header must include legacy stance target"
Assert-True ($Fields -match "leg_legacy_stance_ref_units") "CSV header must include legacy stance reference"
Assert-True ($Fields -match "leg_legacy_stance_norm") "CSV header must include legacy stance norm"
Assert-True ($Fields -match "race_assist_enable") "CSV header must include race assist enable"
Assert-True ($Fields -match "race_u_actual") "CSV header must include race actual request"
Assert-True ($Fields -match "requested_accel_rpm_s") "CSV header must include requested acceleration"
Assert-True ($Fields -match "forward_ramped_rpm") "CSV header must include ramped forward target"
Assert-True ($Fields -match "balance_output_limit_rpm") "CSV header must include balance output limit"
Assert-True ($Fields -match "ik_branch_flags") "CSV header must include IK branch flags"
Assert-True ($Fields -notmatch "leg_actual_height_mm") "CSV header must not imply measured height"
Assert-True ($Fields -notmatch "leg_height_cmd_est_mm") "CSV header must not duplicate the legacy stance reference"
Assert-True ($buffer.Count -eq 0) "buffer should be consumed after frame"

Write-Host "collect_balance_data tests passed"

param(
    [string]$Port = "COM6",
    [int]$Baud = 460800,
    [double]$Duration = 20.0,
    [string]$Commands = "0:STOP;1:B,1;2:B,2;2.2:C,0,0",
    [string]$Out = "",
    [string]$Note = "",
    [switch]$ListenOnly,
    [switch]$NoStopOnExit,
    [switch]$LoadOnly
)

$ErrorActionPreference = "Stop"

$Tail = [byte[]](0x00, 0x00, 0x80, 0x7F)
$FloatCount = 65
$PayloadLen = $FloatCount * 4
$FrameLen = $PayloadLen + $Tail.Length
$Fields = "pc_time_s,elapsed_s,sample_index,last_command,time_ms,balance_mode,roll_deg,pitch_deg,yaw_deg,pitch_rate_dps,balance_rpm,feedback_online,left_motor_rpm,right_motor_rpm,left_duty,right_duty,balance_kp,balance_kd,forward_target_rpm,forward_actual_rpm,speed_pitch_offset_deg,pitch_setpoint_deg,turn_target_dps,gyro_z_dps,turn_rpm,gyro_z_raw_dps,turn_error_dps,turn_integral,turn_kp,turn_ki,imu_age_ms,wheel_age_ms,fast_blend,speed_integral,speed_pitch_limit_deg,speed_ff_rpm,wheel_speed_ks,pitch_term_rpm,rate_term_rpm,speed_term_rpm,pos_term_rpm,ff_term_rpm,leg_mode,leg_target_height_mm,leg_height_cmd_est_mm,leg_height_norm,leg_left_x_mm,leg_left_y_mm,leg_right_x_mm,leg_right_y_mm,leg_ik_valid,leg_output_enable,servo0_output_deg,servo1_output_deg,servo2_output_deg,servo3_output_deg,balance_pitch_kp_eff,balance_pitch_rate_kd_eff,balance_wheel_speed_ks_eff,balance_pitch_setpoint_base_eff_deg,chassis_forward_limit_eff_rpm,chassis_fast_forward_limit_eff_rpm,leg_height_ref_mm,leg_height_rate_mm_s,leg_ik_margin,leg_drive_forward_limit_rpm,leg_motion_state,leg_fault_reason,leg_drive_allowed,note"

function Parse-CommandSchedule {
    param([string]$Text)

    $items = New-Object System.Collections.Generic.List[object]
    if([string]::IsNullOrWhiteSpace($Text)) {
        return @()
    }

    foreach($item in $Text.Split(";")) {
        $part = $item.Trim()
        if($part.Length -eq 0) {
            continue
        }

        if(-not $part.Contains(":")) {
            throw "Command schedule item must use seconds:command format: $part"
        }

        $pieces = $part.Split(":", 2)
        $atSeconds = [double]$pieces[0]
        $command = $pieces[1].Trim()
        if($atSeconds -lt 0.0) {
            throw "Command time must be >= 0: $part"
        }
        if($command.Length -eq 0) {
            throw "Command text is empty: $part"
        }

        $items.Add([pscustomobject]@{
            AtSeconds = $atSeconds
            Command = $command
        })
    }

    return @($items | Sort-Object AtSeconds)
}

function Send-Command {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [string]$Command
    )

    $Serial.Write("$Command`n")
}

function Convert-CsvField {
    param([object]$Value)

    if($null -eq $Value) {
        return ""
    }

    $text = [string]$Value
    if(($text.Contains(",")) -or ($text.Contains('"')) -or ($text.Contains("`r")) -or ($text.Contains("`n"))) {
        return '"' + $text.Replace('"', '""') + '"'
    }

    return $text
}

function Find-Tail {
    param([System.Collections.Generic.List[byte]]$Buffer)

    for($i = 0; $i -le ($Buffer.Count - $Tail.Length); $i++) {
        if(($Buffer[$i] -eq $Tail[0]) -and
           ($Buffer[$i + 1] -eq $Tail[1]) -and
           ($Buffer[$i + 2] -eq $Tail[2]) -and
           ($Buffer[$i + 3] -eq $Tail[3])) {
            return $i
        }
    }

    return -1
}

function Pop-BalanceFrames {
    param([System.Collections.Generic.List[byte]]$Buffer)

    $frames = New-Object System.Collections.Generic.List[object]
    while($true) {
        $tailIndex = Find-Tail -Buffer $Buffer
        if($tailIndex -lt 0) {
            if($Buffer.Count -gt ($FrameLen * 4)) {
                $Buffer.RemoveRange(0, $Buffer.Count - $FrameLen)
            }
            break
        }

        if($tailIndex -ge $PayloadLen) {
            $payloadStart = $tailIndex - $PayloadLen
            $payload = New-Object byte[] $PayloadLen
            for($i = 0; $i -lt $PayloadLen; $i++) {
                $payload[$i] = $Buffer[$payloadStart + $i]
            }

            $values = New-Object double[] $FloatCount
            for($i = 0; $i -lt $FloatCount; $i++) {
                $values[$i] = [BitConverter]::ToSingle($payload, $i * 4)
            }

            $frames.Add([pscustomobject]@{
                time_ms = $values[0]
                balance_mode = $values[1]
                roll_deg = $values[2]
                pitch_deg = $values[3]
                yaw_deg = $values[4]
                pitch_rate_dps = $values[5]
                balance_rpm = $values[6]
                feedback_online = $values[7]
                left_motor_rpm = $values[8]
                right_motor_rpm = $values[9]
                left_duty = $values[10]
                right_duty = $values[11]
                balance_kp = $values[12]
                balance_kd = $values[13]
                forward_target_rpm = $values[14]
                forward_actual_rpm = $values[15]
                speed_pitch_offset_deg = $values[16]
                pitch_setpoint_deg = $values[17]
                turn_target_dps = $values[18]
                gyro_z_dps = $values[19]
                turn_rpm = $values[20]
                gyro_z_raw_dps = $values[21]
                turn_error_dps = $values[22]
                turn_integral = $values[23]
                turn_kp = $values[24]
                turn_ki = $values[25]
                imu_age_ms = $values[26]
                wheel_age_ms = $values[27]
                fast_blend = $values[28]
                speed_integral = $values[29]
                speed_pitch_limit_deg = $values[30]
                speed_ff_rpm = $values[31]
                wheel_speed_ks = $values[32]
                pitch_term_rpm = $values[33]
                rate_term_rpm = $values[34]
                speed_term_rpm = $values[35]
                pos_term_rpm = $values[36]
                ff_term_rpm = $values[37]
                leg_mode = $values[38]
                leg_target_height_mm = $values[39]
                leg_height_cmd_est_mm = $values[40]
                leg_height_norm = $values[41]
                leg_left_x_mm = $values[42]
                leg_left_y_mm = $values[43]
                leg_right_x_mm = $values[44]
                leg_right_y_mm = $values[45]
                leg_ik_valid = $values[46]
                leg_output_enable = $values[47]
                servo0_output_deg = $values[48]
                servo1_output_deg = $values[49]
                servo2_output_deg = $values[50]
                servo3_output_deg = $values[51]
                balance_pitch_kp_eff = $values[52]
                balance_pitch_rate_kd_eff = $values[53]
                balance_wheel_speed_ks_eff = $values[54]
                balance_pitch_setpoint_base_eff_deg = $values[55]
                chassis_forward_limit_eff_rpm = $values[56]
                chassis_fast_forward_limit_eff_rpm = $values[57]
                leg_height_ref_mm = $values[58]
                leg_height_rate_mm_s = $values[59]
                leg_ik_margin = $values[60]
                leg_drive_forward_limit_rpm = $values[61]
                leg_motion_state = $values[62]
                leg_fault_reason = $values[63]
                leg_drive_allowed = $values[64]
            })
        }

        $Buffer.RemoveRange(0, $tailIndex + $Tail.Length)
    }

    return $frames
}

function Read-AvailableBytes {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [byte[]]$ReadBuffer
    )

    try {
        return $Serial.Read($ReadBuffer, 0, $ReadBuffer.Length)
    } catch [System.TimeoutException] {
        return 0
    }
}

if($LoadOnly) {
    return
}

if($Duration -le 0.0) {
    throw "Duration must be positive."
}

if($ListenOnly) {
    $Commands = ""
}

$schedule = Parse-CommandSchedule -Text $Commands
if($Out.Length -eq 0) {
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $Out = "data/balance_capture_$stamp.csv"
}

$outPath = [System.IO.Path]::GetFullPath($Out)
$outDir = [System.IO.Path]::GetDirectoryName($outPath)
if($outDir -and -not (Test-Path -LiteralPath $outDir)) {
    New-Item -ItemType Directory -Path $outDir | Out-Null
}

$serial = [System.IO.Ports.SerialPort]::new($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.ReadTimeout = 20
$serial.WriteTimeout = 1000
$serial.NewLine = "`n"

$writer = [System.IO.StreamWriter]::new($outPath, $false, [System.Text.Encoding]::UTF8)
$rxBuffer = New-Object System.Collections.Generic.List[byte]
$readBuffer = New-Object byte[] 512
$sampleIndex = 0
$commandIndex = 0
$lastCommand = ""
$stopwatch = [System.Diagnostics.Stopwatch]::new()

try {
    $serial.Open()
    $writer.WriteLine($Fields)
    $serial.DiscardInBuffer()
    $stopwatch.Start()

    Write-Host ("collecting balance telemetry on {0} at {1} baud for {2:F2}s" -f $Port, $Baud, $Duration)
    if($schedule.Count -gt 0) {
        Write-Host ("scheduled commands: {0}" -f $Commands)
    } else {
        Write-Host "scheduled commands: none"
    }

    while($stopwatch.Elapsed.TotalSeconds -lt $Duration) {
        $elapsed = $stopwatch.Elapsed.TotalSeconds

        while(($commandIndex -lt $schedule.Count) -and ($elapsed -ge $schedule[$commandIndex].AtSeconds)) {
            $lastCommand = $schedule[$commandIndex].Command
            Send-Command -Serial $serial -Command $lastCommand
            Write-Host ("{0,8:F3}s -> {1}" -f $elapsed, $lastCommand)
            $commandIndex++
        }

        $count = Read-AvailableBytes -Serial $serial -ReadBuffer $readBuffer
        if($count -gt 0) {
            for($i = 0; $i -lt $count; $i++) {
                $rxBuffer.Add($readBuffer[$i])
            }

            foreach($frame in (Pop-BalanceFrames -Buffer $rxBuffer)) {
                $now = [DateTime]::UtcNow
                $pcTime = ([DateTimeOffset]$now).ToUnixTimeMilliseconds() / 1000.0
                $elapsedNow = $stopwatch.Elapsed.TotalSeconds
                $row = @(
                    ("{0:F6}" -f $pcTime),
                    ("{0:F6}" -f $elapsedNow),
                    $sampleIndex,
                    (Convert-CsvField $lastCommand),
                    ("{0:F3}" -f $frame.time_ms),
                    ("{0:F3}" -f $frame.balance_mode),
                    ("{0:F6}" -f $frame.roll_deg),
                    ("{0:F6}" -f $frame.pitch_deg),
                    ("{0:F6}" -f $frame.yaw_deg),
                    ("{0:F6}" -f $frame.pitch_rate_dps),
                    ("{0:F3}" -f $frame.balance_rpm),
                    ("{0:F3}" -f $frame.feedback_online),
                    ("{0:F3}" -f $frame.left_motor_rpm),
                    ("{0:F3}" -f $frame.right_motor_rpm),
                    ("{0:F3}" -f $frame.left_duty),
                    ("{0:F3}" -f $frame.right_duty),
                    ("{0:F6}" -f $frame.balance_kp),
                    ("{0:F6}" -f $frame.balance_kd),
                    ("{0:F3}" -f $frame.forward_target_rpm),
                    ("{0:F3}" -f $frame.forward_actual_rpm),
                    ("{0:F6}" -f $frame.speed_pitch_offset_deg),
                    ("{0:F6}" -f $frame.pitch_setpoint_deg),
                    ("{0:F3}" -f $frame.turn_target_dps),
                    ("{0:F6}" -f $frame.gyro_z_dps),
                    ("{0:F3}" -f $frame.turn_rpm),
                    ("{0:F6}" -f $frame.gyro_z_raw_dps),
                    ("{0:F6}" -f $frame.turn_error_dps),
                    ("{0:F6}" -f $frame.turn_integral),
                    ("{0:F6}" -f $frame.turn_kp),
                    ("{0:F6}" -f $frame.turn_ki),
                    ("{0:F3}" -f $frame.imu_age_ms),
                    ("{0:F3}" -f $frame.wheel_age_ms),
                    ("{0:F6}" -f $frame.fast_blend),
                    ("{0:F6}" -f $frame.speed_integral),
                    ("{0:F6}" -f $frame.speed_pitch_limit_deg),
                    ("{0:F6}" -f $frame.speed_ff_rpm),
                    ("{0:F6}" -f $frame.wheel_speed_ks),
                    ("{0:F6}" -f $frame.pitch_term_rpm),
                    ("{0:F6}" -f $frame.rate_term_rpm),
                    ("{0:F6}" -f $frame.speed_term_rpm),
                    ("{0:F6}" -f $frame.pos_term_rpm),
                    ("{0:F6}" -f $frame.ff_term_rpm),
                    ("{0:F3}" -f $frame.leg_mode),
                    ("{0:F3}" -f $frame.leg_target_height_mm),
                    ("{0:F3}" -f $frame.leg_height_cmd_est_mm),
                    ("{0:F6}" -f $frame.leg_height_norm),
                    ("{0:F3}" -f $frame.leg_left_x_mm),
                    ("{0:F3}" -f $frame.leg_left_y_mm),
                    ("{0:F3}" -f $frame.leg_right_x_mm),
                    ("{0:F3}" -f $frame.leg_right_y_mm),
                    ("{0:F3}" -f $frame.leg_ik_valid),
                    ("{0:F3}" -f $frame.leg_output_enable),
                    ("{0:F6}" -f $frame.servo0_output_deg),
                    ("{0:F6}" -f $frame.servo1_output_deg),
                    ("{0:F6}" -f $frame.servo2_output_deg),
                    ("{0:F6}" -f $frame.servo3_output_deg),
                    ("{0:F6}" -f $frame.balance_pitch_kp_eff),
                    ("{0:F6}" -f $frame.balance_pitch_rate_kd_eff),
                    ("{0:F6}" -f $frame.balance_wheel_speed_ks_eff),
                    ("{0:F6}" -f $frame.balance_pitch_setpoint_base_eff_deg),
                    ("{0:F3}" -f $frame.chassis_forward_limit_eff_rpm),
                    ("{0:F3}" -f $frame.chassis_fast_forward_limit_eff_rpm),
                    ("{0:F3}" -f $frame.leg_height_ref_mm),
                    ("{0:F6}" -f $frame.leg_height_rate_mm_s),
                    ("{0:F6}" -f $frame.leg_ik_margin),
                    ("{0:F3}" -f $frame.leg_drive_forward_limit_rpm),
                    ("{0:F3}" -f $frame.leg_motion_state),
                    ("{0:F3}" -f $frame.leg_fault_reason),
                    ("{0:F3}" -f $frame.leg_drive_allowed),
                    (Convert-CsvField $Note)
                )
                $writer.WriteLine($row -join ",")
                $sampleIndex++
            }
        }
    }
} finally {
    if($serial.IsOpen) {
        if(-not $NoStopOnExit) {
            Send-Command -Serial $serial -Command "STOP"
        }
        $serial.Close()
    }
    $writer.Close()
}

Write-Host "saved $sampleIndex samples to $outPath"

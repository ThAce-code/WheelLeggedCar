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
$FloatCount = 55
$PayloadLen = $FloatCount * 4
$FrameLen = $PayloadLen + $Tail.Length
$Fields = "pc_time_s,elapsed_s,sample_index,last_command,time_ms,balance_mode,roll_deg,pitch_deg,yaw_deg,pitch_rate_dps,balance_rpm,feedback_online,left_motor_rpm,right_motor_rpm,left_duty,right_duty,leg_mode,leg_target_height_mm,leg_height_cmd_est_mm,leg_height_norm,leg_ik_valid,leg_output_enable,servo0_output_deg,servo1_output_deg,servo2_output_deg,servo3_output_deg,servo0_target_deg,servo1_target_deg,servo2_target_deg,servo3_target_deg,servo0_filtered_deg,servo1_filtered_deg,servo2_filtered_deg,servo3_filtered_deg,servo_max_error_deg,servo_settled,servo_s7_progress,leg_left_y_mm,leg_right_y_mm,leg_height_ref_mm,leg_height_rate_mm_s,leg_ik_margin,leg_motion_state,leg_fault_reason,leg_drive_forward_limit_rpm,leg_drive_allowed,servo_fast_mode,servo_direct_bypass,servo_trajectory_mode,servo_s7_remaining_ms,firmware_frame_sequence,telemetry_drop_count,scheduler_missed_tick_count,scheduler_max_gap_ms,servo_tick_count,imu_int_count,imu_invalid_count,imu_age_ms,gyro_y_raw_dps,note"

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
                leg_mode = $values[12]
                leg_target_height_mm = $values[13]
                leg_height_cmd_est_mm = $values[14]
                leg_height_norm = $values[15]
                leg_ik_valid = $values[16]
                leg_output_enable = $values[17]
                servo0_output_deg = $values[18]
                servo1_output_deg = $values[19]
                servo2_output_deg = $values[20]
                servo3_output_deg = $values[21]
                servo0_target_deg = $values[22]
                servo1_target_deg = $values[23]
                servo2_target_deg = $values[24]
                servo3_target_deg = $values[25]
                servo0_filtered_deg = $values[26]
                servo1_filtered_deg = $values[27]
                servo2_filtered_deg = $values[28]
                servo3_filtered_deg = $values[29]
                servo_max_error_deg = $values[30]
                servo_settled = $values[31]
                servo_s7_progress = $values[32]
                leg_left_y_mm = $values[33]
                leg_right_y_mm = $values[34]
                leg_height_ref_mm = $values[35]
                leg_height_rate_mm_s = $values[36]
                leg_ik_margin = $values[37]
                leg_motion_state = $values[38]
                leg_fault_reason = $values[39]
                leg_drive_forward_limit_rpm = $values[40]
                leg_drive_allowed = $values[41]
                servo_fast_mode = $values[42]
                servo_direct_bypass = $values[43]
                servo_trajectory_mode = $values[44]
                servo_s7_remaining_ms = $values[45]
                firmware_frame_sequence = $values[46]
                telemetry_drop_count = $values[47]
                scheduler_missed_tick_count = $values[48]
                scheduler_max_gap_ms = $values[49]
                servo_tick_count = $values[50]
                imu_int_count = $values[51]
                imu_invalid_count = $values[52]
                imu_age_ms = $values[53]
                gyro_y_raw_dps = $values[54]
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
                    ("{0:F3}" -f $frame.leg_mode),
                    ("{0:F3}" -f $frame.leg_target_height_mm),
                    ("{0:F3}" -f $frame.leg_height_cmd_est_mm),
                    ("{0:F6}" -f $frame.leg_height_norm),
                    ("{0:F3}" -f $frame.leg_ik_valid),
                    ("{0:F3}" -f $frame.leg_output_enable),
                    ("{0:F6}" -f $frame.servo0_output_deg),
                    ("{0:F6}" -f $frame.servo1_output_deg),
                    ("{0:F6}" -f $frame.servo2_output_deg),
                    ("{0:F6}" -f $frame.servo3_output_deg),
                    ("{0:F6}" -f $frame.servo0_target_deg),
                    ("{0:F6}" -f $frame.servo1_target_deg),
                    ("{0:F6}" -f $frame.servo2_target_deg),
                    ("{0:F6}" -f $frame.servo3_target_deg),
                    ("{0:F6}" -f $frame.servo0_filtered_deg),
                    ("{0:F6}" -f $frame.servo1_filtered_deg),
                    ("{0:F6}" -f $frame.servo2_filtered_deg),
                    ("{0:F6}" -f $frame.servo3_filtered_deg),
                    ("{0:F6}" -f $frame.servo_max_error_deg),
                    ("{0:F3}" -f $frame.servo_settled),
                    ("{0:F6}" -f $frame.servo_s7_progress),
                    ("{0:F3}" -f $frame.leg_left_y_mm),
                    ("{0:F3}" -f $frame.leg_right_y_mm),
                    ("{0:F3}" -f $frame.leg_height_ref_mm),
                    ("{0:F6}" -f $frame.leg_height_rate_mm_s),
                    ("{0:F6}" -f $frame.leg_ik_margin),
                    ("{0:F3}" -f $frame.leg_motion_state),
                    ("{0:F3}" -f $frame.leg_fault_reason),
                    ("{0:F3}" -f $frame.leg_drive_forward_limit_rpm),
                    ("{0:F3}" -f $frame.leg_drive_allowed),
                    ("{0:F3}" -f $frame.servo_fast_mode),
                    ("{0:F3}" -f $frame.servo_direct_bypass),
                    ("{0:F3}" -f $frame.servo_trajectory_mode),
                    ("{0:F3}" -f $frame.servo_s7_remaining_ms),
                    ("{0:F3}" -f $frame.firmware_frame_sequence),
                    ("{0:F3}" -f $frame.telemetry_drop_count),
                    ("{0:F3}" -f $frame.scheduler_missed_tick_count),
                    ("{0:F3}" -f $frame.scheduler_max_gap_ms),
                    ("{0:F3}" -f $frame.servo_tick_count),
                    ("{0:F3}" -f $frame.imu_int_count),
                    ("{0:F3}" -f $frame.imu_invalid_count),
                    ("{0:F3}" -f $frame.imu_age_ms),
                    ("{0:F6}" -f $frame.gyro_y_raw_dps),
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

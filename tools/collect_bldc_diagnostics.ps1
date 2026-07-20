param(
    [string]$Port = "COM6",
    [int]$Baud = 460800,
    [double]$Duration = 20.0,
    [string]$Commands = "",
    [string]$Out = "",
    [string]$SummaryOut = "",
    [string]$Note = "",
    [double]$RpmDutyLimit = 2000.0,
    [switch]$AllowMotion,
    [switch]$NoStopOnExit,
    [switch]$LoadOnly
)

$ErrorActionPreference = "Stop"

# balance-fast-mode-spec telemetry contract at commit 8a352d2:
# 72 little-endian floats followed by the VOFA JustFloat tail. Legacy
# 55-float frames are rejected rather than shifted into this parser.
$Tail = [byte[]](0x00, 0x00, 0x80, 0x7F)
$FloatCount = 72
$PayloadLen = $FloatCount * 4
$FrameLen = $PayloadLen + $Tail.Length
$script:bldcTailSeeded = $false

function Reset-BldcTailLock {
    $script:bldcTailSeeded = $false
}

$CsvFields = @(
    "pc_time_s", "elapsed_s", "sample_index", "last_command", "command_elapsed_s",
    "expected_mode", "expected_motor_rpm", "expected_open_duty",
    "time_ms", "balance_mode", "roll_deg", "pitch_deg", "balance_rpm", "feedback_online",
    "left_motor_rpm", "right_motor_rpm", "left_duty", "right_duty",
    "firmware_frame_sequence", "telemetry_drop_count", "scheduler_missed_tick_count",
    "scheduler_max_gap_ms", "imu_int_count", "imu_invalid_count", "imu_age_ms", "note"
)

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

function Test-CommandIsNonMotion {
    param([string]$Command)

    $text = $Command.Trim()
    return (($text -match '^(?i:STOP)$') -or
            ($text -match '^(?i:B)\s*,\s*[01]\s*$'))
}

function Assert-CommandScheduleSafe {
    param(
        [object[]]$Schedule,
        [switch]$AllowMotion
    )

    if($AllowMotion) {
        return
    }

    foreach($item in @($Schedule)) {
        if(-not (Test-CommandIsNonMotion -Command $item.Command)) {
            throw ("Command '{0}' can change motor/control output. Re-run with -AllowMotion only after the car is safely supported." -f $item.Command)
        }
    }
}

function Get-CommandExpectation {
    param([string]$Command)

    $result = [ordered]@{
        Mode = "unknown"
        TargetRpm = $null
        TargetDuty = $null
    }

    if([string]::IsNullOrWhiteSpace($Command)) {
        return [pscustomobject]$result
    }

    $text = $Command.Trim()
    if($text -match '^(?i:M)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*$') {
        $result.Mode = "rpm"
        $result.TargetRpm = [double]$Matches[1]
    } elseif($text -match '^(?i:D)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*$') {
        $result.Mode = "duty"
        $result.TargetDuty = [double]$Matches[1]
    } elseif($text -match '^(?i:STOP)$') {
        $result.Mode = "stop"
        $result.TargetRpm = 0.0
        $result.TargetDuty = 0.0
    } elseif($text -match '^(?i:C)\s*,') {
        $result.Mode = "chassis"
    } elseif($text -match '^(?i:B)\s*,') {
        $result.Mode = "balance"
    }

    return [pscustomobject]$result
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

function Find-BldcTail {
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

function Pop-BldcFrames {
    param([System.Collections.Generic.List[byte]]$Buffer)

    $frames = New-Object System.Collections.Generic.List[object]
    while($true) {
        $tailIndex = Find-BldcTail -Buffer $Buffer
        if($tailIndex -lt 0) {
            if($Buffer.Count -gt ($FrameLen * 4)) {
                $Buffer.RemoveRange(0, $Buffer.Count - $FrameLen)
                Reset-BldcTailLock
            }
            break
        }

        # The first observed tail is only a synchronization anchor. Afterwards
        # decode only an adjacent 72-float payload, never by backtracking from
        # an arbitrary tail that could belong to a legacy or corrupt frame.
        if($script:bldcTailSeeded -and ($tailIndex -eq $PayloadLen)) {
            $payload = New-Object byte[] $PayloadLen
            for($i = 0; $i -lt $PayloadLen; $i++) {
                $payload[$i] = $Buffer[$i]
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
                balance_rpm = $values[6]
                feedback_online = $values[7]
                right_motor_rpm = $values[8]
                left_motor_rpm = $values[9]
                right_duty = $values[10]
                left_duty = $values[11]
                firmware_frame_sequence = $values[46]
                telemetry_drop_count = $values[47]
                scheduler_missed_tick_count = $values[48]
                scheduler_max_gap_ms = $values[49]
                imu_int_count = $values[51]
                imu_invalid_count = $values[52]
                imu_age_ms = $values[53]
            })
        }

        $Buffer.RemoveRange(0, $tailIndex + $Tail.Length)
        $script:bldcTailSeeded = $true
    }

    return $frames
}

function Get-MeanValue {
    param([object[]]$Values)

    if(@($Values).Count -eq 0) {
        return 0.0
    }
    return [double](($Values | Measure-Object -Average).Average)
}

function Get-MaxValue {
    param([object[]]$Values)

    if(@($Values).Count -eq 0) {
        return 0.0
    }
    return [double](($Values | Measure-Object -Maximum).Maximum)
}

function Get-BldcDiagnosticSummary {
    param(
        [object[]]$Rows,
        [double]$RpmDutyLimit = 2000.0
    )

    $allRows = @($Rows)
    $findings = New-Object System.Collections.Generic.List[string]
    $recommendations = New-Object System.Collections.Generic.List[string]

    if($allRows.Count -eq 0) {
        $findings.Add("No valid 72-float JustFloat frame was decoded from UART0; legacy 55-float frames are rejected.")
        $recommendations.Add("Close VOFA+, verify COM port/460800 baud, and confirm the flashed firmware uses the 72-float telemetry contract.")
        return [pscustomobject]@{
            status = "no_telemetry"
            sample_count = 0
            capture_duration_s = 0.0
            estimated_sample_hz = 0.0
            feedback_online_ratio = 0.0
            mean_abs_rpm_error = $null
            duty_saturation_ratio = $null
            sign_mismatch_ratio = $null
            one_side_missing_ratio = $null
            frame_sequence_gap_count = 0
            telemetry_drop_delta = 0
            imu_interrupt_delta = 0
            imu_invalid_delta = 0
            imu_age_max_ms = $null
            findings = @($findings)
            recommendations = @($recommendations)
            limitation = "Firmware does not expose UART1 checksum/unknown-frame counters in the current 72-float telemetry."
        }
    }

    $duration = 0.0
    if($allRows.Count -gt 1) {
        $duration = [double]$allRows[-1].elapsed_s - [double]$allRows[0].elapsed_s
    }
    $sampleHz = if($duration -gt 0.0) { ($allRows.Count - 1) / $duration } else { 0.0 }
    $onlineCount = @($allRows | Where-Object { [double]$_.feedback_online -ge 0.5 }).Count
    $onlineRatio = $onlineCount / [double]$allRows.Count

    $sequenceGapCount = 0
    for($i = 1; $i -lt $allRows.Count; $i++) {
        $previousSequence = [long][Math]::Round([double]$allRows[$i - 1].firmware_frame_sequence)
        $currentSequence = [long][Math]::Round([double]$allRows[$i].firmware_frame_sequence)
        if($currentSequence -gt ($previousSequence + 1)) {
            $sequenceGapCount += ($currentSequence - $previousSequence - 1)
        }
    }
    $dropDelta = [long][Math]::Round([double]$allRows[-1].telemetry_drop_count - [double]$allRows[0].telemetry_drop_count)
    if($dropDelta -lt 0) {
        $dropDelta = 0
    }
    $imuInterruptDelta = [long][Math]::Round([double]$allRows[-1].imu_int_count - [double]$allRows[0].imu_int_count)
    if($imuInterruptDelta -lt 0) {
        $imuInterruptDelta = 0
    }
    $imuInvalidDelta = [long][Math]::Round([double]$allRows[-1].imu_invalid_count - [double]$allRows[0].imu_invalid_count)
    if($imuInvalidDelta -lt 0) {
        $imuInvalidDelta = 0
    }
    $imuAgeMax = Get-MaxValue -Values @($allRows | ForEach-Object { [double]$_.imu_age_ms })

    $rpmRows = @($allRows | Where-Object {
        ($_.expected_mode -eq "rpm") -and
        ([double]$_.command_elapsed_s -ge 0.30) -and
        ([Math]::Abs([double]$_.expected_motor_rpm) -ge 20.0)
    })

    $status = "healthy_capture"
    $meanAbsError = $null
    $saturationRatio = $null
    $signMismatchRatio = $null
    $oneSideMissingRatio = $null

    if($onlineRatio -lt 0.90) {
        $status = "feedback_offline"
        $findings.Add(("BLDC feedback_online ratio is only {0:P1}; the fault is at or before the UART1 feedback/parser layer." -f $onlineRatio))
        $recommendations.Add("Inspect UART1 P04.0/P04.1, 460800 baud, driver power/common ground, and whether speed upload is active.")
        $recommendations.Add("Current telemetry cannot distinguish no RX bytes from checksum failures; expose motor_diag counters if this remains ambiguous.")
    } elseif($rpmRows.Count -eq 0) {
        $status = "inconclusive_no_rpm_step"
        $findings.Add("Telemetry and BLDC feedback were captured, but there is no settled M,rpm interval to evaluate the closed loop.")
        $recommendations.Add("With both wheels safely suspended, run a conservative M step schedule using -AllowMotion.")
    } else {
        $errors = New-Object System.Collections.Generic.List[double]
        $saturatedCount = 0
        $signMismatchCount = 0
        $oneSideMissingCount = 0
        $zeroOutputCount = 0
        $asymmetry = New-Object System.Collections.Generic.List[double]
        $targetMagnitude = New-Object System.Collections.Generic.List[double]

        foreach($row in $rpmRows) {
            $target = [double]$row.expected_motor_rpm
            $leftRpm = [double]$row.left_motor_rpm
            $rightRpm = [double]$row.right_motor_rpm
            $leftDuty = [double]$row.left_duty
            $rightDuty = [double]$row.right_duty

            $errors.Add(0.5 * ([Math]::Abs($target - $leftRpm) + [Math]::Abs($target - $rightRpm)))
            $asymmetry.Add([Math]::Abs($leftRpm - $rightRpm))
            $targetMagnitude.Add([Math]::Abs($target))

            if(([Math]::Abs($leftDuty) -ge (0.95 * $RpmDutyLimit)) -or
               ([Math]::Abs($rightDuty) -ge (0.95 * $RpmDutyLimit))) {
                $saturatedCount++
            }
            if((([Math]::Abs($leftRpm) -ge 10.0) -and (($target * $leftRpm) -lt 0.0)) -or
               (([Math]::Abs($rightRpm) -ge 10.0) -and (($target * $rightRpm) -lt 0.0))) {
                $signMismatchCount++
            }
            if((([Math]::Abs($leftRpm) -lt 5.0) -and ([Math]::Abs($rightRpm) -ge 30.0)) -or
               (([Math]::Abs($rightRpm) -lt 5.0) -and ([Math]::Abs($leftRpm) -ge 30.0))) {
                $oneSideMissingCount++
            }
            if(([Math]::Abs($leftDuty) -lt 1.0) -and ([Math]::Abs($rightDuty) -lt 1.0) -and
               ([Math]::Abs($leftRpm) -lt 0.25 * [Math]::Abs($target)) -and
               ([Math]::Abs($rightRpm) -lt 0.25 * [Math]::Abs($target))) {
                $zeroOutputCount++
            }
        }

        $meanAbsError = Get-MeanValue -Values $errors
        $saturationRatio = $saturatedCount / [double]$rpmRows.Count
        $signMismatchRatio = $signMismatchCount / [double]$rpmRows.Count
        $oneSideMissingRatio = $oneSideMissingCount / [double]$rpmRows.Count
        $zeroOutputRatio = $zeroOutputCount / [double]$rpmRows.Count
        $meanAsymmetry = Get-MeanValue -Values $asymmetry
        $meanTargetMagnitude = Get-MeanValue -Values $targetMagnitude
        $largeErrorThreshold = [Math]::Max(30.0, 0.25 * $meanTargetMagnitude)

        if(($zeroOutputRatio -ge 0.30) -and ($meanAbsError -gt $largeErrorThreshold) -and
           ($imuAgeMax -gt 30.0) -and ($imuInterruptDelta -eq 0)) {
            $status = "imu_stale_safety_block"
            $findings.Add(("IMU interrupt count is frozen and IMU age reached {0:F0} ms; the latched global safety fault forces both M and D motor paths to zero." -f $imuAgeMax))
            $recommendations.Add("Power-cycle only after checking LSM6DSV16X INT1/P19.3. Verify imu_int_count resumes continuously before testing BLDC again.")
            $recommendations.Add("Do not bypass APP_STATE_FAULT; repair the IMU interrupt/stale-data path or add an explicit safe recovery design.")
        } elseif($signMismatchRatio -ge 0.30) {
            $status = "rpm_sign_mismatch"
            $findings.Add(("Measured RPM has the opposite sign to M,rpm in {0:P1} of settled samples." -f $signMismatchRatio))
            $recommendations.Add("Check APP_MOTOR_LEFT_RPM_SIGN/APP_MOTOR_RIGHT_RPM_SIGN against wheel mounting and BLDC feedback polarity before tuning PID.")
        } elseif($oneSideMissingRatio -ge 0.30) {
            $status = "one_side_missing"
            $findings.Add(("One wheel reports near-zero RPM while the other is moving in {0:P1} of settled samples." -f $oneSideMissingRatio))
            $recommendations.Add("Inspect the affected encoder/phase/driver channel and per-side feedback path; do not compensate this with PID gains.")
        } elseif(($saturationRatio -ge 0.20) -and ($meanAbsError -gt $largeErrorThreshold)) {
            $status = "output_saturated"
            $findings.Add(("RPM output is near the {0:F0} duty limit in {1:P1} of samples while mean absolute RPM error is {2:F1}." -f $RpmDutyLimit, $saturationRatio, $meanAbsError))
            $recommendations.Add("Check supply/load, duty polarity, BLDC drive acceptance, and mechanical drag before increasing the duty limit or PID gains.")
        } elseif(($zeroOutputRatio -ge 0.30) -and ($meanAbsError -gt $largeErrorThreshold)) {
            $status = "command_or_safety_blocked"
            $findings.Add("A nonzero M,rpm target was scheduled, but both duty outputs stayed near zero while RPM error remained large.")
            $recommendations.Add("Trace host command parsing, motor mode/enable, balance ownership, feedback gate, and safety state.")
        } elseif($meanAsymmetry -gt [Math]::Max(20.0, 0.20 * $meanTargetMagnitude)) {
            $status = "left_right_asymmetry"
            $findings.Add(("Mean left/right RPM difference is {0:F1} RPM for a mean {1:F1} RPM target." -f $meanAsymmetry, $meanTargetMagnitude))
            $recommendations.Add("Compare left/right wiring, friction and per-side PID; verify the asymmetry persists in both directions.")
        } elseif($meanAbsError -gt $largeErrorThreshold) {
            $status = "poor_rpm_tracking"
            $findings.Add(("BLDC feedback is online, but mean absolute settled RPM error is {0:F1} RPM." -f $meanAbsError))
            $recommendations.Add("Plot target/RPM/duty versus time and determine whether the error is lag, oscillation, deadband or load-related before retuning.")
        } else {
            $findings.Add(("BLDC feedback stayed online and the settled closed-loop mean absolute error was {0:F1} RPM." -f $meanAbsError))
            $recommendations.Add("If the operator symptom remains, correlate it with balance/leg mode transitions rather than the basic BLDC RPM loop.")
        }
    }

    if(($sequenceGapCount -gt 0) -or ($dropDelta -gt 0)) {
        $findings.Add(("Telemetry integrity: {0} firmware sequence gaps, telemetry_drop_count delta {1}." -f $sequenceGapCount, $dropDelta))
    }

    return [pscustomobject]@{
        status = $status
        sample_count = $allRows.Count
        capture_duration_s = [Math]::Round($duration, 6)
        estimated_sample_hz = [Math]::Round($sampleHz, 3)
        feedback_online_ratio = [Math]::Round($onlineRatio, 6)
        mean_abs_rpm_error = if($null -eq $meanAbsError) { $null } else { [Math]::Round($meanAbsError, 3) }
        duty_saturation_ratio = if($null -eq $saturationRatio) { $null } else { [Math]::Round($saturationRatio, 6) }
        sign_mismatch_ratio = if($null -eq $signMismatchRatio) { $null } else { [Math]::Round($signMismatchRatio, 6) }
        one_side_missing_ratio = if($null -eq $oneSideMissingRatio) { $null } else { [Math]::Round($oneSideMissingRatio, 6) }
        frame_sequence_gap_count = $sequenceGapCount
        telemetry_drop_delta = $dropDelta
        imu_interrupt_delta = $imuInterruptDelta
        imu_invalid_delta = $imuInvalidDelta
        imu_age_max_ms = [Math]::Round($imuAgeMax, 3)
        findings = @($findings)
        recommendations = @($recommendations)
        limitation = "Current 72-float firmware telemetry exposes combined feedback_online but not UART1 checksum/unknown-frame counters or separate motor targets/integrals."
    }
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
if($RpmDutyLimit -le 0.0) {
    throw "RpmDutyLimit must be positive."
}

$schedule = @(Parse-CommandSchedule -Text $Commands)
Assert-CommandScheduleSafe -Schedule $schedule -AllowMotion:$AllowMotion

if($Out.Length -eq 0) {
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $Out = "data/bldc_diagnostic_$stamp.csv"
}
$outPath = [System.IO.Path]::GetFullPath($Out)
$outDir = [System.IO.Path]::GetDirectoryName($outPath)
if($outDir -and -not (Test-Path -LiteralPath $outDir)) {
    New-Item -ItemType Directory -Path $outDir | Out-Null
}

if($SummaryOut.Length -eq 0) {
    $SummaryOut = [System.IO.Path]::ChangeExtension($outPath, ".summary.json")
}
$summaryPath = [System.IO.Path]::GetFullPath($SummaryOut)
$summaryDir = [System.IO.Path]::GetDirectoryName($summaryPath)
if($summaryDir -and -not (Test-Path -LiteralPath $summaryDir)) {
    New-Item -ItemType Directory -Path $summaryDir | Out-Null
}

$serial = [System.IO.Ports.SerialPort]::new($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.ReadTimeout = 20
$serial.WriteTimeout = 1000
$serial.NewLine = "`n"

$writer = [System.IO.StreamWriter]::new($outPath, $false, [System.Text.Encoding]::UTF8)
$rxBuffer = New-Object System.Collections.Generic.List[byte]
$readBuffer = New-Object byte[] 1024
$rows = New-Object System.Collections.Generic.List[object]
$sampleIndex = 0
$commandIndex = 0
$lastCommand = ""
$lastCommandAt = 0.0
$expectation = Get-CommandExpectation -Command ""
$stopwatch = [System.Diagnostics.Stopwatch]::new()

try {
    try {
        $serial.Open()
    } catch {
        throw ("Could not open {0}. Close VOFA+/other serial tools and verify the port. {1}" -f $Port, $_.Exception.Message)
    }

    $writer.WriteLine($CsvFields -join ",")
    $serial.DiscardInBuffer()
    Reset-BldcTailLock
    $stopwatch.Start()

    Write-Host ("collecting 72-float BLDC diagnostics on {0} at {1} baud for {2:F2}s" -f $Port, $Baud, $Duration)
    if($schedule.Count -eq 0) {
        Write-Host "scheduled commands: none (listen-only)"
    } else {
        Write-Host ("scheduled commands: {0}" -f $Commands)
        if($AllowMotion) {
            Write-Warning "Motion commands are enabled. Keep both wheels safely suspended and stay ready to cut power."
        }
    }

    while($stopwatch.Elapsed.TotalSeconds -lt $Duration) {
        $elapsed = $stopwatch.Elapsed.TotalSeconds
        while(($commandIndex -lt $schedule.Count) -and ($elapsed -ge $schedule[$commandIndex].AtSeconds)) {
            $lastCommand = $schedule[$commandIndex].Command
            $lastCommandAt = $elapsed
            $expectation = Get-CommandExpectation -Command $lastCommand
            Send-Command -Serial $serial -Command $lastCommand
            Write-Host ("{0,8:F3}s -> {1}" -f $elapsed, $lastCommand)
            $commandIndex++
        }

        $count = Read-AvailableBytes -Serial $serial -ReadBuffer $readBuffer
        if($count -le 0) {
            continue
        }
        for($i = 0; $i -lt $count; $i++) {
            $rxBuffer.Add($readBuffer[$i])
        }

        foreach($frame in (Pop-BldcFrames -Buffer $rxBuffer)) {
            $now = [DateTime]::UtcNow
            $elapsedNow = $stopwatch.Elapsed.TotalSeconds
            $commandElapsed = if($lastCommand.Length -gt 0) { $elapsedNow - $lastCommandAt } else { 0.0 }
            $row = [pscustomobject]@{
                pc_time_s = ([DateTimeOffset]$now).ToUnixTimeMilliseconds() / 1000.0
                elapsed_s = $elapsedNow
                sample_index = $sampleIndex
                last_command = $lastCommand
                command_elapsed_s = $commandElapsed
                expected_mode = $expectation.Mode
                expected_motor_rpm = $expectation.TargetRpm
                expected_open_duty = $expectation.TargetDuty
                time_ms = $frame.time_ms
                balance_mode = $frame.balance_mode
                roll_deg = $frame.roll_deg
                pitch_deg = $frame.pitch_deg
                balance_rpm = $frame.balance_rpm
                feedback_online = $frame.feedback_online
                left_motor_rpm = $frame.left_motor_rpm
                right_motor_rpm = $frame.right_motor_rpm
                left_duty = $frame.left_duty
                right_duty = $frame.right_duty
                firmware_frame_sequence = $frame.firmware_frame_sequence
                telemetry_drop_count = $frame.telemetry_drop_count
                scheduler_missed_tick_count = $frame.scheduler_missed_tick_count
                scheduler_max_gap_ms = $frame.scheduler_max_gap_ms
                imu_int_count = $frame.imu_int_count
                imu_invalid_count = $frame.imu_invalid_count
                imu_age_ms = $frame.imu_age_ms
                note = $Note
            }
            $rows.Add($row)

            $values = foreach($field in $CsvFields) {
                $value = $row.$field
                if($value -is [double] -or $value -is [float]) {
                    if([double]::IsNaN([double]$value) -or [double]::IsInfinity([double]$value)) {
                        ""
                    } else {
                        ([double]$value).ToString("0.######", [System.Globalization.CultureInfo]::InvariantCulture)
                    }
                } else {
                    Convert-CsvField $value
                }
            }
            $writer.WriteLine($values -join ",")
            $sampleIndex++
        }
    }
} finally {
    if($serial.IsOpen) {
        if(-not $NoStopOnExit) {
            try {
                Send-Command -Serial $serial -Command "STOP"
            } catch {
                Write-Warning ("Could not send STOP on exit: {0}" -f $_.Exception.Message)
            }
        }
        $serial.Close()
    }
    $writer.Close()
}

$summary = Get-BldcDiagnosticSummary -Rows $rows -RpmDutyLimit $RpmDutyLimit
$summary | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $summaryPath -Encoding UTF8

Write-Host ("saved {0} samples to {1}" -f $sampleIndex, $outPath)
Write-Host ("summary status: {0}" -f $summary.status)
foreach($finding in $summary.findings) {
    Write-Host ("  finding: {0}" -f $finding)
}
foreach($recommendation in $summary.recommendations) {
    Write-Host ("  next: {0}" -f $recommendation)
}
Write-Host ("saved summary to {0}" -f $summaryPath)

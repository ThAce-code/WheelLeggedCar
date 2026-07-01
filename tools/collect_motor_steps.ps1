param(
    [string]$Port = "COM6",
    [int]$Baud = 460800,
    [string]$Sequence = "300:3,500:3,800:3,1000:3,1500:3,2000:3",
    [double]$Settle = 0.2,
    [double]$BetweenStop = 1.0,
    [string]$Out = "data/motor_step.csv"
)

$ErrorActionPreference = "Stop"

$Tail = [byte[]](0x00, 0x00, 0x80, 0x7F)
$FloatCount = 8
$PayloadLen = $FloatCount * 4
$FrameLen = $PayloadLen + $Tail.Length
$Fields = "pc_time_s,sample_index,command_duty,time_ms,mode,target_motor_rpm,left_motor_rpm,right_motor_rpm,left_duty,right_duty,feedback_online"

function Parse-StepSequence {
    param([string]$Text)

    $steps = New-Object System.Collections.Generic.List[object]
    foreach($item in $Text.Split(",")) {
        $part = $item.Trim()
        if($part.Length -eq 0) {
            continue
        }

        $duration = 3.0
        if($part.Contains(":")) {
            $pieces = $part.Split(":", 2)
            $duty = [int]$pieces[0]
            $duration = [double]$pieces[1]
        } else {
            $duty = [int]$part
        }

        if($duration -le 0.0) {
            throw "Step duration must be positive: $part"
        }

        $steps.Add([pscustomobject]@{ Duty = $duty; Duration = $duration })
    }

    if($steps.Count -eq 0) {
        throw "Sequence is empty."
    }

    return $steps
}

function Send-Command {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [string]$Command
    )

    $Serial.Write("$Command`n")
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

function Pop-Frames {
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
                mode = $values[1]
                target_motor_rpm = $values[2]
                left_motor_rpm = $values[3]
                right_motor_rpm = $values[4]
                left_duty = $values[5]
                right_duty = $values[6]
                feedback_online = $values[7]
            })
        }

        $Buffer.RemoveRange(0, $tailIndex + $Tail.Length)
    }

    return $frames
}

function Append-StepData {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [System.IO.StreamWriter]$Writer,
        [System.Collections.Generic.List[byte]]$RxBuffer,
        [int]$Duty,
        [double]$Duration,
        [double]$SettleTime,
        [int]$SampleIndex
    )

    Send-Command -Serial $Serial -Command "D,$Duty"
    $stepStart = [DateTime]::UtcNow
    $keepFrom = $stepStart.AddSeconds($SettleTime)
    $endTime = $stepStart.AddSeconds($Duration)
    $readBuffer = New-Object byte[] 512

    Write-Host ("step D,{0} for {1:F2}s" -f $Duty, $Duration)

    while([DateTime]::UtcNow -lt $endTime) {
        $count = $Serial.Read($readBuffer, 0, $readBuffer.Length)
        if($count -gt 0) {
            for($i = 0; $i -lt $count; $i++) {
                $RxBuffer.Add($readBuffer[$i])
            }

            foreach($frame in (Pop-Frames -Buffer $RxBuffer)) {
                $now = [DateTime]::UtcNow
                if($now -lt $keepFrom) {
                    continue
                }

                $pcTime = ([DateTimeOffset]$now).ToUnixTimeMilliseconds() / 1000.0
                $row = @(
                    ("{0:F6}" -f $pcTime),
                    $SampleIndex,
                    $Duty,
                    ("{0:F3}" -f $frame.time_ms),
                    ("{0:F3}" -f $frame.mode),
                    ("{0:F3}" -f $frame.target_motor_rpm),
                    ("{0:F3}" -f $frame.left_motor_rpm),
                    ("{0:F3}" -f $frame.right_motor_rpm),
                    ("{0:F3}" -f $frame.left_duty),
                    ("{0:F3}" -f $frame.right_duty),
                    ("{0:F3}" -f $frame.feedback_online)
                )
                $Writer.WriteLine($row -join ",")
                $SampleIndex++
            }
        }
    }

    return $SampleIndex
}

$steps = Parse-StepSequence -Text $Sequence
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
$sampleIndex = 0

try {
    $serial.Open()
    $writer.WriteLine($Fields)

    Send-Command -Serial $serial -Command "STOP"
    Start-Sleep -Seconds $BetweenStop
    $serial.DiscardInBuffer()

    foreach($step in $steps) {
        $sampleIndex = Append-StepData `
            -Serial $serial `
            -Writer $writer `
            -RxBuffer $rxBuffer `
            -Duty $step.Duty `
            -Duration $step.Duration `
            -SettleTime $Settle `
            -SampleIndex $sampleIndex

        $writer.Flush()
        Send-Command -Serial $serial -Command "STOP"
        Start-Sleep -Seconds $BetweenStop
    }
} finally {
    if($serial.IsOpen) {
        Send-Command -Serial $serial -Command "STOP"
        $serial.Close()
    }
    $writer.Close()
}

Write-Host "saved $sampleIndex samples to $outPath"

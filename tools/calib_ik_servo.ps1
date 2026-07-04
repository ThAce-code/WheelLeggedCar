param(
    [string]$Port = "COM6",
    [int]$Baud = 460800,
    [string]$Out = "",
    [string]$PointFile = "",
    [switch]$NoStop
)

$ErrorActionPreference = "Stop"

# ── VOFA frame constants (must match firmware telemetry) ──
$Tail = [byte[]](0x00, 0x00, 0x80, 0x7F)
$FloatCount = 58
$PayloadLen = $FloatCount * 4
$FrameLen = $PayloadLen + $Tail.Length

# ── Helper: locate tail in ring buffer ──
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

# ── Helper: pop one balanced frame ──
function Pop-Frame {
    param([System.Collections.Generic.List[byte]]$Buffer)
    $tailIdx = Find-Tail -Buffer $Buffer
    if($tailIdx -lt 0) { return $null }
    if($tailIdx -lt $PayloadLen) {
        $Buffer.RemoveRange(0, $tailIdx + $Tail.Length)
        return $null
    }
    $payloadStart = $tailIdx - $PayloadLen
    $payload = New-Object byte[] $PayloadLen
    for($i = 0; $i -lt $PayloadLen; $i++) {
        $payload[$i] = $Buffer[$payloadStart + $i]
    }
    $values = New-Object double[] $FloatCount
    for($i = 0; $i -lt $FloatCount; $i++) {
        $values[$i] = [BitConverter]::ToSingle($payload, $i * 4)
    }
    $Buffer.RemoveRange(0, $tailIdx + $Tail.Length)
    return @{
        leg_mode             = $values[38]
        leg_target_height_mm = $values[39]
        leg_actual_height_mm = $values[40]
        leg_height_norm      = $values[41]
        leg_ik_valid         = $values[46]
        leg_output_enable    = $values[47]
        servo0_target_deg    = $values[48]
        servo1_target_deg    = $values[49]
        servo2_target_deg    = $values[50]
        servo3_target_deg    = $values[51]
    }
}

# ── Read available bytes ──
function Read-Serial {
    param(
        [System.IO.Ports.SerialPort]$Port,
        [byte[]]$Buf
    )
    try { return $Port.Read($Buf, 0, $Buf.Length) }
    catch [System.TimeoutException] { return 0 }
}

# ── Drain all pending frames, return the latest ──
function Read-LatestFrame {
    param(
        [System.IO.Ports.SerialPort]$Port,
        [System.Collections.Generic.List[byte]]$Buffer,
        [double]$TimeoutS = 0.5
    )
    $readBuf = New-Object byte[] 512
    $deadline = (Get-Date).AddSeconds($TimeoutS)
    $latest = $null
    while((Get-Date) -lt $deadline) {
        $n = Read-Serial -Port $Port -Buf $readBuf
        if($n -gt 0) {
            for($i = 0; $i -lt $n; $i++) { $Buffer.Add($readBuf[$i]) }
        }
        while($true) {
            $f = Pop-Frame -Buffer $Buffer
            if($null -eq $f) { break }
            $latest = $f
        }
        Start-Sleep -Milliseconds 20
    }
    return $latest
}

# ── Predefined calibration point sets ──
$DefaultPoints = @(
    # (a0, a1, a2, a3, label)
    # Mid-height, center
    ,(90, 90, 90, 90, "mid_center")
    # Low height
    ,(80, 80, 80, 80, "low_center")
    # High height
    ,(100, 100, 100, 100, "high_center")
    # Low with slight forward lean
    ,(82, 82, 78, 78, "low_fwd")
    # Low with slight backward lean
    ,(78, 78, 82, 82, "low_bwd")
    # Mid with slight forward lean
    ,(92, 92, 88, 88, "mid_fwd")
    # Mid with slight backward lean
    ,(88, 88, 92, 92, "mid_bwd")
    # High with slight forward lean
    ,(102, 102, 98, 98, "high_fwd")
    # High with slight backward lean
    ,(98, 98, 102, 102, "high_bwd")
    # Left-right differential low
    ,(78, 82, 78, 82, "low_left")
    ,(82, 78, 82, 78, "low_right")
    # Mid differential
    ,(88, 92, 88, 92, "mid_left")
    ,(92, 88, 92, 88, "mid_right")
)

# ── Output file ──
if($Out.Length -eq 0) {
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $Out = "data/ik_calib_$stamp.csv"
}
$outPath = [System.IO.Path]::GetFullPath($Out)
$outDir = [System.IO.Path]::GetDirectoryName($outPath)
if($outDir -and -not (Test-Path $outDir)) {
    New-Item -ItemType Directory -Path $outDir | Out-Null
}

# ── Load point list ──
$points = @()
if($PointFile -and (Test-Path $PointFile)) {
    Write-Host "Loading points from $PointFile"
    Import-Csv $PointFile | ForEach-Object {
        $points += @{
            A0 = [double]$_.a0
            A1 = [double]$_.a1
            A2 = [double]$_.a2
            A3 = [double]$_.a3
            Label = $_.label
        }
    }
    Write-Host "  loaded $($points.Count) points"
} else {
    Write-Host "Using default $($DefaultPoints.Count)-point calibration set"
    foreach($p in $DefaultPoints) {
        $points += @{ A0 = $p[0]; A1 = $p[1]; A2 = $p[2]; A3 = $p[3]; Label = $p[4] }
    }
}

# ── Open serial ──
$serial = [System.IO.Ports.SerialPort]::new($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.ReadTimeout = 50
$serial.WriteTimeout = 1000
$serial.NewLine = "`n"
$rxBuf = New-Object System.Collections.Generic.List[byte]

# ── CSV header ──
$csvFields = "sample_id","label","cmd_a0_deg","cmd_a1_deg","cmd_a2_deg","cmd_a3_deg",
             "servo0_deg","servo1_deg","servo2_deg","servo3_deg",
             "ik_valid","leg_mode","measured_x_mm","measured_y_mm","note"
$writer = [System.IO.StreamWriter]::new($outPath, $false, [System.Text.Encoding]::UTF8)
$writer.WriteLine(($csvFields -join ","))

try {
    $serial.Open()
    $serial.DiscardInBuffer()
    Write-Host "`n============================================="
    Write-Host " IK Servo Calibration"
    Write-Host " Port: $Port  |  Baud: $Baud"
    Write-Host " Points: $($points.Count)"
    Write-Host " Output: $outPath"
    Write-Host "=============================================`n"

    $sampleId = 0

    foreach($pt in $points) {
        $a0 = $pt.A0; $a1 = $pt.A1; $a2 = $pt.A2; $a3 = $pt.A3
        $label = $pt.Label

        Write-Host ("── Sample {0} / {1}  [{2}] ──" -f ($sampleId + 1), $points.Count, $label)
        Write-Host ("    LIK,{0:F0},{1:F0},{2:F0},{3:F0}" -f $a0, $a1, $a2, $a3)

        # Send STOP first to clear state, then LIK
        $serial.WriteLine("STOP")
        Start-Sleep -Milliseconds 100
        $rxBuf.Clear()

        $cmd = "LIK,{0:F0},{1:F0},{2:F0},{3:F0}" -f $a0, $a1, $a2, $a3
        $serial.WriteLine($cmd)

        # Wait for servos to settle and read latest telemetry
        $frame = Read-LatestFrame -Port $serial -Buffer $rxBuf -TimeoutS 1.2

        if($null -eq $frame) {
            Write-Host "    [WARN] No telemetry frame received"
            $s0 = $s1 = $s2 = $s3 = 0
            $ikv = 0
            $lmode = 0
        } else {
            $s0    = "{0:F1}" -f $frame.servo0_target_deg
            $s1    = "{0:F1}" -f $frame.servo1_target_deg
            $s2    = "{0:F1}" -f $frame.servo2_target_deg
            $s3    = "{0:F1}" -f $frame.servo3_target_deg
            $ikv   = "{0:F0}" -f $frame.leg_ik_valid
            $lmode = "{0:F0}" -f $frame.leg_mode
            Write-Host ("    telemetry: servo=({0}, {1}, {2}, {3})  ik_valid={4}  leg_mode={5}" -f $s0, $s1, $s2, $s3, $ikv, $lmode)
        }

        # Prompt user for measured coordinates
        Write-Host "    --- Measure wheel-center (x_mm, y_mm) now ---"
        $mx = Read-Host "    measured_x_mm"
        $my = Read-Host "    measured_y_mm"
        $note = Read-Host "    note (optional)"

        if([string]::IsNullOrWhiteSpace($mx)) { $mx = "" }
        if([string]::IsNullOrWhiteSpace($my)) { $my = "" }
        if([string]::IsNullOrWhiteSpace($note)) { $note = "" }

        # Write CSV row
        $row = @(
            $sampleId, $label,
            ("{0:F1}" -f $a0), ("{0:F1}" -f $a1), ("{0:F1}" -f $a2), ("{0:F1}" -f $a3),
            $s0, $s1, $s2, $s3,
            $ikv, $lmode,
            $mx, $my,
            ('"' + $note.Replace('"', '""') + '"')
        ) -join ","
        $writer.WriteLine($row)
        [Console]::Out.Flush()

        Write-Host ("    recorded: x={0}, y={1}`n" -f $mx, $my)
        $sampleId++
    }

} finally {
    if($serial.IsOpen) {
        if(-not $NoStop) { $serial.WriteLine("STOP") }
        Start-Sleep -Milliseconds 50
        $serial.Close()
    }
    $writer.Close()
}

Write-Host "============================================="
Write-Host " Done.  $sampleId samples → $outPath"
Write-Host "============================================="

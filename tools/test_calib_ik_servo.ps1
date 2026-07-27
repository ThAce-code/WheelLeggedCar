$ErrorActionPreference = "Stop"

$scriptPath = Join-Path $PSScriptRoot "calib_ik_servo.ps1"
if((Get-Content -LiteralPath $scriptPath -Raw) -notmatch '\[switch\]\$LoadOnly') {
    throw "calibration parser must support -LoadOnly test loading"
}
. $scriptPath -LoadOnly

function Assert-True {
    param([bool]$Condition, [string]$Message)
    if(-not $Condition) {
        throw $Message
    }
}

function Assert-Near {
    param([double]$Actual, [double]$Expected, [double]$Tolerance, [string]$Message)
    if([Math]::Abs($Actual - $Expected) -gt $Tolerance) {
        throw ("{0}: actual={1}, expected={2}" -f $Message, $Actual, $Expected)
    }
}

function Add-JustFloatFrame {
    param(
        [System.Collections.Generic.List[byte]]$Buffer,
        [single[]]$FrameValues
    )

    foreach($value in $FrameValues) {
        foreach($byte in [BitConverter]::GetBytes($value)) {
            $Buffer.Add($byte)
        }
    }
    foreach($byte in [byte[]](0x00, 0x00, 0x80, 0x7F)) {
        $Buffer.Add($byte)
    }
}

function Add-Bytes {
    param(
        [System.Collections.Generic.List[byte]]$Buffer,
        [int]$Count,
        [byte]$Value = 0x55
    )

    for($i = 0; $i -lt $Count; $i++) {
        $Buffer.Add($Value)
    }
}

function Pop-AllCalibFrames {
    param([System.Collections.Generic.List[byte]]$Buffer)

    $frames = New-Object System.Collections.Generic.List[hashtable]
    while($true) {
        $beforeCount = $Buffer.Count
        $frame = Pop-Frame -Buffer $Buffer
        if($null -ne $frame) {
            $frames.Add($frame)
        }
        if($Buffer.Count -eq $beforeCount) {
            break
        }
    }
    return @($frames)
}

Assert-True ($FloatCount -eq 72) "calibration parser must require the 72-float UART0 contract"
$values = New-Object 'single[]' 72
$values[12] = 3.0
$values[18] = 88.0
$values[31] = 1.0

$firstValues = [single[]]$values.Clone()
$firstValues[18] = 77.0
$two72Buffer = New-Object System.Collections.Generic.List[byte]
Reset-CalibTailLock
Add-Bytes -Buffer $two72Buffer -Count 68
Add-JustFloatFrame -Buffer $two72Buffer -FrameValues $firstValues
Add-JustFloatFrame -Buffer $two72Buffer -FrameValues $values
$two72Frames = @(Pop-AllCalibFrames -Buffer $two72Buffer)
Assert-True ($two72Frames.Count -eq 1) "prefix plus two 72-float frames must decode only the second frame"
Assert-Near $two72Frames[0].servo0_output_deg 88.0 0.001 "second 72-float calibration frame"

$legacyValues = New-Object 'single[]' 55
$legacyPrefixBuffer = New-Object System.Collections.Generic.List[byte]
Reset-CalibTailLock
Add-Bytes -Buffer $legacyPrefixBuffer -Count 68
Add-JustFloatFrame -Buffer $legacyPrefixBuffer -FrameValues $legacyValues
Add-JustFloatFrame -Buffer $legacyPrefixBuffer -FrameValues $legacyValues
Assert-True (@(Pop-AllCalibFrames -Buffer $legacyPrefixBuffer).Count -eq 0) "68-byte prefix plus two legacy 55-float frames must not decode"

$continuousLegacyBuffer = New-Object System.Collections.Generic.List[byte]
Reset-CalibTailLock
Add-JustFloatFrame -Buffer $continuousLegacyBuffer -FrameValues $legacyValues
Add-JustFloatFrame -Buffer $continuousLegacyBuffer -FrameValues $legacyValues
Assert-True (@(Pop-AllCalibFrames -Buffer $continuousLegacyBuffer).Count -eq 0) "continuous legacy 55-float frames must not decode"

$corruptRecoveryBuffer = New-Object System.Collections.Generic.List[byte]
$corruptFirstValues = [single[]]$values.Clone()
$corruptSecondValues = [single[]]$values.Clone()
$corruptThirdValues = [single[]]$values.Clone()
$corruptFirstValues[18] = 70.0
$corruptSecondValues[18] = 80.0
$corruptThirdValues[18] = 90.0
Reset-CalibTailLock
Add-JustFloatFrame -Buffer $corruptRecoveryBuffer -FrameValues $corruptFirstValues
Add-Bytes -Buffer $corruptRecoveryBuffer -Count 12 -Value 0xA5
Add-JustFloatFrame -Buffer $corruptRecoveryBuffer -FrameValues $corruptSecondValues
Add-JustFloatFrame -Buffer $corruptRecoveryBuffer -FrameValues $corruptThirdValues
$recoveredFrames = @(Pop-AllCalibFrames -Buffer $corruptRecoveryBuffer)
Assert-True ($recoveredFrames.Count -eq 1) "corrupt interval plus two 72-float frames must decode only the re-synchronized frame"
Assert-Near $recoveredFrames[0].servo0_output_deg 90.0 0.001 "re-synchronized calibration frame"

Write-Host "calib_ik_servo parser tests passed"

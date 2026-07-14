$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$projectConfigDir = Join-Path $repoRoot 'project\iar\project_config'
$imagePath = Join-Path $repoRoot 'project\iar\project_config\mt9v03x_cm0plus_capture_service.ewx'
$ewdPath = Join-Path $repoRoot 'project\iar\project_config\cyt4bb7_cm_7_0.ewd'
$provenancePath = Join-Path $repoRoot 'docs\mt9v03x-capture-service-provenance.md'
$expectedHash = '508CD93B33730384804E55794C3A11819E904740B3EC60519318B989DCF6A299'
$failures = [System.Collections.Generic.List[string]]::new()

function Assert-True([bool]$Condition, [string]$Message)
{
    if(-not $Condition)
    {
        $script:failures.Add($Message)
    }
}

function Read-IntelHex([string]$Path)
{
    $base = [uint64]0
    $bytes = [System.Collections.Generic.SortedDictionary[uint64, byte]]::new()
    $startLinear = $null

    foreach($line in Get-Content -LiteralPath $Path)
    {
        if(-not $line.StartsWith(':'))
        {
            throw "Invalid Intel HEX line: $line"
        }

        $count = [Convert]::ToInt32($line.Substring(1, 2), 16)
        $address = [Convert]::ToInt32($line.Substring(3, 4), 16)
        $type = [Convert]::ToInt32($line.Substring(7, 2), 16)
        $record = for($index = 0; $index -lt ($count + 5); $index++)
        {
            [Convert]::ToInt32($line.Substring(1 + (2 * $index), 2), 16)
        }
        Assert-True ((($record | Measure-Object -Sum).Sum -band 0xFF) -eq 0) "Intel HEX checksum mismatch: $line"
        $data = $record[4..(3 + $count)]

        switch($type)
        {
            0 {
                for($index = 0; $index -lt $count; $index++)
                {
                    $bytes[$base + [uint64]$address + [uint64]$index] = [byte]$data[$index]
                }
            }
            2 { $base = [uint64]((([uint32]$data[0] -shl 8) -bor [uint32]$data[1]) -shl 4) }
            4 { $base = [uint64]((([uint32]$data[0] -shl 8) -bor [uint32]$data[1]) -shl 16) }
            5 {
                $startLinear = [uint32](([uint32]$data[0] -shl 24) -bor
                                        ([uint32]$data[1] -shl 16) -bor
                                        ([uint32]$data[2] -shl 8) -bor
                                         [uint32]$data[3])
            }
        }
    }

    return [pscustomobject]@{ Bytes = $bytes; StartLinear = $startLinear }
}

function Find-LittleEndianConstant($Bytes, [uint32]$Value)
{
    $pattern = [BitConverter]::GetBytes($Value)
    foreach($address in $Bytes.Keys)
    {
        if($Bytes.ContainsKey($address + 3) -and
           ($Bytes[$address] -eq $pattern[0]) -and
           ($Bytes[$address + 1] -eq $pattern[1]) -and
           ($Bytes[$address + 2] -eq $pattern[2]) -and
           ($Bytes[$address + 3] -eq $pattern[3]))
        {
            return $true
        }
    }
    return $false
}

Assert-True (Test-Path -LiteralPath $imagePath) 'Capture-service image is missing'
if(Test-Path -LiteralPath $imagePath)
{
    Assert-True ((Get-Item -LiteralPath $imagePath).Length -eq 56964) 'Capture-service file length changed'
    Assert-True ((Get-FileHash -Algorithm SHA256 -LiteralPath $imagePath).Hash -eq $expectedHash) 'Capture-service SHA-256 changed'
    $hex = Read-IntelHex $imagePath
    $hexAddresses = @($hex.Bytes.Keys)
    Assert-True ($hex.Bytes.Count -eq 20234) 'Capture-service Intel HEX payload byte count changed'
    Assert-True ($hexAddresses[0] -eq 0x10000000) 'Capture-service flash start is not 0x10000000'
    Assert-True ($hexAddresses[$hex.Bytes.Count - 1] -eq 0x10004F09) 'Capture-service flash end is not 0x10004F09'
    Assert-True ($hex.StartLinear -eq 0x10004C09) 'Capture-service start-linear address changed'
    foreach($constant in [uint32[]](0x28006BF0, 0x28006BF2, 0x28026024, 0x40581D80))
    {
        Assert-True (Find-LittleEndianConstant $hex.Bytes $constant) ('Capture-service constant 0x{0:X8} is missing' -f $constant)
    }
}

$ewd = Get-Content -Raw -LiteralPath $ewdPath
$captureServiceSetting = '$PROJ_DIR$\mt9v03x_cm0plus_capture_service.ewx'
$cm7_1Setting = '$PROJ_DIR$\..\Debug_m7_1\Exe\cyt4bb7_cm_7_1.hex'
Assert-True ($ewd -match [regex]::Escape("<state>$captureServiceSetting</state>")) 'CM7_0 image 1 is not the repository capture service'
Assert-True ($ewd -match [regex]::Escape("<state>$cm7_1Setting</state>")) 'CM7_0 image 2 is not the current CM7_1 HEX'
$resolvedCaptureService = $captureServiceSetting.Replace('$PROJ_DIR$', $projectConfigDir)
$resolvedCm7_1 = $cm7_1Setting.Replace('$PROJ_DIR$', $projectConfigDir)
Assert-True (Test-Path -LiteralPath $resolvedCaptureService -PathType Leaf) 'CM7_0 image 1 does not resolve to a file from the project directory'
Assert-True (Test-Path -LiteralPath $resolvedCm7_1 -PathType Leaf) 'CM7_0 image 2 does not resolve to a file from the project directory'
Assert-True (($ewd -split '<name>OCImagesUse1</name>', 2)[1] -match '<state>1</state>') 'CM7_0 image 1 is disabled'
Assert-True (($ewd -split '<name>OCImagesUse2</name>', 2)[1] -match '<state>1</state>') 'CM7_0 image 2 is disabled'

Assert-True (Test-Path -LiteralPath $provenancePath) 'Capture-service provenance document is missing'
if(Test-Path -LiteralPath $provenancePath)
{
    $provenance = Get-Content -Raw -LiteralPath $provenancePath
    Assert-True ($provenance.Contains($expectedHash)) 'Provenance omits the exact SHA-256'
    Assert-True ($provenance -match 'active CM7_0') 'Provenance omits the active-CM7_0 launch requirement'
    Assert-True ($provenance -match 'generic CM0\+') 'Provenance omits the generic-CM0+ overwrite warning'
}

if(0 -ne $failures.Count)
{
    $failures | ForEach-Object { Write-Host "FAIL: $_" -ForegroundColor Red }
    exit 1
}

Write-Host 'PASS: MT9V03X capture-service image, provenance, address contract, and CM7_0 loader configuration'

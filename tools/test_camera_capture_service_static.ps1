param(
    [string]$EwdPathOverride = ''
)

$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$projectConfigDir = Join-Path $repoRoot 'project\iar\project_config'
$imagePath = Join-Path $repoRoot 'project\iar\project_config\mt9v03x_cm0plus_capture_service.ewx'
$ewdPath = if([string]::IsNullOrWhiteSpace($EwdPathOverride))
{
    Join-Path $repoRoot 'project\iar\project_config\cyt4bb7_cm_7_0.ewd'
}
else
{
    $EwdPathOverride
}
$provenancePath = Join-Path $repoRoot 'docs\mt9v03x-capture-service-provenance.md'
$gplLicensePath = Join-Path $repoRoot 'libraries\doc\GPL-3.0.txt'
$expectedHash = '508CD93B33730384804E55794C3A11819E904740B3EC60519318B989DCF6A299'
$expectedGplHash = '0B383D5A63DA644F628D99C33976EA6487ED89AAA59F0B3257992DEAC1171E6B'
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

function Test-EwdLoaderContract([xml]$Xml, [bool]$CheckResolvedFiles)
{
    $issues = [System.Collections.Generic.List[string]]::new()
    $expectedOptions = [ordered]@{
        OCImagesPath1          = '$PROJ_DIR$\mt9v03x_cm0plus_capture_service.ewx'
        OCImagesUse1           = '1'
        OCImagesOffset1        = '0'
        OCImagesPath2          = '$PROJ_DIR$\..\Debug_m7_1\Exe\cyt4bb7_cm_7_1.hex'
        OCImagesUse2           = '1'
        OCImagesOffset2        = '0'
        OCDownloadExtraImage   = '1'
    }
    $states = @{}

    foreach($entry in $expectedOptions.GetEnumerator())
    {
        $nodes = @($Xml.SelectNodes("//option[name='$($entry.Key)']"))
        if(1 -ne $nodes.Count)
        {
            $issues.Add("EWD option $($entry.Key) must occur exactly once; found $($nodes.Count)")
            continue
        }

        $stateNodes = @($nodes[0].SelectNodes('./state'))
        if(1 -ne $stateNodes.Count)
        {
            $issues.Add("EWD option $($entry.Key) must contain exactly one state; found $($stateNodes.Count)")
            continue
        }

        $states[$entry.Key] = $stateNodes[0].InnerText
        if($entry.Value -cne $states[$entry.Key])
        {
            $issues.Add("EWD option $($entry.Key) expected '$($entry.Value)' but found '$($states[$entry.Key])'")
        }
    }

    if($CheckResolvedFiles -and
       $states.ContainsKey('OCImagesPath1') -and
       $states.ContainsKey('OCImagesPath2'))
    {
        foreach($pathOption in @('OCImagesPath1', 'OCImagesPath2'))
        {
            $resolved = $states[$pathOption].Replace('$PROJ_DIR$', $projectConfigDir)
            if(-not (Test-Path -LiteralPath $resolved -PathType Leaf))
            {
                $issues.Add("EWD option $pathOption does not resolve to an existing file: $resolved")
            }
        }
    }

    return $issues
}

function Set-UniqueEwdOptionState([xml]$Xml, [string]$Name, [string]$Value)
{
    $nodes = @($Xml.SelectNodes("//option[name='$Name']"))
    if((1 -ne $nodes.Count) -or (1 -ne @($nodes[0].SelectNodes('./state')).Count))
    {
        throw "Self-test fixture cannot uniquely mutate EWD option $Name"
    }
    $nodes[0].state = $Value
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

Assert-True (Test-Path -LiteralPath $ewdPath -PathType Leaf) 'CM7_0 debugger configuration is missing'
if(Test-Path -LiteralPath $ewdPath -PathType Leaf)
{
    [xml]$ewdXml = Get-Content -Raw -LiteralPath $ewdPath
    foreach($issue in (Test-EwdLoaderContract $ewdXml $true))
    {
        $failures.Add($issue)
    }

    [xml]$swappedSlots = $ewdXml.OuterXml
    Set-UniqueEwdOptionState $swappedSlots 'OCImagesPath1' '$PROJ_DIR$\..\Debug_m7_1\Exe\cyt4bb7_cm_7_1.hex'
    Set-UniqueEwdOptionState $swappedSlots 'OCImagesPath2' '$PROJ_DIR$\mt9v03x_cm0plus_capture_service.ewx'
    Assert-True ((Test-EwdLoaderContract $swappedSlots $false).Count -gt 0) 'EWD negative self-test accepted swapped image slots'

    foreach($offsetOption in @('OCImagesOffset1', 'OCImagesOffset2'))
    {
        [xml]$nonzeroOffset = $ewdXml.OuterXml
        Set-UniqueEwdOptionState $nonzeroOffset $offsetOption '1'
        Assert-True ((Test-EwdLoaderContract $nonzeroOffset $false).Count -gt 0) "EWD negative self-test accepted nonzero $offsetOption"
    }

    [xml]$disabledExtraImage = $ewdXml.OuterXml
    Set-UniqueEwdOptionState $disabledExtraImage 'OCDownloadExtraImage' '0'
    Assert-True ((Test-EwdLoaderContract $disabledExtraImage $false).Count -gt 0) 'EWD negative self-test accepted OCDownloadExtraImage=0'
}

Assert-True (Test-Path -LiteralPath $provenancePath) 'Capture-service provenance document is missing'
$gitAttributes = Get-Content -Raw -LiteralPath (Join-Path $repoRoot '.gitattributes')
Assert-True ($gitAttributes -match '(?m)^libraries/doc/GPL-3\.0\.txt\s+-text\s*$') 'GPLv3 license copy is not byte-preserved with -text'
Assert-True (Test-Path -LiteralPath $gplLicensePath -PathType Leaf) 'Complete GPLv3 license copy is missing from libraries/doc/GPL-3.0.txt'
if(Test-Path -LiteralPath $gplLicensePath -PathType Leaf)
{
    Assert-True ((Get-Item -LiteralPath $gplLicensePath).Length -eq 35821) 'GPLv3 license copy length changed'
    Assert-True ((Get-FileHash -Algorithm SHA256 -LiteralPath $gplLicensePath).Hash -eq $expectedGplHash) 'GPLv3 license copy hash changed'
}
if(Test-Path -LiteralPath $provenancePath)
{
    $provenance = Get-Content -Raw -LiteralPath $provenancePath
    Assert-True ($provenance.Contains($expectedHash)) 'Provenance omits the exact SHA-256'
    Assert-True ($provenance -match 'active CM7_0') 'Provenance omits the active-CM7_0 launch requirement'
    Assert-True ($provenance -match 'generic CM0\+') 'Provenance omits the generic-CM0+ overwrite warning'
    Assert-True ($provenance.Contains('D:\smartcar\CYT4BB7_Library\LICENSE')) 'Provenance omits the authoritative library-root GPLv3 source'
    Assert-True ($provenance.Contains($expectedGplHash)) 'Provenance omits the authoritative GPLv3 SHA-256'
    Assert-True ($provenance -match 'current local research') 'Provenance does not limit the vendor image to current local research'
    Assert-True ($provenance -match 'does not include the Corresponding Source or a build recipe') 'Provenance overstates available source/build materials'
    Assert-True ($provenance -match 'before any push or external distribution') 'Provenance omits the pre-distribution compliance gate'
    Assert-True ($provenance -match 'project owner must confirm') 'Provenance omits project-owner confirmation'
    Assert-True ($provenance -match 'does not assign a license to the repository as a whole') 'Provenance improperly implies a repository-wide license'
}

if(0 -ne $failures.Count)
{
    $failures | ForEach-Object { Write-Host "FAIL: $_" -ForegroundColor Red }
    exit 1
}

Write-Host 'PASS: MT9V03X capture-service image, provenance, address contract, and CM7_0 loader configuration'

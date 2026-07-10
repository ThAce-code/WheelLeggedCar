$ErrorActionPreference = "Stop"

function Get-LegTransitionConfig {
    $path = "project/code/leg_config.c"
    if(-not (Test-Path $path)) {
        throw ("Missing file: {0}" -f $path)
    }

    $text = Get-Content $path -Raw
    $names = @(
        "low_height_mm",
        "high_height_mm",
        "max_height_speed_mm_s",
        "max_height_accel_mm_s2",
        "height_settle_error_mm",
        "height_settle_ms",
        "ik_min_margin",
        "safe_support_height_mm"
    )
    $config = @{}

    foreach($name in $names) {
        $match = [regex]::Match($text, "\.$name\s*=\s*([-+]?\d+(?:\.\d+)?)(?:f|U)")
        if(-not $match.Success) {
            throw ("Missing named transition setting: {0}" -f $name)
        }
        $config[$name] = [double]$match.Groups[1].Value
    }

    $safeMatches = [regex]::Matches($text, "\{\d+,\s*([-+]?\d+(?:\.\d+)?)f,")
    if(4 -ne $safeMatches.Count) {
        throw "Expected four configured safe servo commands."
    }
    $config["safe_servo_deg"] = @($safeMatches | ForEach-Object { [double]$_.Groups[1].Value })

    return $config
}

function Assert-Equal {
    param(
        [double]$Actual,
        [double]$Expected,
        [string]$Message
    )

    if($Actual -ne $Expected) {
        throw ("{0}: expected {1}, got {2}" -f $Message, $Expected, $Actual)
    }
}

$config = Get-LegTransitionConfig
Assert-Equal -Actual $config["low_height_mm"] -Expected 35.0 -Message "Low calibrated height"
Assert-Equal -Actual $config["high_height_mm"] -Expected 120.0 -Message "High calibrated height"
Assert-Equal -Actual $config["max_height_speed_mm_s"] -Expected 20.0 -Message "Maximum height speed"
Assert-Equal -Actual $config["max_height_accel_mm_s2"] -Expected 40.0 -Message "Maximum height acceleration"
Assert-Equal -Actual $config["height_settle_error_mm"] -Expected 1.0 -Message "Height settle error"
Assert-Equal -Actual $config["height_settle_ms"] -Expected 300.0 -Message "Height settle time"
Assert-Equal -Actual $config["ik_min_margin"] -Expected 0.20 -Message "IK minimum margin"
Assert-Equal -Actual $config["safe_support_height_mm"] -Expected 80.0 -Message "Provisional safe support height"

foreach($safeServoDeg in $config["safe_servo_deg"]) {
    Assert-Equal -Actual $safeServoDeg -Expected 90.0 -Message "Safe servo command"
}

if(0.0 -ge $config["max_height_accel_mm_s2"]) {
    throw "Maximum height acceleration must be positive."
}
if((0.0 -ge $config["ik_min_margin"]) -or (1.0 -le $config["ik_min_margin"])) {
    throw "IK minimum margin must be strictly between zero and one."
}

Write-Host "leg transition numeric checks passed"

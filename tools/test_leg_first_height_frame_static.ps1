$ErrorActionPreference = "Stop"

$source = Get-Content "project/code/control_leg.c" -Raw
$entryPattern = "uint8 control_leg_set_legacy_stance\(float stance_units, uint32 now_ms\)[\s\S]*?if\([\s\S]*?LEG_MODE_LEGACY_STANCE != control_leg_mode[\s\S]*?\)[\s\S]*?control_leg_last_update_ms = now_ms;"

if($source -notmatch $entryPattern)
{
    throw "First LH legacy-stance command after LOCK must reset the trajectory timestamp before its first integration frame."
}

Write-Host "leg first-legacy-stance-frame static check passed"

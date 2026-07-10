$ErrorActionPreference = "Stop"

$source = Get-Content "project/code/control_leg.c" -Raw
$entryPattern = "uint8 control_leg_set_height\(float height_mm, uint32 now_ms\)[\s\S]*?if\([\s\S]*?LEG_MODE_HEIGHT != control_leg_mode[\s\S]*?\)[\s\S]*?control_leg_last_update_ms = now_ms;"

if($source -notmatch $entryPattern)
{
    throw "First LH command after LOCK must reset the trajectory timestamp before its first integration frame."
}

Write-Host "leg first-height-frame static check passed"

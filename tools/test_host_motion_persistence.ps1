$ErrorActionPreference = "Stop"

function Require-Pattern {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )

    if($Text -notmatch $Pattern) {
        throw $Message
    }
}

$config = Get-Content "project/code/app_config.h" -Raw
$chassis = Get-Content "project/code/control_chassis.c" -Raw

Require-Pattern $config 'APP_HOST_COMMAND_TIMEOUT_MS\s+\(0U\)' 'M/D host motion must persist until an explicit stop.'
Require-Pattern $config 'APP_CHASSIS_CMD_TIMEOUT_MS\s+\(0U\)' 'C host motion must persist until an explicit stop.'
Require-Pattern $chassis '#if\s+\(0U\s*!=\s*APP_CHASSIS_CMD_TIMEOUT_MS\)[\s\S]*APP_CHASSIS_CMD_TIMEOUT_MS\s*<\s*\(now_ms - control_chassis_cmd\.last_cmd_ms\)[\s\S]*#endif' 'A zero C timeout must disable stale-command stopping instead of expiring immediately.'

Write-Host "host motion persistence checks passed"

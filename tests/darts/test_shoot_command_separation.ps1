$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$dartsDef = Get-Content -Raw -Encoding UTF8 (Join-Path $repoRoot "apps\darts\darts_def.h")
$robotFunc = Get-Content -Raw -Encoding UTF8 (Join-Path $repoRoot "apps\darts\single_board\robot_func\robot_func.c")
$shootMode = Get-Content -Raw -Encoding UTF8 (Join-Path $repoRoot "apps\darts\single_board\shoot_func\shoot_mode.c")

if ($dartsDef -notmatch 'typedef\s+enum\s*\{[^}]*shoot_stop[^}]*shoot_start[^}]*\}\s*shoot_cmd_e' -or
    $dartsDef -notmatch 'shoot_cmd_e\s+shoot_cmd') {
    throw "shoot start command is not separated from shoot mode"
}

if ($robotFunc -match 'shoot_ctrl->shoot_mode' -or $robotFunc -match 'shoot_ctrl->shoot_cmd\s*=\s*shoot_fire') {
    throw "remote control writes the physical fire state"
}

if ($shootMode -notmatch 'shoot_cmd->shoot_cmd\s*!=\s*shoot_start') {
    throw "shoot state machine does not consume shoot_start"
}

Write-Output "shoot command separation tests passed"

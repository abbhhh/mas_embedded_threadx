$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$shootMode = Get-Content -Raw -Encoding UTF8 (Join-Path $repoRoot "apps\darts\single_board\shoot_func\shoot_mode.c")
$shootFunc = Get-Content -Raw -Encoding UTF8 (Join-Path $repoRoot "apps\darts\single_board\shoot_func\shoot_func.c")

$span = '125\.66370614f'

if ($shootMode -notmatch "left_span\s*=\s*$span" -or
    $shootMode -notmatch "right_span\s*=\s*-$span" -or
    $shootMode -notmatch "left_home_angle\s*\+\s*$span" -or
    $shootMode -notmatch "right_home_angle\s*-\s*$span") {
    throw "shoot mode does not use the 20-turn left-positive/right-negative target"
}

if ($shootFunc -notmatch "shoot_mode_get_left_home_angle\(\)\s*\+\s*$span" -or
    $shootFunc -notmatch "shoot_mode_get_right_home_angle\(\)\s*-\s*$span") {
    throw "motor output does not use the 20-turn left-positive/right-negative target"
}

Write-Output "sync motor angle mapping tests passed"

$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$shootFunc = Get-Content -Raw -Encoding UTF8 (Join-Path $repoRoot "apps\darts\single_board\shoot_func\shoot_func.c")

$positiveTarget = 'Motor_DJI_SetRef\(friction_r,\s*trigger_station\);'
$negativeTarget = 'Motor_DJI_SetRef\(friction_r,\s*-trigger_station\);'

if ($shootFunc -notmatch 'static float\s+friction_l_home_angle;' -or
    $shootFunc -notmatch 'static float\s+friction_r_home_angle;' -or
    $shootFunc -notmatch 'friction_l_home_angle\s*=\s*friction_l->base\.measure\.total_angle;' -or
    $shootFunc -notmatch 'friction_r_home_angle\s*=\s*friction_r->base\.measure\.total_angle;' -or
    [regex]::Matches($shootFunc, 'Motor_DJI_SetRef\(friction_l,\s*friction_l_home_angle\s*\+\s*trigger_station\);').Count -ne 4 -or
    [regex]::Matches($shootFunc, 'Motor_DJI_SetRef\(friction_r,\s*friction_r_home_angle\s*\+\s*trigger_station\);').Count -ne 4 -or
    $shootFunc -match $negativeTarget) {
    throw "right sync motor does not use the same positive trigger target as the left motor"
}

Write-Output "sync motor angle mapping tests passed"

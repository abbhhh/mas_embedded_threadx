$ErrorActionPreference = 'Stop'

$workspace = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$shootFunc = Join-Path $workspace 'apps\darts\single_board\shoot_func\shoot_func.c'
$content = Get-Content -Raw -Encoding UTF8 $shootFunc

if ($content -notmatch 'Motor_Servo_SetRef\(trigger,\s*5(?:\.0f)?\);\s*Darts_Servo_Sequence_Init') {
    throw 'Trigger initial angle is not 5 degrees'
}

if ($content -notmatch 'case\s+shoot_lock:\s*Motor_Servo_SetRef\(trigger,\s*90(?:\.0f)?\);') {
    throw 'shoot_lock does not set trigger to 90 degrees'
}

foreach ($mode in 'shoot_off', 'shoot_fire', 'shoot_finished', 'shoot_error') {
    if ($content -notmatch "case\s+${mode}:\s*Motor_Servo_SetRef\(trigger,\s*5(?:\.0f)?\);") {
        throw "$mode does not set trigger to 5 degrees"
    }
}

Write-Host 'trigger angle mapping tests passed'

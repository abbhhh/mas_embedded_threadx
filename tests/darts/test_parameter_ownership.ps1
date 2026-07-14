$ErrorActionPreference = 'Stop'

$workspace = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$dartsDef = Join-Path $workspace 'apps\darts\darts_def.h'
$content = Get-Content -Raw -Encoding UTF8 $dartsDef

$forbiddenPatterns = @(
    'DARTS_CONTROL_',
    'DARTS_REMOTE_',
    'DARTS_SYNC_MOTOR_MAX_',
    'DARTS_SYNC_MOTOR_OFFLINE_',
    'DARTS_SYNC_(LEFT|RIGHT)_CAN_ID',
    'DARTS_SYNC_(LEFT|RIGHT)_BEEP_TIMES',
    'DARTS_SYNC_(LEFT|RIGHT)_(ANGLE|SPEED)_',
    'DARTS_ORIGIN_',
    'DARTS_POSITION_',
    'DARTS_SYNC_PROGRESS_',
    'DARTS_SYNC_ERROR_',
    'DARTS_(COCK|HOME|RELOAD|SALVO)_TIMEOUT_',
    'DARTS_SERVO_',
    'DARTS_(RISE|TRANSFER|GRIPPER|TRIGGER)_(MIN|MAX)_ANGLE',
    'DARTS_TRIGGER_(LOCK_SETTLE|FIRE_DWELL)_MS',
    'DARTS_RELOAD[0-2]_STEP[0-9]_WAIT_MS'
)

$violations = foreach ($pattern in $forbiddenPatterns) {
    if ($content -match $pattern) { $pattern }
}

if ($violations.Count -gt 0) {
    throw "darts_def.h 仍包含模块内部参数: $($violations -join ', ')"
}

$requiredPatterns = @(
    'shoot_mode_e',
    'loader_mode_e',
    'Shoot_Ctrl_Cmd_t'
)

foreach ($pattern in $requiredPatterns) {
    if ($content -notmatch $pattern) {
        throw "darts_def.h 缺少机器人级定义: $pattern"
    }
}

Write-Host 'darts 参数归属检查通过'

$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$shootFunc = Get-Content -Raw (Join-Path $repoRoot "apps\darts\single_board\shoot_func\shoot_func.c")
$servoFunc = Get-Content -Raw (Join-Path $repoRoot "apps\darts\single_board\darts_servo_func\darts_servo_func.c")

if ($shootFunc -notmatch 'Motor_Servo_SetRef\(rise_left,\s*145(?:\.0f)?\s*\)') {
    throw "rise_left startup angle is not 145 degrees"
}

$nonZeroRiseAngles = [regex]::Matches(
    $servoFunc,
    'DARTS_SERVO_RISE_[LR]\]\s*=\s*(?!0(?:\.0f)?\b)([0-9]+(?:\.[0-9]+)?f?)'
)

if ($nonZeroRiseAngles.Count -eq 0) {
    throw "no non-zero rise servo target was found"
}

foreach ($match in $nonZeroRiseAngles) {
    if ($match.Groups[1].Value -ne "145.0f") {
        throw "rise servo target is not 145 degrees: $($match.Groups[1].Value)"
    }
}

Write-Output "rise servo angle mapping tests passed"

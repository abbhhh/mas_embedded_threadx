# Darts Double-Salvo Firing Logic Design

## Objective

Replace the current manual loader-mode logic with a non-blocking state machine that fires four darts through two remote-control commands.

The system starts with four darts in fixed locations:

- Dart A is already on the dart plate.
- Dart B is held by the gripper.
- Dart C is stored on the left.
- Dart D is stored on the right.

Each valid channel-8 command fires two darts. After the fourth dart fires, the system enters a permanent completed state until the controller restarts.

## Hardware Scope

The firing state machine controls:

- Two M3508 motors that pull the dart plate together.
- Two lift servos.
- One transfer servo.
- One gripper servo.
- One trigger servo.

The third `loader` M3508 from the current implementation is removed. It has no role in the new firing process.

No limit switch is used. Each M3508 uses its encoder-based multi-turn angle. The mechanism must be placed at its mechanical home position before every power-up.

## Remote Command

SBUS channel 8 provides the firing command.

- The upper position arms the command detector.
- A transition from the upper position to either active position creates one request.
- Holding an active position never creates another request.
- Channel 8 must return to the upper position before another request can be generated.
- The first request starts salvo 0. The second request starts salvo 1.
- Further requests are ignored after salvo 1 completes.

`RemoteControlSet()` emits a one-cycle `fire_request` event. `shoot_func()` consumes it in the same control-task iteration.

## Startup And Homing

At startup, the state machine waits until both M3508 motors:

- are online;
- remain below the configurable stationary-speed threshold; and
- remain stable for the configurable origin-capture duration.

It then records each motor's current `base.measure.total_angle` as its independent home angle. This is a runtime origin and is not persisted.

The accumulated DJI encoder angle is measured at the motor rotor. A mechanical output angle must therefore be converted using the configured reduction ratio.

If either motor becomes unavailable before origin capture completes, the state machine continues waiting. Once firing has started, an offline motor is a latched fault.

## Four-Dart Mission

The mission uses a fixed shot plan:

| Salvo | Shot | Dart | Reload before firing |
|---|---:|---|---|
| 0 | 0 | A | None |
| 0 | 1 | B | Sequence 0 |
| 1 | 0 | C | Sequence 1 |
| 1 | 1 | D | Sequence 2 |

Salvo 0 starts on the first valid remote request. Salvo 1 starts on the second valid request after salvo 0 has completed.

## Reusable Shot Sequence

Every shot executes the same non-blocking state sequence:

1. Command both synchronization motors from their home angles to their independent cocking targets.
2. Wait until both motors are at the cocking targets.
3. Move the trigger servo to the lock position.
4. Wait for the trigger-lock settling time.
5. Command both synchronization motors back to their captured home angles.
6. Wait until both motors are at home.
7. If the shot plan specifies a reload sequence, start it and wait for completion.
8. Move the trigger servo to the fire position.
9. Wait for the configured firing dwell.
10. Mark the shot complete.

The trigger remains in the fire position after every shot. It moves back to the lock position only after the next cocking movement reaches its target. This includes the pause between the two salvos.

The synchronization motors remain at their home targets during reload and firing.

## Motor Position Control

Both M3508 motors use encoder feedback with `ANGLE_AND_SPEED_LOOP` and `CONTROL_PID`.

Each motor has independent configuration for:

- direction sign;
- home-to-cocking travel;
- angle PID gains and limits;
- speed PID gains and limits; and
- maximum torque.

The target equations are:

```text
left_cocking_target  = left_home  + left_direction  * left_travel_rad
right_cocking_target = right_home + right_direction * right_travel_rad
```

A position is accepted only when all conditions remain true for the configured stable duration:

- the left position error is within tolerance;
- the right position error is within tolerance;
- both absolute motor speeds are below the speed tolerance; and
- normalized left and right travel progress differs by no more than the synchronization tolerance.

Every movement has a timeout. A zero or invalid travel configuration prevents firing and raises a configuration fault.

## Reload Sequences

The existing non-blocking servo sequence engine remains responsible for reload sequences 0, 1, and 2.

- Sequence 0 loads dart B from the gripper.
- Sequence 1 transfers dart C from the left storage position.
- Sequence 2 transfers dart D from the right storage position.

Servo target angles are robot-level mechanical calibration values in `apps/darts/darts_def.h`. Reload sequence tables and their step wait times live in `darts_servo_func.c`.

Reload-sequence completion advances the shot state. A sequence error or timeout raises a latched firing fault.

## Mode Model

The implementation follows the existing sentry naming style. It uses separate `shoot_mode` and `load_mode` values instead of generic state and action enums.

```c
typedef enum
{
    shoot_off = 0,
    shoot_lock,
    shoot_fire,
    shoot_finished,
    shoot_error,
} shoot_mode_e;

typedef enum
{
    load_origin = 0,
    load_stop,
    load_cock,
    load_return,
    load_reload,
} loader_mode_e;
```

The mechanical phases are represented by mode combinations:

| Phase | `shoot_mode` | `load_mode` |
|---|---|---|
| Wait for encoder origin | `shoot_off` | `load_origin` |
| Wait for remote request | `shoot_off` | `load_stop` |
| Pull the dart plate | `shoot_off` | `load_cock` |
| Lock the dart plate | `shoot_lock` | `load_cock` |
| Return synchronization motors | `shoot_lock` | `load_return` |
| Run an optional reload sequence | `shoot_lock` | `load_reload` |
| Fire the trigger | `shoot_fire` | `load_stop` |
| Four darts complete | `shoot_finished` | `load_stop` |
| Latched fault | `shoot_error` | `load_stop` |

The mode module owns salvo count, shot count, reload selection, timing, feedback checks, and fault reason. It updates file-level `shoot_mode` and `load_mode` values. It does not return an `output.action` command.

The first shot skips `load_reload`. The other shots select reload sequences 0, 1, and 2 through the fixed mission table. Counters advance directly when `shoot_fire` completes; separate shot-finished and salvo-finished modes are not used.

## Fault Handling

The following conditions enter `FAULT`:

- either M3508 goes offline after startup;
- either movement exceeds its timeout;
- left and right progress differs beyond the synchronization limit;
- a target or PID configuration is invalid;
- a reload sequence reports an error;
- a reload sequence exceeds its timeout; or
- an unexpected state or shot-plan index is detected.

Entering `FAULT` performs these actions once:

- cancel the active servo sequence;
- set both M3508 references to their current positions;
- stop both M3508 motors so their transmitted current becomes zero;
- leave all servos at their current targets to avoid an unverified mechanical movement; and
- latch the fault code for diagnostics.

Remote commands cannot clear a fault. Recovery requires a controller restart and manual verification that the mechanism is at home.

## Parameter Ownership

Parameter placement follows the sentry application style. Robot-wide definitions contain mechanical calibration and public command types only. Module-internal control values stay beside the code that uses them.

`apps/darts/darts_def.h` contains:

- M3508 reduction ratio;
- left and right motor direction signs;
- left and right cocking travel;
- lift, transfer, gripper, and trigger calibration angles;
- `shoot_mode_e`, `loader_mode_e`, and fault enums; and
- `Shoot_Ctrl_Cmd_t`.

`robot_control.c` contains control-task stack size, priority, and period.

`robot_func.c` contains the selected remote channel and directly uses the shared SBUS switch constants.

`shoot_func.c` contains:

- CAN IDs, offline timeouts, and beep identifiers;
- M3508 torque limits and torque constants;
- left and right angle-speed PID values; and
- PWM pulse limits and servo working ranges.

`shoot_mode.c` contains:

- origin and position stability thresholds;
- position, speed, and synchronization tolerances;
- cocking, homing, reload, and salvo timeouts; and
- trigger lock and fire dwell times.

`darts_servo_func.c` contains reload-sequence step wait times.

Unknown travel and PID values remain zero until bench calibration. Configuration validation must prevent motor movement while these values are invalid.

## Application Integration

The darts single-board application becomes firing-only for this change:

- `robot_control.c` reads the remote command and calls `shoot_func()` every two ThreadX ticks.
- Unused sentry, board-communication, INS, vision, and gimbal references are removed from the firing control path.
- `Shoot_Ctrl_Cmd_t` uses `shoot_mode` with sentry-style naming. The remote layer produces a single `shoot_fire` request only after channel 8 has returned to its armed position.
- `shoot_mode.c/.h` owns mode transitions, counters, timing, feedback judgement, and fault latching.
- `darts_servo_func.c/.h` owns the three reload-sequence tables and the non-blocking servo sequence engine.
- `shoot_func.c` contains only two function definitions: `shoot_init()` and `shoot_func()`.
- `shoot_init()` contains the actual two-M3508 and five-servo initialization, matching the sentry module style.
- `shoot_func()` directly uses `switch (shoot_mode)` and `switch (load_mode)` to call `Motor_DJI_Start`, `Motor_DJI_Stop`, `Motor_DJI_SetRef`, `Motor_Servo_SetRef`, and the servo-sequence API.
- No `output.action`, generic state enum, or state-machine wrapper structure appears in `shoot_func.c`.
- `gimbal_func.c` may remain empty, but it is no longer called by the darts firing task.

## Verification

Verification must include:

1. A successful full firmware build for the configured `darts/single` target.
2. Static checks that no third `loader` M3508 remains in the firing module.
3. Host-side mode-transition tests covering all four shots.
4. Confirmation that a held remote switch produces only one request.
5. Confirmation that the shot plan maps reload sequences as `none, 0, 1, 2`.
6. Fault-path checks for motor offline, motion timeout, synchronization error, and reload error.
7. Confirmation that the fifth remote request is ignored after `shoot_finished`.
8. Bench testing without darts before any live firing test.

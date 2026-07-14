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

All servo target angles and all step wait times move to `apps/darts/darts_def.h`. The sequence tables in `shoot_func.c` reference only named macros.

Reload-sequence completion advances the shot state. A sequence error or timeout raises a latched firing fault.

## State Model

The firing state machine contains these states:

```text
WAIT_ORIGIN
IDLE
MOVE_TO_COCK
WAIT_COCK
TRIGGER_LOCK
WAIT_TRIGGER_LOCK
MOVE_TO_HOME
WAIT_HOME
START_RELOAD
WAIT_RELOAD
TRIGGER_FIRE
WAIT_FIRE
SHOT_FINISHED
SALVO_FINISHED
COMPLETED
FAULT
```

State entry performs commands once. Periodic state updates check feedback, elapsed time, and fault conditions without blocking the ThreadX control task.

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

## Configurable Parameters

All unverified mechanical and control values are macros in `apps/darts/darts_def.h`, grouped by purpose:

- task period and remote channel values;
- origin capture speed and stable time;
- left and right direction signs;
- left and right cocking travel;
- M3508 reduction ratio and torque limit;
- left and right angle PID parameters;
- left and right speed PID parameters;
- cocking and home position tolerances;
- low-speed and synchronization tolerances;
- stable-position duration;
- cocking, homing, reload, and total-salvo timeouts;
- trigger lock and fire angles;
- trigger lock and fire dwell times;
- lift, transfer, and gripper angles for every reload step; and
- wait time for every reload step.

Initial macro values are conservative placeholders. Hardware testing must tune travel, PID gains, tolerances, servo angles, and timing before live firing.

## Application Integration

The darts single-board application becomes firing-only for this change:

- `robot_control.c` reads the remote command and calls `shoot_func()` every two ThreadX ticks.
- Unused sentry, board-communication, INS, vision, and gimbal references are removed from the firing control path.
- `Shoot_Ctrl_Cmd_t` contains the fire-request event needed by the new state machine.
- Obsolete `friction_mode` and manual `loader_mode_e` control paths are removed.
- `gimbal_func.c` may remain empty, but it is no longer called by the darts firing task.

## Verification

Verification must include:

1. A successful full firmware build for the configured `darts/single` target.
2. Static checks that no third `loader` M3508 remains in the firing module.
3. State-transition tests or a host-side state-machine harness covering all four shots.
4. Confirmation that a held remote switch produces only one request.
5. Confirmation that the shot plan maps reload sequences as `none, 0, 1, 2`.
6. Fault-path checks for motor offline, motion timeout, synchronization error, and reload error.
7. Confirmation that the fifth remote request is ignored after `COMPLETED`.
8. Bench testing without darts before any live firing test.

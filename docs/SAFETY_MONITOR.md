# Safety Monitor (shared)

Latches a stop on a critical fault or collision risk, independently of the Vehicle Controller (design rule 3, FR11). The latch is motion control's own (`motion_control_latch()`), kept in that one place, so no task can drive the car until the stop is cleared.

| File (`app_program/shared/`) | Contents                                                  |
| ---------------------------- | --------------------------------------------------------- |
| `safety_monitor.c/.h`        | Stop API (I11) and task `safety`, priority 5, every 10 ms, any core |

`usermain()` starts it in normal builds only, before the controller: the `TEST=1` bench sequences drive without a safety latch.

## Checks

Every 10 ms the task latches a stop for any of these. The cause is printed with the first stop only:

| Cause           | Value | Condition                                                                                                   |
| --------------- | ----- | ----------------------------------------------------------------------------------------------------------- |
| `IMU`           | 1     | `s4_get_fault()` reports an impact or excessive tilt (I9)                                                    |
| `COLLISION`     | 2     | Motion `RUNNING` a forward move, or a drive with `left_speed + right_speed > 0`, and a forward reading under 150 mm that is under 200 ms old (I8) |
| `STALL`         | 4     | Motion `FAILED`: a wheel stalled during a move, turn or drive                                                |
| `CONTROLLER`    | 8     | Raised by the Vehicle Controller for its own stops: GP20, an avoidance safe stop, a failed junction move     |

The 150 mm limit is a backstop inside avoidance's 350 mm, which stays the normal reaction to an obstacle. Turns are never checked, so the look turns during profiling cannot trip it. Forward readings are only taken while avoidance runs; when they stop, the check ignores the old reading.

## API

| Call                          | Effect                                                                                       |
| ----------------------------- | -------------------------------------------------------------------------------------------- |
| `safety_monitor_stop(cause)`  | If not latched yet: `motion_control_latch(true)` (brake and refuse every command) and print the cause |
| `motion_control_latched()`    | True while a stop is latched                                                                  |
| `safety_monitor_clear()`      | Release the latch. Returns false, and stays latched, while an IMU fault is set (those latch until reset) |

The controller enters SAFE_STOP whenever the latch is set, and a GP20 press there calls `safety_monitor_clear()`. Moving again then needs a second press (FR11).

## Console

```
[safety] stop, cause 0x2
[vc] FOLLOW -> SAFE_STOP: safety latched
[safety] cleared
```

## Calibration

`SAFETY_MONITOR_CLEAR_MM` (150) must leave room to brake from line-following speed, and stay below avoidance's `SCAN_PROFILE_BLOCK_MM` (350) so it only acts when avoidance has not.

## Period

The design's 5 ms period needs a 1 ms kernel tick. With the shipped 10 ms tick the monitor runs every 10 ms, like the other control tasks.

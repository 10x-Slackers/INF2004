# Safety Monitor (shared)

Latches a stop on a critical fault, independently of the Vehicle Controller (design rule 3, FR11). The latch is motion control's own (`motion_control_latch()`), kept in that one place, so no task can drive the car until the stop is cleared.

| File (`app_program/shared/`) | Contents                                                  |
| ---------------------------- | --------------------------------------------------------- |
| `safety_monitor.c/.h`        | Stop API (I11) and task `safety`, priority 5, every 10 ms, any core |

`usermain()` starts it in normal builds only, before the controller: the `TEST=1` bench sequences drive without a safety latch.

## Checks

Every 10 ms the task latches a stop for any of these. The cause is printed with the first stop only:

| Cause           | Value | Condition                                                                                                   |
| --------------- | ----- | ----------------------------------------------------------------------------------------------------------- |
| `IMU`           | 1     | `APP_FAULT_IMPACT` or `APP_FAULT_TILT` is set in the shared fault bitmask (I9)                               |
| `STALL`         | 4     | Motion `FAILED`: a wheel stalled during a move, turn or drive                                                |
| `CONTROLLER`    | 8     | Raised by the Vehicle Controller for its own stops: GP20, an avoidance safe stop, a lost line, a failed junction move     |

Collision risk is not checked here: obstacle avoidance stops on the first forward reading within 350 mm and safe-stops when no side is open, and its safe stop latches through the controller (`CONTROLLER`). It reads the range only while following and on the straight bypass steps, so nothing checks it during turns (look turns, bypass turns, junction turns). A backstop on the forward range cannot act first: avoidance brakes on the same reading, and only avoidance requests readings, so without it the range goes stale.

## API

| Call                          | Effect                                                                                       |
| ----------------------------- | -------------------------------------------------------------------------------------------- |
| `safety_monitor_stop(cause)`  | `motion_control_latch(true)` (brake and refuse every command); if it was not latched yet, print the cause |
| `motion_control_latched()`    | True while a stop is latched                                                                  |
| `safety_monitor_clear()`      | Release the latch. Returns false, and stays latched, while an IMU fault is set (those latch until reset) |

The controller enters SAFE_STOP whenever the latch is set, and a GP20 press there calls `safety_monitor_clear()`. Moving again then needs a second press (FR11).

## Console

```
[safety] stop, cause 0x4
[vc] FOLLOW -> SAFE_STOP: safety latched
[safety] cleared
```

## Period

The design's 5 ms period needs a 1 ms kernel tick. With the shipped 10 ms tick the monitor runs every 10 ms, like the other control tasks.

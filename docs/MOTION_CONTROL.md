# Motion Control (Buddy 2)

Motors, wheel encoders, speed control, odometry and encoder-based turns. Other subsystems never touch the motor or encoder pins; they call [`motion_control.h`](../app_program/subsystem2/motion_control.h), which is safe from any task on either core.

| File (`app_program/subsystem2/`) | Contents                                                 |
| -------------------------------- | -------------------------------------------------------- |
| `motion_control.c/.h`            | Public API, 10 ms control step, odometry, telemetry      |
| `drv_motion.c/.h`                | Motor PWM, encoder interrupt, state lock                 |
| `subsystem2.c/.h`                | Start-up, motion task, 10 ms cyclic handler              |
| `subsystem2_test.c`              | On-target tests and bench sequence (`make TEST=1`)       |

Tests and results: [testing/subsystem2](../testing/subsystem2/TEST_LOG.md).

## API

| Call                                 | Effect                                                                                           |
| ------------------------------------ | ------------------------------------------------------------------------------------------------ |
| `motion_control_move_forward(mm)`    | Drive straight, 1–5000 mm                                                                        |
| `motion_control_move_backward(mm)`   | Reverse straight, 1–5000 mm                                                                      |
| `motion_control_turn_left(deg)`      | Turn anticlockwise on the spot, 1–360° (180 is a U-turn)                                         |
| `motion_control_turn_right(deg)`     | Turn clockwise on the spot, 1–360°                                                               |
| `motion_control_drive(left, right)`  | Hold wheel speeds, −1000 to 1000 mm/s. Lasts 200 ms, so call it every cycle; otherwise both wheels brake. After a stall it returns false until `stop()` or another command |
| `motion_control_stop()`              | Brake both wheels now and cancel the command                                                     |
| `motion_control_latch(latched)`      | Safety latch ([SAFETY_MONITOR.md](SAFETY_MONITOR.md)): `true` brakes like `stop()` and makes every move, turn and drive return false until `false`. Returns true if it was already latched. `motion_control_latched()` reads it |
| `motion_control_get_status(&status)` | Copy the status (below)                                                                          |
| `motion_control_distance_um()`       | Distance rolled up to now in µm, wraps. Counts the edges since the last step too, so it is finer and fresher than `distance_mm` |
| `motion_control_set_speed(mm_s)`     | Speed of later moves and turns, 10–1000 mm/s (default 100)                                       |
| `motion_control_pack(&status, buf)`  | 12-byte telemetry payload                                                                        |

A command returns `false` and does nothing if an argument is out of range. Moves and turns run in the background; a new command replaces the one running.

```c
motion_control_turn_right(90u);                     /* barcode "right" */
do
{
    tk_dly_tsk(10);
    motion_control_get_status(&status);
} while (MOTION_CONTROL_RUNNING == status.state);

motion_control_drive(base - correction, base + correction); /* every 10 ms */
```

### Status (`motion_control_status_t`, 12 bytes)

| Field                       | Range              | Meaning                                                                                       |
| --------------------------- | ------------------ | --------------------------------------------------------------------------------------------- |
| `state`                     | 0–3                | `IDLE`, `RUNNING`, `COMPLETE`, `FAILED`                                                       |
| `command`                   | 0–5                | `NONE`, `FORWARD`, `BACKWARD`, `LEFT`, `RIGHT`, `DRIVE`                                       |
| `left_speed`, `right_speed` | −1023–1023         | Wheel speed in mm/s, forward positive                                                         |
| `left_count`, `right_count` | 0–65535, wraps     | Encoder edges, forward counts up                                                              |
| `distance_mm`               | 0–16 777 215, wraps| Distance rolled, either direction                                                             |
| `heading_deg`               | 0–359              | Anticlockwise from power-up, from the encoders                                                |

To measure part of a route, save `distance_mm` or `heading_deg` at the start and subtract.

Faults are bits in the shared fault bitmask (`app_faults()`, `app_fault.h`), cleared by the next command: `APP_FAULT_STALL_LEFT` and `_RIGHT` (no encoder edge for 1 s: the move fails and both wheels brake), `APP_FAULT_ENCODER` (a missed phase: counted, the move continues).

## How it works

- **Encoders:** 2537 edges per wheel turn (measured), 0.080 mm per edge. Each edge interrupts core 0. If a phase is missed, the ISR counts two edges in the last direction; the next 10 ms step sets `APP_FAULT_ENCODER` (bit 2) if a command is running.
- **Speed control:** every 10 ms, one PID per wheel: `duty = (Kp·e + Ki·Σe + Kd·Δe) / 256 + 100‰`, limited to 800‰. The integral holds while the output is limited. Defaults Kp 256, Ki 16, Kd 0. Measured speed is the edges counted since the last step over the kernel time since then (10 ms, or 20 ms then 0 ms after a late wake-up); the hardware timer is not used. The 0 ms step only counts edges: speeds, stall counts, the PID and the outputs wait for the next step.
- **Moves and turns:** each wheel rolls a set number of edges: `mm × 2537 / (π × 65)` for a move, `track × angle × 2537 / (360 × 65)` for a turn. Speed ramps up at 500 mm/s² and slows to 30 mm/s at the end, so stops are repeatable.
- **Going straight:** the wheel that is ahead slows by (edge difference ÷ 4) mm/s and the other speeds up by the same amount.
- **PWM:** 20 kHz on slices 4 (left, GP8/9) and 5 (right, GP10/11). Zero duty brakes.

## Calibration

Measured values are in the [test log](../testing/subsystem2/TEST_LOG.md). Constants are in `motion_control.c`; change one, rebuild and rerun the bench sequence.

1. **Edges per turn:** wheels lifted, after `bench done`, turn a wheel 10 turns by hand between two status lines. The count should rise by about 25 370; otherwise set `MOTION_COUNTS_PER_REV` to the rise ÷ 10.
2. **Distance:** measure the 300 mm moves. If off by more than 1%, scale `MOTION_WHEEL_DIAMETER_UM` by actual ÷ commanded.
3. **Turns:** measure the turns. Set `MOTION_TRACK_UM` to `113500 × commanded ÷ actual` and repeat until within ±3°.
4. **PID:** plot the `[trace] time_ms left right` lines of the three `forward 300` moves (100, 200 and 100 mm/s). Adjust `MOTION_KP`, `MOTION_KI`, `MOTION_KD`.

## Bench sequence

The `make -C build_make TEST=1` image starts task `s2_bench` after the checks. It waits for a tap of the GP21 button, under 0.5 s (a hold of 1 s or more starts the line following bench instead; a press in between starts neither), so other subsystems' `TEST=1` runs never move the robot, then waits 5 s to let you put it down, and runs:

1. `drive(100, 100)` for 2 s, then `drive(-100, -100)` for 2 s
2. `move_forward(300)`, `move_backward(300)`
3. `turn_left(90)`, `turn_right(90)`, `turn_left(180)`
4. `move_forward(300)` at 200 mm/s
5. `turn_left(180)`, `move_forward(300)`: back on the start position, facing the start direction, so the end position shows the error of the whole sequence

While each step runs it prints `[trace] time_ms left_mm_s right_mm_s` every 20 ms (two kernel ticks), and a `[motion] state=… max_tick_us=…` status line when the step ends (before the stop, for drive steps), then holds still for 3 s so the step can be measured. After `[motion] bench done` it prints the status every second.

## Telemetry payload

`motion_control_pack()` writes 96 bits, most significant bit first. Speeds are sent as mm/s + 1024:

| Bits  | 0–1     | 2–4       | 5–13          | 14–24        | 25–35         | 36–51        | 52–67         | 68–91         | 92–95 |
| ----- | ------- | --------- | ------------- | ------------ | ------------- | ------------ | ------------- | ------------- | ----- |
| Field | `state` | `command` | `heading_deg` | `left_speed` | `right_speed` | `left_count` | `right_count` | `distance_mm` | 0     |

```python
v = int.from_bytes(payload, "big")
def field(start, width): return (v >> (96 - start - width)) & ((1 << width) - 1)
left_mm_s = field(14, 11) - 1024
```

## Resources

| Resource                     | Use                                                                                         |
| ---------------------------- | ------------------------------------------------------------------------------------------- |
| GP8–GP11, PWM slices 4, 5    | Motors                                                                                      |
| GP2, GP3, GP16, GP17         | Encoders (pull-ups, both edges)                                                             |
| GP21                         | Bench start button (pull-up), `TEST=1` only                                                 |
| IRQ 13 (GPIO bank 0), core 0 | Encoder edge callback, registered with `app_gpio_register()` (`app.h`)                      |
| Tasks                        | `s2_motion` priority 6, core 0; `s2_bench` priority 7, core 0, `TEST=1` only                 |
| Kernel objects               | 1 mutex, 1 cyclic handler                                                                   |

## Coding standard

The code follows BARR-C:2018, with the team's deviations listed at the top of `app_program/app.h`.

Abbreviations (rule 1.5):

| Abbreviation | Meaning                      | Abbreviation  | Meaning                                  |
| ------------ | ---------------------------- | ------------- | ---------------------------------------- |
| `us`         | microseconds                 | `um`          | micrometres                              |
| `ms`         | milliseconds                 | `mm_s`        | millimetres per second                   |
| `deg`        | degrees                      | `cmd`         | command                                  |
| `pwm`        | pulse width modulation       | `pid`, `kp`, `ki`, `kd` | PID controller and its gains   |
| `isr`        | interrupt service routine    | `mm`          | millimetres                              |
| `drv`        | driver                       | `pri`, `prc`  | task priority, processor (core)          |
| `s2`..`s5`   | subsystem 2 to 5             | `accel`, `decel` | acceleration, deceleration            |
| `rev`        | revolution                   | `div`         | clock divider                            |
| `uturn`      | U-turn                       | `trig`        | trigger                                  |
| `chr`        | character                    | `pct`         | percent                                  |
| `q8`         | fixed point, 8 fraction bits (value × 256) | `permille` / `‰` | parts per thousand (duty)  |

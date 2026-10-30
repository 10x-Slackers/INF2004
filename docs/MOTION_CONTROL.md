# Motion Control (Buddy 2)

Motors, wheel encoders, speed control, odometry and encoder-based turns. Other subsystems never touch the motor or encoder pins; they call [`motion_control.h`](../app_program/subsystem2/motion_control.h), which is safe from any task on either core.

| File (`app_program/subsystem2/`) | Contents                                                 |
| -------------------------------- | -------------------------------------------------------- |
| `motion_control.c/.h`            | Public API, 10 ms control step, odometry, telemetry      |
| `drv_motion.c/.h`                | Motor PWM, encoder interrupt, state lock                 |
| `subsystem2.c/.h`                | Start-up, motion task, 10 ms cyclic handler              |
| `console_motion.c/.h`            | USB console for testing and calibration                  |
| `subsystem2_test.c`              | On-target tests (`make TEST=1`)                          |

Tests and results: [testing/subsystem2](../testing/subsystem2/TEST_LOG.md).

## API

| Call                                 | Effect                                                                                           |
| ------------------------------------ | ------------------------------------------------------------------------------------------------ |
| `motion_control_move_forward(mm)`    | Drive straight, 1–5000 mm                                                                        |
| `motion_control_move_backward(mm)`   | Reverse straight, 1–5000 mm                                                                      |
| `motion_control_turn_left(deg)`      | Turn anticlockwise on the spot, 1–360° (180 is a U-turn)                                         |
| `motion_control_turn_right(deg)`     | Turn clockwise on the spot, 1–360°                                                               |
| `motion_control_drive(left, right)`  | Hold wheel speeds, −1000 to 1000 mm/s. Lasts 200 ms, so call it every cycle; otherwise both wheels brake |
| `motion_control_stop()`              | Brake both wheels now and cancel the command                                                     |
| `motion_control_get_status(&status)` | Copy the status (below)                                                                          |
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
| `faults`                    | bits               | 1 left stall, 2 right stall (no encoder edge for 1 s: move fails, both brake). 4 missed encoder phase (counted, move continues). Cleared by the next command |
| `left_speed`, `right_speed` | 1–2047             | mm/s + 1024; use `motion_control_decode_speed()`                                              |
| `left_count`, `right_count` | 0–65535, wraps     | Encoder edges, forward counts up                                                              |
| `distance_mm`               | 0–16 777 215, wraps| Distance rolled, either direction                                                             |
| `heading_deg`               | 0–359              | Anticlockwise from power-up, from the encoders                                                |

To measure part of a route, save `distance_mm` or `heading_deg` at the start and subtract.

## How it works

- **Encoders:** 2537 edges per wheel turn (measured), 0.080 mm per edge. Each edge interrupts core 0. If a phase is missed, the ISR counts two edges in the last direction and sets fault bit 4.
- **Speed control:** every 10 ms, one PID per wheel: `duty = (Kp·e + Ki·Σe + Kd·Δe) / 256 + 100‰`, limited to 800‰. The integral holds while the output is limited. Defaults Kp 256, Ki 16, Kd 0.
- **Moves and turns:** each wheel rolls a set number of edges: `mm × 2537 / (π × 65)` for a move, `track × angle × 2537 / (360 × 65)` for a turn. Speed ramps up at 500 mm/s² and slows to 30 mm/s at the end, so stops are repeatable.
- **Going straight:** the wheel that is ahead slows by (edge difference ÷ 4) mm/s and the other speeds up by the same amount.
- **PWM:** 20 kHz on slices 4 (left, GP8/9) and 5 (right, GP10/11). Zero duty brakes.

## Calibration

Measured values are in [CALIBRATION.md](../testing/subsystem2/CALIBRATION.md). Console changes last until reset; put final values in `motion_control.c`.

1. **Edges per turn:** wheels lifted, `status`, turn a wheel 10 turns by hand, `status`. The count should rise by about 25 370; otherwise set `MOTION_COUNTS_PER_REV` to the rise ÷ 10.
2. **Distance:** `forward 500` and measure. If off by more than 1%, scale `MOTION_WHEEL_DIAMETER_UM` and `MOTION_CIRCUMFERENCE_UM` by actual ÷ commanded.
3. **Turns:** `right 360` and measure. Set `track` to `113500 × 360 ÷ actual` and repeat until within ±3°.
4. **PID:** `trace`, then `forward 300`, and plot the `[trace] time left right` lines. Adjust with `pid kp ki kd`.

## Console

USB serial at 115200 8N1, one command per line. On a robot with only the motion hardware, build `make S2_ONLY=1`; the image starts only this subsystem and its name ends in `_s2only`.

| Command                    | Effect                                                  |
| -------------------------- | ------------------------------------------------------- |
| `forward 300`, `backward 300` | Move, mm                                             |
| `left 90`, `right 90`      | Turn, degrees                                           |
| `drive 150 100`            | Wheel speeds, mm/s, until the next command              |
| `stop`                     | Brake                                                   |
| `speed 150`                | Move and turn speed, mm/s                               |
| `track 113500`             | Effective track, µm                                     |
| `pid 256 16 0`             | PID gains, Q8                                           |
| `status`                   | Status and the longest control step (`max_tick_us`)     |
| `trace`                    | Print wheel speeds while a command runs (on/off)        |

## Telemetry payload

`motion_control_pack()` writes 96 bits, most significant bit first:

| Bits  | 0–1     | 2–4       | 5–7      | 8–16          | 17–27        | 28–38         | 39–54        | 55–70         | 71–94         | 95 |
| ----- | ------- | --------- | -------- | ------------- | ------------ | ------------- | ------------ | ------------- | ------------- | -- |
| Field | `state` | `command` | `faults` | `heading_deg` | `left_speed` | `right_speed` | `left_count` | `right_count` | `distance_mm` | 0  |

```python
v = int.from_bytes(payload, "big")
def field(start, width): return (v >> (96 - start - width)) & ((1 << width) - 1)
left_mm_s = field(17, 11) - 1024
```

## Resources

| Resource                     | Use                                                                                         |
| ---------------------------- | ------------------------------------------------------------------------------------------- |
| GP8–GP11, PWM slices 4, 5    | Motors                                                                                      |
| GP2, GP3, GP16, GP17         | Encoders (pull-ups, both edges)                                                             |
| IRQ 13 (GPIO bank 0), core 0 | Encoder ISR. Other subsystems needing GPIO edge interrupts call `drv_motion_gpio_register()` in their init, not `tk_def_int(13)` |
| Tasks                        | `s2_motion` priority 6, `s2_console` priority 22, core 0                                     |
| Kernel objects               | 1 mutex, 1 cyclic handler                                                                   |

## Coding standard

The code follows BARR-C:2018. Deviations:

| Rule      | Deviation and reason                                                                                 |
| --------- | ---------------------------------------------------------------------------------------------------- |
| 3.1.j, 3.2 | No space before `(` in definitions, and no column alignment of declarations: the repository's `.clang-format` enforces this layout |
| 6.1.i     | `tm_console_receive_byte()` has no module prefix: its name is fixed by the weak hook in `lib/libtm`    |
| 4.2.a     | `subsystem2_test.c` has no header: `subsystem2_test()` is declared in `app.h` by the course template  |
| 5.2       | Task and ISR entry points take kernel types (`INT`, `UINT`) to match the µT-Kernel prototypes         |

Abbreviations (rule 1.5):

| Abbreviation | Meaning                      | Abbreviation  | Meaning                                  |
| ------------ | ---------------------------- | ------------- | ---------------------------------------- |
| `us`         | microseconds                 | `um`          | micrometres                              |
| `ms`         | milliseconds                 | `mm_s`        | millimetres per second                   |
| `deg`        | degrees                      | `cmd`         | command                                  |
| `cpr`        | encoder edges per wheel turn | `pid`, `kp`, `ki`, `kd` | PID controller and its gains   |
| `pwm`        | pulse width modulation       | `isr`         | interrupt service routine                |
| `q8`         | fixed point, 8 fraction bits (value × 256) | `permille` / `‰` | parts per thousand (duty)  |

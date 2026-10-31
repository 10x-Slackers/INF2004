# Obstacle Avoidance (Buddy 5)

Ultrasonic ranging, obstacle profiling and the bypass. Other subsystems never touch the HC-SR04 or the servo; they call [`ultrasonic_scan.h`](../app_program/subsystem5/ultrasonic_scan.h) and [`obstacle_avoidance.h`](../app_program/subsystem5/obstacle_avoidance.h), which are safe from any task on either core.

| File (`app_program/subsystem5/`) | Contents                                                      |
| -------------------------------- | ------------------------------------------------------------- |
| `obstacle_avoidance.c/.h`        | Avoidance API (I6) and task: stop, look each side, profile, bypass |
| `ultrasonic_scan.c/.h`           | Scan API (I7, I8) and scan task, the owner of sensor and servo |
| `scan_profile.c/.h`              | Echo conversion, profile and side choice; no hardware calls    |
| `drv_hcsr04.c/.h`                | HC-SR04 trigger and echo timing                                |
| `drv_sg90.c/.h`                  | Servo PWM                                                      |
| `subsystem5.c/.h`                | Start-up and task priorities                                   |
| `subsystem5_test.c`              | On-target tests and bench sequence (`make TEST=1`)             |

Tests and results: [testing/subsystem5](../testing/subsystem5/TEST_LOG.md).

## API

| Call                                   | Effect                                                                                          |
| -------------------------------------- | ----------------------------------------------------------------------------------------------- |
| `obstacle_avoidance_start()`           | Clear the faults and watch the range ahead. Also the only way out of a safe stop. Does nothing if already started |
| `obstacle_avoidance_cancel()`          | Stop watching. Brakes the car if avoidance is driving it (profiling or bypassing)                |
| `obstacle_avoidance_get_status(&s)`    | Copy the avoidance status (below)                                                               |
| `ultrasonic_scan_request(mode)`        | Ask for a scan: `ULTRASONIC_SCAN_FORWARD` (one reading ahead, about 60 ms) or `ULTRASONIC_SCAN_PROFILE` (servo sweep, about 2 s, car still). Returns the request number |
| `ultrasonic_scan_get(&scan)`           | Copy the latest results; `request` is the number of the request the latest scan served |

```c
obstacle_avoidance_start();                 /* with the line follower */
motion_control_drive(left, right);          /* every 10 ms, while... */
obstacle_avoidance_get_status(&s);          /* ...s.state is MONITOR */
```

While avoidance is profiling or bypassing it drives the car itself through the motion API (I3). On seeing the obstacle it calls `line_following_stop()` (I1), so the line follower stops steering, and the controller must send no motion commands until the state is back to `MONITOR` (the car is then stopped on the original line and heading) or `STOPPED`.

### Avoidance status (`obstacle_avoidance_status_t`, 8 bytes)

| Field      | Range | Meaning                                                                                           |
| ---------- | ----- | ------------------------------------------------------------------------------------------------- |
| `state`    | 0–4   | `IDLE`, `MONITOR`, `PROFILE`, `BYPASS`, `STOPPED` (safe stop, until the next `start()`)          |
| `side`     | 0–2   | Side of the last bypass: none, left, right                                                         |
| `step`     | 0–7   | Bypass step running, 0 outside a bypass                                                            |
| `bypasses` | 0–15  | Bypasses finished, wraps                                                                           |
| `left_mm`, `right_mm` | 0–4000 | Clearance of the last profile, with the car turned 45° left and right; 0 if not measured |

Faults are bits in the shared fault bitmask (`app_faults()`, `app_fault.h`), cleared by `start()`: `APP_FAULT_AVOID_SENSOR` (no echo, or no scan result in 8 s), `_BLOCKED` (no open side, or the way ahead closed during a bypass), `_MOTION` (did not stop in 1 s, a move failed or took over 10 s, or someone else stopped it).

### Scan results (`ultrasonic_scan_t`, 16 bytes)

| Field                  | Range         | Meaning                                                                        |
| ---------------------- | ------------- | ------------------------------------------------------------------------------ |
| `forward_mm`           | 20–4000       | Latest range straight ahead. 4000 means nothing within the sensor's range      |
| `forward_ms`           | wraps         | Kernel time of `forward_mm`, to check its age                                  |
| `request`              | 0–15, wraps   | Number of the request the latest scan served                                   |
| `profile.closest_mm`   | 20–4000       | Closest point of the last profile scan, at `profile.closest_deg`               |
| `profile.right_deg`, `.left_deg` | 80–100 | Obstacle edges: outermost rays within 350 mm next to the closest point |
| `profile.b_obstacle`   | 0–1           | Closest point within 350 mm                                                    |
| `profile.width_mm`     | 0–122         | Rough obstacle width: the arc between the edges at `closest_mm`; 0 with no obstacle |

Each scan also sets `APP_FAULT_NO_ECHO` in the shared fault bitmask when one of its rays got no echo pulse (sensor missing or faulty), and clears it otherwise, before `request` moves on.

Servo angles: 80° points right, 90° ahead, 100° left. The sensor's mount stops the servo about 15° either side, so the servo only does the fine scan; the car turns for the coarse one. The width (FR6) is rough: the beam is about as wide as the sweep, and an obstacle that fills the sweep reads at most about 120 mm at 350 mm, whatever its real width.

## How it works

- **Ranging:** a 10 µs trigger, then the scan task sleeps while the echo is out. ECHO's edges arrive through subsystem 2's GPIO interrupt (`drv_motion_gpio_register()`), which times them on the 1 MHz TIMER and wakes the task. Distance is `echo_us × 10 / 58` mm. An echo longer than 4 m, or none ending within 50 ms, reads 4000 mm: open space is clear. Only an echo pulse that never starts is a fault. Triggers are at least 60 ms apart, and wait for ECHO to be low.
- **Coarse scan (car):** with the car still, avoidance turns it 45° left and takes a forward reading, turns 90° right and takes another, then turns 45° back to the original heading. These are `left_mm` and `right_mm`.
- **Fine scan (servo):** rays every 5° from 80 to 100°, each the median of three readings after a 200 ms servo settle. The obstacle is the run of rays within 350 mm around the closest one. The servo returns ahead and takes a forward reading at the end of every scan.
- **Side:** the more open side, if it has at least 400 mm; left on a tie. With neither, a safe stop (no reverse: nothing checks behind the car).
- **Bypass:** seven steps through the motion API, each waited on until `COMPLETE`: turn 90° towards the side, forward 200 mm (300 mm if the obstacle fills the whole fine sweep, so it may be wider than the sweep can see), turn back, forward 700 mm (the 350 mm gap, the obstacle, the car and the beam's spread), turn back, forward the same side-step, turn 90° to the original heading. During each forward step the range ahead is read continuously, and anything within 350 mm is a safe stop, so a side that looked open but is not ends in a stop instead of a collision. The obstacle being passed stays further than that while it is inside the beam. The car ends stopped on the original line, past the obstacle, and avoidance watches again. Finding the line with the IR sensors is up to the controller.
- **Monitoring:** while started, forward readings follow each other (about one every 60 ms). The first within 350 mm makes avoidance stop line following and brake, wait for both wheels to read 0 mm/s, look each side and profile. If the fine scan finds nothing within 350 mm, it goes back to watching with the car stopped.

## Calibration

Constants are in the `.c` files; change one, rebuild and rerun the bench sequence.

1. **Range:** flat target at 100, 300 and 1000 mm. If readings are off by more than 1%, change the 58 µs/cm in `scan_profile_mm()`.
2. **Servo:** watch a short bench press. Set `DRV_SG90_MID_US` so the sensor points straight ahead, and `DRV_SG90_MIN_US`, `_MAX_US` so it points 10° right and 10° left without reaching the mount's stops (a higher pulse turns it left; on the car's servo 10 µs moves it well under 1°, and steps under 10 µs may not move it at all).
3. **Look angle:** `OBSTACLE_AVOIDANCE_LOOK_DEG` (45°) must point the sensor past the widest obstacle at 350 mm.
4. **Bypass size:** `OBSTACLE_AVOIDANCE_SIDE_MM` (200) must clear half the car's width plus half a narrow obstacle's, and `_WIDE_MM` (300) half a wide one's; `_PASS_MM` the obstacle's length plus the car's. Measure on the track.
5. **Thresholds:** `SCAN_PROFILE_BLOCK_MM` (350) must leave room to brake; `SCAN_PROFILE_CLEAR_MM` (400) must fit the car beside the obstacle; the range watch during each forward bypass step catches a side that is narrower than it looked.

## Bench sequence

The `make -C build_make TEST=1` image starts task `s5_bench` after the checks. It waits for the GP20 button, so other subsystems' `TEST=1` runs never move the robot:

1. **Short press (any number of times):** one servo profile scan with the car still, then `[scan] …` with the forward range, closest point and edges. The car does not move.
2. **Press held for 1 s:** after 5 s, `obstacle_avoidance_start()` and `motion_control_move_forward(1500)`. It prints `[scan] …` and `[avoid] state=… faults=… side=… step=… bypasses=… right_mm=… left_mm=… motion=state` whenever the state or step changes, and stops after one bypass, a safe stop, or the 1.5 m drive ending with no obstacle. Then `[avoid] bench done`, and it waits for GP20 again (no reset needed).

## Resources

| Resource              | Use                                                                                 |
| --------------------- | ----------------------------------------------------------------------------------- |
| GP0, GP1              | HC-SR04 TRIG, ECHO (level shifted). Unplug the sensor for `CRASH_REPORT=1` builds    |
| GP12, PWM slice 6 A   | Servo, 50 Hz                                                                        |
| GP20                  | Bench button (pull-up), `TEST=1` only                                              |
| IRQ 13 callback       | ECHO edges, registered with `drv_motion_gpio_register()`                             |
| Tasks                 | `s5_avoid` priority 11; `s5_scan` priority 12; `s5_bench` priority 12, `TEST=1` only |
| Kernel objects        | 2 mutexes, 1 semaphore                                                              |

## Coding standard

The code follows BARR-C:2018 with the same deviations as subsystem 2 ([MOTION_CONTROL.md](MOTION_CONTROL.md#coding-standard)), for `subsystem5_test.c` and the task entry points. Abbreviations as listed there.

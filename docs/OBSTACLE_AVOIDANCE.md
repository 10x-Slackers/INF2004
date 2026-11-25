# Obstacle Avoidance (Buddy 5)

Ultrasonic ranging, obstacle profiling and the bypass. Other subsystems never touch the HC-SR04 or the servo; they call [`ultrasonic_scan.h`](../app_program/subsystem5/ultrasonic_scan.h) and [`obstacle_avoidance.h`](../app_program/subsystem5/obstacle_avoidance.h), which are safe from any task on either core.

| File (`app_program/subsystem5/`) | Contents                                                      |
| -------------------------------- | ------------------------------------------------------------- |
| `obstacle_avoidance.c/.h`        | Avoidance API (I6) and task: stop, profile, bypass             |
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
| `obstacle_avoidance_line()`            | A line sensor sees black. Near the line on the return diagonal, ends the diagonal with the axle on the line; otherwise nothing |
| `obstacle_avoidance_get_status(&s)`    | Copy the avoidance status (below)                                                               |
| `ultrasonic_scan_request(mode)`        | Ask for a scan: `ULTRASONIC_SCAN_FORWARD` (one reading ahead, about 60 ms) or `ULTRASONIC_SCAN_PROFILE` (coarse and fine servo sweep, 3 to 8 s, car still). Returns the request number |
| `ultrasonic_scan_get(&scan)`           | Copy the latest results; `request` is the number of the request the latest scan served |

```c
obstacle_avoidance_start();                 /* with the line follower */
motion_control_drive(left, right);          /* every 10 ms, while... */
obstacle_avoidance_get_status(&s);          /* ...s.state is MONITOR */
```

While avoidance is profiling or bypassing it drives the car itself through the motion API (I3). On seeing the obstacle it calls `line_following_stop()` (I1), so the line follower stops steering, and the controller must send no motion commands until the state is back to `MONITOR` (the car is then stopped on the original line and heading) or `STOPPED`. The Vehicle Controller cancels avoidance while the line follower pivots to find a lost line (the sweep would look like an obstacle ahead) and starts it again once the line is found; it also cancels it for a junction turn and on a safe stop ([VEHICLE_CONTROLLER.md](VEHICLE_CONTROLLER.md#states)).

### Avoidance status (`obstacle_avoidance_status_t`, 8 bytes)

| Field      | Range | Meaning                                                                                           |
| ---------- | ----- | ------------------------------------------------------------------------------------------------- |
| `state`    | 0–4   | `IDLE`, `MONITOR`, `PROFILE`, `BYPASS`, `STOPPED` (safe stop, until the next `start()`)          |
| `side`     | 0–2   | Side of the last bypass: none, left, right                                                         |
| `step`     | 0–7   | Bypass step running, 0 outside a bypass                                                            |
| `bypasses` | 0–15  | Bypasses finished, wraps                                                                           |
| `left_mm`, `right_mm` | 0–4000 | Clearance of the last profile, 45° left and right (`profile.left_mm`, `.right_mm`); 0 if not measured |

Faults are bits in the shared fault bitmask (`app_faults()`, `app_fault.h`), cleared by `start()`: `APP_FAULT_AVOID_SENSOR` (no echo, or no scan result in 12 s), `_BLOCKED` (no open side or clear path after backing off, the obstacle not seen again after backing off, or the way ahead closed during a bypass), `_MOTION` (did not stop in 1 s, a move failed or took over 10 s, or someone else stopped it).

### Scan results (`ultrasonic_scan_t`, 20 bytes)

| Field                  | Range         | Meaning                                                                        |
| ---------------------- | ------------- | ------------------------------------------------------------------------------ |
| `forward_mm`           | 20–4000       | Latest range straight ahead. 4000 means nothing within the sensor's range      |
| `forward_ms`           | wraps         | Kernel time of `forward_mm`, to check its age                                  |
| `request`              | 0–15, wraps   | Number of the request the latest scan served                                   |
| `profile.closest_mm`   | 20–4000       | Closest point of the last profile scan, at `profile.closest_deg`               |
| `profile.right_deg`, `.left_deg` | 45–135 | Obstacle edges: outermost rays of the run within 200 mm around straight ahead |
| `profile.b_obstacle`   | 0–1           | Ray straight ahead within 200 mm                                               |
| `profile.width_mm`     | 0–255         | Obstacle width: the arc at `closest_mm` across the rays on the same obstacle (see Fine scan), less 5° beam spread each side; 0 with no obstacle |
| `profile.left_mm`, `.right_mm` | 20–4000 | Clearance: the rays at 135° and 45° (45° left and right)               |

Each scan also sets `APP_FAULT_NO_ECHO` in the shared fault bitmask when one of its rays got no echo pulse (sensor missing or faulty), and clears it otherwise, before `request` moves on.

Servo angles: 45° points right, 90° ahead, 135° left. The sensor's mount limits the servo to a little under 60° either side, so the sweep stays 45° either side; the car stays still for the whole scan. The width (FR6) is rough: the beam spreads, so the outermost rays still see the obstacle past its edges; 5° comes off each edge, which fits a 180 mm flat obstacle at 160–190 mm to within about 15%. A small obstacle still reads wider than it is.

## How it works

- **Ranging:** a 10 µs trigger, then the scan task sleeps while the echo is out. ECHO's edges arrive through the shared GPIO interrupt (`app_gpio_register()`), which times them on the 1 MHz TIMER and wakes the task. Distance is `echo_us × 10 / 58` mm. An echo longer than 4 m, or none ending within 50 ms, reads 4000 mm: open space is clear. Only an echo pulse that never starts is a fault. Triggers are at least 60 ms apart, and wait for ECHO to be low.
- **Coarse scan:** rays every 15° from 45 to 135°, each the median of three readings after a 200 ms servo settle. The rays at 135° and 45° are `left_mm` and `right_mm`.
- **Fine scan:** from the ray ahead, the rays every 5° each way, while the last one read was on the obstacle ahead: within 50 mm past the ray ahead, or within 200 mm. The obstacle is the run of blocked rays (within 200 mm) around straight ahead, its edges the outermost, and its closest point the nearest of them. The width spans the wider run on the same obstacle: a flat face seen at a slant reads further at its sides, past 200 mm when the car stopped close to that distance, and those rays would otherwise drop out. A blocked run elsewhere (a wall to the side) is not the obstacle. The servo returns ahead and takes a forward reading at the end of every scan. A profile takes about 3 s with nothing ahead, about 4 s for a box, and at most 19 rays (about 7 s).
- **Side:** the more open side, if it has at least 250 mm; left on a tie. The 45° rays cannot see the bypass path itself, so after the first turn, with the sensor facing along the diagonal, a forward reading must show the diagonal plus 200 mm. If it does not, the car turns 120° to the other side and checks it the same way. With neither side open, or neither path clear (turned back to the original heading), the car may have stopped too close: the 45° rays then hit the obstacle itself. So it takes one forward reading and reverses by what that is short of `OBSTACLE_AVOIDANCE_RETRY_MM` (180 mm), at most `OBSTACLE_AVOIDANCE_BACK_MM` (80 mm), and profiles and tries again once. It aims to stay within 200 mm, as the profile only sees an obstacle that close. The reverse is blind, since nothing checks behind the car, hence the cap. Still blocked is a safe stop, and so is the second profile seeing no obstacle: the car would otherwise drive up to it again and back off again for ever.
- **Bypass:** seven steps through the motion API, each waited on until `COMPLETE`: turn 60° towards the side, a diagonal out, turn back parallel to the line, a straight pass, turn 60° in, the diagonal back, turn 60° to the original heading. The legs come from the profile. The diagonal ends with the car's inner side 40 mm clear of the obstacle's edge on that side (the edge's offset is taken as 200 mm × its angle off ahead, as an arc: a blocked edge ray is never further than 200 mm, so this is never under the true offset), so it is that offset plus half the car (85 mm) and 40 mm, over sin 60°. The pass takes the car's back 40 mm past the obstacle, its length taken as the distance between its edges at `closest_mm`. During each forward step the range ahead is read continuously, and anything within 200 mm is a safe stop, so a side that looked open but is not ends in a stop instead of a collision. The obstacle being passed stays further than that while it is inside the beam. The return diagonal ends where the line actually is, not where dead reckoning puts it: the line sensors are 115 mm ahead of the axle, so they should reach the line 115 mm before the car is back where it would be on the line by dead reckoning. The return diagonal is 150 mm longer than the outward one, so heading error from the turns (the 120° turn round most of all) cannot leave the car short of the line. From 100 mm before that point to the end of the diagonal, the first black on either sensor (passed on by the controller, which reads the line every 10 ms; `obstacle_avoidance_line()`) replaces the rest of the diagonal with a 115 mm move, which puts the axle on the line. The last turn then leaves the car on the line and parallel to it, and avoidance watches again. If the sensors see nothing, the diagonal runs its full length, 150 mm past where dead reckoning puts the line, and the line follower's lost-line search looks for the line.
- **Monitoring:** while started, forward readings follow each other (about one every 60 ms). The second in a row within 200 mm (one stray echo is ignored) makes avoidance stop line following and brake, wait for both wheels to read 0 mm/s, and profile. If nothing straight ahead is within 200 mm, it goes back to watching with the car stopped.

## Calibration

Constants are in the `.c` files; change one, rebuild and rerun the bench sequence.

1. **Range:** flat target at 100, 300 and 1000 mm. If readings are off by more than 1%, change the 58 µs/cm in `scan_profile_mm()`.
2. **Servo:** watch a short bench press. Set `DRV_SG90_MID_US` so the sensor points straight ahead, and `DRV_SG90_MIN_US`, `_MAX_US` so it points 45° right and 45° left without reaching the mount's stops (a higher pulse turns it left; the defaults assume about 11 µs per degree).
3. **Bypass size:** `OBSTACLE_AVOIDANCE_HALF_MM` (85, half the 170 mm car), `_FRONT_MM` (100, axle to the ultrasonic sensor) and `_REAR_MM` (80, axle to the back) are the car's size; `_GAP_MM` (40) is the clearance kept round the obstacle. Raise the gap if the car brushes the obstacle, lower it if the route is wider than needed. `_LINE_MM` (115) is the line sensors' distance ahead of the axle: raise it if the car ends short of the line after the last turn, lower it if past. `_EARLY_MM` (100) is how early the return diagonal starts watching for the line: lower it if a mark beside the line ends the diagonal early. `_OVER_MM` (150) is how much longer the return diagonal is than the outward one: raise it if the car still ends short of the line, lower it if a car that misses the line runs too far on. `_SPEED` (250 mm/s) is the cruise speed of the bypass moves and turns. `_TURN_DEG` (60) is the diagonals' angle; `_SIN_TURN` and `_COS_TURN` must match it.
4. **Thresholds:** `SCAN_PROFILE_BLOCK_MM` (200) must leave room to brake; `SCAN_PROFILE_CLEAR_MM` (250) must fit the car beside the obstacle; the range watch during each forward bypass step catches a side that is narrower than it looked.

## Bench sequence

The `make -C build_make TEST=1` image starts task `s5_bench` after the checks. It waits for the GP20 button, so other subsystems' `TEST=1` runs never move the robot:

1. **Short press (any number of times):** the last run's log again (`[avoid] last run: N records`, then its lines; for a run made with USB unplugged), then one servo profile scan with the car still, then `[scan] forward_mm= fault= closest=…@… edges=… width= obstacle= clear=right/left` and an `[avoid] …` line as below. The car does not move.
2. **Press held for 1 s:** after 5 s, `obstacle_avoidance_start()` and `motion_control_move_forward(1500)`. It prints `[scan] …` and `[avoid] t=… ms state=… faults=… side=… step=… bypasses=… right_mm=… left_mm=… motion=state` whenever the state or step changes, and stops after one bypass, a safe stop, or the car standing still while watching (the 1.5 m drive ended, or a profile found nothing straight ahead). It prints one last record, then `[avoid] bench done`, and it waits for GP20 again (no reset needed).

## Resources

| Resource              | Use                                                                                 |
| --------------------- | ----------------------------------------------------------------------------------- |
| GP0, GP1              | HC-SR04 TRIG, ECHO (level shifted). Unplug the sensor for `CRASH_REPORT=1` builds    |
| GP12, PWM slice 6 A   | Servo, 50 Hz                                                                        |
| GP20                  | Bench button (pull-up), `TEST=1` only                                              |
| IRQ 13 callback       | ECHO edges, registered with `app_gpio_register()`                                    |
| Tasks                 | `s5_avoid` priority 11; `s5_scan` priority 12; `s5_bench` priority 12, `TEST=1` only |
| Kernel objects        | 2 mutexes, 1 semaphore                                                              |

## Coding standard

The code follows BARR-C:2018 with the same deviations as subsystem 2 ([MOTION_CONTROL.md](MOTION_CONTROL.md#coding-standard)), for `subsystem5_test.c` and the task entry points. Abbreviations as listed there.

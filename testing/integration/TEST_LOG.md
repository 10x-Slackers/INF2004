# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | | | | | |
| INT-02 | Barcode and controller commands drive the robot | I3, I4 | manual | Controller calls `motion_control_turn_left(90)`, `_turn_right(90)`, `_move_forward(200)`, `_turn_left(180)`; polls `motion_control_get_status()` | Each ends `COMPLETE`; turns within ±3°; controller waits for completion |
| INT-03 | Line follower and bypass use continuous drive | I3, I5 | manual | Buddy 3 calls `motion_control_drive()` every cycle; Buddy 5 uses `distance_mm` / `heading_deg` deltas during a bypass; `motion_control_stop()` at the end | Smooth tracking; stop brakes at once |
| INT-04 | Motion telemetry over MQTT | Motion telemetry | manual | Buddy 1 publishes `motion_control_pack()` every 200 ms; the dashboard decodes it as in `docs/MOTION_CONTROL.md` | 12-byte payload; decoded state, speeds and distance match the console `status` |
| INT-05 | Controller start, stop and reset with GP20 | I1, I11, FR11 | manual | Normal build, car on the line. 1. Wait for `READY`. 2. Press GP20. 3. After 1 m press GP20. 4. Press GP20 twice | 2: follows the line. 3: brakes, `SAFE_STOP: GP20`. 4: `READY`, then follows again |
| INT-06 | Controller hands the car to avoidance and back | I1, I6 | manual | Obstacle on a straight line 1 m ahead with one side open; then repeat with both sides blocked | Open side: `FOLLOW -> AVOID`, bypass, `AVOID -> FOLLOW`, follows the line on. Blocked: `SAFE_STOP: avoidance stopped`, no contact |
| INT-07 | Controller executes barcodes at the next junction | I2, I3, FR3 | manual | For each command in `g_actions` (on the example course, `*Z*` before the left loop's crossing): barcode before a junction, start 0.5 m before it | `barcode N pending`, then at the junction a left 90°, right 90°, straight on, or U-turn, and follows the outgoing line |
| INT-08 | IMU fault latches a safe stop | I9, FR11 | manual | Car following; tilt it past 25° for 1 s; then press GP20 | `[safety] stop, cause 0x1`, `SAFE_STOP: safety latched`, motors stop; GP20 does not leave the safe stop until reset |
| INT-09 | Retired: Safety Monitor collision backstop. The backstop was removed after its FAIL (see Results); avoidance (INT-06, S5-U48) covers collision risk | I8, I11, NFR5 | manual | Normal build. Start following; hold a flat target 100 mm in front of the sensor while the car drives | `[safety] stop, cause 0x2`, motors brake at once, `SAFE_STOP: safety latched`; drive commands are refused until GP20 |
| INT-10 | Safety Monitor latches a wheel stall | I4, I11, FR11 | manual | Normal build, car following. Hold one wheel still for over 1 s | `[safety] stop, cause 0x4`, `SAFE_STOP: safety latched`; GP20 clears it, a second press follows again |
| INT-11 | RAM before/after minimum-bit: Vehicle Controller and the whole image | Minimum-bit CR | host | Commands in `logs/2026-10-31_ram.txt` | Packed is smaller |
| INT-12 | Controller stops on a lost line | I1, I11, FR2, NFR5 | manual | Normal build. A straight line that ends with no line within 300 mm; car on the line, GP20 to start. After it stops, GP20, then GP20 again | After the search, `[vc] FOLLOW -> SAFE_STOP: line lost` and `[safety] stop, cause 0x8`. The first press returns to INIT then READY, the second follows again |
| INT-13 | Avoidance pauses during a line search | I1, I6, FR7 | manual | Normal build. An L: a straight line ending in a 90° left corner, with a box 200 mm outside the corner (to the right of where the straight ends). Car on the straight, GP20 | The car runs off the corner or follows it; any pivot sweeps past the box with no `[vc] FOLLOW -> AVOID`, and the car follows the new branch |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-31 | INT-05 | `644e2e8` | commit2main | PASS (partial) | `logs/2026-10-31_safety_car.txt` | Wheels lifted, no line. Three start/stop/reset cycles: each GP20 stop gave `[safety] stop, cause 0x8`, the next press `[safety] cleared` and `READY`, the next `FOLLOW`. Not yet run on the track |
| 2026-10-31 | INT-10 | `644e2e8` | commit2main | PASS (partial) | `logs/2026-10-31_safety_car.txt` | Wheels lifted, one wheel held: `[safety] stop, cause 0x4`, `FOLLOW -> SAFE_STOP: safety latched`; GP20 cleared it to `READY` |
| 2026-10-31 | INT-09 | `644e2e8` | commit2main | FAIL | `logs/2026-10-31_safety_car.txt` | Wheels lifted, book swung in to about 100 mm: avoidance reacted first (`FOLLOW -> AVOID: obstacle`, then `AVOID -> FOLLOW: obstacle passed` once the book was gone). The 150 mm backstop never latched: avoidance stops line following on the first reading within 350 mm, so the car is no longer driving forward when the monitor looks. The test cannot isolate the backstop as written |
| 2026-10-31 | INT-08 | `644e2e8` | commit2main | PASS (partial) | `logs/2026-10-31_safety_car.txt` | Wheels lifted, car propped and tilted past 25° while following: `[safety] stop, cause 0x1`, `FOLLOW -> SAFE_STOP: safety latched`. GP20 afterwards: no `cleared` and no `READY` in the log, so the fault stayed latched |
| 2026-10-31 | INT-11 | `2aabbff` | commit2main | PASS | `logs/2026-10-31_ram.txt` | Controller 14 → 4 B; app_program .bss 1500 → 1318 B; image .bss 26252 → 26148 B (the 512-byte-aligned barcode ring absorbs part of the saving); .text +1096 B |
| 2026-10-31 | INT-11 | `42a79fa` | commit2main | PASS | `logs/2026-10-31_ram.txt` (re-run section) | Same image figures as `2aabbff`; controller object text 1524 B, the unused getter is dropped at link |
| 2026-10-31 | INT-05 | `7739da3` + `cd9aa1f` (#23 merged with #25) | commit2main | PASS (partial) | `logs/2026-10-31_int05_packed.txt` | Wheels lifted, no line; packed controller state. Six GP20 stops (`[safety] stop, cause 0x8`), five resets each `SAFE_STOP -> INIT: GP20` then `INIT -> READY: IMU calibrated`, every start `READY -> FOLLOW`. One stop came from AVOID (something passed in front of the sensor: `FOLLOW -> AVOID: obstacle`), handled the same. An impact fault after the last stop came from handling the car. Line following and junctions not covered (INT-07) |
| 2026-10-31 | INT-08 | `af4be0d` | commit2main | PASS (partial) | `../subsystem4/logs/2026-10-31_pr31_faults.txt` | Normal image, car in READY, not following, no line. Tilt past 45°: `[imu] FAULT: tilt`, `[safety] stop, cause 0x1`, `READY -> SAFE_STOP: safety latched`. GP20 not pressed; the following case is still owed |
| 2026-11-03 | INT-08 | `2c7101f` | commit2main | PASS (partial) | `logs/2026-11-03_course_stops.txt` | On the course while following: driving over a 30 mm hump raised `[imu] FAULT: impact`, then `[safety] stop, cause 0x1` and `FOLLOW -> SAFE_STOP: safety latched`. Impact instead of the defined tilt; GP20 not pressed after (the board was reset) |
| 2026-11-03 | INT-12 | `2c7101f` | commit2main | PASS (partial) | `logs/2026-11-03_course_stops.txt` | Line ending with no line beyond: `FOLLOW -> SAFE_STOP: line lost`, `[safety] stop, cause 0x8`. The GP20 reset and second run were not done (the board was reset) |

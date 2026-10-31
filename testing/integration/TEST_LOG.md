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
| INT-04 | Motion telemetry over MQTT | Motion telemetry | manual | Buddy 1 publishes `motion_control_pack()` every 100 ms; the dashboard decodes it as in `docs/MOTION_CONTROL.md` | 12-byte payload; decoded state, speeds and distance match the console `status` |
| INT-05 | Controller start, stop and reset with GP20 | I1, I11, FR11 | manual | Normal build, car on the line. 1. Wait for `READY`. 2. Press GP20. 3. After 1 m press GP20. 4. Press GP20 twice | 2: follows the line. 3: brakes, `SAFE_STOP: GP20`. 4: `READY`, then follows again |
| INT-06 | Controller hands the car to avoidance and back | I1, I6 | manual | Obstacle on a straight line 1 m ahead with one side open; then repeat with both sides blocked | Open side: `FOLLOW -> AVOID`, bypass, `AVOID -> FOLLOW`, follows the line on. Blocked: `SAFE_STOP: avoidance stopped`, no contact |
| INT-07 | Controller executes barcodes at the next junction | I2, I3, FR3 | manual | For each of A, B, C, D: barcode before a junction, start 0.5 m before it | `barcode N pending`, then at the junction a left 90°, right 90°, straight on, or U-turn, and follows the outgoing line |
| INT-08 | IMU fault latches a safe stop | I9, FR11 | manual | Car following; tilt it past 25° for 1 s; then press GP20 | `SAFE_STOP: IMU fault`, motors stop; GP20 does not leave the safe stop until reset |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |

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

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |

# Subsystem 3 — test log

**Subsystem:** Line following and barcodes  ·  **Student ID:** 2502564  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | Line Following task samples the sensors | FR2, I1 | auto | `TEST=1` image, read the console | `S3-U01 PASS`: the sample count changes within 100 ms |
| S3-U02 | Line state is bit-packed | Minimum-bit data | auto | As above | `S3-U02 PASS`: `line_following_t` is 8 bytes |
| S3-U03 | Each sensor reports black and white | FR2, A3 | manual | Sensors at their final height over the track. Move the car so the line passes under the left sensor, then the right, then both | `[line]` lines show `left=1` only over black on the left, `right=1` only on the right, `junction=1` with both, `last` = 1 (left) or 2 (right) |
| S3-U04 | Out-of-range speeds are rejected | I1 | auto | `TEST=1` image, read the console | `S3-U04 PASS`: `line_following_start()` refuses 0 and 501 mm/s |
| S3-U05 | Line following boots stopped | FR2 | auto | As above | `S3-U05 PASS`: `following=0` until started |
| S3-U06 | The car follows the line | FR2, NFR3 | manual | `TEST=1` image. Press GP20 briefly (under 1 s), put the car on a straight line, then a curved one, with a cross line at the end | Within 5 s the car follows at 200 mm/s without losing the line and stops at the cross line (`[line] bench done`); record the run with a video |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-31 | S3-U01 | `195db26` | mykacx | PASS | `logs/2026-10-31_unit.txt` | |
| 2026-10-31 | S3-U02 | `195db26` | mykacx | PASS | `logs/2026-10-31_unit.txt` | |
| 2026-10-31 | S3-U04 | `195db26` | mykacx | PASS | `logs/2026-10-31_unit.txt` | |
| 2026-10-31 | S3-U05 | `195db26` | mykacx | PASS | `logs/2026-10-31_unit.txt` | |
| 2026-10-31 | S3-U03 | `195db26` | mykacx | PASS | `logs/2026-10-31_unit.txt` | Left, right and both over the line read as expected; DO is 1 over black |
| 2026-10-31 | S3-U06 | `ca4b8b1` | mykacx | FAIL | `logs/2026-10-31_follow.txt` | Ran off the line: after one left=1 reading neither sensor saw the line again. Cause under investigation |

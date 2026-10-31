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

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |

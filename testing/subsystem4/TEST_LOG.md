# Subsystem 4 — test log

**Subsystem:** IMU and terrain  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | GY-511 answers on I2C0 and readings arrive (≥ 3 in 500 ms) | FR10 | auto | `subsystem4_test.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | flat reading: z ≈ +1000 mg, x and y ≈ 0 (±200 mg) | FR10, A4 | auto | Board flat, component side up; `subsystem4_test.c` | `[TEST] S4-U02 PASS` |
| S4-U03 | each axis sees gravity when tilted | FR10, A4 | manual | 1. Flash normal image. 2. Rest the board on each of its six sides for ~2 s. 3. Read `[imu]` lines. | the axis pointing up reads ≈ +1000 mg, pointing down ≈ −1000 mg, others ≈ 0 |
| S4-U04 | packed reading keeps −2048..2047 mg per axis and a wrapping 8-bit count, in 8 bytes | Minimum-bit CR | auto | `subsystem4_test.c` | `[TEST] S4-U04 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-25 | S4-U01 | `f5befac` | commit2main | PASS | `logs/2026-10-25_unit.txt` | Maker Pi Pico, GY-511 on Grove 3 |
| 2026-10-25 | S4-U02 | `f5befac` | commit2main | FAIL | `logs/2026-10-25_unit.txt` | Flat z ≈ 795 mg, below the 800 limit: Z offset ≈ −230 mg (see S4-U03) |
| 2026-10-25 | S4-U03 | `f5befac` | commit2main | PASS | `logs/2026-10-25_tilt.txt` | ±1 g per axis: X −1008/+1057, Y −961/+1044, Z −1255/+794 mg. Offsets ≈ X +25, Y +42, Z −231 mg |
| 2026-10-25 | S4-U04 | `f5befac` | commit2main | PASS | `logs/2026-10-25_unit.txt` | |

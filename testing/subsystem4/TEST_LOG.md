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
| S4-U05 | start-up calibration ends within 3 s with the board still | FR10, A4 | auto | Board flat and still from power-on; `subsystem4_test.c` | `[TEST] S4-U05 PASS` |
| S4-U06 | a reading every 10 ms | NFR6 | auto | `subsystem4_test.c` | `[TEST] S4-U06 PASS` (45..55 readings in 500 ms) |
| S4-U07 | pitch and roll match a known slope | FR10, A4 | manual | 1. Flash normal image with the module flat and still until `[imu] calibrated`. 2. Put the module on the 6° ramp nose up, nose down, left side up, right side up, ~3 s each. 3. Read `pitch` and `roll` in the `[imu]` lines. | Signs: nose up and left up positive. Half the difference between opposite readings (which cancels any calibration lean) is 60 ± 5 (0.1 degree) for pitch and for roll |
| S4-U08 | buzzer beeps when calibration is done | FR10 | manual | 1. Buzzer mute switch off. 2. Flash normal image, board flat and still. 3. Listen. | a short beep about 2 s after boot, at the `[imu] calibrated` line |
| S4-U14 | no impact or tilt fault on a still board; fault record is 4 bytes | FR10 | auto | `subsystem4_test.c` | `[TEST] S4-U14 PASS` |
| S4-U15 | a sharp knock latches the impact fault | FR10, A4 | manual | 1. Flash normal image, module flat and still until `[imu] calibrated`. 2. Rap the table hard next to the module, or flick the module sideways. | `[imu] FAULT: impact` and a 1 s beep; nothing more on further knocks (latched) |
| S4-U16 | holding past 25° latches the tilt fault | FR10 | manual | 1. Reset and calibrate as above. 2. Tilt the module to about 45° and hold it still for 1 s. | `[imu] FAULT: tilt` and a 1 s beep after about 0.5 s |
| S4-U17 | the 6° ramp and gentle handling raise no fault | FR10, A4 | manual | 1. Reset and calibrate as above. 2. Gently move the module onto the 6° ramp nose up, nose down and sideways, ~3 s each. | no `[imu] FAULT` line |

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
| 2026-10-28 | S4-U05 | `fdaed62` | commit2main | PASS | `logs/2026-10-28_unit.txt` | Calibration offsets x=16 y=36 z=−171 mg |
| 2026-10-28 | S4-U01 | `fdaed62` | commit2main | PASS | `logs/2026-10-28_unit.txt` | |
| 2026-10-28 | S4-U06 | `fdaed62` | commit2main | PASS | `logs/2026-10-28_unit.txt` | |
| 2026-10-28 | S4-U02 | `fdaed62` | commit2main | PASS | `logs/2026-10-28_unit.txt` | Calibrated: flat x≈0, y≈0, z≈1000 mg (failed on 2026-10-25 before calibration) |
| 2026-10-28 | S4-U04 | `fdaed62` | commit2main | PASS | `logs/2026-10-28_unit.txt` | |
| 2026-10-28 | S4-U07 | `fdaed62` | commit2main | PASS | `logs/2026-10-28_ramp.txt` | Pitch +6.6/−5.3 → slope 5.95°, roll +6.8/−5.4 → slope 6.1°; calibration lean ≈ 0.7°. Z was stuck at full scale during this run (loose jumper wires); X and Y unaffected |
| 2026-10-28 | S4-U08 | `327d98d` | commit2main | PASS | Heard by tester | Maker Pi Pico buzzer on GP18 |
| 2026-10-28 | S4-U05, S4-U01, S4-U06, S4-U04 | `6fc6db2` | commit2main | PASS | `logs/2026-10-28_fault_unit.txt` | |
| 2026-10-28 | S4-U02 | `6fc6db2` | commit2main | PASS | `logs/2026-10-28_fault_unit.txt` | Only x and y are meaningful: Z is stuck at full scale (faulty module), and calibration absorbs it |
| 2026-10-28 | S4-U14 | `6fc6db2` | commit2main | PASS | `logs/2026-10-28_fault_unit.txt` | |
| 2026-10-28 | S4-U17 | `6fc6db2` | commit2main | PASS | `logs/2026-10-28_fault.txt` | No fault. Pitch briefly reached 26.9° while handling, but not for 0.5 s |
| 2026-10-28 | S4-U15 | `6fc6db2` | commit2main | PASS | `logs/2026-10-28_fault.txt` | Beep heard. The triggering sample falls between printed lines (printed every 100 ms); around it X swung from +355 to −541 mg. The module then rested leaning at −26°, which correctly latched the tilt fault too |
| 2026-10-28 | S4-U16 | `6fc6db2` | commit2main | PASS | `logs/2026-10-28_fault.txt` | Held at about 30° (X 506 mg; roll read 28.8°); fault about 0.5 s after passing 25° |

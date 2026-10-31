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
| S4-U09 | hump height and detection, bench (module not on the car) | FR5 | manual | 1. Flash normal image, module flat and still until `[imu] calibrated`. 2. Lay the module on the 6° ramp with its right side (−X) up for ~3 s, then flat again. 3. Read `h=` and the `[imu] hump:` line. | `h` ≈ 114 × sin 6° = 12 ± 1 mm on the ramp; exactly one `[imu] hump: peak 11..13 mm` after it is laid flat |
| S4-U10 | right wheel height on a block | FR5, NFR4 | manual | Sensor mounted on the car. 1. Calibrate on flat ground. 2. Put the right wheel on a block of ruler-measured height (~10 mm). 3. Read `h=`. | `h` within ±1 mm of the block |
| S4-U11 | hump detected and measured when the car drives over it | FR5, NFR4, A4 | manual | Sensor mounted on the car. 1. Flash the test image (`make TEST=1`) with the car flat and still, right wheel ~5 cm before the hump, until `[imu] calibrated`. 2. Press GP21: the motion bench's `drive ±100`, `forward 300` and `backward 300` take the right wheel onto the hump, over it and back. 3. Read the `[imu] hump:` lines up to the first 180° turn. | exactly one hump per crossing; each peak within ±2 mm of the hump's ruler height |
| S4-U12 | no false humps on flat ground | FR5, A4 | manual | Sensor mounted on the car. 1. Flash the test image with the car flat and still on the floor until `[imu] calibrated`. 2. Press GP21 and let the whole motion bench sequence run (drives, moves, turns). | no `[imu] hump:` line |
| S4-U13 | packed hump record keeps 0..63 mm and a wrapping 4-bit count, in 4 bytes | Minimum-bit CR | auto | `subsystem4_test.c` | `[TEST] S4-U13 PASS` |
| S4-U14 | no impact or tilt fault on a still board; fault record is 4 bytes | FR10 | auto | `subsystem4_test.c` | `[TEST] S4-U14 PASS` |
| S4-U15 | a sharp knock latches the impact fault | FR10, A4 | manual | 1. Flash normal image, module flat and still until `[imu] calibrated`. 2. Rap the table hard next to the module, or flick the module sideways. | `[imu] FAULT: impact` and a 1 s beep; nothing more on further knocks (latched) |
| S4-U16 | holding past 25° latches the tilt fault | FR10 | manual | 1. Reset and calibrate as above. 2. Tilt the module to about 45° and hold it still for 1 s. | `[imu] FAULT: tilt` and a 1 s beep after about 0.5 s |
| S4-U17 | the 6° ramp and gentle handling raise no fault | FR10, A4 | manual | 1. Reset and calibrate as above. 2. Gently move the module onto the 6° ramp nose up, nose down and sideways, ~3 s each. | no `[imu] FAULT` line |
| S4-U18 | motion is steady on a still board; motion record is 4 bytes | FR10 | auto | `subsystem4_test.c` | `[TEST] S4-U18 PASS` |
| S4-U19 | sliding the module forward and stopping it reads accelerating then braking | FR10 | manual | 1. Flash normal image, module flat and still until `[imu] calibrated`. 2. Slide it sharply forward (−Y edge leading) about 10 cm and stop it sharply. | `[imu] motion: accelerating`, then `braking`, then `steady` |
| S4-U20 | a hump under the right wheel reads climbing then descending | FR10 | manual | 1. As above. 2. Lay the module on the 6° ramp right side up for ~3 s, then slowly lower that side back to flat. | `climbing` while on the ramp, `descending` while lowering, then `steady` |
| S4-U21 | an impact reads impact and stays | FR10 | manual | 1. As above. 2. Rap the table next to the module. | `[imu] motion: impact`, with no further motion lines |

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
| 2026-10-28 | S4-U05, S4-U01, S4-U06, S4-U04 | `81680a9` | commit2main | PASS | `logs/2026-10-28_hump_unit.txt` | Test image built just before the 200 ms hump guard was added; that change does not touch these tests |
| 2026-10-28 | S4-U02 | `81680a9` | commit2main | PASS | `logs/2026-10-28_hump_unit.txt` | Only x and y are meaningful: Z is stuck at full scale (faulty module), and calibration absorbs it |
| 2026-10-28 | S4-U13 | `81680a9` | commit2main | PASS | `logs/2026-10-28_hump_unit.txt` | Same build note as above |
| 2026-10-28 | S4-U09 | `81680a9` | commit2main | FAIL | `logs/2026-10-28_hump_bench.txt` | Height 11 mm on the ramp (expected 12 ± 1) and exactly one hump logged, but its peak was 24 mm: the jolt of placing the module by hand set it. Without the 200 ms guard the same handling logged 8 humps (`logs/2026-10-28_hump_bench_noguard.txt`). Peak smoothing is left until the car tests (S4-U11, S4-U12) |
| 2026-10-28 | S4-U05, S4-U01, S4-U06, S4-U04 | `6fc6db2` | commit2main | PASS | `logs/2026-10-28_fault_unit.txt` | |
| 2026-10-28 | S4-U02 | `6fc6db2` | commit2main | PASS | `logs/2026-10-28_fault_unit.txt` | Only x and y are meaningful: Z is stuck at full scale (faulty module), and calibration absorbs it |
| 2026-10-28 | S4-U14 | `6fc6db2` | commit2main | PASS | `logs/2026-10-28_fault_unit.txt` | |
| 2026-10-28 | S4-U17 | `6fc6db2` | commit2main | PASS | `logs/2026-10-28_fault.txt` | No fault. Pitch briefly reached 26.9° while handling, but not for 0.5 s |
| 2026-10-28 | S4-U15 | `6fc6db2` | commit2main | PASS | `logs/2026-10-28_fault.txt` | Beep heard. The triggering sample falls between printed lines (printed every 100 ms); around it X swung from +355 to −541 mg. The module then rested leaning at −26°, which correctly latched the tilt fault too |
| 2026-10-28 | S4-U16 | `6fc6db2` | commit2main | PASS | `logs/2026-10-28_fault.txt` | Held at about 30° (X 506 mg; roll read 28.8°); fault about 0.5 s after passing 25° |
| 2026-10-28 | S4-U05, S4-U01, S4-U06, S4-U02, S4-U04, S4-U14, S4-U13 | `dea91c7` | commit2main | PASS | `logs/2026-10-28_motion_unit.txt` | First board run of the hump and fault code together. Test image built before the accelerating/braking fix and the descending latch, which these tests do not touch; buzzer on GP22 |
| 2026-10-28 | S4-U18 | `dea91c7` | commit2main | PASS | `logs/2026-10-28_motion_unit.txt` | Same build note as above |
| 2026-10-28 | S4-U19 | `dea91c7` | commit2main | PASS | `logs/2026-10-28_motion_bench_nolatch.txt` | accelerating → steady → braking → steady. Built before the descending latch, which does not touch this. A 4° tilt within 0.5 s (wires) also read as braking for 0.7 s until the 2.5 s average caught up. The first version, comparing against zero, flickered and missed braking (`logs/2026-10-28_motion_bench_nofilter.txt`) |
| 2026-10-28 | S4-U20 | `dea91c7` | commit2main | FAIL | `logs/2026-10-28_motion_bench.txt` | climbing lasted 20 ms: the landing jolt set the peak to 20 mm while the module rested at 13–14 mm, so descending started at once. Same cause as S4-U09 (a jolt sets the peak); left for the car tests. Before the descending latch, slow lowering by hand flipped climbing/descending repeatedly (`logs/2026-10-28_motion_bench_nolatch.txt`) |
| 2026-10-28 | S4-U21 | `dea91c7` | commit2main | PASS | `logs/2026-10-28_motion_bench.txt` | Laying the module down on the table latched the impact fault; motion stayed impact with no further changes |
| 2026-10-31 | S4-U05, S4-U01, S4-U06, S4-U02, S4-U04, S4-U14, S4-U13, S4-U18 | `644aed3` | commit2main | PASS | `logs/2026-10-31_car_unit.txt` | First run with the sensor on the car (Robo Pico), with subsystem 2 from `main`. An earlier boot the same day failed S4-U05, U01, U06 and U02 because the car moved during calibration, which restarts it |
| 2026-10-31 | S4-U12 | `644aed3` | commit2main | PASS | `logs/2026-10-31_car_flat.txt` | Whole bench sequence, no hump and no fault. Height at most 5 mm, never for 200 ms; roll at most 6.9°. Starts and stops rock the car on its caster (Y −709 to +670 mg, pitch −25.8° for one sample). Accelerating and braking flicker at constant speed |
| 2026-10-31 | S4-U11 | `644aed3` | commit2main | PASS | `logs/2026-10-31_car_hump.txt` | Replica hump, 10 mm by ruler. Three crossings, one hump each: peaks 12, 11 and 11 mm. Each read climbing, descending, then steady, which S4-U20 could not show on the bench. On the car a jolt adds at most 2 mm to the peak, so the S4-U09 and S4-U20 bench FAILs came from placing the module by hand; no peak smoothing is added. S4-U10 not run: no block |

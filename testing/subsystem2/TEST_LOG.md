# Subsystem 2 — test log

**Subsystem:** Motion control  ·  **Student ID:** 2503068  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results. Manual tests use the `TEST=1` bench sequence in [docs/MOTION_CONTROL.md](../../docs/MOTION_CONTROL.md#bench-sequence).

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U11 | Encoder counts and direction | FR4, A2 | manual | Wheels lifted. After `bench done`, turn each wheel 10 turns forward by hand between two status lines | Each count rises by about 25 370 |
| S2-U12 | PID tuning | FR4, A2 | manual | Bench sequence on the floor; plot the `[trace]` lines of both `forward 300` moves (100 and 200 mm/s) | Follows the start ramp within 0.2 s, overshoot < 15%, steady error < 10 mm/s |
| S2-U13 | Distance and turn accuracy | FR4, NFR4 | manual | Bench sequence on the floor, 5 runs. In the 3 s pause after each move and turn, mark the wheel centres and the heading; measure each step afterwards | Distance within ±1%, turns within ±3° |
| S2-U14 | Control timing under load | FR11, NFR6 | manual | Full team `TEST=1` image with Wi-Fi; read `max_tick_us` after `bench done` | `max_tick_us` < 5000 (half the period) |
| S2-U16 | Boots idle with the motors braked | FR11 | auto | `subsystem2_test.c` | `[TEST] S2-U16 PASS` |
| S2-U17 | Out-of-range commands are rejected | FR4 | auto | `subsystem2_test.c` | `[TEST] S2-U17 PASS` |
| S2-U18 | A rejected command does not start the motors | FR11 | auto | `subsystem2_test.c` | `[TEST] S2-U18 PASS` |
| S2-U19 | 12-byte status and telemetry | Minimum-bit CR | auto | `subsystem2_test.c` | `[TEST] S2-U19 PASS` |
| S2-U20 | Control step runs and takes under 1 ms | NFR6 | auto | `subsystem2_test.c` | `[TEST] S2-U20 PASS` |
| S2-U21 | Moves, turns, drive and its expiry, stop, stall (also during renewed drive), late step, encoder flag, counter wrap, limits, telemetry layout | FR4, FR11, I3/I4/I5 | host | `cc -std=c99 -Wall -Wextra -Werror -pedantic -O2 app_program/subsystem2/motion_control.c testing/subsystem2/host/test_motion.c -lm -o /tmp/test_motion && /tmp/test_motion` (simulated robot, right motor 15% weaker) | `ALL PASS` |
| S2-U22 | RAM and message size before/after minimum-bit, same behaviour | Minimum-bit CR | host | Commands in `logs/2026-10-31_ram.txt` | Packed is smaller; full-width version passes S2-U21 |
| S2-U24 | Motor direction and speed, wheels lifted | FR4 | manual | Wheels lifted. Bench sequence, drive steps | Forward then backward; 100 ± 15 mm/s; `faults=0` |
| S2-U25 | Stall protection, wheels lifted | FR11 | manual | Wheels lifted. Bench sequence: hold the left wheel during the forward drive, the right during the backward drive | `state=3`, `faults=2` (right) or `1` (left) within about 1 s; both wheels stop |
| S2-U26 | Moves and turns finish, wheels lifted | FR4 | manual | Wheels lifted. Bench sequence, move and turn steps | Each ends with `state=2 faults=0`; turns change `heading` by about 90° |
| S2-U27 | Whole bench sequence returns to its start | FR4, NFR4 | manual | Floor, power bank, USB unplugged. Tape the start position (wheel centres) and direction, press GP21, place the car within 5 s. After `bench done`, measure the wheel centres from the tape and the heading from the start direction | Within 20 mm of the start and 5° of the start direction (about 1.6 m of driving and 540° of turns) |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-31 | S2-U11 | `673b8ca` + changes | xxcin | FAIL | `logs/2026-10-31_manual.txt` | Counts went up (direction right), but 5 turns gave 12 719 / 12 619, not about 11 700: the 2340 edges per turn in the code was wrong |
| 2026-10-31 | S2-U11 | `673b8ca` + changes | xxcin | PASS | `logs/2026-10-31_manual.txt` | 10 turns: left 25 390, right 25 358 (2539 / 2536 per turn). `MOTION_COUNTS_PER_REV` set to 2537 |
| 2026-10-31 | S2-U12 | `673b8ca` + changes, gains 256/16/0 | xxcin | FAIL | `logs/2026-10-31_manual.txt` | 100 mm/s: rise 0.18 s, overshoot 9%, steady 99.5 / 99.7. 200 mm/s: rise 0.52 s, overshoot 2.5%, steady 193 / 195. Failed the criterion then written (rise < 0.3 s) because the 500 mm/s² start ramp alone takes 0.34 s to 200 mm/s; criterion changed to "follows the ramp within 0.2 s", which both speeds meet. Gains kept |
| 2026-10-31 | S2-U12 | `dfe3e46` | commit2main | PASS (partial) | `logs/2026-10-31_floor.txt` | `forward 300` at 100 mm/s: 90% after 120 ms (ramp 140 ms), peak 110 (10%), steady 101.7 / 101.8. 200 mm/s move too short to judge |
| 2026-10-31 | S2-U12 | `523159c` | commit2main | FAIL | `logs/2026-10-31_pid.txt` | `forward 200` at 100 mm/s: left wheel at 0 for about 150 ms at each start, 90% after 920 ms, steady 95 / 95. Same with the previous controller; the left motor is weaker than in the `dfe3e46` run, likely since S2-U25. Recheck after looking at the motor |
| 2026-10-31 | S2-U12 | `00be97c` | commit2main | FAIL | `logs/2026-10-31_return.txt` | Floor, two runs. Start-ramp lag 0.12 and 0.24 s at 100 mm/s, 0.24 and 0.22 s at 200 mm/s (limit 0.2 s): the left wheel starts late. Overshoot 12% / 6–8% (100 ms averages; single samples step by 8 mm/s), steady error 4 mm/s or less |
| 2026-10-31 | S2-U12 | `2335313` | commit2main | PASS | `logs/2026-10-31_offset125.txt` | Feedforward 125‰ (was 100‰; 150‰ tried: 200 mm/s lag 0.46 s). Start-ramp lag 0.06 / 0.16 s at 100 mm/s, 0.20 s at 200 mm/s; overshoot 10%; steady error 5 mm/s or less |
| 2026-10-31 | S2-U13 | `673b8ca` + changes | xxcin | PASS (partial) | Measured by tester | `forward` and `backward` 50 and 100 mm within ±2 mm. 500 mm not possible on the USB cable |
| 2026-10-31 | S2-U13 | `673b8ca` + changes, track 113 500 µm | xxcin | PASS | Measured by tester | `right 90`, `left 90`, `left 180`, `right 360` within ±2°; track unchanged |
| 2026-10-31 | S2-U13 | `673b8ca` + changes | xxcin | PASS | `logs/2026-10-31_accuracy.txt` | 5 runs each. `forward 100`: 101, 100, 100, 101, 100 mm. `backward 100`: 100, 100, 101, 100, 100 mm. `right 90`: 89, 90, 88, 90, 90°. `left 180`: 178, 179, 180, 180, 181°. `left 90` angles not written down |
| 2026-10-31 | S2-U13 | `dfe3e46` | commit2main | PASS (partial) | `logs/2026-10-31_floor.txt` | Floor, USB connected: moves 302 and 300 mm and turns to 90°, 0°, 180° by odometry, `faults=0`. Not measured by hand |
| 2026-10-31 | S2-U14 | `673b8ca` + changes | xxcin | PASS (partial) | `logs/2026-10-31_accuracy.txt` | Motion-only bench image with Wi-Fi running, not the full team image: 45 s drive, `faults=0`, `max_tick_us=1939` |
| 2026-10-31 | S2-U14 | `dfe3e46` | commit2main | PASS | `logs/2026-10-31_bench.txt` | Full team image, Wi-Fi up but not joined: `max_tick_us=541` |
| 2026-10-31 | S2-U16–U20 | `b0f93f7` + changes | xxcin | PASS | `logs/2026-10-31_unit.txt` | Full team image. The S4 failures in the log are expected: no IMU fitted |
| 2026-10-31 | S2-U16–U20 | `dfe3e46` | commit2main | PASS | `logs/2026-10-31_bench.txt` | Full team image on the car |
| 2026-10-31 | S2-U21 | `673b8ca` + changes | xxcin | PASS | `logs/2026-10-31_host.txt` | Simulation: turns within 1.3°, moves within 1.5 mm. Also passes with ASan and UBSan |
| 2026-10-31 | S2-U21 | `508f9fb` | commit2main | PASS | `logs/2026-10-31_host_rerun.txt` | Re-run after the cleanup and review fixes: all earlier results unchanged; new stall-during-`drive()` and late-step checks pass |
| 2026-10-31 | S2-U22 | `48f7ea3` | commit2main | PASS | `logs/2026-10-31_ram.txt` | RAM 112 → 52 B; status 36 → 12 B; telemetry 12 B |
| 2026-10-31 | S2-U24 | `673b8ca` + changes | xxcin | FAIL | Observed by tester | `drive 100 100` turned both wheels backwards, then fault 4. Both motor signs flipped in `drv_motion.c` |
| 2026-10-31 | S2-U24 | `673b8ca` + changes, motor signs flipped | xxcin | PASS | `logs/2026-10-31_manual.txt` | Right directions; 95–104 mm/s; wheels within 3 edges; `faults=0` |
| 2026-10-31 | S2-U24 | `dfe3e46` | commit2main | PASS | `logs/2026-10-31_bench.txt` | Wheels lifted. Forward then backward (seen by tester); 107 / 107 then −102 / −94 mm/s; `faults=0` |
| 2026-10-31 | S2-U25 | `673b8ca` + changes, 2537 edges/turn | xxcin | PASS | `logs/2026-10-31_manual.txt` | Right held: `faults=2`; left held: `faults=1`; both braked |
| 2026-10-31 | S2-U25 | `508f9fb` | commit2main | PASS | `logs/2026-10-31_stall.txt` | Wheels lifted. Left held: each step `state=3 faults=1` within about 1 s, both braked. Right held: `faults=2`. Renewed `drive()` did not restart a stall |
| 2026-10-31 | S2-U26 | `673b8ca` + changes | xxcin | PASS | `logs/2026-10-31_manual.txt` | `left 90`: heading +90°. `right 90`: heading −89°. All `faults=0` |
| 2026-10-31 | S2-U26 | `dfe3e46` | commit2main | PASS | `logs/2026-10-31_bench.txt` | Wheels lifted. All `state=2 faults=0`; heading +90°, −90°, +181°; moves 303 and 300 mm |
| 2026-10-31 | S2-U27 | `00be97c` | commit2main | PASS (partial) | `logs/2026-10-31_return.txt` | USB connected (laptop carried) instead of the power bank. Run B: 10 mm forward and 5 mm left of the start (11 mm). Heading 4° by the encoders (run A: 1°), not measured by hand. `faults=0` |
| 2026-10-31 | S2-U27 | `2335313` | commit2main | PASS (partial) | `logs/2026-10-31_offset125.txt` | USB connected (laptop carried). 10 mm forward and 3 mm left of the start. Heading 3° by the encoders, not measured by hand. `faults=0` |
| 2026-10-31 | S2-U27 | `2335313` | commit2main | PASS (partial) | Measured by tester | Power bank, USB unplugged, 3 runs: about 15 mm left of the start each time (worse than the USB runs). Forward error and heading not recorded |
| 2026-10-31 | S2-U27 | `36cfa5f`, feedforward 100‰ | commit2main | FAIL | Measured by tester | Power bank, 3 runs: about 20 mm left each time, at the limit and worse than 125‰. Turns looked right. Forward error not recorded. 125‰ kept |
| 2026-10-31 | S2-U27 | `36cfa5f` | commit2main | FAIL | Measured by tester | Rerun at 125‰ on the power bank: about 20 mm left, turns looked worse than at 100‰. Forward error not recorded. Feedforward set back to 100‰ for the turns |
| 2026-10-31 | S2-U16, S2-U17, S2-U18, S2-U19, S2-U20 | `4b89bd3` | commit2main | PASS | `logs/2026-10-31_barr_shared_unit.txt` | After the BARR-C refactor of the shared app files (`app_main.c`, `app.h`, `test_report.h`) |

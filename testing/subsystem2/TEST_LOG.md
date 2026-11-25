# Subsystem 2 — test log

**Subsystem:** Motion control  ·  **Student ID:** 2503068  ·  **Code:** `app_program/subsystem2/`

Methods and how to record results: `testing/README.md`. Manual tests use the `TEST=1` bench sequence in [docs/MOTION_CONTROL.md](../../docs/MOTION_CONTROL.md#bench-sequence).

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | Encoder counts and direction | FR4, A2 | manual | Wheels lifted. After `bench done`, turn each wheel 10 turns forward by hand between two status lines | Each count rises by about 25 370 |
| S2-U02 | PID tuning | FR4, A2 | manual | Bench sequence on the floor; plot the `[trace]` lines of both `forward 300` moves (100 and 200 mm/s) | Follows the start ramp within 0.2 s, overshoot < 15%, steady error < 10 mm/s |
| S2-U03 | Distance and turn accuracy | FR4, NFR4 | manual | Bench sequence on the floor, 5 runs. In the 3 s pause after each move and turn, mark the wheel centres and the heading; measure each step afterwards | Distance within ±1%, turns within ±3° |
| S2-U04 | Control timing under load | FR11, NFR6 | manual | Full team `TEST=1` image with Wi-Fi; read `max_tick_us` after `bench done` | `max_tick_us` < 5000 (half the period) |
| S2-U05 | Boots idle with the motors braked | FR11 | auto | `subsystem2_test.c` | `[TEST] S2-U05 PASS` |
| S2-U06 | Out-of-range commands are rejected | FR4 | auto | `subsystem2_test.c` | `[TEST] S2-U06 PASS` |
| S2-U07 | A rejected command does not start the motors | FR11 | auto | `subsystem2_test.c` | `[TEST] S2-U07 PASS` |
| S2-U08 | 12-byte status and telemetry | Minimum-bit CR | auto | `subsystem2_test.c` | `[TEST] S2-U08 PASS` |
| S2-U09 | Control step runs and takes under 1 ms | NFR6 | auto | `subsystem2_test.c` | `[TEST] S2-U09 PASS` |
| S2-U10 | Start-up latch, moves, turns, drive and its expiry, stop, stall (also during renewed drive), late step, encoder flag, counter wrap, limits, telemetry layout | FR4, FR11, I3/I4/I5 | host | `cc -std=c99 -Wall -Wextra -Werror -pedantic -O2 app_program/subsystem2/motion_control.c testing/subsystem2/host/test_motion.c -lm -o /tmp/test_motion && /tmp/test_motion` (simulated robot, right motor 15% weaker) | `ALL PASS` |
| S2-U11 | RAM and message size before/after minimum-bit, same behaviour | Minimum-bit CR | host | Commands in `logs/2026-10-31_ram.txt` | Packed is smaller; full-width version passes S2-U10 |
| S2-U12 | Motor direction and speed, wheels lifted | FR4 | manual | Wheels lifted. Bench sequence, drive steps | Forward then backward; 100 ± 15 mm/s; `faults=0` |
| S2-U13 | Stall protection, wheels lifted | FR11 | manual | Wheels lifted. Bench sequence: hold the left wheel during the forward drive, the right during the backward drive | `state=3`, `faults=2` (right) or `1` (left) within about 0.5 s; both wheels stop |
| S2-U14 | Moves and turns finish, wheels lifted | FR4 | manual | Wheels lifted. Bench sequence, move and turn steps | Each ends with `state=2 faults=0`; turns change `heading` by about 90° |
| S2-U15 | Whole bench sequence returns to its start | FR4, NFR4 | manual | Floor, power bank, USB unplugged. Tape the start position (wheel centres) and direction, press GP21, place the car within 5 s. After `bench done`, measure the wheel centres from the tape and the heading from the start direction | Within 20 mm of the start and 5° of the start direction (about 1.6 m of driving and 540° of turns) |
| S2-U16 | A latched safety stop refuses every command | FR11 | auto | `subsystem2_test.c` | `[TEST] S2-U16 PASS` |
| S2-U17 | Commands are accepted again after the latch is released | FR11 | auto | `subsystem2_test.c` | `[TEST] S2-U17 PASS` |

## Results

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-31 | S2-U01 | `673b8ca` + changes | xxcin | FAIL | `logs/2026-10-31_manual.txt` | Direction right, but 5 turns gave 12 719 / 12 619, not about 11 700: the 2340 edges per turn in the code was wrong |
| 2026-10-31 | S2-U01 | `673b8ca` + changes | xxcin | PASS | `logs/2026-10-31_manual.txt` | 10 turns: left 25 390, right 25 358 (2539 / 2536 per turn). `MOTION_COUNTS_PER_REV` set to 2537 |
| 2026-10-31 | S2-U02 | `673b8ca` + changes, gains 256/16/0 | xxcin | FAIL | `logs/2026-10-31_manual.txt` | 200 mm/s rise 0.52 s against the criterion then written (rise < 0.3 s), but the 500 mm/s² start ramp alone takes 0.34 s. Criterion changed to "follows the ramp within 0.2 s", which both speeds meet (overshoot 9% / 2.5%, steady 99.5–99.7 and 193–195 mm/s). Gains kept |
| 2026-10-31 | S2-U02 | `00be97c` | commit2main | FAIL | `logs/2026-10-31_return.txt` | Floor, two runs. The left wheel starts late: start-ramp lag up to 0.24 s (limit 0.2 s). Overshoot 12% at 100 mm/s, steady error 4 mm/s or less |
| 2026-10-31 | S2-U02 | `2335313` | commit2main | PASS | `logs/2026-10-31_offset125.txt` | Fix: feedforward 125‰ (was 100‰; 150‰ gave 0.46 s lag at 200 mm/s). Start-ramp lag 0.06 / 0.16 s at 100 mm/s, 0.20 s at 200 mm/s; overshoot 10%; steady error 5 mm/s or less |
| 2026-10-31 | S2-U03 | `673b8ca` + changes, track 113 500 µm | xxcin | PASS | Measured by tester | `right 90`, `left 90`, `left 180`, `right 360` within ±2°; track width kept |
| 2026-10-31 | S2-U03 | `673b8ca` + changes | xxcin | PASS | `logs/2026-10-31_accuracy.txt` | 5 runs each. `forward 100`: 100–101 mm. `backward 100`: 100–101 mm. `right 90`: 88–90°. `left 180`: 178–181° |
| 2026-10-31 | S2-U04 | `af4be0d` | commit2main | PASS | `logs/2026-10-31_pr31_bench.txt` | Full team image with Wi-Fi: `max_tick_us=540` |
| 2026-10-31 | S2-U10 | `673b8ca` + changes | xxcin | PASS | `logs/2026-10-31_host.txt` | Simulation: turns within 1.3°, moves within 1.5 mm. Also passes with ASan and UBSan |
| 2026-10-31 | S2-U11 | `48f7ea3` | commit2main | PASS | `logs/2026-10-31_ram.txt` | RAM 112 → 52 B; status 36 → 12 B; telemetry 12 B |
| 2026-10-31 | S2-U12 | `673b8ca` + changes | xxcin | FAIL | Observed by tester | `drive 100 100` turned both wheels backwards, then fault 4. Both motor signs flipped in `drv_motion.c` |
| 2026-10-31 | S2-U12 | `673b8ca` + changes, motor signs flipped | xxcin | PASS | `logs/2026-10-31_manual.txt` | Right directions; 95–104 mm/s; wheels within 3 edges; `faults=0` |
| 2026-10-31 | S2-U13 | `508f9fb` | commit2main | PASS | `logs/2026-10-31_stall.txt` | Left held: each step `state=3 faults=1` within about 1 s, both braked. Right held: `faults=2`. Renewed `drive()` did not restart a stall |
| 2026-10-31 | S2-U14 | `dfe3e46` | commit2main | PASS | `logs/2026-10-31_bench.txt` | All `state=2 faults=0`; heading +90°, −90°, +181°; moves 303 and 300 mm |
| 2026-10-31 | S2-U15 | `2335313` | commit2main | PASS (partial) | `logs/2026-10-31_offset125.txt` | USB connected (laptop carried), not the power bank. 10 mm forward and 3 mm left of the start; heading 3° by the encoders, not measured by hand |
| 2026-10-31 | S2-U15 | `2335313` | commit2main | PASS (partial) | Measured by tester | Power bank, 3 runs: about 15 mm left of the start each time, worse than on USB. Forward error and heading not recorded |
| 2026-10-31 | S2-U15 | `36cfa5f`, feedforward 100‰ | commit2main | FAIL | Measured by tester | Power bank, 3 runs: about 20 mm left each time, at the limit; turns looked right |
| 2026-10-31 | S2-U15 | `36cfa5f` | commit2main | FAIL | Measured by tester | At 125‰ on the power bank: still about 20 mm left, and turns looked worse than at 100‰. Feedforward set back to 100‰ for the turns |
| 2026-10-31 | S2-U13 | `45f32a5` | commit2main | PASS | `logs/2026-10-31_faults_stall_left.txt`, `logs/2026-10-31_faults_stall_right.txt` | Shared fault bitmask. Left held: every step `state=3 faults=1`, both braked. Right held: `faults=2`; the next command cleared the bit before it latched again. With the wheels free (`logs/2026-10-31_faults_wheels_free.txt`) every step ended `faults=0` apart from one `faults=4` (missed encoder phase), cleared by the next command |
| 2026-10-31 | S2-U05, S2-U06, S2-U07, S2-U08, S2-U09, S2-U16, S2-U17 | `af4be0d` | commit2main | PASS | `../integration/logs/2026-10-31_pr31_unit.txt` | Full team `TEST=1` image. Encoder edges through the shared GPIO interrupt (PR #31) |
| 2026-10-31 | S2-U12 | `af4be0d` | commit2main | PASS | `logs/2026-10-31_pr31_bench.txt` | Forward then backward, `speed=96,96` then `-112,-112`, `faults=0` |
| 2026-10-31 | S2-U14 | `af4be0d` | commit2main | PASS (partial) | `logs/2026-10-31_pr31_bench.txt` | Every move and turn ends `state=2`; turns read heading 90, 1, 180, 1. The `forward 300` at 200 mm/s ended `faults=4` (one missed encoder phase), as before PR #31; every other step `faults=0` |
| 2026-10-31 | S2-U01 | `af4be0d` | commit2main | PASS | `logs/2026-10-31_pr31_bench.txt` | Left +25 353 for 10 turns; right +27 902 for 11 turns (expected 27 907) |
| 2026-11-19 | S2-U10 | `3e7007a` | commit2main | PASS | `logs/2026-11-19_exact_periods_host.txt` | `motion_control.c` unchanged |
| 2026-11-23 | S2-U10 | `354bea5` (#58) | commit2main | PASS | `logs/2026-11-23_stall_latch_host.txt` | Blocked wheel: stall fault after 0.5 s. Before `motion_control_init()` the latch is set and a release is ignored |
| 2026-11-24 | S2-U13 | `05bb0ff` (#58) | commit2main | PASS | `logs/2026-11-24_stall_bench.txt` | Right wheel held in the backward drive: at 0 from 170080 to 170580 ms, then `state=3 faults=2`. Left wheel held in the left 90: `state=3 faults=1` about 0.48 s after it stopped. Right held in the right 90: `faults=2` |
| 2026-11-24 | S2-U12 | `05bb0ff` (#58) | commit2main | PASS | `logs/2026-11-24_stall_bench.txt` | Wheels free, whole bench sequence: no stall. Every step `faults=0` except `forward 300` at 200 mm/s, `faults=4` (one missed encoder phase, as before) |
| 2026-11-25 | S2-U05, S2-U06, S2-U07, S2-U08, S2-U09, S2-U16, S2-U17 | `dbb28bd` (#70) | commit2main | PASS | `logs/2026-11-25_auto.txt` | Auto checks on the refactored code: 7 of 7 |
| 2026-11-25 | S2-U14 | `dbb28bd` (#70) | commit2main | PASS (partial) | `logs/2026-11-25_s2_u14_bench.txt` | Wheels lifted. Every logged step ended `state=2`; `forward 300` at 200 mm/s again ended with `faults=4` (encoder). The `[trace]` lines overfill the console buffer, so 5 of the 10 step reports are missing from the log |
| 2026-11-25 | S2-U15 | `dbb28bd` (#70) | commit2main | PASS (partial) | Measured by tester | Floor, power bank: ended 15 mm from the start (limit 20 mm), after the FAIL of 20 mm. Heading not measured, so the 5° part is not shown |

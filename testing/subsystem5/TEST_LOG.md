# Subsystem 5 — test log

**Subsystem:** Ultrasonic scanning and obstacle avoidance  ·  **Student ID:** 2503992  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results. Manual tests use the `TEST=1` bench sequence at the end of `subsystem5_test.c`: it requests a forward scan and, if that calls for a detailed scan, confirms the stop itself (the sensor is on the desk) and prints the profile. Wiring: TRIG on GP0, level-shifted ECHO on GP1, SG90 on GP12, USB console.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | Packed record sizes | Minimum-bit CR | auto | `subsystem5_test.c` | sample 8, profile 16, scan 100, control 32, health 36 bytes; unpacked sample 16 |
| S5-U02 | Echo time to distance | FR6 | auto | `subsystem5_test.c` | 5800 µs is 1000 mm |
| S5-U03 | No echo is invalid and never clears the path | FR6, NFR5 | auto | `subsystem5_test.c` | `[TEST] S5-U03 PASS` |
| S5-U04 | Closest point of a coarse scan | FR6 | auto | `subsystem5_test.c` | 200 mm at 90° |
| S5-U05 | Obstacle edges of a centred obstacle | FR6 | auto | `subsystem5_test.c` | Both edges at 90° |
| S5-U06 | Only the left side is clear | FR7 | auto | `subsystem5_test.c` | `BYPASS_LEFT` |
| S5-U07 | Only the right side is clear | FR7 | auto | `subsystem5_test.c` | `BYPASS_RIGHT` |
| S5-U08 | Both sides blocked | FR7 | auto | `subsystem5_test.c` | `SAFE_STOP`, no reverse |
| S5-U09 | All readings invalid | FR7, NFR5 | auto | `subsystem5_test.c` | Profile invalid, one retry requested |
| S5-U10 | Forward reading age and angle | NFR6 | auto | `subsystem5_test.c` | Clear at 200 ms old, not at 201 ms or sideways |
| S5-U11 | Scan retries are bounded | NFR5 | auto | `subsystem5_test.c` | Two retries, then latched `SAFE_STOP` |
| S5-U12 | Fault bitmask | Minimum-bit CR | auto | `subsystem5_test.c` | Three faults combine to 97 in 7 bits |
| S5-U13 | Median filter | FR6 | auto | `subsystem5_test.c` | Ignores the invalid reading, takes the nearer of two |
| S5-U14 | Timestamp wrap | NFR6 | auto | `subsystem5_test.c` | Age correct across 2³² ms |
| S5-U15 | Detailed scan waits for the stop | FR7 | auto | `subsystem5_test.c` | `WAIT_STOP`, no scan requested |
| S5-U16 | Confirmed stop starts the coarse scan | FR6, FR7 | auto | `subsystem5_test.c` | `PROFILING`, scan pending |
| S5-U17 | Cancel latches the stop | FR7 | auto | `subsystem5_test.c` | `STOPPED`; activation does not unlatch |
| S5-U18 | Line recovery is bounded | FR7, NFR5 | auto | `subsystem5_test.c` | Two retries, then `STOPPED` with recovery fault |
| S5-U19 | Tie-break and calibration gate | FR7, NFR5 | auto | `subsystem5_test.c` | Left wins a tie; uncalibrated gives `SAFE_STOP` |
| S5-U20 | An unknown side is never clear | NFR5 | auto | `subsystem5_test.c` | `SAFE_STOP` |
| S5-U21 | Missing stop confirmation times out | NFR5 | auto | `subsystem5_test.c` | `STOPPED` with scan timeout after 2000 ms |
| S5-U22 | Fine reading replaces the coarse one at the same angle | FR6 | auto | `subsystem5_test.c` | Closest 150 mm, oldest timestamp kept |
| S5-U23 | Old profile is not used | NFR6 | auto | `subsystem5_test.c` | `SAFE_STOP` after 6000 ms |
| S5-U24 | Obstacle within 100 mm | NFR5 | auto | `subsystem5_test.c` | `STOPPED`, `SAFE_STOP` |
| S5-U25 | Activation cannot clear a stop | FR7 | auto | `subsystem5_test.c` | Stays `STOPPED` |
| S5-U26 | Reset needs standstill and a new activation | FR7 | auto | `subsystem5_test.c` | `MONITOR`, inactive, no forward reading kept |
| S5-U27 | Range limits | FR6 | auto | `subsystem5_test.c` | 20 and 4000 mm valid, outside invalid |
| S5-U28 | Filter of invalid readings stays invalid | FR6 | auto | `subsystem5_test.c` | `[TEST] S5-U28 PASS` |
| S5-U29 | Planner stop keeps the profile, cancel drops it | FR7 | auto | `subsystem5_test.c` | `[TEST] S5-U29 PASS` |
| S5-U31 | Simulated 1160 µs echo | FR6 | host | `bash testing/subsystem5/host/hcsr04/run.sh` (real driver, simulated registers) | About 200 mm |
| S5-U32 | No rising echo | NFR5 | host | As above | Invalid, `S5_NO_ECHO` |
| S5-U33 | Echo stuck high | NFR5 | host | As above | Invalid, `S5_ECHO_TIMEOUT` |
| S5-U38 | 10 µs trigger pulse without `WaitUsec` | FR6 | host | As above | TRIG high ≥ 10 µs |
| S5-H13-A | Clear path | FR6 | manual | Bench sequence; flat target 350–4000 mm ahead | Valid forward range, `MONITOR`/`CONTINUE`, no detailed scan |
| S5-H13-B | Obstacle profile | FR6, A5 | manual | Bench sequence; box 200–300 mm ahead, background within 4 m | Coarse then fine scan, valid profile; `SAFE_STOP` while the calibration gate is off |
| S5-H13-C | Obstacle too close | NFR5 | manual | Bench sequence; flat target 20–100 mm ahead | `STOPPED`/`SAFE_STOP`, no detailed scan |
| S5-H13-D | No echo | NFR5 | manual | Bench sequence; nothing in range | Never clear; bounded retries, then `SAFE_STOP` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console; also built on the PC by `bash testing/subsystem5/host/run.sh`;
**host** = runs on the PC only;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-29 | S5-U01–U29 | Before `f7ec9a7` | calebwee04 | PASS | `logs/2026-10-29_host_unit.txt` | Host build, not on the Pico |
| 2026-10-29 | S5-U30–U39 | Before `f7ec9a7` | calebwee04 | PASS | `logs/2026-10-29_host_driver.txt` | |
| 2026-10-29 | S5-H13 | Unknown, before the TIMER and trigger fixes | calebwee04 | FAIL | `logs/2026-10-29_bench.txt` | Forward reading invalid; the coarse scan never finished and the 42 s bench deadline hit with `pending=1`. `drv_hcsr04.c` then started TIMER itself and replaced `WaitUsec` (stalls on core 1) for the trigger pulse. Not re-run on hardware since |
| 2026-10-31 | S5-U01–U29 | `9cacfa8` | commit2main | PASS | `logs/2026-10-31_host_unit.txt` | Re-run after the width fields were removed. Host build, not on the Pico. Sizes unchanged: sample 8 (unpacked 16), profile 16, scan 100, control 32, health 36 bytes; the Arm build checks the same sizes with `_Static_assert` |
| 2026-10-31 | S5-U30–U39 | `9cacfa8` | commit2main | PASS | `logs/2026-10-31_host_driver.txt` | |
| 2026-10-31 | S5-U31–U34, S5-U36–U39 | `62915d5` | commit2main | PASS | `logs/2026-10-31_host_driver_tick.txt` | Re-run after the TIMER start-up moved to `usermain()`; S5-U30 and S5-U35 retired with that code |
| 2026-10-31 | S5-U01–U29 | `c9b7543` | commit2main | PASS | `logs/2026-10-31_host_unit_simplify.txt` | Re-run after the cleanup review. Host build, not on the Pico |
| 2026-10-31 | S5-U31–U33, S5-U38 | `c9b7543` | commit2main | PASS | `logs/2026-10-31_host_driver_simplify.txt` | S5-U34, U36, U37 and U39 retired with the backup deadlines they tested |

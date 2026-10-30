# Subsystem 2 — test log

**Subsystem:** Motion control  ·  **Student ID:** 2503068  ·  **Code:** `app_program/subsystem2/`

How to run and record tests: `testing/README.md`. Console commands: [docs/MOTION_CONTROL.md](../../docs/MOTION_CONTROL.md). PID tuning and accuracy report: [CALIBRATION.md](CALIBRATION.md).

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01–U09, S2-U15 | Retired: these tested the code replaced on 2026-10-31. The IDs are not reused | — | — | — | — |
| S2-U10 | PWM frequency, braked at boot and after stop | FR4 | manual | Wheels lifted. Scope GP8–GP11 at boot, during `drive 150 150`, after `stop` | 20 kHz; all low at boot and after stop |
| S2-U11 | Encoder counts and direction | FR4, A2 | manual | Wheels lifted. `status`, turn each wheel 10 turns forward by hand, `status` | Each count rises by about 25 370 |
| S2-U12 | PID tuning | FR4, A2 | manual | `trace`, then `forward 300` at 100 and at 200 mm/s; plot the trace | Follows the start ramp within 0.2 s, overshoot < 15%, steady error < 10 mm/s |
| S2-U13 | Distance and turn accuracy | FR4, NFR4 | manual | On the floor, 5 runs each of `forward`, `backward`, `left 90`, `right 90`, `left 180`; measure. Record in CALIBRATION.md | Distance within ±1%, turns within ±3° |
| S2-U14 | Control timing under load | FR11, NFR6 | manual | Full team image with Wi-Fi; `drive 100 100` for 1 min, then `status` | `max_tick_us` < 5000 (half the period) |
| S2-U16 | Boots idle with the motors braked | FR11 | auto | `make S2_ONLY=1 TEST=1` or `make TEST=1`, flash, read the console | `[TEST] S2-U16 PASS` |
| S2-U17 | Out-of-range commands are rejected | FR4 | auto | Same image | `[TEST] S2-U17 PASS` |
| S2-U18 | A rejected command does not start the motors | FR11 | auto | Same image | `[TEST] S2-U18 PASS` |
| S2-U19 | 12-byte status and telemetry | Minimum-bit CR | auto | Same image | `[TEST] S2-U19 PASS` |
| S2-U20 | Control step runs and takes under 1 ms | NFR6 | auto | Same image | `[TEST] S2-U20 PASS` |
| S2-U21 | Moves, turns, drive and its expiry, stop, stall, encoder flag, counter wrap, limits, track setting, telemetry layout | FR4, FR11, I3/I4/I5 | host | `make test` (simulated robot, right motor 15% weaker) | `ALL PASS` |
| S2-U22 | RAM and message size before/after minimum-bit, same behaviour | Minimum-bit CR | host | Commands in `logs/2026-10-31_ram.txt` | Packed is smaller; full-width version passes S2-U21 |
| S2-U23 | Builds without warnings, formatting, sanitizers | NFR1, NFR2 | host | `make`, `make TEST=1`, `clang-format --dry-run`, `make test` with ASan/UBSan | No warnings, no format changes, all pass |
| S2-U24 | Motor direction and speed, wheels lifted | FR4 | manual | `drive 100 100`, `status`, `stop`, `drive -100 -100`, `status`, `stop` | Forward then backward; 100 ± 15 mm/s; `faults=0` |
| S2-U25 | Stall protection, wheels lifted | FR11 | manual | `drive 100 100` and hold one wheel 2 s; repeat for the other | `state=3`, `faults=2` (right) or `1` (left) within about 1 s; both wheels stop |
| S2-U26 | Moves and turns finish, wheels lifted | FR4 | manual | `forward 300`, `left 90`, `right 90` | Each ends with `state=2 faults=0`; turns change `heading` by about 90° |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-26 | S2-U04–U08 | Uncommitted working tree; fingerprint in log | Codex host fixture | PASS | `logs/2026-10-26_host.txt` | Software simulation, not hardware |
| 2026-10-26 | S2-U09 | Same worktree fingerprint | Codex cross-build | PASS | `logs/2026-10-26_representation.txt` | Generated full-width reference, not deleted/historical code |
| 2026-10-26 | S2-U15 | Same worktree fingerprint | Codex toolchain | PASS | `logs/2026-10-26_build.txt`, `logs/2026-10-26_sanitizers.txt` | Fresh source snapshot, static checks and sanitizers |
| 2026-10-31 | S2-U16–U20 | `b0f93f7` + changes | 2503068 | PASS | `logs/2026-10-31_unit.txt` | Full team image. The S4 failures in the log are expected: no IMU fitted |
| 2026-10-31 | S2-U16–U20 | `673b8ca` + changes | 2503068 | PASS | `logs/2026-10-31_unit_s2only.txt` | `S2_ONLY=1 TEST=1` image |
| 2026-10-31 | S2-U11 | `673b8ca` + changes | 2503068 | FAIL | `logs/2026-10-31_manual.txt` | Counts went up (direction right), but 5 turns gave 12 719 / 12 619, not about 11 700: the 2340 edges per turn in the code was wrong |
| 2026-10-31 | S2-U24 | `673b8ca` + changes | 2503068 | FAIL | Observed by tester | `drive 100 100` turned both wheels backwards, then fault 4. Both motor signs flipped in `drv_motion.c` |
| 2026-10-31 | S2-U24 | `673b8ca` + changes, motor signs flipped | 2503068 | PASS | `logs/2026-10-31_manual.txt` | Right directions; 95–104 mm/s; wheels within 3 edges; `faults=0` |
| 2026-10-31 | S2-U11 | Same | 2503068 | PASS | `logs/2026-10-31_manual.txt` | 10 turns: left 25 390, right 25 358. `MOTION_COUNTS_PER_REV` set to 2537 |
| 2026-10-31 | S2-U25 | `673b8ca` + changes, 2537 edges/turn | 2503068 | PASS | `logs/2026-10-31_manual.txt` | Right held: `faults=2`; left held: `faults=1`; both braked |
| 2026-10-31 | S2-U26 | Same | 2503068 | PASS | `logs/2026-10-31_manual.txt` | `left 90`: heading +90°. `right 90`: heading −89°. All `faults=0` |
| 2026-10-31 | S2-U13 | Same | 2503068 | PASS (partial) | Measured by tester | `forward` and `backward` 50 and 100 mm within ±2 mm. 500 mm not possible on the USB cable |
| 2026-10-31 | S2-U13 | Same, track 113 500 µm | 2503068 | PASS | Measured by tester | `right 90`, `left 90`, `left 180`, `right 360` within ±2°; track unchanged |
| 2026-10-31 | S2-U12 | Same, gains 256/16/0 | 2503068 | FAIL | `logs/2026-10-31_manual.txt` | Against the criterion then written (rise < 0.3 s): 200 mm/s took 0.52 s, because the start ramp alone takes 0.34 s. Criterion changed to "follows the ramp within 0.2 s", which both speeds meet. Details in CALIBRATION.md |
| 2026-10-31 | S2-U13 | Same | 2503068 | PASS | CALIBRATION.md, `logs/2026-10-31_accuracy.txt` | 5 runs each: `forward 100` and `backward 100` within 1 mm; `right 90` and `left 180` within 2°. `left 90` angles not written down |
| 2026-10-31 | S2-U14 | Same | 2503068 | PASS (partial) | `logs/2026-10-31_accuracy.txt` | `S2_ONLY=1` image with Wi-Fi running, not the full team image: 45 s drive, `faults=0`, `max_tick_us=1939` |
| 2026-10-31 | S2-U10 | — | 2503068 | NOT RUN | — | No oscilloscope. Braking at boot and on `stop` seen by eye in S2-U24/U25 |
| 2026-10-31 | S2-U21 | `673b8ca` + changes | `make test` | PASS | `logs/2026-10-31_host.txt` | Simulation: turns within 1.3°, moves within 1.5 mm |
| 2026-10-31 | S2-U22 | `673b8ca` + changes | Cross-compile | PASS | `logs/2026-10-31_ram.txt` | RAM 132 → 68 B; status 36 → 12 B; telemetry 12 B |
| 2026-10-31 | S2-U23 | `673b8ca` + changes | Build | PASS | `logs/2026-10-31_build.txt`, `logs/2026-10-31_host.txt` | All images, 0 warnings, 0 format changes, sanitizers pass |

## Still to do

- S2-U13: `left 90` ×5 with angles written down, and `forward`/`backward 500` once the robot runs without the USB cable.
- S2-U14 on the full team image.
- S2-U10 if an oscilloscope becomes available.

# Subsystem 5 — test log

**Subsystem:** Ultrasonic scanning and obstacle avoidance  ·  **Student ID:** 2503992  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results. Manual tests use the `TEST=1` bench sequence in [docs/OBSTACLE_AVOIDANCE.md](../../docs/OBSTACLE_AVOIDANCE.md#bench-sequence). Wiring: TRIG on GP0, level-shifted ECHO on GP1, SG90 on GP12.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U40 | Avoidance boots idle | FR7 | auto | `subsystem5_test.c` | `[TEST] S5-U40 PASS` |
| S5-U41 | A forward scan finishes within 500 ms | FR6, I7/I8 | auto | `subsystem5_test.c` | `[TEST] S5-U41 PASS`; the `[scan]` line shows `fault=0` with the sensor fitted |
| S5-U42 | Packed status 8, scan 16, profile 8 bytes (status 4 bytes before the car-turn scan) | Minimum-bit CR | auto | `subsystem5_test.c` | `[TEST] S5-U42 PASS` |
| S5-U43 | Echo conversion, median, profile (closest point, edges, width), side choice, packed sizes | FR6, FR7 | host | `cc -std=c99 -Wall -Wextra -Werror -pedantic -O2 app_program/subsystem5/scan_profile.c testing/subsystem5/host/test_profile.c -o /tmp/test_profile && /tmp/test_profile` | `ALL PASS` |
| S5-U44 | RAM and record sizes before/after minimum-bit, same behaviour | Minimum-bit CR | host | Commands in `logs/2026-10-31_ram.txt` | Packed is smaller; full-width version passes S5-U43 except its size check |
| S5-U45 | Range accuracy | FR6, NFR4, A5 | manual | Car still, flat target ahead at 100, 300 and 1000 mm, then nothing within 4 m. Short bench press at each | `forward_mm` within ±10 mm of 100 and 300, ±1% of 1000; 4000 with nothing in range; `fault=0` |
| S5-U46 | Servo angles | FR6 | manual | Flash the normal image, then the `TEST=1` image; watch the sensor during a short bench press | Points straight ahead (within 3°) at start-up; sweeps from about 10° right to 10° left and back ahead; never reaches the mount's stops; no buzzing |
| S5-U47 | Profile of a box | FR6, A5 | manual | Object about 50–100 mm wide (box, bottle, book on edge), 250 mm ahead, nothing else within 350 mm. Short bench press; repeat with the box 40 mm left and right of centre | `obstacle=1`; `closest` within ±10% of the true distance; the edges include the box's direction (90° centred, above 90 left, below 90 right); `width` printed (rough, at most about 120 mm) |
| S5-U48 | Avoidance on the floor | FR7, I3, I6 | manual | Box on the car's path about 800 mm ahead, open floor around it. Held bench press | Stops with the box at about 350 mm, turns 45° left, 90° right and back, profiles, `right_mm` or `left_mm` at least 400, `BYPASS` steps 1–7, `bypasses=1`, `faults=0`; car ends past the box, within 50 mm of the original line and 10° of the original heading |
| S5-U49 | No open side | FR7, NFR5 | manual | As S5-U48, with walls 250 mm either side of the box | `state=4` (stopped), `faults=0x80`, before any forward bypass move (it may look both ways and turn towards a side first); no reverse |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**host** = runs on the PC;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-31 | S5-U40 | `774f912` | commit2main | PASS | `logs/2026-10-31_boot.txt` | On the car |
| 2026-10-31 | S5-U41 | `774f912` | commit2main | PASS | `logs/2026-10-31_boot.txt` | `forward_mm=207 fault=0`; plain HC-SR04 powered from Grove 1 (3.3 V) |
| 2026-10-31 | S5-U42 | `774f912` | commit2main | PASS | `logs/2026-10-31_boot.txt` | |
| 2026-10-31 | S5-U44 | `b66c3c3` | commit2main | PASS | `logs/2026-10-31_ram_look.txt` | g_scan 36 → 16 B, g_status 28 → 8 B; widened version fails only the size check; measured before the servo trims that are also in this commit |
| 2026-10-31 | S5-U46 | `b66c3c3` | commit2main | PASS | `logs/2026-10-31_servo.txt` | Straight ahead at 1700 µs; ends at ±250 µs, sweep clear of the mount, no buzzing |
| 2026-10-31 | S5-U45 | `b66c3c3` | commit2main | FAIL | `logs/2026-10-31_range.txt` | 100 mm → 91 (−9), 300 mm → 291–296 (within ±10): pass. 1000 mm → 981 (−1.9%, limit ±1%), but the 1 m distance was estimated without a metre ruler, so this may be measurement error; code unchanged. Open space read 3221–3245 mm (a far wall or ceiling, not 4000), `fault=0` throughout. Re-run the 1 m and open-space points with a tape measure |
| 2026-10-31 | S5-U47 | `b66c3c3` | commit2main | PASS | `logs/2026-10-31_profile.txt` | 7 presses: `obstacle=1`, closest 247–259 mm, edges 80..100 every time. At 250 mm the object fills the whole sweep in every position, so the fine scan does not tell left from right; side choice relies on the car-turn coarse scan (S5-U48) |
| 2026-10-31 | S5-U49 | `a4f227c` | commit2main | PASS | `logs/2026-10-31_no_side.txt` | Braked at 336 mm, looked left 58 / right 289 mm (both under 400), safe-stopped with `faults=2` and `side=0`; no bypass step, no reverse. Run before this commit shortened the pass to 700 mm and made the bench wait for GP20 again |
| 2026-10-31 | S5-U48 | `a4f227c` | commit2main | PASS | `logs/2026-10-31_floor.txt` | 700 mm pass, 200 mm side-step, bottle. Braked at 349 mm, looked right 547 / left 616 mm, chose left, steps 1–7 COMPLETE, `bypasses=1`, `faults=0`. Ended past the bottle on the original line and heading. The bench returned to waiting for GP20 without a reset |
| 2026-10-31 | S5-U43 | `4b22bb9` | commit2main | PASS | `logs/2026-10-31_host_profile.txt` | With the width estimate. The review fixes and the width-based side-step in this commit are not tested on the car |
| 2026-10-31 | S5-U40, S5-U41, S5-U42 | `644e2e8` | commit2main | PASS | `logs/2026-10-31_safety_unit.txt` | Regression run after #20 (avoidance now stops line following) and #21 |
| 2026-10-31 | S5-U40, S5-U41, S5-U42 | `45f32a5` | commit2main | PASS | `logs/2026-10-31_faults_unit.txt` | Shared fault bitmask: no echo is `APP_FAULT_NO_ECHO`, avoidance faults `APP_FAULT_AVOID`. Tested as `b9b0666` before the rebase onto #25; `45f32a5` has the same code apart from the controller change from main, which the `TEST=1` image does not start |
| 2026-10-31 | S5-U49 | `45f32a5` | commit2main | PASS | `logs/2026-10-31_faults_no_side.txt` | Wheels lifted, book held about 250 mm ahead instead of walls, so the turns did not move the sensor. Braked, looked right 241 / left 253 mm, then `state=4 faults=0x80 side=0`, no forward bypass move. Tested as `b9b0666` before the rebase onto #25; `45f32a5` has the same code apart from the controller change from main, which the `TEST=1` image does not start |

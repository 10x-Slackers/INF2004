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
| S5-U43 | Echo conversion, median, profile (closest point, edges), side choice, packed sizes | FR6, FR7 | host | `cc -std=c99 -Wall -Wextra -Werror -pedantic -O2 app_program/subsystem5/scan_profile.c testing/subsystem5/host/test_profile.c -o /tmp/test_profile && /tmp/test_profile` | `ALL PASS` |
| S5-U44 | RAM and record sizes before/after minimum-bit, same behaviour | Minimum-bit CR | host | Commands in `logs/2026-10-31_ram.txt` | Packed is smaller; full-width version passes S5-U43 except its size check |
| S5-U45 | Range accuracy | FR6, NFR4, A5 | manual | Car still, flat target ahead at 100, 300 and 1000 mm, then nothing within 4 m. Short bench press at each | `forward_mm` within ±10 mm of 100 and 300, ±1% of 1000; 4000 with nothing in range; `fault=0` |
| S5-U46 | Servo angles | FR6 | manual | Flash the normal image, then the `TEST=1` image; watch the sensor during a short bench press | Points straight ahead (within 3°) at start-up; sweeps from about 10° right to 10° left and back ahead; never reaches the mount's stops; no buzzing |
| S5-U47 | Profile of a box | FR6, A5 | manual | Object about 50–100 mm wide (box, bottle, book on edge), 250 mm ahead, nothing else within 350 mm. Short bench press; repeat with the box 40 mm left and right of centre | `obstacle=1`; `closest` within ±10% of the true distance; the edges include the box's direction (90° centred, above 90 left, below 90 right) |
| S5-U48 | Avoidance on the floor | FR7, I3, I6 | manual | Box on the car's path about 800 mm ahead, open floor around it. Held bench press | Stops with the box at about 350 mm, turns 45° left, 90° right and back, profiles, `right_mm` and `left_mm` over 500, `BYPASS` steps 1–7, `bypasses=1`, `faults=0`; car ends past the box, within 50 mm of the original line and 10° of the original heading |
| S5-U49 | No open side | FR7, NFR5 | manual | As S5-U48, with walls 300 mm either side of the box | Looks left and right and returns to the original heading, then `state=4` (stopped), `faults=2`; no bypass step, no reverse |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**host** = runs on the PC;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-31 | S5-U43 | `fbdac9e` | commit2main | PASS | `logs/2026-10-31_host_profile.txt` | Host only |
| 2026-10-31 | S5-U44 | `fbdac9e` | commit2main | PASS | `logs/2026-10-31_ram.txt` | g_scan 44 → 16 B, g_status 20 → 4 B; widened version fails only the size check |
| 2026-10-31 | S5-U40 | `1937ba5` | commit2main | PASS | `logs/2026-10-31_boot.txt` | On the car |
| 2026-10-31 | S5-U41 | `1937ba5` | commit2main | PASS | `logs/2026-10-31_boot.txt` | `forward_mm=207 fault=0`; plain HC-SR04 powered from Grove 1 (3.3 V) |
| 2026-10-31 | S5-U42 | `1937ba5` | commit2main | PASS | `logs/2026-10-31_boot.txt` | |
| 2026-10-31 | S5-U43 | `26a4ae2` | commit2main | PASS | `logs/2026-10-31_host_profile_look.txt` | After the car-turn coarse scan |
| 2026-10-31 | S5-U44 | `26a4ae2` | commit2main | PASS | `logs/2026-10-31_ram_look.txt` | g_scan 36 → 16 B, g_status 28 → 8 B; widened version fails only the size check |
| 2026-10-31 | S5-U46 | `6e2ca23` | commit2main | PASS | `logs/2026-10-31_servo.txt` | About 0.5° right of ahead after trimming the centre to 1640 µs; sweep clear of the mount, no buzzing |
| 2026-10-31 | S5-U46 | `fc9b321` | commit2main | PASS | `logs/2026-10-31_servo_2.txt` | Straight ahead at 1700 µs; ends at ±250 µs, sweep clear of the mount, no buzzing |
| 2026-10-31 | S5-U45 | `fc9b321` | commit2main | FAIL | `logs/2026-10-31_range.txt` | 100 mm → 91 (−9), 300 mm → 291–296 (within ±10): pass. 1000 mm → 981 (−1.9%, limit ±1%), but the 1 m distance was estimated without a metre ruler, so this may be measurement error; code unchanged. Open space read 3221–3245 mm (a far wall or ceiling, not 4000), `fault=0` throughout. Re-run the 1 m and open-space points with a tape measure |
| 2026-10-31 | S5-U47 | `fc9b321` | commit2main | PASS | `logs/2026-10-31_profile.txt` | 7 presses: `obstacle=1`, closest 247–259 mm, edges 80..100 every time. At 250 mm the object fills the whole sweep in every position, so the fine scan does not tell left from right; side choice relies on the car-turn coarse scan (S5-U48) |

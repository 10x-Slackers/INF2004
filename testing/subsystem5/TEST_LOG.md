# Subsystem 5 — test log

**Subsystem:** Ultrasonic scanning and obstacle avoidance  ·  **Student ID:** 2503992  ·  **Code:** `app_program/subsystem5/`

Methods and how to record results: `testing/README.md`. Manual tests use the `TEST=1` bench sequence in [docs/OBSTACLE_AVOIDANCE.md](../../docs/OBSTACLE_AVOIDANCE.md#bench-sequence). Wiring: TRIG on GP0, level-shifted ECHO on GP1, SG90 on GP12.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | Avoidance boots idle | FR7 | auto | `subsystem5_test.c` | `[TEST] S5-U01 PASS` |
| S5-U02 | A forward scan finishes within 500 ms | FR6, I7/I8 | auto | `subsystem5_test.c` | `[TEST] S5-U02 PASS`; the `[scan]` line shows `fault=0` with the sensor fitted |
| S5-U03 | Packed status 8, scan 20, profile 12 bytes | Minimum-bit CR | auto | `subsystem5_test.c` | `[TEST] S5-U03 PASS` |
| S5-U04 | Echo conversion, median, profile (closest point, edges, width), side choice, packed sizes | FR6, FR7 | host | `cc -std=c99 -Wall -Wextra -Werror -pedantic -O2 app_program/subsystem5/scan_profile.c testing/subsystem5/host/test_profile.c -o /tmp/test_profile && /tmp/test_profile` | `ALL PASS` |
| S5-U05 | RAM and record sizes before/after minimum-bit, same behaviour | Minimum-bit CR | host | Commands in `logs/2026-10-31_ram.txt` | Packed is smaller; full-width version passes S5-U04 except its size check |
| S5-U06 | Range accuracy | FR6, NFR4, A5 | manual | Car still, flat target ahead at 100, 300 and 1000 mm, then nothing within 4 m. Short bench press at each | `forward_mm` within ±10 mm of 100 and 300, ±1% of 1000; 4000 with nothing in range; `fault=0` |
| S5-U07 | Servo angles | FR6 | manual | Flash the normal image, then the `TEST=1` image; watch the sensor during a short bench press | Points straight ahead (within 3°) at start-up; coarse rays every 15° from about 45° right to 45° left (within 5°), then fine steps around anything ahead, and back ahead; never reaches the mount's stops; no buzzing |
| S5-U08 | Profile of a box | FR6, A5 | manual | Object about 50–100 mm wide (box, bottle, book on edge), 100 mm ahead, nothing else within 150 mm. Short bench press; repeat with the box 40 mm left and right of centre | `obstacle=1`; `closest` within ±10% of the true distance; the edges include the box's direction (90° centred, above 90 left, below 90 right) and stay within 30° of ahead; `width` printed (rough, wider than the box); `clear` shows the open floor each side |
| S5-U09 | Avoidance on the floor | FR7, I3, I6 | manual | Box on the car's path about 800 mm ahead, open floor around it. Held bench press. Note the largest sideways offset and the gap to the box while passing | Stops with the box at about 150 mm, profiles with the servo while the car stays still, `right_mm` or `left_mm` at least 250, `BYPASS` steps 1–7, `bypasses=1`, `faults=0`; the car's side passes the box with a gap of at least 40 mm, never touching (wider for a wide box: the edge offset is never taken as less than it is); car ends past the box within 10° of the original heading. With no line on the floor the return diagonal runs its full length, so it ends up to 150 mm along it (about 130 mm sideways) past the original line; ending on the line is checked on the course (SYS-08) |
| S5-U10 | No open side | FR7, NFR5 | manual | As S5-U09, with walls 150 mm either side of the box (about 210 mm along the 45° rays) | `right_mm` and `left_mm` under 250; the car backs off up to 80 mm, profiles again, then `state=4` (stopped), `faults=0x80` |

## Results

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-31 | S5-U05 | `b66c3c3` | commit2main | PASS | `logs/2026-10-31_ram_look.txt` | g_scan 36 → 16 B, g_status 28 → 8 B; the widened version fails only the size check |
| 2026-10-31 | S5-U06 | `b66c3c3` | commit2main | FAIL | `logs/2026-10-31_range.txt` | 100 mm → 91, 300 mm → 291–296: pass. 1000 mm → 981 (−1.9%, limit ±1%), but the 1 m was estimated without a tape; re-measured in the 2026-11-03 row. Open space read 3221–3245 mm (a far wall, not 4000), `fault=0` |
| 2026-10-31 | S5-U08 | `b66c3c3` | commit2main | PASS | `logs/2026-10-31_profile.txt` | Object at 250 mm: `obstacle=1`, closest 247–259 mm, edges 80..100 in every position. At 250 mm the object fills the sweep, so the profile cannot tell left from right; the test distance later moved to 100 mm |
| 2026-10-31 | S5-U10 | `45f32a5` | commit2main | PASS | `logs/2026-10-31_faults_no_side.txt` | Wheels lifted, book about 250 mm ahead instead of walls. Braked, right 241 / left 253 mm, then `state=4 faults=0x80 side=0`, no bypass move |
| 2026-10-31 | S5-U01, S5-U02, S5-U03 | `af4be0d` | commit2main | PASS | `../integration/logs/2026-10-31_pr31_unit.txt` | Echo edges through the shared GPIO interrupt (PR #31); `[scan] forward_mm=1351 fault=0` |
| 2026-11-03 | S5-U06 | `21463ff` | commit2main | PASS (partial) | `logs/2026-11-03_range.txt` | 1000 mm by tape → 1000, 1000, 1000, `fault=0`. 100 and 300 mm not re-run; open space not tested (no clear 4 m in the lab) |
| 2026-11-12 | S5-U03 | `1ba47cb` | commit2main | PASS | `logs/2026-11-12_servo_45.txt` | `packed status 8, scan 20, profile 12 bytes` |
| 2026-11-12 | S5-U07 | `1ba47cb` | commit2main | PASS | `logs/2026-11-12_servo_45.txt` | Rebuilt mount. A ±60° sweep (1033–2367 µs) was too wide for the mount, so it was cut to ±45° (1200–2200 µs): clear of the mount, straight ahead at the middle, no buzzing. Observed by eye |
| 2026-11-12 | S5-U08 | `1ba47cb` | commit2main | PASS | `logs/2026-11-12_profile_45.txt` | 65 mm box at 100 mm: centred 96 mm @ 90°, edges 50..120; 40 mm left 96 mm, edges 65..120; 40 mm right 106 mm @ 65°, edges 50..100. The edges follow the box; beam spread makes the width 92–117 mm |
| 2026-11-12 | S5-U09 | `1ba47cb` | commit2main | PASS | `logs/2026-11-12_floor_angled.txt` | On battery, log read back over USB. 65 mm box: braked at 144 mm, closest 127 mm, clear right 570 / left 837. Turned left, the diagonal was blocked, turned 120° right and bypassed on the right: steps 1–7, `bypasses=1`, `faults=0`, about 15 s. Passed clear of the box and ended on the line |
| 2026-11-12 | S5-U09 | `b24dd4f` | commit2main | PASS | `logs/2026-11-12_blocked_turn_back.txt` | After the refactor (edge offset at 150 mm). Bypassed on the right (right 498 / left 167 mm), `bypasses=1`; passed without touching and ended on the line. The log is the next run's; its first record holds this run's result |
| 2026-11-12 | S5-U10 | `b24dd4f` | commit2main | PASS | `logs/2026-11-12_blocked_turn_back.txt` | Right ray clear (417 mm), so this ran the turn-back path: turned right, the diagonal read 165 mm, left under 250, turned back and safe-stopped: `state=4`, `faults=0x80`, `side=0`, ending on its original heading; no reverse |
| 2026-11-19 | S5-U04 | `3e7007a` | commit2main | PASS | `logs/2026-11-19_exact_periods_host.txt` | `ALL PASS` |
| 2026-11-24 | S5-U08 | `fix/obstacle-width` | commit2main | PASS | `logs/2026-11-24_width.txt` | Width only. Before: a 170 mm flat face stopped 192 mm away read edges 90..100 (its slanted sides read past 200 mm), and 206–231 mm when closer. After: 170 mm obstacle at 160–169 mm → 176, 181, 162 mm (±6%); 230 mm obstacle at 176–195 mm → 214–251 mm (±9%), and 191 mm at 169 mm with the car off-centre, where it ran past the 45° end of the sweep |
| 2026-11-24 | S5-U04 | `fix/obstacle-width` | commit2main | PASS | `logs/2026-11-24_width.txt` | `ALL PASS`, with the slanted-face and reach checks |
| 2026-11-25 | S5-U01, S5-U02, S5-U03 | `dbb28bd` (#70) | commit2main | PASS | `logs/2026-11-25_auto.txt` | Auto checks on the refactored code: 3 of 3 |
| 2026-11-25 | S5-U07 | `dbb28bd` (#70) | commit2main | PASS | `logs/2026-11-25_s5_u07_servo.txt`, [video](https://youtu.be/S1qvIX3HW8I) | Straight ahead at start, about 45° each way, back ahead, no buzzing |
| 2026-11-25 | S5-U09 | `dbb28bd` (#70) | commit2main | PASS (partial) | `logs/2026-11-25_s5_u09_bypass.txt`, [video](https://youtu.be/Tlqxwjrnfc8) | Open floor, 230 mm box. Stopped at about 180 mm, width 234 mm, right side open (567 mm), steps 1–7, `bypasses=1`, `faults=0x0`. Ended 100 mm off the original line (limit 50) and passed 400 mm from the box (expected about 40): with no line on the floor nothing ends the return leg, which #70 made 150 mm longer than the leg out. On the course it rejoined the line (SYS-08) |
| 2026-11-25 | S5-U10 | `dbb28bd` (#70) | commit2main | PASS | `logs/2026-11-25_s5_u10_blocked.txt`, [video](https://youtu.be/L4oBsRPK4iM) | Walls 150 mm either side: profiled, backed off, profiled again (both sides under 250 mm both times), then `state=4`, `faults=0x80`; no further movement |

# Videos

Short videos of tests and demos on the real car. A video shows what a log cannot: the car following the line, stopping, passing an obstacle. Only record one when the log alone doesn't show that the test passed. Auto, host and RAM tests, and anything fully shown by console output, don't need one.

## What to record

- **Short:** 15–60 seconds, one test or one feature per video.
- **Say what it is:** show or say the test ID (`SYS-08`, `INT-09`, …) at the start.
- **Show the commit:** run `git rev-parse --short HEAD` before recording and keep it in shot for a moment, or put it in the table below.
- **Show the result:** keep the car and, if it matters, the console or dashboard in view.
- **Landscape**, steady, good light.

## How to share it: unlisted YouTube only

1. Upload to YouTube with visibility **Unlisted**: anyone with the link can watch, but it doesn't show up in search. Not *Private* (the link won't work for the team or the instructor) and not *Public*.
2. Copy the link (`https://youtu.be/...`).
3. Add a row to the table below, and put the same link in the *Log / evidence* column of the test's result row in `testing/.../TEST_LOG.md`.

Don't commit video files or attach them on GitHub: a video stays in the git history for good and slows every clone. `.gitignore` ignores common video files for this reason.

## Video index

Newest at the bottom.

| Date | Subsystem | Test ID / demo | Commit | Student ID | Link | Notes |
|---|---|---|---|---|---|---|
| 2026-11-25 | System | SYS-02 | `dbb28bd` (#70) | 2503999 | [link](https://youtu.be/HWoQPpom_Fo) | Barcode decode on the dashboard: `Barcode F: ignored` |
| 2026-11-25 | System | SYS-02 | `dbb28bd` (#70) | 2503999 | [link](https://youtu.be/QJA3phgBQOg) | Barcode decode, dashboard and car in one shot |
| 2026-11-25 | 5 | S5-U07 | `dbb28bd` (#70) | 2503999 | [link](https://youtu.be/S1qvIX3HW8I) | Ultrasonic scan: the servo turns the sensor each way and back |
| 2026-11-25 | 4 | S4-U10 | `dbb28bd` (#70) | 2503999 | [link](https://youtu.be/GCucvy4EUPQ) | Hump on the car: bench drives over an 11 mm hump |
| 2026-11-25 | 5 | S5-U09 | `dbb28bd` (#70) | 2503999 | [link](https://youtu.be/Tlqxwjrnfc8) | Obstacle bypass on open floor |
| 2026-11-25 | 5 | S5-U10 | `dbb28bd` (#70) | 2503999 | [link](https://youtu.be/L4oBsRPK4iM) | No open side: the car stays still |
| 2026-11-25 | Integration | INT-01 | `dbb28bd` (#70) | 2503999 | [link](https://youtu.be/Y6-EYzI0AEg) | GP20 start, stop and reset on the line |
| 2026-11-25 | Integration | INT-02 | `dbb28bd` (#70) | 2503999 | [link](https://youtu.be/9PdUJxX5ZrU) | Barcode right turn |
| 2026-11-25 | 4 | S4-U08 | `dbb28bd` (#70) | 2503999 | [link](https://youtu.be/UxXhBs-dmYk) | Calibration beep |
| 2026-11-25 | Integration | INT-02 | `dbb28bd` (#70) | 2503999 | [link](https://youtu.be/TdOTd3a-Un0) | Barcode straight on |
| 2026-11-25 | Integration | INT-02 | `dbb28bd` (#70) | 2503999 | [link](https://youtu.be/8fd2H3VnJ00) | Barcode U-turn |
| 2026-11-25 | Integration, System | INT-05, INT-09, INT-10, SYS-10 | `dbb28bd` (#70) | 2503999 | [link](https://youtu.be/yFbUaGETJ9I) | Wheel stall and lift stop |
| 2026-11-25 | System | SYS-11 | `b721ac5` (#71) | 2503999 | [link](https://youtu.be/kfQtLfntgPY) | Full course run, started from the dashboard |

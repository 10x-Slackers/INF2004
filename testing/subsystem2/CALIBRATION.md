# Buddy 2 PID tuning and motion accuracy report

Measured on the robot. Methods: [docs/MOTION_CONTROL.md](../../docs/MOTION_CONTROL.md#calibration). Test IDs: [TEST_LOG.md](TEST_LOG.md). Leave a cell empty rather than estimate it.

## Encoders and geometry (S2-U11)

| Parameter | In the code | Measured (2026-10-31) |
| --- | --- | --- |
| Edges per wheel turn, left / right | 2537 / 2537 | 2539 / 2536, over 10 turns by hand |
| Wheel diameter | 65 mm | |
| Track, wheel centre to wheel centre | 113.5 mm | Turns within ±2° with this value |
| Direction, left / right | motor −1 / +1, encoder −1 / +1 | Confirmed (S2-U11, S2-U24) |

Earlier hand counts gave 2340 (left) and 2541 (right); the 10-turn measurement replaces them. The right motor needs about 15% duty to start, which the 10% feedforward plus the integral covers.

## PID tuning (S2-U12)

Gains are Q8 per 10 ms sample, plus a fixed 100‰ feedforward. Measured with `forward 300` on the floor.

| Target mm/s | Kp / Ki / Kd | Rise to 90% (s) | Overshoot | Steady speed, left / right (mm/s) |
| --- | --- | --- | --- | --- |
| 100 | 256 / 16 / 0 | 0.18 | 9% (peak 109) | 99.5 / 99.7 |
| 200 | 256 / 16 / 0 | 0.52 | 2.5% (peak 205) | 193 / 195 |

Each speed reading is edges per 10 ms, so it moves in steps of 8 mm/s; most of the ±4–6 mm/s spread is that step. Speed ramps up at 500 mm/s² to avoid wheel slip, so 200 mm/s takes at least 0.34 s; the PID trails the ramp by about 0.2 s. The default gains are kept. Data: `logs/2026-10-31_manual.txt`.

## Distance and turn accuracy (S2-U13)

Five runs per command, measured with tape and floor marks. Targets: distance within ±1%, turns within ±3°.

| Command | Run 1 | Run 2 | Run 3 | Run 4 | Run 5 | Mean abs error | Max abs error |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `forward 100` (mm) | 101 | 100 | 100 | 101 | 100 | 0.4 | 1 |
| `backward 100` (mm) | 100 | 100 | 101 | 100 | 100 | 0.2 | 1 |
| `left 90` (deg) | | | | | | | |
| `right 90` (deg) | 89 | 90 | 88 | 90 | 90 | 0.6 | 2 |
| `left 180` (deg) | 178 | 179 | 180 | 180 | 181 | 0.8 | 2 |

2026-10-31, on the floor, robot on its USB cable (so straight moves were limited to 100 mm). Every measured run meets its target. Still to do: `left 90`, and 500 mm moves without the cable.

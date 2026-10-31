# Line Following and Barcodes (Buddy 3)

The two line sensors, the barcode sensor, line following, lost-line search and the Code 39 decoder. Other subsystems never touch these sensors or the ADC; they call [`line_following.h`](../app_program/subsystem3/line_following.h) and [`barcode_decoder.h`](../app_program/subsystem3/barcode_decoder.h), which are safe from any task on either core.

| File (`app_program/subsystem3/`) | Contents                                                          |
| -------------------------------- | ----------------------------------------------------------------- |
| `line_following.c/.h`            | Line Following API (I1) and task: steering and lost-line search   |
| `barcode_decoder.c/.h`           | Barcode Decoder API (I2) and task: bar widths along the odometer  |
| `code39.c/.h`                    | Code 39 decoding of `*X*` from its bar widths; no hardware calls  |
| `drv_line.c/.h`                  | Left and right IR module DO pins                                  |
| `drv_barcode.c/.h`               | Barcode IR module AO: ADC at 2 ksample/s, DMA into a ring         |
| `subsystem3.c/.h`                | Start-up, task priorities, the 10 ms cyclic handler               |
| `subsystem3_test.c`              | On-target tests and bench run (`make TEST=1`)                     |

Tests and results: [testing/subsystem3](../testing/subsystem3/TEST_LOG.md).

## API

| Call                                 | Effect                                                                                           |
| ------------------------------------ | ------------------------------------------------------------------------------------------------ |
| `line_following_start(mm_s)`         | Steer the car along the line at 1–500 mm/s until stopped; clears `b_lost`. Returns false, and changes nothing, out of range |
| `line_following_stop()`              | Stop steering and brake. The task only drives under its lock, so no drive command follows the brake |
| `line_following_get(&line)`          | Copy the latest line state (below)                                                               |
| `barcode_decoder_get(&barcode)`      | Copy the last validated barcode (below). Act when `count` changes                                |
| `barcode_decoder_get_raw(&raw)`      | Barcode sensor's min and max over the last 10 ms, for calibration                                |

```c
line_following_start(200u);
line_following_get(&line);          /* every 10 ms */
if (line.b_lost) { /* safe stop */ }
barcode_decoder_get(&barcode);      /* barcode.count changed: new command */
```

A junction does not stop line following: the controller decides what to do at one. A lost line that the search cannot find does stop it (below).

### Line state (`line_following_t`, 8 bytes)

| Field         | Range     | Meaning                                                                       |
| ------------- | --------- | ----------------------------------------------------------------------------- |
| `bits`        | 0–3       | `DRV_LINE_LEFT` (1), `DRV_LINE_RIGHT` (2): that sensor sees black now          |
| `last_side`   | 0–2       | Last single sensor to see black; 0 before the first                           |
| `b_junction`  | 0–1       | Both sensors black for 3 samples in a row (30 ms, 6 mm at 200 mm/s)           |
| `b_following` | 0–1       | Steering the car                                                              |
| `b_searching` | 0–1       | Pivoting to find a lost line                                                  |
| `b_lost`      | 0–1       | The search did not find the line: braked and stopped following, until the next `start()` |
| `count`       | 0–255, wraps | Samples taken                                                              |
| `sample_ms`   | wraps     | Kernel time of the sample, to check its age                                   |

### Barcode (`barcode_decoder_t`, 8 bytes)

| Field         | Range        | Meaning                                                                  |
| ------------- | ------------ | ------------------------------------------------------------------------ |
| `command`     | 0–4          | `NONE`, `LEFT` (A), `RIGHT` (B), `STRAIGHT` (C), `UTURN` (D)              |
| `count`       | 0–15, wraps  | Barcodes validated                                                       |
| `rejects`     | 0–15, wraps  | Barcodes read but not valid: a letter outside A to D, or a misread       |
| `distance_mm` | wraps        | Motion odometer (`distance_mm`) where the barcode ended                  |

An unvalidated barcode only bumps `rejects`; its command is never published.

## How it works

- **Sampling:** a cyclic handler wakes each task every 10 ms (`app_start_cyclic()`; `tk_dly_tsk()` would add a tick to every period).
- **Line following:** the sensors straddle the 18 mm line. Both white means centred, one black means the line has drifted under that sensor, so the car turns that way: that wheel slows and the other speeds up by 50% of the speed. Both black is a junction (or a marking across the line) and the car drives straight on.
- **Lost line:** both white also looks like a lost line, so the line counts as lost only after `LINE_FOLLOWING_LOST_MM` (150 mm) of travel with both sensors white (`distance_mm` from the motion status). The car then pivots on the spot towards `last_side`, where a curve usually takes the line, up to `LINE_FOLLOWING_SEARCH_DEG` (135°), then the other way to as far past its starting heading (`heading_deg`). Either sensor seeing black resumes normal following. If the whole sweep finds nothing, the task brakes, stops following and sets `b_lost`; the Vehicle Controller takes a safe stop. On a straight, a false alarm costs a small pivot: the line soon reaches one sensor. While `b_searching` is set the Vehicle Controller pauses obstacle avoidance, since the pivot sweeps the ultrasonic sensor across the sides; nothing checks collisions during the pivot, as during every other turn.
- **Search reach:** the sensors are 110 mm ahead of the axle, so the pivot sweeps a circle of that radius. The longer the all-white threshold, the further the car rolls past the point where the line left. On a 90° corner, a 90° sweep stops reaching the new line after about 110 mm of all-white travel, a 135° sweep after about 185 mm, and any sweep after about 220 mm (it would need reversing). So the threshold must stay short however long the straights run without touching a sensor, and the sweep must match it: 150 mm needs about 110°.
- **Barcodes:** the barcode sensor's AO is converted every 0.5 ms; DMA copies each sample into a 256-sample ring with no interrupt. Every 10 ms the decoder spreads the new samples along the distance rolled in that block, in µm, so bar widths do not depend on speed. The distance comes from `motion_control_distance_um()`, read just after the samples: it counts encoder edges up to that moment, not up to the last 10 ms motion step. A bar is black above 2500 counts and ends below 1500 (hysteresis). A barcode starts at the first black after white and ends at a white run over 30 mm.
- **Code 39:** exactly 29 elements (`*`, the letter, `*`, with narrow gaps between) are split into narrow and wide at halfway between the narrowest and widest. Each character must have exactly 3 wide elements and match `*`, A, B, C or D, read in either direction.

## Calibration

Constants are at the top of each `.c` file; change one, rebuild and rerun the bench.

1. **Line sensors:** at the final mounting height over the real track, turn each module's potentiometer until its LED lights over the line only (S3-U03). DO reads 1 over black (`DRV_LINE_BLACK`).
2. **Steering:** `LINE_FOLLOWING_TURN_PCT` (50). Raise it if the car runs off curves, lower it if it weaves on straights.
3. **Lost-line threshold:** `LINE_FOLLOWING_LOST_MM` (150). Run the bench on the longest straight: `[line]` lines print the odometer (`mm=`) on every change, so the longest all-white stretch is the gap between a `left=0 right=0` line and the next line. Set the threshold about 1.5 times that, but under about 185 mm, the reach of the 135° sweep (`LINE_FOLLOWING_SEARCH_DEG`) on a sharp corner (above). If 1.5 times is more than that, make the car weave more (step 2) instead. A narrower sweep is quicker if the course has no sharp corners.
4. **Barcode levels:** hold the sensor still over white and over a black bar (S3-U10) and keep `BARCODE_BLACK` and `BARCODE_WHITE` well inside the two levels. White reads about 170 counts and black about 4070.

## Bench sequence

The `make -C build_make TEST=1` image starts task `s3_bench` after the checks. It prints the line state (`[line] left= right= junction= last= following= searching= lost= mm= deg=`) on every change, the barcode level (`[bar] min= max=`) every 0.5 s, and every barcode read (`[barcode] command= count= rejects= at_mm=`, with the measured widths).

- **GP21 held for 1 s or more:** after 5 s the car follows the line at 200 mm/s until a junction, a lost line or 15 s, then stops and prints `[line] bench done after … ms`. Taps under 0.5 s start subsystem 2's bench instead; a press in between starts neither.

## Resources

| Resource              | Use                                                                                 |
| --------------------- | ----------------------------------------------------------------------------------- |
| GP6, GP7              | Left and right IR module DO (Grove 5, Grove 7)                                       |
| GP26, ADC0            | Barcode IR module AO (Grove 6). The kernel's `adca` device is not opened             |
| DMA channel 11        | ADC FIFO to the sample ring                                                          |
| GP21                  | Bench button (pull-up), `TEST=1` only                                                |
| Tasks                 | `s3_barcode` priority 8; `s3_line` priority 9; `s3_bench` priority 9, `TEST=1` only  |
| Kernel objects        | 2 mutexes, 2 cyclic handlers                                                        |

## Coding standard

The code follows BARR-C:2018 with the same deviations as subsystem 2 ([MOTION_CONTROL.md](MOTION_CONTROL.md#coding-standard)), for `subsystem3_test.c` and the task entry points. Abbreviations as listed there.

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
| `line_following_start(mm_s)`         | Steer the car along the line at 1–500 mm/s until stopped; clears `b_lost` and `APP_FAULT_LINE_SENSOR`. Returns false, and changes nothing, out of range |
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

A junction does not stop line following: the controller decides what to do at one. A lost line that the search cannot find does stop it, and so does a line sensor stuck on black (below).

### Line state (`line_following_t`, 8 bytes)

| Field         | Range     | Meaning                                                                       |
| ------------- | --------- | ----------------------------------------------------------------------------- |
| `bits`        | 0–3       | `DRV_LINE_LEFT` (1), `DRV_LINE_RIGHT` (2): that sensor sees black now          |
| `last_side`   | 0–2       | Last single sensor to see black; 0 before the first                           |
| `b_junction`  | 0–1       | Both sensors black for 6 samples in a row (60 ms, 9.6 mm at 160 mm/s)           |
| `b_following` | 0–1       | Steering the car                                                              |
| `b_searching` | 0–1       | Pivoting to find a lost line                                                  |
| `b_lost`      | 0–1       | The search did not find the line, or a sensor is stuck on black: braked and stopped following, until the next `start()` |
| `count`       | 0–255, wraps | Samples taken                                                              |
| `sample_ms`   | wraps     | Kernel time of the sample, to check its age                                   |

### Barcode (`barcode_decoder_t`, 8 bytes)

| Field         | Range        | Meaning                                                                  |
| ------------- | ------------ | ------------------------------------------------------------------------ |
| `command`     | 0–4          | `NONE`, `LEFT`, `RIGHT`, `STRAIGHT`, `UTURN`, from the letter (`g_actions`, set over MQTT) |
| `count`       | 0–15, wraps  | Barcodes validated                                                       |
| `rejects`     | 0–15, wraps  | Barcodes read but not valid: a letter set to ignore in `g_actions`, a digit, or a misread with at least 9 elements |
| `letter`      | 0–26         | Last letter read, validated or rejected: 1–26 for A–Z, 0 for a misread or a digit |
| `distance_mm` | wraps        | Motion odometer (`distance_mm`) where the barcode ended                  |
| `b_reading`   | 0–1          | Over a barcode now: from its third bar, each bar and space under 30 mm, until it ends |

An unvalidated barcode only bumps `rejects`; its command is never published. Shorter runs (specks, the line itself) and runs with a bar over 30 mm are not barcodes and are ignored without counting.

## How it works

- **Sampling:** a cyclic handler wakes each task every 10 ms (`app_start_cyclic()`; `tk_dly_tsk()` would add a tick to every period).
- **Line following:** the sensors straddle the 18 mm line. Both white means centred, one black means the line has drifted under that sensor, so the car turns that way: that wheel slows and the other speeds up. The turn is gradual: it starts at once at `LINE_FOLLOWING_TURN_MIN` (30% of the speed) towards that sensor's side, rises the further the car rolls with the sensor black to the full turn (inner wheel stopped, outer at twice the speed) after `LINE_FOLLOWING_RISE_MM` (30 mm), and falls back to straight over `LINE_FOLLOWING_FALL_MM` (10 mm) once neither sensor is black. The other sensor turning black switches sides at once, so a fading turn never carries the car past the line. After `LINE_FOLLOWING_DRIFT_MM` (40 mm) with both sensors white, the car turns gently towards `last_side`, growing to `LINE_FOLLOWING_TURN_MIN` by `LINE_FOLLOWING_LOST_MM`: a centred car drifts back onto that sensor instead of pivoting, and on a curve it keeps bending the way the line went. A small drift on a straight clears the sensor while the turn is still small; a curve keeps it black and the turn keeps growing. It is measured along the odometer, so it does not depend on speed. Both black is a junction (or a marking across the line) and the car drives straight on.
- **Lost line:** both white also looks like a lost line, so the line counts as lost only after `LINE_FOLLOWING_LOST_MM` (80 mm) of travel with both sensors white (`distance_mm` from the motion status). The car then pivots on the spot towards `last_side`, where a curve usually takes the line, up to `LINE_FOLLOWING_SEARCH_DEG` (135°), then the other way to as far past its starting heading (`heading_deg`). Either sensor seeing black resumes normal following. If the whole sweep finds nothing, the task brakes, stops following and sets `b_lost`; the Vehicle Controller takes a safe stop. The sweep ends only when the encoder heading gets there, and a heading that stops following the pivot (a wheel that does not turn, a bad encoder) never does, so the search also fails after `LINE_FOLLOWING_SEARCH_MAX` samples (6 s; a whole sweep takes under 5 s). On a straight, a false alarm costs a small pivot: the line soon reaches one sensor. While `b_searching` is set the Vehicle Controller pauses obstacle avoidance, since the pivot sweeps the ultrasonic sensor across the sides; nothing checks collisions during the pivot, as during every other turn.
- **Stuck sensor:** a line sensor stuck on black holds the car in a full turn, and the line never looks lost because the sensors never both read white. So when one sensor has read black for more than `LINE_FOLLOWING_STUCK_MM` (300 mm of `distance_mm`, 1.9 s at 160 mm/s; the longest run on the course is about 120 mm), the task stops as for a failed search and sets `APP_FAULT_LINE_SENSOR` until the next `start()`. The run belongs to the sensors that have read black on every sample since it began and starts again when none of them is black, so the other sensor crossing a line or a junction does not hide a stuck one; a sensor that turns black part-way is counted from when the first reads white.
- **Wobble hold:** three sensor hits in a row on alternating sides, each starting within `LINE_FOLLOWING_WOBBLE_MM` (150 mm) of the last, mean the car is weaving along a straight. Hits are still steered as usual, but while both sensors are white the car holds the heading halfway between the last left and right hits (from the encoder heading) instead of drifting towards the last side, turning 5% of the speed per degree off it, at most `LINE_FOLLOWING_TURN_MIN`. Each new alternating hit updates that heading, at any distance while holding. A curve ends the hold: two hits in a row on the same side, a hit longer than `LINE_FOLLOWING_HOLD_EXIT_MM` (40 mm), a new halfway heading more than `LINE_FOLLOWING_HOLD_BEND_DEG` (8°) from the held one (the weave itself is turning), or the car more than `LINE_FOLLOWING_HOLD_OFF_DEG` (15°) off the held heading. While holding, all white counts as lost only after `LINE_FOLLOWING_HOLD_LOST_MM` (120 mm), since a held car runs centred with both sensors white; the white gaps of a weave are up to about 80 mm. It is kept short because a held car that loses the line on a curve drives straight on until the search starts.
- **Straight over barcodes:** barcodes lie on straights, so while `b_reading` is set the car holds the heading it had when the barcode was recognised (the wobble hold heading if holding, as that is the better estimate of the straight), the same way as the wobble hold. It also slows to `LINE_FOLLOWING_CODE_MM_S` (80 mm/s, or the following speed if lower), so each narrow bar stays under the sensor long enough to read black. A sensor hit still turns it towards the line, but only at `LINE_FOLLOWING_TURN_MIN`, never ramping up, and all white counts as lost only after `LINE_FOLLOWING_CODE_LOST_MM` (300 mm, longer than the 141 mm barcode; still a limit, as `b_reading` stays set if the barcode sensor stops). Where the barcode ends, all white is counted from the start again: a car that ran centred over the barcode has already gone further than `LINE_FOLLOWING_LOST_MM`, and would otherwise start a search at once. `b_reading` is set at the third bar of a barcode (about 20 mm in), with each bar and space so far under 30 mm, which a lone speck, the line or a crossing line does not give, and drops when the barcode ends.
- **Search reach:** the sensors are 115 mm ahead of the axle, so the pivot sweeps a circle of that radius. The longer the all-white threshold, the further the car rolls past the point where the line left. On a 90° corner, a 90° sweep stops reaching the new line after about 115 mm of all-white travel, a 135° sweep after about 195 mm, and any sweep after about 230 mm (it would need reversing). So the threshold must stay short however long the straights run without touching a sensor, and the sweep must match it: 80 mm needs no more than 90°.
- **Barcodes:** the barcode sensor's AO is converted every 0.5 ms; DMA copies each sample into a 256-sample ring with no interrupt. Every 10 ms the decoder spreads the new samples along the distance rolled in that block, in µm, so bar widths do not depend on speed. The distance comes from `motion_control_distance_um()`, read just after the samples: it counts encoder edges up to that moment, not up to the last 10 ms motion step. It can dip when a wheel rolls back or reverses between two motion steps, as when the car stops; the decoder then stays at its position until the car has rolled past it again. A roll back far enough to take the sensor back across the edge of a bar still adds two elements, and that barcode is rejected. A bar is black above 600 counts and ends below 400 (hysteresis). The thresholds sit low because a narrow bar never fills the sensor's spot: it reads black only briefly, and higher thresholds missed some narrow bars entirely (S3-U14). A barcode starts at the first black after white and ends at a white run over 30 mm. If the ADC or DMA stops, the car would ignore every barcode without a sign, so the task counts the samples of every 10 wake-ups (100 ms, 200 samples due) and sets `APP_FAULT_BARCODE_SENSOR` while a count is under 100. The ADC is started before the task, so start-up does not set it. Driving does not change.
- **Code 39:** 29 elements (`*`, the letter, `*`, with narrow gaps between) are split into narrow and wide, bars and spaces separately, since the sensor's spot makes bars read narrower and spaces wider. Every `*X*` with a letter or digit has exactly 6 wide bars and 3 wide spaces, so the split sits below the 6th widest bar and the 3rd widest space; one bar stretched by a turn cannot move it. `$ / + %` (no wide bars) are not decoded. Each character must have exactly 3 wide elements and match one of the Code 39 characters or `*`, read in either direction; the three must be `*`, a character other than `*`, then `*`, with narrow gaps between them. The decoder keeps the first 31 elements (any after them, such as a crossing line, are ignored) and guesses at two misreads, only where the answer is certain: `*X*` is read exactly starting from any bar of them, so a speck and its gap before the barcode are skipped; and with 27 elements, a narrow bar the sensor missed (it and the spaces either side then read as one space) is put back in each space in turn, accepted only if exactly one letter then reads. Often more than one does (`*A*` missing one bar reads the same as `*U*` missing another), and the barcode is then rejected rather than risk a wrong turn. `g_actions` in `barcode_decoder.c` gives each letter A to Z its command. Every letter is ignored until the supervisor dashboard sends a table over MQTT ([TELEMETRY.md](TELEMETRY.md#map-78-bits)).

## Calibration

Constants are at the top of each `.c` file; change one, rebuild and rerun the bench.

1. **Line sensors:** at the final mounting height over the real track, turn each module's potentiometer until its LED lights over the line only (S3-U03). Space the sensors so that, centred, each sits a few mm clear of the line edge: on this car they are 20 mm apart at their inner edges and about 30 mm centre to centre. With almost no gap the line jumps from one sensor to the other and the car wobbles on straights. Sliding the car across the line by hand must give `left=1`, then both white, then `right=1`, with no `left=1 right=1` between. DO reads 1 over black (`DRV_LINE_BLACK`).
2. **Steering:** `LINE_FOLLOWING_TURN_MIN` (300 permille), `LINE_FOLLOWING_RISE_MM` (30) and `LINE_FOLLOWING_FALL_MM` (10). Raise the min if the car reacts late to a drift, lower it if small corrections look abrupt. Lower the rise if the car runs off curves, raise it if it weaves on straights. Lower the fall if the car overshoots to the other side after a correction, raise it if corrections end abruptly. `LINE_FOLLOWING_DRIFT_MM` (40): lower it if the car still pivots in small false searches on straights, raise it if it weaves while centred. `LINE_FOLLOWING_TURN_MAX` (1000 permille) caps the turn.
3. **Lost-line threshold:** `LINE_FOLLOWING_LOST_MM` (80). Run the bench on the longest straight: `[line]` lines print the odometer (`mm=`) on every sensor change, so the longest all-white stretch is the gap between a `left=0 right=0` line and the next line. Set the threshold about 1.5 times that, but under about 195 mm, the reach of the 135° sweep (`LINE_FOLLOWING_SEARCH_DEG`) on a sharp corner (above). If 1.5 times is more than that, make the car weave more (step 2) instead. A narrower sweep is quicker if the course has no sharp corners. The same lines give the longest stretch with one sensor black: keep `LINE_FOLLOWING_STUCK_MM` (300) at least twice it. **Wobble hold:** `LINE_FOLLOWING_WOBBLE_MM` (150) and `_HOLD_EXIT_MM` (40) decide when holding starts and ends: raise the exit if the hold drops out on straights, lower it if the car holds too long into curves. Lower `_HOLD_BEND_DEG` (8) and `_HOLD_OFF_DEG` (15) if the hold still fights curves, raise them if it drops out on straights. Lower `_HOLD_LOST_MM` (120) if the car runs too far before searching after losing the line while holding, raise it if a held car starts searching on a straight. `_CODE_LOST_MM` (300) must stay over the barcode's length. `_HOLD_GAIN` (50 permille per degree) sets how firmly it holds the heading.
4. **Barcode levels:** hold the sensor still over white and over a black bar (S3-U10) and keep `BARCODE_BLACK` and `BARCODE_WHITE` well inside the two levels, close to white. White reads about 170 counts and black about 4080. Mount the barcode sensor low, with its emitter and detector side by side across the direction of travel (parallel to the bars): along the direction of travel the spot then covers less than a narrow bar. Mounted the other way it covers about 6 mm, and narrow bars read white.
5. **Barcode letters:** set the announced letters under Barcode actions on the supervisor dashboard and send them ([tools/supervisor/](../tools/supervisor/README.md)). An image built without MQTT ignores every barcode.

## Bench sequence

The `make -C build_make TEST=1` image starts task `s3_bench` after the checks. It prints the line state (`[line] left= right= junction= last= following= searching= lost= stuck= mm= deg=`, `stuck` being `APP_FAULT_LINE_SENSOR`) whenever a sensor, the junction, searching or lost flag changes, the barcode level (`[bar] min= max= fault=`, `fault` being `APP_FAULT_BARCODE_SENSOR`) every 0.5 s while no run is going, and every barcode read (`[barcode] command= count= rejects= at_mm=`, with the measured widths).

- **GP21 held for 1 s or more:** after 5 s the car follows the line at 150 mm/s until a lost line or 15 s. At the first junction (printed as `junction=1`) after a barcode with a command, it does what the Vehicle Controller does: prints `[line] junction: command N`, stops following, rolls 115 mm so the axle is on the junction, turns 90° left or right (180° for a U-turn, none for straight on) and follows the line again. It drives straight over other junctions. At the end it stops and prints `[line] bench done after … ms`. Taps under 0.5 s start subsystem 2's bench instead; a press in between starts neither.

## Resources

| Resource              | Use                                                                                 |
| --------------------- | ----------------------------------------------------------------------------------- |
| GP6, GP7              | Right and left IR module DO (Grove 5, Grove 7)                                       |
| GP26, ADC0            | Barcode IR module AO (Grove 6). The kernel's `adca` device is not opened             |
| DMA channel 11        | ADC FIFO to the sample ring                                                          |
| GP21                  | Bench button (pull-up), `TEST=1` only                                                |
| Tasks                 | `s3_barcode` priority 8; `s3_line` priority 9; `s3_bench` priority 9, `TEST=1` only  |
| Kernel objects        | 2 mutexes, 2 cyclic handlers                                                        |

## Coding standard

The code follows BARR-C:2018 with the same deviations as subsystem 2 ([MOTION_CONTROL.md](MOTION_CONTROL.md#coding-standard)), for `subsystem3_test.c` and the task entry points. Abbreviations as listed there.

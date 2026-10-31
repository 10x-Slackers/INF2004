# Subsystem 3 — test log

**Subsystem:** Line following and barcodes  ·  **Student ID:** 2502564  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | Line Following task samples the sensors | FR2, I1 | auto | `TEST=1` image, read the console | `S3-U01 PASS`: the sample count changes within 100 ms |
| S3-U02 | Line state is bit-packed | Minimum-bit data | auto | As above | `S3-U02 PASS`: `line_following_t` is 8 bytes |
| S3-U03 | Each sensor reports black and white | FR2, A3 | manual | Sensors at their final height over the track. Move the car so the line passes under the left sensor, then the right, then both | `[line]` lines show `left=1` only over black on the left, `right=1` only on the right, `junction=1` with both, `last` = 1 (left) or 2 (right) |
| S3-U04 | Out-of-range speeds are rejected | I1 | auto | `TEST=1` image, read the console | `S3-U04 PASS`: `line_following_start()` refuses 0 and 501 mm/s |
| S3-U05 | Line following boots stopped | FR2 | auto | As above | `S3-U05 PASS`: `following=0` until started |
| S3-U06 | The car follows the line | FR2, NFR3 | manual | `TEST=1` image. Hold GP21 for about 2 s and release, put the car on a straight line, then a curved one, with a cross line at the end | Within 5 s the car follows at 200 mm/s without losing the line and stops at the cross line (`[line] bench done`); record the run with a video |
| S3-U07 | Sensors are sampled every 10 ms | FR2, NFR6 | auto | `TEST=1` image, read the console | `S3-U07 PASS`: 10 to 12 samples in a 100 ms wait |
| S3-U08 | Barcode ADC samples at 2 ksample/s | FR3 | auto | `TEST=1` image, read the console | `S3-U08 PASS`: 180 to 240 samples in a 100 ms wait |
| S3-U09 | Raw barcode level is bit-packed | Minimum-bit data | auto | As above | `S3-U09 PASS`: `barcode_decoder_raw_t` is 8 bytes |
| S3-U10 | Barcode sensor tells black from white | FR3, A3 | manual | Sensor at its final height over the track. Hold it still over white, then over a black bar of a sample barcode, 5 s each; read the `[bar]` lines | White and black readings differ by well over the min-max spread of either, so one threshold separates them. Record both levels |
| S3-U11 | `*A*` decodes in both reading directions | FR3 | auto | `TEST=1` image, read the console | `S3-U11 PASS`: synthetic 3 mm / 9 mm widths of `*A*`, forwards and reversed, give A |
| S3-U12 | Invalid barcodes are rejected | FR3 | auto | As above | `S3-U12 PASS`: `*Z*`, and `*A*` with a wide gap, give 0 |
| S3-U13 | Barcode result is bit-packed | Minimum-bit data | auto | As above | `S3-U13 PASS`: `barcode_decoder_t` is 8 bytes |
| S3-U14 | Barcodes are read while driving | FR3, NFR3, A3 | manual | `TEST=1` image. Lay a printed `*A*` 20 mm beside a straight line, then a `*Z*`. Hold GP21 for 1 s, put the car on the line before them | `[barcode] command=1` with count +1 for `*A*`; rejects +1 and no new command for `*Z*`; then repeat driving the other way along the line |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-31 | S3-U01 | `195db26` | mykacx | PASS | `logs/2026-10-31_unit.txt` | |
| 2026-10-31 | S3-U02 | `195db26` | mykacx | PASS | `logs/2026-10-31_unit.txt` | |
| 2026-10-31 | S3-U04 | `195db26` | mykacx | PASS | `logs/2026-10-31_unit.txt` | |
| 2026-10-31 | S3-U05 | `195db26` | mykacx | PASS | `logs/2026-10-31_unit.txt` | |
| 2026-10-31 | S3-U03 | `195db26` | mykacx | PASS | `logs/2026-10-31_unit.txt` | Left, right and both over the line read as expected; DO is 1 over black |
| 2026-10-31 | S3-U06 | `ca4b8b1` | mykacx | FAIL | `logs/2026-10-31_follow.txt` | Ran off the line: after one left=1 reading neither sensor saw the line again. Cause under investigation |
| 2026-10-31 | S3-U03 | `2e899b0` | mykacx | PASS | `logs/2026-10-31_follow_straight.txt` | Re-run after the left and right sensor cables were swapped back to the pin map (left on Grove 5, right on Grove 7); the earlier PASS was taken with them mirrored |
| 2026-10-31 | S3-U06 | `2e899b0` | mykacx | PASS | `logs/2026-10-31_follow_straight.txt` | Straight line only, no curve available: followed about 75 cm in 3.8 s without wobbling and stopped at the junction. The curve still needs a run |
| 2026-10-31 | S3-U01, U02, U04, U05, U07, U08, U09 | `c02263f` | mykacx | PASS | `logs/2026-10-31_barcode_raw.txt` | U07 confirms the 10 ms cyclic wake-up; U08 the ADC and DMA rate. Tested as `4bbee29` before a rebase; `c02263f` has the same subsystem 3 code apart from the GP21 bench timing. |
| 2026-10-31 | S3-U10 | `c02263f` | mykacx | PASS | `logs/2026-10-31_barcode_raw.txt` | White 144 to 195, black 4058 to 4095 (near full scale): AO rises over black, and any threshold around 2000 separates them. Tested as `4bbee29` before a rebase; `c02263f` has the same subsystem 3 code apart from the GP21 bench timing. |
| 2026-10-31 | S3-U01, U02, U04, U05, U07, U08, U09, U11, U12, U13 | `76d20ea` | mykacx | PASS | `logs/2026-10-31_barcode_decode.txt` | Tested as `a150618` before a rebase; `76d20ea` has the same subsystem 3 code apart from the GP21 bench timing. |
| 2026-10-31 | S3-U14 | `76d20ea` | mykacx | FAIL | `logs/2026-10-31_barcode_drive.txt` | No barcode validated; one pattern of 9 or more elements rejected. The line sensors never changed during the 15 s run, so the setup was in doubt. The bar widths were not logged; `783ab28` prints them. Re-run once the sensor mount is repaired. Tested as `a150618` before a rebase; `76d20ea` has the same subsystem 3 code apart from the GP21 bench timing. |

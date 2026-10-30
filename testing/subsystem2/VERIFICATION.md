# Buddy 2 verification

## Fresh cycle — 2026-10-30

Sources reviewed: *Project Write-up.pdf* (11 pages), *Project Briefing.pdf*
(10 slides), *Change Requirement 1.docx* and *BARR-C:2018* (87 pages).
The [requirement checklist and C contract](../../docs/MOTION_CONTROL.md)
distinguish official requirements from team policies and unspecified values.

No hardware was flashed or operated. All physical cases below are **NOT RUN**.
Earlier hardware trials/calibration/fault theories are superseded for this cycle.
The replacement encoder is not presumed faulty or calibrated.

Base revision: `b0f93f754eabaf2363c02e7fdba56da6528e89fd`, with pre-existing
uncommitted changes. No commit is created. The source hashes in [current.txt](current.txt)
identify the tested working files; the base revision alone does not contain them.

## Software checks

`make verify` records the host suite, sanitizer suite, packed/full-width
representation comparison, firmware builds and source checks in `current.txt`.
Host fixtures use synthetic geometry/CPR/gains, never robot calibration values.

| Case | Coverage | Result |
| --- | --- | --- |
| S2-N01 | All 16 quadrature transitions, both directions, invalid jumps and saturation | PASS, C host fixture |
| S2-N02 | Actual ISR: all 16 startup phase pairs, GPIO mapping, sample/ack interleavings, W1C and shared callback | PASS, production C sections compiled against mock registers |
| S2-N03 | Actual PWM output mapping, bounded duty, zero shutdown and brake on reversal | PASS, production C output section with mock registers |
| S2-N04 | Speed/distance units, total-count rounding, independent CPR and straight correction | PASS, C host fixture |
| S2-N05 | Forward/reverse/left/right signs, independent completion and count-capacity rejection | PASS, C host fixture |
| S2-N06 | All 16 motor/encoder polarity combinations normalize synthetic forward correctly | PASS, C host fixture |
| S2-N07 | PID bounded output, anti-windup/reset, 5/10/15 ms integration and derivative scaling | PASS, C host fixture |
| S2-N08 | Locked count/time pairing, stale entry-time hint, configuration baseline and timer wrap | PASS, C host/register fixtures |
| S2-N09 | Stall, timeout, invalid transitions, overflow, numeric range, stale data and execution overrun | PASS, C host fixture |
| S2-N10 | Synchronous stop/disarm/safety, early-notification fault, latched reset/no automatic resume, console stop/backlog | PASS, C host fixture |
| S2-N11 | Coherent concurrent snapshots, rejection of overlapping requests and binary boundary/roundtrip validation | PASS, pthread C host fixture; peer integration unrun |
| S2-N12 | Real task/init/startup selection, event coalescing, bounded waits and init failure; SMP 0/1, TEST 0/1, standalone 0/1 | PASS, kernel API model |
| S2-N13 | Uncalibrated boot, explicit confirmation/arm, runtime edits and invalid default rejection | PASS, three synthetic/default profiles |
| S2-N14 | AddressSanitizer and UndefinedBehaviorSanitizer | PASS, no sanitizer findings |
| S2-N15 | Integrated, standalone, diagnostic and direct build_make normal/TEST firmware | PASS, no warnings; build only, not target execution |
| S2-N16 | Field widths, packed/full-width RAM/message comparison, focused source checks | PASS; measurements below; full BARR review remains separate |

An intermediate draft of S2-N05 failed because its fixture jumped close to the
count limit too quickly, producing an out-of-range synthetic speed. The RANGE
guard correctly stopped. The fixture interval was corrected; the guard was not
weakened. Subsequent host run passed. This is a test setup correction, not a
hardware observation or a firmware fix.

## Minimum-bit change evidence

`compare_representation.py` compiles identical algorithms with the real packed
fields and a temporary reference where unsigned bit-fields become `uint32_t`.
It runs the same functional suite on both and compiles the portable modules
for ARM. This is a controlled representation comparison, **not** a historical
pre-change firmware or physical performance baseline.

| Measurement | Full-width reference | Packed | Difference |
| --- | --- | --- | --- |
| ARM static RAM, control + math + console (`.data + .bss` of their objects) | 336 B | 260 B | 76 B less |
| Configuration RAM struct | 68 B | 28 B | 40 B less |
| Status snapshot / I4/I5 copy | 60 B | 44 B | 16 B less |
| PID state per wheel | 8 B | 4 B | 4 B less |
| Encoder snapshot | 32 B | 16 B | 16 B less |
| Telemetry application payload | 60 B as 15 native-width fields | 36 B explicit binary | 24 B less |

Struct sizes include compiler padding; field widths are documented in
[DATA_REPRESENTATION](../../docs/DATA_REPRESENTATION.md). No inter-task command
queue exists, so there is no queue-size saving to claim. The telemetry baseline
is a native-width binary representation of the same fields, not an invented
previous MQTT text payload. The packet has 287 data bits plus one padding bit.

A fresh pre-edit production build in this session measured text 293492 B,
data 88 B, bss 24952 B (static RAM 25040 B). This pre-edit firmware already used
packed fields; comparing it with the revised image measures cleanup, not the
minimum-bit change itself. Final image sizes are recorded below/current.txt.
Task stacks and RTOS allocation/high-water usage require target measurement.
Host packing/control timings in the transcript are host-only observations;
no claim of acceptable RP2040 timing degradation follows from them.

## Physical acceptance and calibration

| Case | Procedure / evidence needed | Result |
| --- | --- | --- |
| S2-H01 | Boot motor-isolated, diagnostic banner, S2-U01/U02/U03 boot checks, arm rejection and zero PWM | NOT RUN |
| S2-H02 | Each wheel forward/backward manually; correct GPIO ownership, raw signs and no new invalids | NOT RUN |
| S2-H03 | Repeated marked revolutions per wheel/direction, x4 CPR and spread; actual encoder output levels/type | NOT RUN |
| S2-H04 | Loaded wheel circumference and wheel-centre track measurements | NOT RUN |
| S2-H05 | Electrical motor polarity and normalized encoder signs agree with physical forward | NOT RUN |
| S2-H06 | Supervised bounded motor pulses, measured PWM/reversal brake and stop timing | NOT RUN |
| S2-H07 | Loaded independent-wheel PID tuning and straight-line drift | NOT RUN |
| S2-H08 | Repeated forward/reverse distances and left/right 90°/180° turns, external ground truth and agreed tolerances | NOT RUN |
| S2-H09 | Safety/stop behavior and latency; fault-reset/new-Start integration | NOT RUN |
| S2-H10 | I3/I4/I5/I11 peers, shared IRQ13, binary MQTT transport and line/avoidance handoff | NOT RUN |
| S2-H11 | Sensor/Wi-Fi load: jitter/capture bandwidth, CPU, stack, memory, power and physical stopping | NOT RUN |

Retain setup, image hash, runtime configuration, supply/load, raw results and
failed trials. Follow [README](README.md); unresolved decisions are in [ISSUES](ISSUES.md).
Do not infer physical accuracy from encoder-based completion or host tests.

## PID tuning report

No hardware tuning data supplied for this cycle. Record each wheel's gains
(Q8), desired/measured/external speed, sample timing, supply/load, output limit,
overshoot, settling time, steady error and saturation. Include the chosen
acceptance limits and observations that justify final gains. Zero defaults
are unconfigured, not a selected controller tuning.

## Motion accuracy evaluation

No physical accuracy data supplied for this cycle. For each forward/reverse
and left/right turn trial record requested travel/angle, external measured
value, encoder outcome, signed error, drift, elapsed time and faults. Calculate
mean and spread over repeated trials against team-agreed tolerances. Record
circumference, track, CPR, polarity and image hash with the measurements.

## Final automated run

`make verify` completed with exit status 0 on 2026-10-30 using ARM GNU
Toolchain 15.3.Rel2. Each packed production, diagnostic and full-width reference
motion run passed **13657 assertions**, repeated under ASan/UBSan. Additional
C task/startup, default, real ISR/PWM fixtures and source checks passed. All
required builds completed without warnings, including `make -C build_make -j8`
and `make -C build_make TEST=1 -j8` (8 jobs used because macOS has no `nproc`).
`git diff --check` passed. Full teammate/hardware integration is not claimed.

| Final image | Text | Data | BSS | Static RAM (data + BSS) |
| --- | --- | --- | --- | --- |
| Integrated production | 291264 B | 88 B | 24824 B | 24912 B |
| Standalone diagnostic TEST | 75900 B | 36 B | 22532 B | 22568 B |

Compared with the freshly measured pre-edit production image, static RAM is
128 B lower and text is 2228 B lower. This is cleanup evidence; the separate
packed/full-width comparison above isolates representation savings.

Diagnostic artifact:
`build/smp1-usb_cdc-none-none-mqtt0-crash0-test1-s2only-s2diag/mtk3pico_smp1_usb_cdc_test_s2only_s2diag.uf2`

SHA256: `6ced8358277db16e72f1a3e4682b64ab268ca5efa8a7428be77b4cb5c31fdf9b`.

Removed superseded per-event range/invalid/ISR timing captures, transition
matrices/history, paired-pulse/duty traces and their dedicated fixtures. Kept
raw encoder counts/phases/invalid totals, normal status/faults and the standard
bounded single-wheel bench endpoint. Orphaned `motion_diag` object/dependency
files and the old host history executable were deleted. Official source
specifications, teammate source and the shared build/startup contract were
preserved. No flashing, motor operation, commit, push or merge was performed.


## Encoder configuration syntax inspection

Request: investigate rejected `config elrev 1` and `config errev 0` with
controller disarmed/fault-free. The user reports fresh raw forward/backward
deltas LEFT −2488/+2414 and RIGHT +2550/−2455, both with zero invalids. These
support left reversal 1 and right reversal 0; they are not used to select CPR,
motor polarity, gains or thresholds. Hardware execution/image/configuration
was not independently observed during this inspection.

Finding: `console_motion_config()` recognizes only `el` and `er`. Printed
`elrev`/`errev` are report labels. Unknown keys return false before the
short-circuited call to `motion_control_configure()`, leaving live configuration
unchanged. Supported commands are `config el 1` and `config er 0`.

The API exposes one-bit reversal fields and applies sign normalization during
measurement; encoder polarity is configurable, not fixed. API configuration
requires disarmed state and representable normalized counts. Console edits
clear confirmation, allowing incomplete calibration to be staged. Reconfirmation
requires complete valid configuration; arming additionally requires fresh data
and zero faults. RAM edits are lost on reboot.

The official Buddy 2 allocation (Project Write-up p. 6) requires motor/encoder
integration, motion calibration, speed/distance/PID/straight correction/turning
and reusable motion APIs; it does not prescribe console key spellings. This
polarity configuration meets that calibration/interface need. Whole-subsystem
physical accuracy, PID tuning/reporting and joint integration remain separate
acceptance work; the reported counts alone do not close those requirements.

No firmware source correction or alias was needed. Added
`test_encoder_config_syntax()` to the existing C fixture: unknown-key rejection
without mutation, armed edit rejection, supported keys, Boolean range checks,
confirmation clearing, forward/backward normalization, no output while disarmed
and polarity staging before geometry/CPR/gains. Clarified README syntax only.

Validation: `make verify` passed with 13683 assertions per production,
diagnostic and full-width reference motion run, including ASan/UBSan. Source
checks, kernel/ISR/PWM fixtures, all firmware profiles and both direct
`build_make` normal/TEST entrypoints passed without warnings. `git diff --check`
passed. The updated transcript and source hashes are in `current.txt`.

All Buddy 2 firmware source/header hashes are unchanged from the start of this
inspection. Diagnostic UF2 SHA256 remains
`6ced8358277db16e72f1a3e4682b64ab268ca5efa8a7428be77b4cb5c31fdf9b`.
No firmware settings or outputs were changed on hardware. Rebuilding/reflashing
is not needed to use supported syntax; only host-test source, documentation
and the automated transcript changed. No commit, push, merge or flash occurred.

## 2026-10-30 — Reviewed boot calibration defaults

Loaded the operator-supplied hardware-verified working profile into
`motion_defaults.h`: circumference 204 mm, track 113 mm, left/right four-edge
CPR 2466/2524, Q8 gains 256/1/0, speed 100 mm/s, duty limit 100 permille,
timeout 1500 ms, stall 500 ms, motor reversal 1/0 and encoder reversal 1/0.
Shared CPR remains unused (zero). Confirmation remains zero on every boot.
Startup/control logic, GPIO assignments and configuration layout are unchanged.

Extended the defaults/reboot host fixture to verify all gains and operating
limits, zero outputs at boot, rejection before confirmation, successful explicit
confirmation followed by a fresh sample and arm, zero output after arming, and
restoration of defaults rather than runtime edits after simulated reboot.
All three default profiles pass, including the checked-in per-wheel profile.

`make -s test` and `make -s verify` passed. Verification includes production,
diagnostic and full-width reference motion fixtures (13683 assertions each),
ASan/UBSan, startup/scheduling and actual ISR/PWM fixtures, source checks,
integrated normal/TEST, standalone normal/TEST, diagnostic and direct
`build_make` normal/TEST firmware builds. No compiler warnings or errors were
reported. The saved transcript and source hashes are in `current.txt`.

No new physical measurements were made, and no hardware configuration was
changed. These defaults require rebuilding and later operator-controlled
flashing to take effect on the Pico. No flash, commit, push or merge occurred.
Powered emergency-stop timing and physical turn-angle acceptance remain
separate verification work.

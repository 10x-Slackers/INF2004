# Subsystem 5 — test log

Subsystem: adaptive ultrasonic scanning, obstacle profiling and recovery.
See `testing/README.md` for formal committed-run evidence rules.

## Test cases

All automatic cases run the same pure C code on the host and in TEST=1.
Host shims replace only kernel declarations/console, never the planner.

| ID | What is tested | Requirement / interface | Method | Expected result |
| --- | --- | --- | --- | --- |
| S5-U01 | Minimum-bit layouts and sizes | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | sample=8, profile=16, scan=100, control=32, health=36 |
| S5-U02 | Echo conversion | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | 5800 us = 1000 mm |
| S5-U03 | No echo | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | Invalid and never forward clear |
| S5-U04 | Closest point | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | 200 mm at 90 degrees |
| S5-U05 | Centred coarse sector | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | Valid edges at 90; width unknown |
| S5-U06 | Left-only clearance | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | BYPASS_LEFT with calibration enabled in test |
| S5-U07 | Right-only clearance | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | BYPASS_RIGHT with calibration enabled in test |
| S5-U08 | Both blocked | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | SAFE_STOP, no reverse |
| S5-U09 | All invalid | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | Invalid profile and bounded retry |
| S5-U10 | Forward age and angle | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | 200 ms inclusive; 201 ms / sideways rejected |
| S5-U11 | Scan retry bound | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | Two retries then latched SAFE_STOP |
| S5-U12 | Fault bitmask | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | Seven-bit mask combines to 97 |
| S5-U13 | Filter | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | Invalid ignored; conservative nearer of two |
| S5-U14 | Timestamp wrap | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | Unsigned age remains correct |
| S5-U15 | Missing stop confirmation | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | Wait for stop, no detailed request |
| S5-U16 | Confirmed stop | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | Coarse request starts |
| S5-U17 | Cancellation | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | Latched stop; pending scan drains |
| S5-U18 | Recovery bound | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | Two retries then fault/stop |
| S5-U19 | Tie and calibration | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | Left deterministic; uncalibrated stop |
| S5-U20 | Unknown side | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | Never infer clearance |
| S5-U21 | Stop deadline | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | 2000 ms expiry stops |
| S5-U22 | Fine replacement | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | Duplicate angle replaced; oldest timestamp retained |
| S5-U23 | Profile age | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | Older than 6000 ms stops |
| S5-U24 | Emergency range | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | 100 mm stops |
| S5-U25 | Stop latch | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | ACTIVATE cannot clear stop |
| S5-U26 | Reset | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | Standstill required; no cached forward authorization |
| S5-U27 | Range bounds | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | 20/4000 valid; below/above invalid |
| S5-U28 | Invalid filter | Minimum-bit / FR6 / FR7 / NFR5 / I6–I10 | auto | No fabricated valid sample |

Run from the repository root in the Dev Container/WSL:

```sh
bash testing/subsystem5/host/run.sh
make -C build_make -j$(nproc)
make -C build_make TEST=1 -j$(nproc)
```

The host runner treats FAIL output as failure. Flash the TEST=1 UF2 to run
the same tests on the Pico; host success is not target execution evidence.

## Physical test plan (not performed)

Before power: HC-SR04 VCC=5 V; common ground; TRIG=GP0;
ECHO through a resistor divider/level shifter before GP1 (e.g. 1 kohm series,
2 kohm to ground). Confirm GP1 never exceeds 3.3 V. GPIO is not 5 V tolerant.
Use USB CDC, CRASH_REPORT=0. Never enable UART on GP0/GP1 with sensor connected.
Support the chassis for initial actuator tests. Keep bypass calibration gate
off until geometry and swept-body clearance have been measured.

| ID | Test and steps | Expected / record | Dependency |
| --- | --- | --- | --- |
| S5-H01 | Flat target at 100, 200, 300, 500, 1000 mm; 10 readings each | Record actual, measured, absolute error, validity and repeatability | HC-SR04 + tape |
| S5-H02 | Scope TRIG/ECHO; remove target, hold ECHO high via safe 3.3 V source | TRIG >=10 us, trigger interval >=60 ms; waits <=25 ms each; invalid never clear | Scope / safe signal source |
| S5-H03 | Command calibrated 30/90/150 positions; observe mounting | No mechanical stop; measure actual direction, angle, pulse width, settle; 50 Hz | Servo + protractor/scope |
| S5-H04 | Centre/left/right obstacles; request stop then coarse scan | Correct closest point and candidate sector, logged against ground truth | Stop-confirmation adapter |
| S5-H05 | Narrow target near coarse angle; request fine after coarse | Seven local 5-degree rays; edges refined if within window; no full resweep | Scanner + angle ground truth |
| S5-H06 | Object of measured width at known distances | Width remains explicitly invalid; compare angular edges; do not record zero as measured width | Width method deferred |
| S5-H07 | Forward reading, pan sideways, wait >200 ms | front_clear false sideways and stale; fresh forward required before movement | Scanner / snapshot adapter |
| S5-H08 | Repeat H01 with motors stopped/running/accelerating | Log invalid fraction/error; tune filter/settle; detailed scans remain stopped | Buddy 2 |
| S5-H09 | Block both sides; repeat with absent echoes and <=100 mm obstacle | SAFE_STOP; no blind reverse; retry bound respected | Integrated stop path |
| S5-H10 | Bypass; supply line success, then deliberately fail reacquisition | RECOVERY request; max two retries; SAFE_STOP on exhaustion | Buddy 2 + Buddy 3 + controller |
| S5-H11 | Scope motor PWM while panning servo; safety cancel mid-scan under network load | Slices 4/5 unchanged; only slice 6 at 50 Hz; measure stop latency, CPU, stack high-water and power | Buddy 2 + safety/network integration |
| S5-H12 | Measure body width, sonar offset, track geometry and required margin; test swept bypass envelope | Establish safe clearance/age/deadlines; enable calibration gate only after passing | Complete chassis and track |

HC-SR04 measurement worksheet (repeat rows for each trial):

| Actual mm | Measured mm | Absolute error mm | Valid | Evidence |
| --- | --- | --- | --- | --- |
| 100 | | | | |
| 200 | | | | |
| 300 | | | | |
| 500 | | | | |
| 1000 | | | | |

## Representation and RAM evidence

See `docs/DATA_REPRESENTATION.md` for every field/range/owner. S5-U01 prints
GCC host sizes; target compile-time assertions independently check all final
record sizes, including private message=104 bytes and request=4 bytes.
The explicitly unpacked reference sample has four unsigned int fields:
16 bytes versus the 8-byte final sample on these ABIs (8 bytes/sample,
96 bytes per twelve-sample array). This is a reference-layout comparison,
not a historical firmware RAM saving: Buddy 5 previously contained no data.
Profile=16, scan=100, control=32, health=36 bytes. Queue backing arrays are
432+8=440 bytes. Two existing 2048-byte startup task stacks are requested;
measure stack high-water on hardware. No measured CPU/power savings claimed.

## Results and evidence

Formal result rows require a committed revision and saved target/bench logs.
The preliminary working-tree run below is explicitly not that formal record;
no commit or hardware result is invented. `logs/verification.json` records
base commit plus SHA-256 hashes of tested source files, commands and tool
versions. Host console output is in `logs/host-unit.txt`; firmware compiler
logs are `logs/build-normal.txt` and `logs/build-test.txt`.

| Date | Test ID | Revision | Tester | Result | Evidence | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-10-26 | S5-U01–S5-U28 | Working tree; base + source hashes in verification.json | Codex host runner | 28 PASS, 0 FAIL | logs/host-unit.txt | Host execution only; repeat after commit on target |
| 2026-10-26 | Normal / TEST=1 build | Same working tree | Codex WSL Arm GCC | PASS | logs/build-normal.txt; logs/build-test.txt | Compile/link/UF2 only; no physical execution |

No S5-H test has been performed. No real bypass, line recovery, stop latency,
servo angle, width accuracy or chassis clearance has been validated.

## Temporary Buddy 5-only build verification

2026-10-29: normal and TEST=1 firmware builds passed without warnings with
APP_BUDDY5_ONLY=1 in app_main.c. Buddies 1–4 init/start/test calls are excluded;
Buddy 5 remains enabled. Kernel boot output is outside this application switch.
Set APP_BUDDY5_ONLY=0 to restore the combined application.

Evidence: logs/2026-10-29_buddy5-only-normal.txt and
logs/2026-10-29_buddy5-only-test.txt. Compile/link verification only; no new
physical run was performed. Uncommitted app_main.c SHA-256: 3E4CE2595F40643E2FB6B1BF2ABF315BEA2F1788539918D28ABF4DF96DE6C966.


## S5-H13 — temporary on-target API bench test

Run TEST=1 with only HC-SR04 and SG90 attached; keep the assembly stationary
on the desk. `S5_RUN_HARDWARE_BENCH_TEST` in subsystem5_test.c enables this
one-shot test; set it to zero to remove the bench sequence from test firmware.
S5_HOST_TEST and normal firmware exclude it entirely. Existing S5-U01–U28
remain unchanged. S5-U29 separately checks completed-profile validity on a
planner SAFE_STOP; cancellation/motion still invalidate the profile.

The bench requests FORWARD via the production API, polls every 20 ms and
requires a changed forward timestamp or observed busy-to-idle transition to
recognize completion. It never mistakes the initial idle snapshot for a
completed request. When WAIT_STOP follows an obstacle/invalid forward range,
it explicitly labels and queues a stationary-desk BENCH-TEST-ONLY stop
confirmation, then observes the existing coarse/fine/profile sequence.
It prints forward data, mode/retry progress, the final profile and names for
state/action. It never commands servo angles or motor pins, reports motion
completion, or simulates line recovery.

The shared overall deadline is `(S5_MAX_SCAN_RETRIES + 3) * S5_SCAN_MAX_MS +
S5_WAIT_STOP_MS` = 42000 ms with the initial constants. A deadline or API
failure prints an error and requests CANCEL; the current driver measurement
may finish afterward. Health uses the existing short-held snapshot mutex;
there are no new kernel objects. All exits print HARDWARE TEST COMPLETE.

| ID | Method | Steps | Expected |
| --- | --- | --- | --- |
| S5-H13-A | manual/on-target | Aim at a flat target >350 mm and <=4000 mm | Valid forward range, MONITOR/CONTINUE, no detailed scan |
| S5-H13-B | manual/on-target | Box 200–300 mm ahead; provide valid side/background returns within 4 m | Bench stop confirmation; COARSE then FINE; real profile; calibration-gated SAFE_STOP is expected |
| S5-H13-C | manual/on-target | Flat target 20–100 mm ahead | Valid critical range, STOPPED/SAFE_STOP; no coarse/fine |
| S5-H13-D | manual/on-target | No Echo/invalid returns | Invalid never clear; bench stop confirmation; bounded coarse retries then SAFE_STOP, or overall timeout |

Start Serial Monitor before resetting so the unit tests and bench header are
captured. Repeat from reset for each arrangement. GP0=TRIG, GP1=level-shifted
ECHO, GP12=SG90; common ground, sensor 5 V, USB CDC / CRASH_REPORT=0.
Sideways no-Echo readings invalidate a coarse profile and can prevent FINE;
this is existing behaviour, not a bench failure hidden by fake samples.
No physical S5-H13 result has been recorded.


### Preliminary software verification — 2026-10-29

Working-tree host run: S5-U01–U29 PASS (29/29). The existing U01–U28 cases
were retained; U29 covers the narrow completed-profile validity fix. Both
normal and TEST=1 firmware compiled/linked/generated UF2 with no warnings.
Pinned clang-format 23.1.3 and diff whitespace checks passed.

Evidence: logs/2026-10-29_bench-host-unit.txt,
logs/2026-10-29_bench-build-test.txt,
logs/2026-10-29_bench-build-normal.txt and
logs/2026-10-29_bench-verification.json (source hashes/base revision).
A temporary software-only replay executed the actual bench polling helper
with synthetic API responses for A/B/C/D and a scanner-never-starts timeout;
all five assertions passed. Its explicitly labelled output is in
logs/2026-10-29_bench-control-replay.txt. This does not exercise GPIO, PWM,
real-time scheduling or sensors. The hardware bench code is absent from the
normal firmware and host unit binary and present in the target test image.
No on-target or physical test result is claimed; repeat S5-H13 on the Pico
and save the actual serial log before recording a physical PASS/FAIL.


## Timer startup and stalled-clock regression tests

The range timer is separate from the kernel CPU-clock SysTick. Driver init
must enable the watchdog tick reference at 1 MHz, release TIMER without
asserting reset, unpause and verify counter advancement before ranging.
It does not enable watchdog resets or allocate/use timer alarms. Measurement
spacing has an independent 100 ms RTOS deadline. Echo waits retain their
25 ms raw-timer deadline and check a 35 ms RTOS fallback every 128 polls.
The occasional fallback checks leave the microsecond timer as the measurement
clock; they are not a replacement timing source.

Run `bash testing/subsystem5/host/hcsr04/run.sh` from the repository root.
This compiles the actual driver against mock registers/time, not GPIO hardware.

| ID | Method | Expected |
| --- | --- | --- |
| S5-U30 | host driver/MMIO simulation | Set 12-cycle watchdog tick, release TIMER only and verify advancement |
| S5-U31 | host driver/MMIO simulation | 1160 us simulated pulse converts to approximately 200 mm |
| S5-U32 | host driver/MMIO simulation | No rising Echo returns invalid within deadline |
| S5-U33 | host driver/MMIO simulation | Stuck-high pulse returns invalid within deadline |
| S5-U34 | host driver/MMIO simulation | Frozen timer cannot trap spacing wait; SCAN_TIMEOUT |
| S5-U35 | host driver/MMIO simulation | Frozen timer fails initialization before GPIO configuration |
| S5-U36 | host driver/MMIO simulation | Frozen counter cannot trap rising-edge wait |
| S5-U37 | host driver/MMIO simulation | Frozen counter cannot trap high-pulse wait |

For a standalone desk bench use `make -C build_make TEST=1 WIFI=none`;
flash `build_make/mtk3pico_smp1_usb_cdc_test.uf2`. The project Wi-Fi default
is unchanged. The bench prints TIMER delta_us/elapsed_ms after the forward
request and reports scanner state/servo angle at completion. If initialization
fails the scan reports SCAN_TIMEOUT instead of mislabelling it a servo fault.
No physical rerun is claimed by these software regressions.


### Timer fix evidence — 2026-10-29

User-reported S5-H13 hardware run before the fix: FAIL (42-second timeout,
SCAN_TIMEOUT/stale flags, pending=1). The exact flashed revision is unknown;
raw user-supplied console text is preserved in
logs/2026-10-29_user-reported-timeout.txt. This was reported by the user,
not physically executed by Codex.

Working-tree software verification after the fix: 29 logic checks and eight
actual-driver/MMIO simulations PASS. Normal, TEST=1 and TEST=1 WIFI=none
builds compile/link/generate UF2 without warnings. Pinned formatter and
whitespace checks pass. Evidence: logs/2026-10-29_timer-logic-unit.txt,
logs/2026-10-29_timer-driver-unit.txt, logs/2026-10-29_timer-build-normal.txt,
logs/2026-10-29_timer-build-test.txt, logs/2026-10-29_timer-build-bench.txt
and logs/2026-10-29_timer-verification.json (source and UF2 hashes).
The standalone bench ELF contains the timer diagnostic and no Wi-Fi startup
message. No physical rerun or success after the fix is claimed.


## Migratable trigger-pulse regression

The SMP port disables core 1 SysTick in `knl_smp_core1_entry`, but the BSP's
WaitUsec implementation polls the calling core's SysTick counter. Buddy 5's
scan task is migratable. Therefore WaitUsec can stall with TRIG high before
the existing Echo deadlines are reached. The driver generates its 10 us
trigger with the shared raw timer instead; no core affinity, kernel, obstacle
threshold, freshness limit or retry/scan deadline is changed.

| ID | Method | Expected |
| --- | --- | --- |
| S5-U38 | actual driver, mock MMIO | TRIG high for at least 10 us without any WaitUsec call; mocked WaitUsec fails immediately if called |
| S5-U39 | actual driver, mock MMIO | A frozen timer during TRIG releases the pin LOW and exits invalid with SCAN_TIMEOUT |

U36/U37 freeze the reference after the trigger's falling edge to continue
checking the separate Echo deadlines. Use the same hcsr04/run.sh runner.
These are simulations; multicore timing/accuracy must be rerun on the Pico.


### Trigger fix verification — 2026-10-29

Pre-fix user logs still timed out after entering FINE, with scanner measuring
and pending=1. Source review identifies an additional unbounded path before
Echo polling: WaitUsec on core 1's disabled SysTick. Timer startup alone did
not address this path. The correction stays in the Buddy 5 driver.

Working-tree software results: 29 logic tests and 10 actual-driver/mock-MMIO
tests PASS. Normal, TEST=1 and TEST=1 WIFI=none builds PASS without warnings.
Formatter 23.1.3 and diff whitespace checks PASS. The HC-SR04 object has no
WaitUsec reference (other kernel drivers may still use it); the standalone
bench ELF has no Wi-Fi startup message. Evidence:
logs/2026-10-29_trigger-driver-unit.txt,
logs/2026-10-29_trigger-logic-unit.txt,
logs/2026-10-29_trigger-build-normal.txt,
logs/2026-10-29_trigger-build-test.txt,
logs/2026-10-29_trigger-build-bench.txt,
logs/2026-10-29_trigger-driver-symbols.txt and
logs/2026-10-29_trigger-verification.json (source/UF2 hashes).
No physical pass after this trigger correction is claimed. Reflash the newly
built build_make/mtk3pico_smp1_usb_cdc_test.uf2 and repeat S5-H13. A completed
valid obstacle profile can still recommend SAFE_STOP because calibration is
gated; expect pending=0 and scanner idle, not an indefinite pending request.

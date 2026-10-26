# Subsystem 2 — test log

**Subsystem:** Motion control  ·  **Student ID:** 2503068  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | Boot disarmed | FR4, FR11 | auto target | `make TEST=1`, boot | No arm permission |
| S2-U02 | Reject disarmed movement | FR11 | auto target | Same image | No movement accepted |
| S2-U03 | Binary snapshot round trip | I5, minimum-bit CR | auto target | Same image | Preserved fields |
| S2-U04 | PWM limits/reversal, quadrature and PID boundaries | FR4 | host | `make test` | Assertions pass |
| S2-U05 | Speeds, distance, straight correction, both turns | FR4, I3/I4/I5 | host | `make test` | Synthetic geometry completes with expected signs |
| S2-U06 | Stop, combined faults, stall, timeout, stale/range faults | FR11 | host | `make test` | Braked outputs and explicit recovery |
| S2-U07 | Packed boundaries, byte order and concurrent access | Minimum-bit CR | host | `make test` | No lost flags; valid binary contract |
| S2-U08 | Console bounds, polarity changes, timer wrap | FR4, FR11 | host | `make test` | Reject invalid inputs, preserve valid samples |
| S2-U09 | RAM/message/timing reference comparison | Minimum-bit CR | host + cross-build | `python3 testing/subsystem2/host/compare_representation.py --firmware` | Identical logic passes with full-width fields |
| S2-U10 | PWM frequency, boot brake and safe reversal | FR4 | manual | Lift wheels; scope GP8–11 during bounded pulses | 20 kHz, low/low at boot/stop, braked reversal |
| S2-U11 | Actual encoder counts, polarity and capture rate | FR4, A2 | manual | Rotate wheels; compare external counter under load | Calibrated counts and no lost edges |
| S2-U12 | PID tuning | FR4, A2 | manual | `CALIBRATION.md` procedure | Measured step response within agreed limits |
| S2-U13 | Distance/turn accuracy | FR4, NFR4 | manual | Repeated measured forward/reverse and left/right trials | Errors within team-agreed limits |
| S2-U14 | Loaded timing and physical stop | FR11, NFR6 | manual | Scope brake and record `status` under sensor/network load | Bounded measured latency; no missed control deadlines |
| S2-U15 | C99, focused BARR checks, sanitizers and fresh firmware builds | NFR1/NFR2 | host + cross-build | Test log commands | No diagnostics; ELF/MAP/UF2 generated |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-26 | S2-U04–U08 | Uncommitted working tree; fingerprint in log | Codex host fixture | PASS | `logs/2026-10-26_host.txt` | Software simulation, not hardware |
| 2026-10-26 | S2-U09 | Same worktree fingerprint | Codex cross-build | PASS | `logs/2026-10-26_representation.txt` | Generated full-width reference, not deleted/historical code |
| 2026-10-26 | S2-U15 | Same worktree fingerprint | Codex toolchain | PASS | `logs/2026-10-26_build.txt`, `logs/2026-10-26_sanitizers.txt` | Fresh source snapshot, static checks and sanitizers |

## Hardware evidence

S2-U01–U03 are compiled but not executed on a Pico. S2-U10–U14 await physical
measurements. [CALIBRATION.md](CALIBRATION.md) is the combined PID-tuning and
motion-accuracy report template. No host result establishes wheel accuracy,
interrupt capture limits, RP2040 worst-case execution time or stop latency.

## Representation evidence

`make test` generates a reference by replacing Buddy 2 unsigned bit-fields with
`uint32_t` members, preserving the same algorithms, offsets and fault mask. The
optional `--firmware` command cross-compiles that variant separately. This is a
reproducible full-width reference, **not** the removed prior implementation or a
measurement of its savings. There is no claim that RAM structs are wire packets.

A full-width status has fifteen 32-bit fields (60 bytes); the explicit protocol
uses 287 data bits plus one reserved bit (36 bytes), a reproducibly calculated
24-byte reduction against a canonical full-width-field message. The reference
firmware deliberately retains the production packer, so its actual wire packet
is also 36 bytes. No earlier 60-byte network message was transmitted.

Logs contain measured struct sizes, ELF `.text`/`.data`/`.bss`, the static RAM
comparison and host-only timing samples. Static RAM is `.data + .bss`; this
excludes live stack/heap high-water measurements. The same tests run against
both representations. Timing differences are host observations, not embedded
performance acceptance. Re-run the comparison on the Pico before signing off.

The user requested local changes without commits. Evidence therefore identifies
the base commit plus a SHA-256 digest of the tested application/build/test source
files; it does not label that base commit as containing the new implementation.

### Software comparison measurements (2026-10-26)

| Measurement | Full-width reference | Packed | Difference |
| --- | --- | --- | --- |
| Config struct (host) | 60 B | 24 B | −36 B |
| Status struct (host) | 60 B | 44 B | −16 B |
| Two PID states (host) | 16 B | 8 B | −8 B |
| Encoder snapshot (host) | 16 B | 12 B | −4 B |
| Default ELF `.data` | 88 B | 88 B | 0 B |
| Default ELF `.bss` | 24896 B | 24796 B | −100 B |
| Static RAM `.data + .bss` | 24984 B | 24884 B | −100 B |
| Default ELF `.text` | 287620 B | 288652 B | +1032 B |
| Canonical full-field message vs explicit wire format | 60 B | 36 B | −24 B (calculated reference) |

Packed fields save static RAM at a modest code-size cost. Both software
representations pass 11244 assertions each, including concurrency and console
stop queue invalidation; AddressSanitizer/UndefinedBehaviorSanitizer also pass.
Portable Clang analysis reports no diagnostics. See the saved logs for host-only
per-operation timing samples and fresh-build profiles. These checks do not
certify physical accuracy, embedded deadlines or every BARR-C rule.

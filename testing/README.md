# Testing

Test records for the project, used as evidence in the write-up and tribunals.

| Folder | Contents |
|---|---|
| `subsystemN/` | `TEST_LOG.md` (cases and results), `logs/` (saved console output, photos), `host/` (host test programs), kept by the subsystem's owner |
| `integration/` | Tests across two or more subsystems, usually one per interface in `TEAM.md` section 5 |

Test IDs are `S1-U01`, `S2-U01`, … for a subsystem and `INT-01`, … for integration. Number cases in order and give a new case the next free number.

## Kinds of test

- **auto:** a `TEST_CHECK` with the test's ID in `app_program/subsystemN/subsystemN_test.c` (see `app_program/test_report.h`). `make -C build_make TEST=1` builds the test image, which runs every check at power-up and prints `[TEST] <ID> PASS|FAIL`.
- **host:** logic with no hardware calls (packing, decoding, control maths) as a C program in `subsystemN/host/`, built with the laptop's `cc`. Put the build-and-run command in the case's *How* column.
- **manual:** a person watches the car or the console, usually through a subsystem's `TEST=1` bench sequence.

## Recording a result

Add a row to the *Results* table: date, test ID, commit (with the pull request number if it was on a branch, as `1a2b3c4 (#42)`), who ran it, PASS / FAIL / PARTIAL, the log file if any, and a note. To save console output, run `cat /dev/ttyACM0 | tee testing/subsystemN/logs/YYYY-MM-DD_<what>.txt` (macOS: `/dev/cu.usbmodem*`) and then reset the Pico.

Add a new row for a re-run rather than overwriting an earlier result. Fixing a typo or a wrong reference in an old row is fine.

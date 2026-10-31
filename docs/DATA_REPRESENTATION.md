# Data Representation

Application data uses only the bits its valid range needs (change requirement "Minimum-Bit Data Representation"). This page states the rules and lists every application data field. Measurements that compare before and after (RAM, message sizes, timing) are test evidence and live in each subsystem's `testing/` log.

## Rules

- **Size each field to its range.** A value from 1 to 5 needs 3 bits, a state machine with up to 8 states needs 3 bits, a signed value from −2048 to 2047 needs 12 bits.
- **Stored and shared structs use C bit-fields.** Standard C only guarantees bit-fields of `signed int`, `unsigned int` and `_Bool`, so use those, and always write `signed` or `unsigned`: whether a plain `int` bit-field is signed is up to the compiler.
- **Anything sent over the network uses explicit masks and shifts**, never bit-fields. The C standard leaves bit-field layout to the compiler, so the receiver cannot rely on it. MQTT payloads are compact binary, not text.
- **Faults are single bits in one shared fault bitmask**, not separate variables.
- **Packed data shared between tasks, or between an ISR and a task, is accessed safely.** Writing one bit-field rewrites the whole word it shares with its neighbours, so guard packed shared data with a mutex, pass copies through a message buffer, or use an event flag (its set and clear calls are atomic).
- **Out of scope:** device bus buffers in the sensor's own format, temporary variables inside a calculation, and kernel types such as `ID` and `ER`.

## Fields

| Subsystem | Field | Valid range | Bits | Representation | Shared with |
| --- | --- | --- | --- | --- | --- |
| 4 | `S4_ACCEL.x`, `.y`, `.z`: latest acceleration per axis | −2048 to 2047 mg (sensor's 12-bit output at ±2 g) | 12 each | `signed int` bit-field | Readers copy it with `s4_get_accel()`, under mutex `s4_mtx_accel` |
| 4 | `S4_ACCEL.count`: readings taken | 0 to 255, wraps | 8 | `unsigned int` bit-field | As above |

## Buddy 5

All fields below use explicit `unsigned int` bit-fields. Enum constants and
function parameters are unpacked; stored enums have the widths below.
Timestamps are wrapping 32-bit monotonic milliseconds. Normal word alignment
is retained. These are local ABI records; network encoding requires masks
and shifts, never C bit-fields.

| Subsystem | Field | Valid range | Bits | Representation | Shared with |
| --- | --- | --- | --- | --- | --- |
| 5 | `S5_SAMPLE.distance_mm` | zero invalid; 20–4000 mm valid | 12 | unsigned bit-field | Scanner → avoidance / I8 readers |
| 5 | `S5_SAMPLE.angle_deg` | 0–180 degrees; travel 30–150 | 8 | unsigned bit-field | As above |
| 5 | `S5_SAMPLE.valid` | false/true | 1 | unsigned bit-field | As above |
| 5 | `S5_SAMPLE.stamp_ms` | 0–4294967295, wraps | 32 | unsigned bit-field | As above |
| 5 | `S5_PROFILE.closest_mm`, `left_mm`, `right_mm`, `width_mm` | 0–4000 mm; zero unknown with validity false | 12 each | unsigned bit-field | Avoidance / I8/I10 readers |
| 5 | `S5_PROFILE.closest_angle`, `left_edge`, `right_edge` | 0–180 degrees | 8 each | unsigned bit-field | As above |
| 5 | `S5_PROFILE.detected`, `valid`, `width_valid`, `left_valid`, `right_valid` | false/true | 1 each | unsigned bit-field | As above |
| 5 | `S5_PROFILE.valid_count` | 0–12 unique rays | 4 | unsigned bit-field | As above |
| 5 | `S5_PROFILE.faults` | mask 0–127 | 7 | unsigned bit-field | I8/I10 readers |
| 5 | `S5_PROFILE.stamp_ms` | 0–4294967295, wraps; oldest contributor | 32 | unsigned bit-field | I8/I10 readers |
| 5 | `S5_SCAN.count` | 0–12 | 4 | unsigned bit-field | Scanner → avoidance copy |
| 5 | `S5_SCAN.mode`, private `S5_REQUEST.mode` | FORWARD/COARSE/FINE, 0–2 | 2 each | unsigned bit-field | Internal queues |
| 5 | `S5_SCAN.faults` | mask 0–127 | 7 | unsigned bit-field | Scanner → avoidance copy |
| 5 | private `S5_MESSAGE.kind` | 15 events, 0–14 | 4 | unsigned bit-field | Input/result queue |
| 5 | `S5_CONTROL.state` | seven states, 0–6 | 3 | unsigned bit-field | Avoidance → readers |
| 5 | `S5_CONTROL.action` | seven actions, 0–6 | 3 | unsigned bit-field | I6 readers |
| 5 | `S5_CONTROL.scan_retries`, `recovery_retries` | 0–2 | 2 each | unsigned bit-field | I6/I10 readers |
| 5 | `S5_CONTROL.faults` | mask 0–127 | 7 | unsigned bit-field | I10 readers |
| 5 | `S5_CONTROL.active`, `motion_stopped`, `pending`, `scan_due` | false/true | 1 each | unsigned bit-field | Internal / readers |
| 5 | `S5_CONTROL.mode` | 0–2 | 2 | unsigned bit-field | Internal / readers |
| 5 | `S5_CONTROL.entered_ms` | 0–4294967295, wraps | 32 | unsigned bit-field | Internal / readers |
| 5 | `S5_HEALTH.servo_angle` | 0–180 degrees | 8 | unsigned bit-field | Scanner → readers under mutex |
| 5 | `S5_HEALTH.scan_state` | idle/settling/measuring, 0–2 | 2 | unsigned bit-field | Scanner → readers under mutex |
| 5 | `S5_HEALTH.front_clear` | false/true | 1 | unsigned bit-field | Computed on caller's private copy |

Sample: 53 payload bits, 8 bytes. Profile: 120 payload bits, 16 bytes.
Scan: twelve samples + 13 metadata bits, 100 bytes. Control: 23 control bits
+ 32 timer bits + sample + profile, 32 bytes. Health: control + 11 bits,
36 bytes. Internal message: four event bits + scan, 104 bytes. Request: two
bits, 4 bytes. Sizes include GCC ABI padding, checked by tests.

Avoidance owns control; scanner owns local scans/cache. Queues pass copies.
Every shared snapshot read/write uses one inheritance mutex; each writer
modifies only its owned members. Readers get private copies. Fault fields
carry one seven-bit mask format (NO_ECHO, ECHO_TIMEOUT, SCAN_TIMEOUT,
STALE_RANGE, NO_SAFE_PATH, RECOVERY_FAULT, SERVO_ERROR), copied/accumulated
at the scan/profile/health boundaries; no per-fault booleans. Faults latch
until reset except the transient stale-forward flag.

Packing evidence and the labelled unpacked reference comparison are in
`testing/subsystem5/TEST_LOG.md`; no historical firmware saving is claimed.

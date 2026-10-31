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
| 4 | `S4_TILT.pitch`, `.roll`: tilt from the start surface | −900 to 900 (0.1 degree) | 11 each | `signed int` bit-field | Readers copy it with `s4_get_tilt()`, under mutex `s4_mtx_accel` |
| 4 | `S4_TILT.cal`: start-up calibration done | 0 or 1 | 1 | `unsigned int` bit-field | As above |
| 4 | `S4_FAULT.impact`, `.tilt`: impact and excessive-tilt faults, latched until reset | 0 or 1 | 1 each | `unsigned int` bit-field (moves into the shared fault bitmask once agreed) | Readers copy it with `s4_get_fault()`, under mutex `s4_mtx_accel` |
| 5 | `S5_SAMPLE.distance_mm`: one range reading | 20 to 4000 mm, 0 when invalid | 12 | `unsigned int` bit-field | Scan task passes copies to the avoidance task in message buffer `inbox`; readers copy health with `s5_get_health()`, under mutex `snapshot_mutex` |
| 5 | `S5_SAMPLE.angle_deg`: servo angle of the reading | 30 to 150 degrees | 8 | `unsigned int` bit-field | As above |
| 5 | `S5_SAMPLE.valid`: reading in range | 0 or 1 | 1 | `unsigned int` bit-field | As above |
| 5 | `S5_SAMPLE.stamp_ms`: time of the reading | 0 to 2³²−1 ms, wraps | 32 | `unsigned int` bit-field | As above |
| 5 | `S5_PROFILE.closest_mm`, `.left_mm`, `.right_mm`: closest point and side clearances | 0 to 4000 mm, 0 when unknown | 12 each | `unsigned int` bit-field | Readers copy it with `s5_get_health()`, under mutex `snapshot_mutex` |
| 5 | `S5_PROFILE.closest_angle`, `.left_edge`, `.right_edge`: angles of the closest point and obstacle edges | 30 to 150 degrees | 8 each | `unsigned int` bit-field | As above |
| 5 | `S5_PROFILE.detected`, `.valid`, `.left_valid`, `.right_valid` | 0 or 1 | 1 each | `unsigned int` bit-field | As above |
| 5 | `S5_PROFILE.valid_count`: valid rays in the profile | 0 to 12 | 4 | `unsigned int` bit-field | As above |
| 5 | `S5_PROFILE.stamp_ms`: time of the oldest reading used | 0 to 2³²−1 ms, wraps | 32 | `unsigned int` bit-field | As above |
| 5 | `S5_SCAN.count`: readings in the scan | 0 to 12 | 4 | `unsigned int` bit-field | Scan task passes copies to the avoidance task in message buffer `inbox` |
| 5 | `S5_SCAN.mode`, `S5_REQUEST.mode`: forward, coarse or fine scan | 0 to 2 | 2 each | `unsigned int` bit-field | `S5_REQUEST` is copied from the avoidance task to the scan task in message buffer `requests` |
| 5 | `S5_MESSAGE.kind`: event in the avoidance task's inbox | 0 to 14 | 4 | `unsigned int` bit-field | Copied in message buffer `inbox` |
| 5 | `S5_SCAN.faults`, `S5_PROFILE.faults`, `S5_CONTROL.faults`: fault bitmask (no echo, echo timeout, scan timeout, stale range, no safe path, recovery fault, servo error) | 7 single-bit flags | 7 each | `unsigned int` bit-field (moves into the shared fault bitmask once agreed) | As each record |
| 5 | `S5_CONTROL.state`: avoidance state | 0 to 6 | 3 | `unsigned int` bit-field | Written only by the avoidance task; readers copy it with `s5_get_health()`, under mutex `snapshot_mutex` |
| 5 | `S5_CONTROL.action`: recommended action | 0 to 6 | 3 | `unsigned int` bit-field | As above |
| 5 | `S5_CONTROL.scan_retries`, `.recovery_retries` | 0 to 2 | 2 each | `unsigned int` bit-field | As above |
| 5 | `S5_CONTROL.active`, `.motion_stopped`, `.pending`, `.scan_due` | 0 or 1 | 1 each | `unsigned int` bit-field | As above |
| 5 | `S5_CONTROL.mode`: scan mode to request | 0 to 2 | 2 | `unsigned int` bit-field | As above |
| 5 | `S5_CONTROL.entered_ms`: time the state was entered | 0 to 2³²−1 ms, wraps | 32 | `unsigned int` bit-field | As above |
| 5 | `S5_HEALTH.servo_angle`: current servo angle | 30 to 150 degrees | 8 | `unsigned int` bit-field | Written only by the scan task, under mutex `snapshot_mutex`; readers copy it with `s5_get_health()` |
| 5 | `S5_HEALTH.scan_state`: idle, settling or measuring | 0 to 2 | 2 | `unsigned int` bit-field | As above |
| 5 | `S5_HEALTH.front_clear`: fresh forward range beyond 350 mm | 0 or 1 | 1 | `unsigned int` bit-field | Computed in the caller's copy by `s5_get_health()` |
| 5 | `previous_us` in `drv_hcsr04.c`: TIMER value at the last trigger, to keep 60 ms between readings | 0 to 2³²−1 µs, wraps | 32 | `uint32_t` | Scan task only |

# Data Representation

Application data uses only the bits its valid range needs (change requirement "Minimum-Bit Data Representation"). This page states the rules and lists every application data field. Measurements that compare before and after (RAM, message sizes, timing) are test evidence and live in each subsystem's `testing/` log.

## Rules

- **Size each field to its range.** A value from 1 to 5 needs 3 bits, a state machine with up to 8 states needs 3 bits, a signed value from −2048 to 2047 needs 12 bits.
- **Stored and shared structs use C bit-fields.** Standard C only guarantees bit-fields of `signed int`, `unsigned int` and `_Bool`, so use unsigned bit-fields for new Buddy 2 data; signed values use explicit
  offsets. Existing Buddy 4 signed fields retain their original encoding.
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

## Buddy 2 application fields

Signed values are stored unsigned with a bias, because BARR-C 5.3.a forbids signed bit-fields. The motion state (`g_status`, `g_config`, `g_run`, `g_pid`) is guarded by one priority-inheriting mutex, so a read-modify-write of any packed field cannot race. Telemetry is packed with explicit shifts (see [MOTION_CONTROL.md](MOTION_CONTROL.md#telemetry-payload)).

| Field | Valid range | Bits | Representation | Shared with |
| --- | --- | --- | --- | --- |
| `status.state` | 0–3 (idle, running, complete, failed) | 2 | `unsigned int` bit-field | Any task, copied by `motion_control_get_status()` under the mutex; MQTT |
| `status.command` | 0–5 | 3 | as above | as above |
| `status.faults` | stall left, stall right, encoder | 3 | one bit per fault | as above |
| `status.heading_deg` | 0–359° | 9 | as above | as above |
| `status.left_speed`, `.right_speed` | −1023 to 1023 mm/s | 11 each | value + 1024 | as above |
| `status.left_count`, `.right_count` | 0–65535 edges, wrapping | 16 each | as above | as above |
| `status.distance_mm` | 0–16 777 215 mm, wrapping | 24 | as above | as above |
| `config.speed_mm_s` | 10–1000 mm/s | 10 | bit-field | API callers and motion task, mutex |
| `config.track_um` | 50 000–200 000 µm | 18 | bit-field | as above |
| `config.kp`, `.ki`, `.kd` | 0–4095 (Q8) | 12 each | bit-field | as above |
| `run.goal` | 0–62 119 edges (5 m) | 16 | bit-field | as above |
| `run.left_progress`, `.right_progress` | 0–131 071 edges | 17 each | clamped at 0 | motion task |
| `run.profile_speed` | 0–1000 mm/s | 10 | bit-field | motion task |
| `run.left_target`, `.right_target` | −1000 to 1000 mm/s | 11 each | value + 1024 | `motion_control_drive()` and motion task, mutex |
| `run.left_raw`, `.right_raw` | 0–65535 edges | 16 each | last driver count | motion task |
| `run.skipped` | 0–255, wrapping | 8 | last driver value | motion task |
| `run.left_stall`, `.right_stall` | 0–100 periods (1 s) | 7 each | bit-field | motion task |
| `run.drive_age` | 0–20 periods (200 ms target expiry) | 5 | bit-field | `motion_control_drive()` and motion task, mutex |
| `run.heading` | 0 to 2 × track × 2537 (< 2^30) | 30 | edge difference × wheel diameter, modulo one turn | motion task |
| `run.odometer` | 0 to 2^30 − 1 edges, wrapping | 30 | bit-field | motion task |
| `pid[n].integral` | −32767 to 32767 | 16 each | value + 32768 | motion task |
| `pid[n].last_error` | −2047 to 2047 mm/s | 12 each | value + 2048 | motion task |
| Driver `left_count`, `right_count` | 0–65535 edges, wrapping | 16 each | `volatile uint16_t` | Encoder ISR writes; motion task reads. A 16-bit load is atomic on Cortex-M0+, so no lock is needed |
| Driver `skipped` | 0–255, wrapping | 8 | `volatile uint8_t` | as above |
| Driver wheel `phase`, `b_backward` | 0–3, 0/1 | 2 + 1 per wheel | bit-field | Encoder ISR only |
| Console receive ring head, tail | 0–63 | 8 each | `volatile uint8_t`, for an atomic access | USB task writes head, console task writes tail (both core 0) |
| Console `drive` speeds | −1000 to 1000 mm/s | 16 each | `int16_t`, the smallest signed standard type | Console task only |

`run.last_us` stays a full 32-bit timer reading, because elapsed time is a wrap-around difference of the hardware counter. Measured savings are in `testing/subsystem2/logs/2026-10-31_ram.txt`.

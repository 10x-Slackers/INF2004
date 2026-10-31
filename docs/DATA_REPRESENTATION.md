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
| 4 | `S4_HUMP.peak`, `.max`: last hump peak and highest of the run | 0 to 63 mm | 6 each | `unsigned int` bit-field | Readers copy it with `s4_get_hump()`, under mutex `s4_mtx_accel` |
| 4 | `S4_HUMP.count`: humps seen | 0 to 15, wraps | 4 | `unsigned int` bit-field | As above |
| 4 | `S4_HUMP.on`: right wheel on a hump now | 0 or 1 | 1 | `unsigned int` bit-field | As above |
| 4 | `S4_MOTION.cls`: current motion class (steady, accelerating, braking, climbing, descending, impact) | 0 to 5 | 3 | `unsigned int` bit-field | Readers copy it with `s4_get_motion()`, under mutex `s4_mtx_accel` |
| 2 | `motion_control_status_t.state`: idle, running, complete, failed | 0 to 3 | 2 | `unsigned int` bit-field | Readers copy it with `motion_control_get_status()`, under the motion mutex; packed with shifts for telemetry |
| 2 | `.command`: none, forward, backward, left, right, drive | 0 to 5 | 3 | As above | As above |
| 2 | `.faults`: stall left, stall right, missed encoder phase | One bit each | 3 | As above (moves into the shared fault bitmask once agreed) | As above |
| 2 | `.heading_deg`: anticlockwise from power-up | 0 to 359° | 9 | As above | As above |
| 2 | `.left_speed`, `.right_speed` | −1023 to 1023 mm/s | 11 each | `signed int` bit-field | As above |
| 2 | `.left_count`, `.right_count`: encoder edges | 0 to 65535, wraps | 16 each | `unsigned int` bit-field | As above |
| 2 | `.distance_mm`: distance rolled | 0 to 16 777 215 mm, wraps | 24 | `unsigned int` bit-field | As above |
| 2 | `g_run.speed_mm_s`: cruise speed of moves and turns | 10 to 1000 mm/s | 10 | `unsigned int` bit-field | `motion_control_set_speed()` and the motion task, under the motion mutex |
| 2 | `g_run.goal`: edges each wheel must roll | 0 to 62 119 (5 m) | 16 | `unsigned int` bit-field | Command calls and the motion task, under the motion mutex |
| 2 | `g_run.left_target`, `.right_target`: drive speeds | −1000 to 1000 mm/s | 11 each | `signed int` bit-field | As above |
| 2 | `g_run.drive_age`: periods since the last `drive()` | 0 to 20 (200 ms expiry) | 5 | `unsigned int` bit-field | As above |
| 2 | `g_run.left_progress`, `.right_progress`: edges rolled | 0 to 131 071, clamped | 17 each | `unsigned int` bit-field | Motion task only |
| 2 | `g_run.profile_speed`: current ramp speed | 0 to 1000 mm/s | 10 | `unsigned int` bit-field | Motion task only |
| 2 | `g_run.left_raw`, `.right_raw`, `.skipped`: driver values at the last step | 0 to 65535; 0 to 255 | 16 each; 8 | `unsigned int` bit-field | Motion task only |
| 2 | `g_run.left_stall`, `.right_stall`: periods driven without an edge | 0 to 100 (1 s) | 7 each | `unsigned int` bit-field | Motion task only |
| 2 | `g_run.heading`: edge difference × wheel diameter, modulo one turn | 0 to 2 × track × 2537 (< 2^30) | 30 | `unsigned int` bit-field | Motion task only |
| 2 | `g_run.odometer`: edges rolled by both wheels | 0 to 2^30 − 1, wraps | 30 | `unsigned int` bit-field | Motion task only |
| 2 | `g_pid[n].integral`, `.last_error`: PID state per wheel | −32767 to 32767; −2047 to 2047 mm/s | 16; 12 | `signed int` bit-field | Motion task only |
| 2 | `g_left_count`, `g_right_count`, `g_skipped` (driver): encoder edges, missed phases | 0 to 65535; 0 to 255, wrap | 16 each; 8 | `volatile uint16_t`, `uint8_t`: the ISR writes them, and these loads are atomic on Cortex-M0+ | Encoder ISR writes, motion task reads, no lock |
| 2 | `drv_motion_wheel_t.phase`, `.b_backward` (driver) | 0 to 3; 0 or 1 | 2; 1 | `unsigned int` bit-field | Encoder ISR only |
| 2 | `g_max_tick_us`: longest control step | 0 to 65535 µs, saturates | 16 | `volatile uint16_t`, for an atomic load | Motion task writes; tests read it with `subsystem2_max_tick_us()` |
| 2 | `g_callback_pins` (driver): GPIO pins of the registered edge callback | Bank 0 pin mask | 32 | `uint32_t` in the hardware register's format (out of scope) | Set once at init, read by the encoder ISR |

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

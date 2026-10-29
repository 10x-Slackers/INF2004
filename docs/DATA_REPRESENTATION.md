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

## Buddy 2 application fields

All storage below is unsigned. Fields grouped on a row share range/encoding.
C layout padding is measured, never assumed to be the sum of bit widths.
Transport offsets and invalid codes are in [MOTION_CONTROL.md](MOTION_CONTROL.md).

| Field(s) | Valid value / resolution | Bits each | Encoding / ownership |
| --- | --- | --- | --- |
| config circumference / track | 50–1000 mm, 1 mm; 0 unconfigured | 10 | direct, motion state lock |
| config left/right_counts_per_rev | 4–65535 counts/rev; 0 selects shared fallback | 16 each | direct; confirmation validates both resolved values |
| config counts_per_rev | 4–65535 four-edge counts, 1 count; 0 unconfigured | 16 | direct |
| config kp / ki / kd | 0–4095, 1/256 gain; Kp must be nonzero when confirmed | 12 | Q8; Ki/Kd in 10 ms units, scaled by actual elapsed time |
| config speed_mm_s | 10–1000 mm/s | 10 | direct; zero only while unconfigured |
| config duty_limit | 1–1000 permille | 10 | direct; zero only while unconfigured |
| config timeout_ms | 100–60000 ms | 16 | direct |
| config stall_ms | 100–10000 ms | 14 | direct |
| config motor/encoder left/right reverse, confirmed | 0/1 | 1 | direct |
| status state | 0–4 | 3 | 5–7 reserved |
| status command | 0–5 | 3 | 6–7 reserved |
| status armed | 0/1 | 1 | permission, not motor-on indication |
| status faults | 8 independent flags | 8 | combined mask; unknown sensor reading is not zero/clear |
| status left/right count | −8388608..8388607, 1 edge | 24 | signed value + 8388608 |
| status left/right/average mm | −8388608..8388607, 1 mm | 24 | signed value + 8388608 |
| status left/right mm_s | −2000..2000, 1 mm/s | 12 | signed value + 2048; others reserved |
| status timestamp_us, max_execution_us, max_period_us | 0..4294967295 µs | 32 | timer-wrap arithmetic; maxima diagnostic |
| status missed_periods | 0..4294967295 releases | 32 | saturating counter |
| PID integral, each wheel | −65535..65535 error-samples | 17 | value + 65535; all-ones reserved |
| PID previous_error, each wheel | −3250..3250 mm/s | 13 | value + 4096; outside range reserved |
| run start_left/right, last_left/right | normalized signed encoder range | 24 | value + 8388608 |
| run left/right_goal | 1..8388607 edges; 0 idle | 23 each | direct |
| run invalid | 0–65535 transitions | 16 | last observed ISR diagnostic |
| run test_wheel | 0/1 | 1 | left/right |
| run test_duty | −200..200 permille | 9 | value + 200 |
| run test_duration_ms | 0–1000 ms | 10 | zero idle |
| run start_us, last_us, left/right_edge_us | full 32-bit µs timer | 32 | wrap-safe differences |
| driver left/right counts | signed encoder range | 24 | value + 8388608, lock 4 |
| driver invalid | 0–65535 transitions | 16 | saturates, then raises overflow |
| driver left/right invalid | 0–65535 transitions each | 16 each | diagnostic saturation; aggregate retains existing fault policy |
| driver overflow | 0/1 | 1 | latched until reboot |
| driver left/right quadrature phase | 0–3 | 2 | A*2+B |
| driver previous left/right duty | −1000..1000 permille | 11 | value + 1000 |
| driver additional GPIO owner mask | GP0–29 | 30 | one bit/pin; fixed startup registration |
| driver started | 0/1 | 1 | unsigned bit-field |
| console RX bytes | 0–255 | 8 | byte stream, fixed 128-byte ring |
| console RX head/tail | 0–127 | 7 | state lock protects producer/consumer |

Temporary calculations use wider signed arithmetic deliberately. IDs, function
pointers, lock addresses and bus register formats are kernel/device data.
Host fixtures and the generated full-width reference are test data, not robot
application storage. See the subsystem test log for measured sizes and evidence.

The startup `started` flag is one logical bit and is written only during
single-threaded startup. The console's temporary token/line buffers are local
text data, not MQTT application payloads. Kernel object IDs, callbacks and
hardware register values retain their native representation.

Compare sizes using `make test`: the same portable control, math and console
algorithms are compiled for ARM with packed fields and with a temporary
full-width reference. No generated reference source becomes firmware. Actual
RAM/message measurements, commands and limitations belong in
[verification](../testing/subsystem2/VERIFICATION.md). Count/time snapshots use
encoder lock 4; task configuration/status use lock 3; no raw bit-field layout
crosses the network. Per-consumer status copies are the I4/I5 messages, with no
single-consumer queue. The explicit telemetry packet is 36 bytes.

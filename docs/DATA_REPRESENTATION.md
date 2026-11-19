# Data Representation

Application data uses only the bits its valid range needs (change requirement "Minimum-Bit Data Representation"). This page states the rules and lists every application data field. Measurements that compare before and after (RAM, message sizes, timing) are test evidence and live in each subsystem's `testing/` log; [Evidence](#evidence) lists them.

## Rules

- **Size each field to its range.** A value from 1 to 5 needs 3 bits, a state machine with up to 8 states needs 3 bits, a signed value from −2048 to 2047 needs 12 bits.
- **Stored and shared structs use C bit-fields.** Standard C only guarantees bit-fields of `signed int`, `unsigned int` and `_Bool`, so use those, and always write `signed` or `unsigned`: whether a plain `int` bit-field is signed is up to the compiler.
- **Anything sent over the network uses explicit masks and shifts**, never bit-fields. The C standard leaves bit-field layout to the compiler, so the receiver cannot rely on it. MQTT payloads are compact binary, not text.
- **Faults are single bits in one shared fault bitmask**, not separate variables.
- **Packed data shared between tasks, or between an ISR and a task, is accessed safely.** Writing one bit-field rewrites the whole word it shares with its neighbours, so guard packed shared data with a mutex, pass copies through a message buffer, or use an event flag (its set and clear calls are atomic). A value with a single writer may also be read without a lock when the read is one load, which is atomic on the Cortex-M0+.
- **Out of scope:** device bus buffers in the sensor's own format, temporary variables inside a calculation, and kernel types such as `ID` and `ER`.

## Fields

| Subsystem | Field | Valid range | Bits | Representation | Shared with |
| --- | --- | --- | --- | --- | --- |
| Shared | Fault bitmask: one bit per fault of S1, S2, S4 and S5 (`APP_FAULT_...` in `app_fault.h`) | One bit each | 11 | Kernel event flag pattern, read with `app_faults()` | Every task and ISR: `tk_set_flg` and `tk_clr_flg` are atomic across both cores, and each owner changes only its own bits |
| 4 | `s4_accel_t.x`, `.y`, `.z`: latest acceleration per axis | −2048 to 2047 mg (sensor's 12-bit output at ±2 g) | 12 each | `signed int` bit-field | Readers copy it with `s4_get_accel()`, under mutex `gh_s4_mutex` |
| 4 | `s4_accel_t.count`: readings taken | 0 to 255, wraps | 8 | `unsigned int` bit-field | As above |
| 4 | `s4_tilt_t.pitch`, `.roll`: tilt from the start surface | −900 to 900 (0.1 degree) | 11 each | `signed int` bit-field | Readers copy it with `s4_get_tilt()`, under mutex `gh_s4_mutex` |
| 4 | `s4_tilt_t.cal`: start-up calibration done | 0 or 1 | 1 | `unsigned int` bit-field | As above |
| 4 | `s4_hump_t.peak`, `.max`: last hump peak and highest of the run | 0 to 63 mm | 6 each | `unsigned int` bit-field | Readers copy it with `s4_get_hump()`, under mutex `gh_s4_mutex` |
| 4 | `s4_hump_t.count`: humps seen | 0 to 15, wraps | 4 | `unsigned int` bit-field | As above |
| 4 | `s4_hump_t.on`: right wheel on a hump now | 0 or 1 | 1 | `unsigned int` bit-field | As above |
| 4 | `s4_motion_t.cls`: current motion class (steady, accelerating, braking, climbing, descending) | 0 to 4 | 3 | `unsigned int` bit-field | Readers copy it with `s4_get_motion()`, under mutex `gh_s4_mutex` |
| 2 | `motion_control_status_t.state`: idle, running, complete, failed | 0 to 3 | 2 | `unsigned int` bit-field | Readers copy it with `motion_control_get_status()`, under the motion mutex; packed with shifts for telemetry |
| 2 | `.command`: none, forward, backward, left, right, drive | 0 to 5 | 3 | As above | As above |
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
| 2 | `drv_motion_wheel_t.phase`, `.b_backward` (driver) | 0 to 3; 0 or 1 | 2; 1 | `volatile unsigned int` bit-field | Encoder ISR only |
| 2 | `g_max_tick_us`: longest control step | 0 to 65535 µs, saturates | 16 | `volatile uint16_t`, for an atomic load | Motion task writes; tests read it with `subsystem2_max_tick_us()` |
| 2 | `gb_latched`: safety stop latched | 0 or 1 | 1 | `bool`, the smallest type | Safety Monitor and Controller write it with `motion_control_latch()`, any task reads it with `motion_control_latched()`, under the motion mutex |
| Shared | `g_gpio[].pins` (`app_main.c`): GPIO pins of each registered edge callback, 0 in a free slot | Bank 0 pin mask | 32 each | `volatile uint32_t` in the hardware register's format (out of scope) | Set once at init, read by the GPIO ISR |
| 5 | `ultrasonic_scan_t.forward_mm`: latest range straight ahead | 20 to 4000 mm (4000: nothing in range) | 12 | `unsigned int` bit-field | Readers copy it with `ultrasonic_scan_get()`, under the scan mutex |
| 5 | `.request`: request number the latest scan served | 0 to 15, wraps | 4 | As above | As above |
| 5 | `.forward_ms`: kernel time of `forward_mm` | 0 to 2³²−1 ms, wraps | 32 | `uint32_t`, the kernel time's width | As above |
| 5 | `scan_profile_t.closest_mm`: closest point of the obstacle ahead | 20 to 4000 mm | 12 | `unsigned int` bit-field | Inside `ultrasonic_scan_t`, as above |
| 5 | `.closest_deg`, `.right_deg`, `.left_deg`: servo angles of the closest point and the obstacle edges | 45 to 135° | 8 each | As above | As above |
| 5 | `.b_obstacle`: ray ahead within 150 mm | 0 or 1 | 1 | As above | As above |
| 5 | `.width_mm`: rough obstacle width, the arc between the edges at the closest distance | 0 to 235 mm (150 mm × 90°) | 8 | As above | As above |
| 5 | `.left_mm`, `.right_mm`: rays at 135° and 45°, the clearance each side | 20 to 4000 mm | 12 each | As above | As above |
| 5 | `obstacle_avoidance_status_t.state`: idle, monitor, profile, bypass, stopped | 0 to 4 | 3 | `unsigned int` bit-field | Readers copy it with `obstacle_avoidance_get_status()`, under the avoidance mutex |
| 5 | `.side`: none, left, right | 0 to 2 | 2 | As above | As above |
| 5 | `.step`: bypass step | 0 to 7 | 3 | As above | As above |
| 5 | `.bypasses`: bypasses finished | 0 to 15, wraps | 4 | As above | As above |
| 5 | `.left_mm`, `.right_mm`: clearance 45° left and right, copied from the profile | 0 to 4000 mm (0: not measured) | 12 each | As above | As above |
| 5 | `g_next.mode` (scan): requested scan, forward or profile | 0 or 1 | 1 | `unsigned int` bit-field; written under the scan mutex | `ultrasonic_scan_request()` and the scan task |
| 5 | `g_next.request` (scan): requests made | 0 to 15, wraps | 4 | As above | As above |
| 5 | `g_angle` (scan): servo angle last set | 45 to 135° | 8 | `uint8_t` | Scan task only |
| 5 | `g_rise_us`, `g_width_us`, `g_trigger_us` (driver): echo rise time, echo width, last trigger time | 0 to 2³²−1 µs, wraps | 32 each | `uint32_t`, in the TIMER register's format (out of scope) | Edge callback writes the first two, the scan task reads them after it wakes |
| 5 | `gb_rose`, `gb_fell` (driver): echo edges seen | 0 or 1 | 1 each | `volatile bool`: written by the edge callback, these loads are atomic | As above |
| 3 | `line_following_t.bits`: left and right sensor over black | One bit each | 2 | `unsigned int` bit-field | Readers copy it with `line_following_get()`, under the line mutex |
| 3 | `.last_side`: last single sensor to see black | `DRV_LINE_LEFT` or `DRV_LINE_RIGHT`, 0 before the first | 2 | As above | As above |
| 3 | `.b_junction`: both sensors black for 3 samples | 0 or 1 | 1 | As above | As above |
| 3 | `.b_following`: steering the car | 0 or 1 | 1 | As above | As above |
| 3 | `.b_searching`: pivoting to find a lost line | 0 or 1 | 1 | As above | As above |
| 3 | `.b_lost`: the lost-line search failed | 0 or 1 | 1 | As above | As above |
| 3 | `.count`: samples taken | 0 to 255, wraps | 8 | As above | As above |
| 3 | `.sample_ms`: kernel time of the sample | 0 to 2³²−1 ms, wraps | 32 | As above | As above |
| 3 | `g_search.from_mm` (line following): odometer when a sensor last saw black | 0 to 16 777 215 mm, wraps (motion's `distance_mm`) | 24 | `unsigned int` bit-field | Line Following task and `line_following_start()`, under the line mutex |
| 3 | `g_search.phase`: none, pivoting towards `last_side`, pivoting back | 0 to 2 | 2 | As above | As above |
| 3 | `g_search.start_deg`: heading when the search began | 0 to 359° (motion's `heading_deg`) | 9 | As above | As above |
| 3 | `g_speed_mm_s` (line following): forward speed while following | 1 to 500 mm/s | 16 | `uint16_t`, the smallest standard type that fits | `line_following_start()` and the Line Following task, under the line mutex |
| 3 | `barcode_decoder_raw_t.min`, `.max`: barcode sensor level over the last 10 ms | 0 to 4095 ADC counts | 12 each | `unsigned int` bit-field | Readers copy it with `barcode_decoder_get_raw()`, under the barcode mutex |
| 3 | `.samples`: samples taken | 0 to 65535, wraps | 16 | As above | As above |
| 3 | `g_ring` (driver): barcode samples | 0 to 4095 | 16 each | `volatile uint16_t`, the ADC FIFO's format written by DMA (out of scope) | DMA writes, the Barcode Decoder task reads |
| 3 | `barcode_decoder_t.command`: last validated command (none, left, right, straight, U-turn) | 0 to 4 | 3 | `unsigned int` bit-field | Readers copy it with `barcode_decoder_get()`, under the barcode mutex |
| 3 | `.count`, `.rejects`: barcodes validated and rejected | 0 to 15 each, wrap | 4 each | As above | As above |
| 3 | `.distance_mm`: odometer at the end of the barcode | 0 to 16 777 215 mm, wraps (motion's `distance_mm`) | 24 | As above | As above |
| 3 | `g_widths` (barcode decoder): bar and space widths of one barcode | 0 to 30 mm in µm (0 to 30000) | 16 each | `uint16_t`, the smallest standard type that fits | Barcode Decoder task only |
| Shared | `g_vc.state`: Vehicle Controller state (INIT, READY, FOLLOW, AVOID, TURN, SAFE_STOP) | 0 to 5 | 3 | `volatile unsigned int` bit-field | Controller task writes; telemetry reads it with `vehicle_controller_state()`, one load, no lock |
| Shared | `g_vc.cycles`: 10 ms cycles since the state or junction move began | 0 to 1023, wraps (only TURN reads it, below 502) | 10 | As above | Controller task only |
| Shared | `g_vc.released`: cycles GP20 has been released | 0 to 5, stops at 5 | 3 | As above | Controller task only |
| Shared | `g_vc.pending`: barcode command for the next junction | 0 to 4 | 3 | As above | Controller task only |
| Shared | `g_vc.b_turning`: TURN past the roll forward | 0 or 1 | 1 | `bool` bit-field | Controller task only |
| 1 | MQTT payloads `telemetry`, `motion`, `scan`, `heartbeat`, `cmd`: every field in [TELEMETRY.md](TELEMETRY.md) | Per field | 83, 92, 68, 16, 8 | Explicit shifts, most significant bit first | Wi-Fi service and the supervisor |
| 1 | Message buffer entry (topic number + payload) and command event flag: [TELEMETRY.md](TELEMETRY.md#inside-the-car) | Topic 0 to 3; commands one bit each | 8 + payload; 3 | `uint8_t` topic, the smallest unit of a message; event flag bits, set and clear atomic | Telemetry task → Wi-Fi service, both `TMO_POL`; Wi-Fi service → Vehicle Controller |
| 1 | `payload_telemetry_t`: the telemetry fields gathered for packing | Per field, as sent | 83 | `unsigned int` and `signed int` bit-fields | Telemetry task only |
| 1 | Telemetry task: heartbeat count; cycles since the last heartbeat | 0 to 65535, wraps; 0 to 4 | 16; 3 | `uint16_t`, the payload's own width; `uint8_t`, the smallest standard type | Telemetry task only |

## Evidence

Change Requirement 1 asks for this table, RAM and message sizes before and after, and proof that nothing was lost. "Before" is the same source with every bit-field widened to a 32-bit integer, or the code's earlier unpacked version where one exists; each log states which.

| Evidence | Where |
| --- | --- |
| RAM, whole image and Vehicle Controller | INT-11, `testing/integration/logs/2026-10-31_ram.txt` |
| RAM and record sizes, per subsystem | S2-U22, S3-U15, S4-U22, S5-U44 in each `testing/subsystemN/TEST_LOG.md` |
| Inter-task record sizes (each `get()` copies one) | The same logs; S3-U02, U09, U13, S4-U04, U13, U18 and S5-U42 check them on the car |
| MQTT payload sizes against the Week 6 JSON | S1-U01, `testing/subsystem1/logs/2026-10-31_payloads.txt` |
| No loss of function | Every subsystem's unit tests run on the packed code; the widened versions pass the same host tests (S2-U21, S5-U43) except their size checks; S1-U01 round-trips every payload field at its limits, and S1-U02 checks the C packers against the same layout |
| No loss of timing | S2-U14 (`max_tick_us` of the 10 ms control step) |

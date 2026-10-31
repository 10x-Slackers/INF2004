# Telemetry and Commands (Subsystem 1)

The MQTT topics and binary payloads between the car and the supervisor (FR9, interface I12). They follow [DATA_REPRESENTATION.md](DATA_REPRESENTATION.md): every field uses only the bits its range needs, packed with explicit shifts, never C bit-fields. They replace the Week 6 review's JSON messages, and drop its session, challenge and acknowledgement fields for simplicity.

## Topics

All topics start with `csc13/`. Every message is QoS 0, not retained.

| Topic       | Direction        | Sent                                                   | Payload                                          |
| ----------- | ---------------- | ------------------------------------------------------ | ------------------------------------------------ |
| `telemetry` | car → supervisor | Every 200 ms                                           | 10 bytes, below                                  |
| `motion`    | car → supervisor | Every 100 ms                                           | 12 bytes, `motion_control_pack()` ([MOTION_CONTROL.md](MOTION_CONTROL.md#telemetry-payload)) |
| `scan`      | car → supervisor | Once per obstacle profile, when avoidance leaves `PROFILE` | 8 bytes, below                               |
| `heartbeat` | car → supervisor | Every 1 s                                              | 2 bytes: a count, wraps                          |
| `cmd`       | supervisor → car | On demand                                              | 1 byte, below                                    |

Events (barcode read, hump measured, bypass done, state change) need no topic of their own: each shows as a changed count or state in the next `telemetry`.

## Encoding

Each payload is one bit stream, most significant bit first, padded with zero bits to whole bytes, as in `motion_control_pack()`. A signed field is sent as value + offset so it is never negative. `testing/subsystem1/host/payloads.py` packs and unpacks every payload and is the reference decoder.

### `telemetry`: 80 bits

| Bits  | Field          | Source                                                    | Range                                                          |
| ----- | -------------- | --------------------------------------------------------- | -------------------------------------------------------------- |
| 0–2   | `state`        | Vehicle Controller `g_vc.state`                           | 0 INIT, 1 READY, 2 FOLLOW, 3 AVOID, 4 TURN, 5 SAFE_STOP        |
| 3     | `latched`      | `motion_control_latched()`                                | 0 or 1                                                         |
| 4–11  | `faults`       | Fault byte, below                                         | One bit each                                                   |
| 12–13 | `line`         | `line_following_t.bits`                                   | Left and right sensor over black                               |
| 14    | `junction`     | `line_following_t.b_junction`                             | 0 or 1                                                         |
| 15    | `following`    | `line_following_t.b_following`                            | 0 or 1                                                         |
| 16–18 | `barcode`      | `barcode_decoder_t.command`                               | 0 none, 1 left, 2 right, 3 straight, 4 U-turn                  |
| 19–22 | `barcodes`     | `barcode_decoder_t.count`                                 | 0 to 15, wraps                                                 |
| 23–25 | `motion_class` | `s4_motion_t.cls`                                         | 0 to 5                                                         |
| 26–36 | `pitch`        | `s4_tilt_t.pitch` + 1024                                  | −900 to 900 (0.1°)                                             |
| 37–47 | `roll`         | `s4_tilt_t.roll` + 1024                                   | −900 to 900 (0.1°)                                             |
| 48–53 | `hump_max`     | `s4_hump_t.max`                                           | 0 to 63 mm                                                     |
| 54–57 | `humps`        | `s4_hump_t.count`                                         | 0 to 15, wraps                                                 |
| 58    | `on_hump`      | `s4_hump_t.on`                                            | 0 or 1                                                         |
| 59–61 | `avoid_state`  | `obstacle_avoidance_status_t.state`                       | 0 to 4                                                         |
| 62–63 | `avoid_side`   | `obstacle_avoidance_status_t.side`                        | 0 none, 1 left, 2 right                                        |
| 64–67 | `bypasses`     | `obstacle_avoidance_status_t.bypasses`                    | 0 to 15, wraps                                                 |
| 68–79 | `forward_mm`   | `ultrasonic_scan_t.forward_mm`                            | 20 to 4000 mm                                                  |

### Fault byte

The one shared fault bitmask on the network. Bits 0–2 are motion's `faults` as they are, and bits 5–6 avoidance's sensor and blocked bits shifted up by 5.

| Bit | Fault                                           | Source                                      |
| --- | ----------------------------------------------- | ------------------------------------------- |
| 0   | Left wheel stall                                | `MOTION_CONTROL_FAULT_STALL_LEFT`           |
| 1   | Right wheel stall                               | `MOTION_CONTROL_FAULT_STALL_RIGHT`          |
| 2   | Encoder missed a phase                          | `MOTION_CONTROL_FAULT_ENCODER`              |
| 3   | Impact                                          | `s4_fault_t.impact`                         |
| 4   | Excessive tilt                                  | `s4_fault_t.tilt`                           |
| 5   | No echo                                         | `OBSTACLE_AVOIDANCE_FAULT_SENSOR`           |
| 6   | No safe path                                    | `OBSTACLE_AVOIDANCE_FAULT_BLOCKED`          |
| 7   | Wi-Fi lost since the last `telemetry`           | `cyw43_utk_get_status()`                    |

Avoidance's motion fault is left out: a failed bypass move is a stall, which bits 0–1 already carry.

### `scan`: 64 bits

| Bits  | Field         | Source (`ultrasonic_scan_t.profile`, `obstacle_avoidance_status_t`) | Range           |
| ----- | ------------- | ------------------------------------------------------------------- | --------------- |
| 0–11  | `closest_mm`  | `closest_mm`                                                        | 20 to 4000 mm   |
| 12–18 | `closest_deg` | `closest_deg`                                                       | 80 to 100°      |
| 19–25 | `right_deg`   | `right_deg`                                                         | 80 to 100°      |
| 26–32 | `left_deg`    | `left_deg`                                                          | 80 to 100°      |
| 33–39 | `width_mm`    | `width_mm`                                                          | 0 to 122 mm     |
| 40–51 | `left_mm`     | `left_mm`                                                           | 0 to 4000 mm    |
| 52–63 | `right_mm`    | `right_mm`                                                          | 0 to 4000 mm    |

### `cmd`: 8 bits

Bits 0–1 hold the command: 1 START, 2 STOP, 3 RESET. Any other payload is ignored. Bits 2–7 are zero. The commands do what GP20 does in the same state ([VEHICLE_CONTROLLER.md](VEHICLE_CONTROLLER.md#gp20)): START from READY, STOP from FOLLOW, AVOID or TURN, RESET from SAFE_STOP. STOP is always accepted. No command steers or sets a speed.

## Inside the car

Only the kernel's Wi-Fi service task may call MQTT (`TEAM.md` section 3), so the payloads cross two kernel objects:

| Path                                   | Object         | Message                                                              |
| -------------------------------------- | -------------- | -------------------------------------------------------------------- |
| Telemetry task → Wi-Fi service         | Message buffer | 1 byte topic number (0 `telemetry`, 1 `motion`, 2 `scan`, 3 `heartbeat`) + the payload: at most 13 bytes |
| Wi-Fi service → Vehicle Controller     | Event flag     | One bit per command: 1 START, 2 STOP, 4 RESET                        |

The Telemetry task sends with `TMO_POL` and drops a message when the buffer is full, so a slow network never blocks it. The service reads the buffer with `TMO_POL` too. Event flag set and clear calls are atomic, so the command bits need no lock.

## Size

Payload bytes per message and per second at the rates above, measured by S1-U01 (`testing/subsystem1/`):

| Topic       | Bits | Binary (bytes) | Week 6 JSON (bytes) |
| ----------- | ---- | -------------- | ------------------- |
| `telemetry` | 80   | 10             | 275                 |
| `motion`    | 96   | 12             | 182                 |
| `scan`      | 64   | 8              | 141                 |
| `heartbeat` | 16   | 2              | 50                  |
| `cmd`       | 8    | 1              | 101                 |

At the rates above the periodic topics need 172 bytes/s of payload instead of 3245.

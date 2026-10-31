# Telemetry and Commands (Subsystem 1)

The MQTT topics and binary payloads between the car and the supervisor (FR9, interface I12). They follow [DATA_REPRESENTATION.md](DATA_REPRESENTATION.md): every field uses only the bits its range needs, packed with explicit shifts, never C bit-fields. They replace the Week 6 review's JSON messages, and drop its version, robot, sequence, session, challenge and acknowledgement fields for simplicity. Without a sequence number the supervisor cannot spot a lost `telemetry` message; MQTT over TCP keeps them in order, and the `heartbeat` count shows gaps in the link.

## Topics

All topics start with `csc13/`. Every message is QoS 0, not retained.

| Topic       | Direction        | Sent                                                   | Payload                                          |
| ----------- | ---------------- | ------------------------------------------------------ | ------------------------------------------------ |
| `telemetry` | car → supervisor | Every 200 ms                                           | 10 bytes, below                                  |
| `motion`    | car → supervisor | With every `telemetry`                                 | 12 bytes, `motion_control_pack()` ([MOTION_CONTROL.md](MOTION_CONTROL.md#telemetry-payload)) |
| `scan`      | car → supervisor | Once per obstacle profile, when avoidance leaves `PROFILE` | 8 bytes, below                               |
| `heartbeat` | car → supervisor | Every 1 s                                              | 2 bytes: a count, wraps                          |
| `cmd`       | supervisor → car | On demand                                              | 1 byte, below                                    |

Events (barcode read, hump measured, bypass done, state change) need no topic of their own: each shows as a changed count or state in the next `telemetry`.

## Encoding

Each payload is one bit stream, most significant bit first, padded with zero bits to whole bytes, as in `motion_control_pack()`. A signed field is sent as value + offset so it is never negative. `testing/subsystem1/host/payloads.py` packs and unpacks every payload and is the reference decoder.

### `telemetry`: 80 bits

Each field's range is in [DATA_REPRESENTATION.md](DATA_REPRESENTATION.md#fields).

| Bits  | Field          | Source |
| ----- | -------------- | ------ |
| 0–2 | `state` | `vehicle_controller_state()`: 0 INIT, 1 READY, 2 FOLLOW, 3 AVOID, 4 TURN, 5 SAFE_STOP |
| 3 | `latched` | `motion_control_latched()` |
| 4–11 | `faults` | Fault byte, below |
| 12–13 | `line` | `line_following_t.bits`: left and right sensor over black |
| 14 | `junction` | `line_following_t.b_junction` |
| 15 | `following` | `line_following_t.b_following` |
| 16–18 | `barcode` | `barcode_decoder_t.command`: 0 none, 1 left, 2 right, 3 straight, 4 U-turn |
| 19–22 | `barcodes` | `barcode_decoder_t.count` |
| 23–25 | `motion_class` | `s4_motion_t.cls` |
| 26–36 | `pitch` | `s4_tilt_t.pitch` + 1024 |
| 37–47 | `roll` | `s4_tilt_t.roll` + 1024 |
| 48–53 | `hump_max` | `s4_hump_t.max` |
| 54–57 | `humps` | `s4_hump_t.count` |
| 58 | `on_hump` | `s4_hump_t.on` |
| 59–61 | `avoid_state` | `obstacle_avoidance_status_t.state` |
| 62–63 | `avoid_side` | `obstacle_avoidance_status_t.side`: 0 none, 1 left, 2 right |
| 64–67 | `bypasses` | `obstacle_avoidance_status_t.bypasses` |
| 68–79 | `forward_mm` | `ultrasonic_scan_t.forward_mm` |

### Fault byte

The one shared fault bitmask on the network. Bits 0–2 are motion's `faults` as they are, and bits 5–6 are avoidance's sensor and blocked bits: `(faults & 3u) << 5`.

| Bit | Fault                                           | Source                                      |
| --- | ----------------------------------------------- | ------------------------------------------- |
| 0   | Left wheel stall                                | `MOTION_CONTROL_FAULT_STALL_LEFT`           |
| 1   | Right wheel stall                               | `MOTION_CONTROL_FAULT_STALL_RIGHT`          |
| 2   | Encoder missed a phase                          | `MOTION_CONTROL_FAULT_ENCODER`              |
| 3   | Impact                                          | `s4_fault_t.impact`                         |
| 4   | Excessive tilt                                  | `s4_fault_t.tilt`                           |
| 5   | No echo                                         | `OBSTACLE_AVOIDANCE_FAULT_SENSOR`           |
| 6   | No safe path                                    | `OBSTACLE_AVOIDANCE_FAULT_BLOCKED`          |
| 7   | Wi-Fi lost since the last `telemetry`           | The Telemetry task's latch, below           |

Avoidance's motion fault is left out: a failed bypass move is a stall, which bits 0–1 already carry.

### `scan`: 64 bits

Fields of `ultrasonic_scan_t.profile` and `obstacle_avoidance_status_t`, same names.

| Bits  | Field         |
| ----- | ------------- |
| 0–11 | `closest_mm` |
| 12–18 | `closest_deg` |
| 19–25 | `right_deg` |
| 26–32 | `left_deg` |
| 33–39 | `width_mm` |
| 40–51 | `left_mm` |
| 52–63 | `right_mm` |

### `cmd`: 8 bits

One bit per command, the same bits as the controller's event flag: 1 START, 2 STOP, 4 RESET. Other bits are ignored. START and RESET do what GP20 does ([VEHICLE_CONTROLLER.md](VEHICLE_CONTROLLER.md#gp20)): START from READY, RESET from SAFE_STOP, and are ignored in any other state. STOP is accepted in every state except INIT: the controller calls `safety_monitor_stop()`, so the car brakes and the controller goes to SAFE_STOP. In INIT nothing moves yet, and a stop there would let RESET skip the IMU calibration. No command steers or sets a speed.

## Inside the car

Only the kernel's Wi-Fi service task may call MQTT (`TEAM.md` section 3), so the payloads cross two kernel objects:

| Path                                   | Object         | Message                                                              |
| -------------------------------------- | -------------- | -------------------------------------------------------------------- |
| Telemetry task → Wi-Fi service         | Message buffer | 1 byte topic number (0 `telemetry`, 1 `motion`, 2 `scan`, 3 `heartbeat`) + the payload: at most 13 bytes |
| Wi-Fi service → Vehicle Controller     | Event flag     | The `cmd` byte as it is: `tk_set_flg(flag, cmd & 7u)`                |

The Telemetry task (Subsystem 1, priority band 20–21 in `TEAM.md` section 4) sends with `TMO_POL` and drops a message when the buffer is full, so a slow network never blocks it. The service reads the buffer with `TMO_POL` too. Event flag set and clear calls are atomic, so the command bits need no lock.

Every 10 ms the Telemetry task reads `cyw43_utk_get_status()` and sets its Wi-Fi lost latch, a `bool` only it touches, whenever the link is down. The latch goes out as fault bit 7 in each `telemetry` after the link returns, and is cleared only once one of them is queued: a send dropped by a full buffer keeps it set.

## Size

S1-U01 (`testing/subsystem1/`) measures every payload against the same data as Week 6 JSON.

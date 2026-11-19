# Telemetry and Commands (Subsystem 1)

The MQTT topics and binary payloads between the car and the supervisor (FR9, interface I12). They follow [DATA_REPRESENTATION.md](DATA_REPRESENTATION.md): every field uses only the bits its range needs, packed with explicit shifts, never C bit-fields. They replace the Week 6 review's JSON messages, and drop its version, robot, sequence, session, challenge and acknowledgement fields for simplicity. Without a sequence number the supervisor cannot spot a lost `telemetry` message; MQTT over TCP keeps them in order, and the `heartbeat` count shows gaps in the link.

## Topics

All topics start with `csc13/`. Every message is QoS 0, not retained.

| Topic       | Direction        | Sent                                                   | Payload                                          |
| ----------- | ---------------- | ------------------------------------------------------ | ------------------------------------------------ |
| `telemetry` | car → supervisor | Every 200 ms                                           | 11 bytes, below                                  |
| `motion`    | car → supervisor | With every `telemetry`                                 | 12 bytes, `motion_control_pack()` ([MOTION_CONTROL.md](MOTION_CONTROL.md#telemetry-payload)) |
| `scan`      | car → supervisor | Once per obstacle profile, when avoidance leaves `PROFILE` | 9 bytes, below                               |
| `heartbeat` | car → supervisor | Every 1 s                                              | 2 bytes: a count, wraps                          |
| `cmd`       | supervisor → car | On demand                                              | 1 byte, below                                    |

Events (barcode read, hump measured, bypass done, state change) need no topic of their own: each shows as a changed count or state in the next `telemetry`.

## Encoding

Each payload is one bit stream, most significant bit first, padded with zero bits to whole bytes, as in `motion_control_pack()`. A signed field is sent as value + offset so it is never negative. `testing/subsystem1/host/payloads.py` packs and unpacks every payload and is the reference decoder.

### `telemetry`: 83 bits

Each field's range is in [DATA_REPRESENTATION.md](DATA_REPRESENTATION.md#fields).

| Bits  | Field          | Source |
| ----- | -------------- | ------ |
| 0–2 | `state` | `vehicle_controller_state()`: 0 INIT, 1 READY, 2 FOLLOW, 3 AVOID, 4 TURN, 5 SAFE_STOP |
| 3 | `latched` | `motion_control_latched()` |
| 4–5 | `line` | `line_following_t.bits`: left and right sensor over black |
| 6 | `junction` | `line_following_t.b_junction` |
| 7 | `following` | `line_following_t.b_following` |
| 8–10 | `barcode` | `barcode_decoder_t.command`: 0 none, 1 left, 2 right, 3 straight, 4 U-turn |
| 11–14 | `barcodes` | `barcode_decoder_t.count` |
| 15–17 | `motion_class` | `s4_motion_t.cls` |
| 18–28 | `pitch` | `s4_tilt_t.pitch` + 1024 |
| 29–39 | `roll` | `s4_tilt_t.roll` + 1024 |
| 40–45 | `hump_max` | `s4_hump_t.max` |
| 46–49 | `humps` | `s4_hump_t.count` |
| 50 | `on_hump` | `s4_hump_t.on` |
| 51–53 | `avoid_state` | `obstacle_avoidance_status_t.state` |
| 54–55 | `avoid_side` | `obstacle_avoidance_status_t.side`: 0 none, 1 left, 2 right |
| 56–59 | `bypasses` | `obstacle_avoidance_status_t.bypasses` |
| 60–71 | `forward_mm` | `ultrasonic_scan_t.forward_mm` |
| 72–82 | `faults` | `app_faults()`: the shared fault bitmask, `APP_FAULT_...` in `app_fault.h` |

### `scan`: 68 bits

Fields of `ultrasonic_scan_t.profile` and `obstacle_avoidance_status_t`, same names.

| Bits  | Field         |
| ----- | ------------- |
| 0–11 | `closest_mm` |
| 12–19 | `closest_deg` |
| 20–27 | `right_deg` |
| 28–35 | `left_deg` |
| 36–43 | `width_mm` |
| 44–55 | `left_mm` |
| 56–67 | `right_mm` |

### `cmd`: 8 bits

One bit per command, the same bits as the controller's event flag: 1 START, 2 STOP, 4 RESET. Other bits are ignored. START and RESET do what GP20 does ([VEHICLE_CONTROLLER.md](VEHICLE_CONTROLLER.md#gp20)): START from READY, RESET from SAFE_STOP, and are ignored in any other state. STOP is accepted in every state except INIT: the controller calls `safety_monitor_stop()`, so the car brakes and the controller goes to SAFE_STOP. In INIT nothing moves yet, and a stop there would let RESET skip the IMU calibration. No command steers or sets a speed.

## Inside the car

Only the kernel's Wi-Fi service task may call MQTT (`TEAM.md` section 3), so the payloads cross two kernel objects:

| Path                                   | Object         | Message                                                              |
| -------------------------------------- | -------------- | -------------------------------------------------------------------- |
| Telemetry task → Wi-Fi service         | Message buffer | 1 byte topic number (0 `telemetry`, 1 `motion`, 2 `scan`, 3 `heartbeat`) + the payload: at most 13 bytes |
| Wi-Fi service → Vehicle Controller     | Event flag     | The `cmd` byte as it is: `tk_set_flg(flag, cmd & 7u)`                |

The Telemetry task (Subsystem 1, priority band 20–21 in `TEAM.md` section 4) sends with `TMO_POL` and drops a message when the buffer is full, so a slow network never blocks it. The service reads the buffer with `TMO_POL` too. Event flag set and clear calls are atomic, so the command bits need no lock.


## Size

S1-U01 (`testing/subsystem1/`) measures every payload against the same data as Week 6 JSON.

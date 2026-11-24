# Telemetry and Commands (Subsystem 1)

The MQTT topics and binary payloads between the car and the supervisor (FR9, interface I12). They follow [DATA_REPRESENTATION.md](DATA_REPRESENTATION.md): every field uses only the bits its range needs, packed with explicit shifts, never C bit-fields. They replace the Week 6 review's JSON messages, and drop its version, robot, sequence, session, challenge and acknowledgement fields for simplicity. Without a sequence number the supervisor cannot spot a lost `telemetry` message; MQTT over TCP keeps them in order, and the `heartbeat` count shows gaps in the link.

## Topics

All topics start with `csc13/`. Every message is QoS 0, and only `map` is retained.

| Topic       | Direction        | Sent                                                   | Payload                                          |
| ----------- | ---------------- | ------------------------------------------------------ | ------------------------------------------------ |
| `telemetry` | car → supervisor | Every 200 ms                                           | 12 bytes, below                                  |
| `motion`    | car → supervisor | With every `telemetry`                                 | 12 bytes, `motion_control_pack()` ([MOTION_CONTROL.md](MOTION_CONTROL.md#telemetry-payload)) |
| `scan`      | car → supervisor | Once per obstacle profile, when avoidance moves on to a bypass or stops with both sides blocked | 9 bytes, below                               |
| `heartbeat` | car → supervisor | Every 1 s                                              | 2 bytes: a count, wraps                          |
| `cmd`       | supervisor → car | On demand                                              | 1 byte, below                                    |
| `map`       | supervisor → car | On demand, retained                                    | 10 bytes, below                                  |

Events (barcode read, hump measured, bypass done, state change) need no topic of their own: each shows as a changed count or state in the next `telemetry`.

## Encoding

Each payload is one bit stream, most significant bit first, padded with zero bits to whole bytes, as in `motion_control_pack()`. A signed field is sent as value + offset so it is never negative. `tools/supervisor/payloads.py` packs and unpacks every payload and is the reference decoder.

### `telemetry`: 96 bits

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
| 40–45 | `hump_max` | `s4_hump_t.max`: the highest hump of the run, reset at each Start |
| 46–49 | `humps` | `s4_hump_t.count` |
| 50 | `on_hump` | `s4_hump_t.on` |
| 51–53 | `avoid_state` | `obstacle_avoidance_status_t.state` |
| 54–55 | `avoid_side` | `obstacle_avoidance_status_t.side`: 0 none, 1 left, 2 right |
| 56–59 | `bypasses` | `obstacle_avoidance_status_t.bypasses` |
| 60–71 | `forward_mm` | `ultrasonic_scan_t.forward_mm` |
| 72–86 | `faults` | `app_faults()`: the shared fault bitmask, `APP_FAULT_...` in `app_fault.h` |
| 87–90 | `rejects` | `barcode_decoder_t.rejects` |
| 91–95 | `letter` | `barcode_decoder_t.letter`: the last letter read, valid or rejected; 0 none, misread or digit, 1–26 A–Z |

### `scan`: 68 bits

Fields of `ultrasonic_scan_t.profile`, same names.

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

One bit per command, the same bits as the controller's event flag: 1 START, 2 STOP, 4 RESET. Other bits are ignored, and so is a payload that is not exactly one byte. The states in which the controller accepts each command, and what it does, are in [VEHICLE_CONTROLLER.md](VEHICLE_CONTROLLER.md#mqtt-commands). No command steers or sets a speed.

### `map`: 78 bits

The barcode action of every letter, 3 bits each, A in bits 0–2 to Z in bits 75–77: 0 ignore, 1 left, 2 right, 3 straight, 4 U-turn (`BARCODE_DECODER_...`). It replaces the whole of `g_actions` in `barcode_decoder.c` from the next barcode read, at any time, even mid-run; a command already read keeps its action. An ignored letter only counts as a reject, as a misread does; so does a letter sent as 5 to 7, which are not actions. A payload that is not exactly 10 bytes is ignored.

It is published retained, so the broker keeps the last one and sends it to the car each time the car subscribes: a car that restarts or reconnects gets it back, and one sent while the car is off arrives when it connects. Until a map arrives every letter is ignored, so a car that has had no map executes no barcode. To ignore them all again, send a map of zeros (**Ignore all** on the dashboard); a broker restart also drops the retained map, but a running car keeps the map it has.

## Inside the car

Only the kernel's Wi-Fi service task may call MQTT (`TEAM.md` section 3), so the payloads cross three kernel objects:

| Path                                   | Object         | Message                                                              |
| -------------------------------------- | -------------- | -------------------------------------------------------------------- |
| Telemetry task → Wi-Fi service         | Message buffer | 1 byte topic number (0 `telemetry`, 1 `motion`, 2 `scan`, 3 `heartbeat`) + the payload: at most 13 bytes |
| Wi-Fi service → Vehicle Controller     | Event flag     | The `cmd` byte as it is: `tk_set_flg(flag, cmd & 7u)`                |
| Wi-Fi service → Barcode Decoder        | Barcode mutex  | The `map` payload, unpacked by `payload_unpack_map()` and copied with `barcode_decoder_set_actions()` |

The Telemetry task (`s1_telemetry`, priority 20, in the band of `TEAM.md` section 4) sends with `TMO_POL` and drops a message when the buffer is full, so a slow network never blocks it. The service reads the buffer with `TMO_POL` too, on each of its 2 ms polls while the broker is connected, and publishes until the buffer is empty or lwIP has no room; a message lwIP refuses is kept and offered again on the next poll. The Vehicle Controller takes the command bits once per 10 ms cycle, also with `TMO_POL`, which clears them. Event flag set and clear calls are atomic, so the command bits need no lock.

The task and the message buffer are built only with `NET=lwip MQTT=1`, the default of the normal build ([DEVELOPMENT.md](DEVELOPMENT.md#building-with-mqtt)); the event flag exists in every build, and without MQTT nothing sets it.

| Code | Contents |
| ---- | -------- |
| `app_program/subsystem1/subsystem1.c/.h` | Telemetry task, the two kernel objects, and the four functions the service calls: next message, command received, map received, broker up or down |
| `app_program/subsystem1/payload.c/.h` | Packs `telemetry` and `scan`, unpacks `map`; no kernel headers, so the host test builds it |
| `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c/.h` | The service's side: connects to the broker in `config/wifi_secrets.h`, publishes, subscribes to `cmd` and `map` |

### Connection

The service connects once the Wi-Fi link is up. When an attempt fails or the connection drops it tries again 3 s later, from one timer. `APP_FAULT_WIFI` is set from start-up until the broker accepts the connection, and again while it is lost. A connection the broker closes is noticed at once; one that goes silent, such as the car leaving Wi-Fi range, is noticed by the MQTT keep-alive after about 90 s. Nothing in the car reacts to the fault: the car drives the same with or without the supervisor. While the broker is away the buffer fills and then drops new messages; when the broker accepts the connection the service empties it unpublished, so the supervisor sees only live data, and a gap in the heartbeat count shows how long the link was down. Live telemetry therefore never shows `WIFI`.

## Supervisor dashboard

[tools/supervisor/](../tools/supervisor/README.md) has the laptop side: `dashboard.py` subscribes to `csc13/#`, decodes each payload with `payloads.py`, and serves a local page with the live state, an event log, the Start, Stop and Reset commands and the barcode actions. Its README covers running the broker and the dashboard for a demo.

## Size

S1-U01 (`testing/subsystem1/`) measures every payload against the same data as Week 6 JSON.

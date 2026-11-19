# Subsystem 1 — test log

**Subsystem:** Wi-Fi, command and telemetry  ·  **Student ID:** 2503014  ·  **Code:** `app_program/subsystem1/`

Methods and how to record results: `testing/README.md`.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | MQTT payloads round-trip every field at its limits; sizes against the Week 6 JSON | FR9, Minimum-bit CR | host | `python3 -I tools/supervisor/payloads.py` | `ALL PASS`; every binary payload smaller than its JSON |
| S1-U02 | The C packers `payload_pack_telemetry()` and `payload_pack_scan()` give the bytes `payloads.pack()` gives, with every field's top bit set and a negative roll | FR9, Minimum-bit CR | host | `cc -std=c99 -Wall -Wextra -Werror -pedantic -O2 app_program/subsystem1/payload.c testing/subsystem1/host/test_payload.c -o /tmp/test_payload && /tmp/test_payload` | `ALL PASS` |
| S1-U03 | The dashboard decodes the bytes of S1-U02, with enumerated fields and faults by name; ignores a wrong size and a topic that is not ours; logs state changes, the run summary on a safe stop, barcodes and faults, and heartbeat gaps across the count's wrap and a restart | FR9, FR12, I12 | host | `cd tools/supervisor && pip install -r requirements.txt && python3 -m unittest -v test_dashboard` | `OK` |
| S1-U04 | The dashboard against a Mosquitto broker on the PC, without the car: payloads sent with `mosquitto_pub` show decoded on the page's `/state`; each button's request publishes its byte on `csc13/cmd` | FR9, I12 | host | Commands in `logs/2026-11-19_host_mqtt.txt` | `/state` holds the values of S1-U02; `mosquitto_sub` prints `01`, `02`, `04` for START, STOP, RESET |

## Results

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-31 | S1-U01 | `3a71771` | commit2main | PASS | `logs/2026-10-31_payloads.txt` | Telemetry 11 B, motion 12 B (92 bits), scan 8 B, heartbeat 2 B, cmd 1 B; periodic topics 117 B/s against 2280 B/s as JSON (motion at 200 ms, cmd as event flag bits) |
| 2026-11-19 | S1-U01 | `3735a6a` | commit2main | PASS | `logs/2026-11-19_payloads_faults11.txt` | Fault field widened to 11 bits for the lifted fault (bit 10); telemetry is 83 bits, still 11 bytes |
| 2026-11-19 | S1-U02 | `94c5bc4` | commit2main | PASS | `logs/2026-11-19_host_mqtt.txt` | Faults 0x600, angles 130° and 135°, pitch 900, roll −900. Packing the fault, pitch or `closest_deg` field one bit narrower makes the test fail. Run by the coding agent |
| 2026-11-19 | S1-U04 | `94c5bc4` | commit2main | PASS | `logs/2026-11-19_host_mqtt.txt` | Mosquitto 2.1.2. A telemetry message of the wrong size was ignored. In a browser the page showed the values and STOP published `02`. Run by the coding agent |
| 2026-11-19 | S1-U03 | `35c3cca` | commit2main | PASS | `logs/2026-11-19_supervisor.txt` | With the event log. In a browser against Mosquitto and a scripted stand-in for the car, READY, FOLLOW, SAFE_STOP and the car going silent each showed as expected, and Start and Esc (Stop) changed the stand-in's state |
| 2026-11-19 | S1-U01 | `3e7007a` | commit2main | PASS | `logs/2026-11-19_exact_periods_host.txt` | Payload layouts unchanged |
| 2026-11-19 | S1-U02 | `3e7007a` | commit2main | PASS | `logs/2026-11-19_exact_periods_host.txt` | Same run |
| 2026-11-19 | S1-U03 | `3e7007a` | commit2main | PASS | `logs/2026-11-19_exact_periods_host.txt` | Run-summary text now says the highest hump of this run |

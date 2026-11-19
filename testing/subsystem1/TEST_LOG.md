# Subsystem 1 — test log

**Subsystem:** Wi-Fi, command and telemetry  ·  **Student ID:** 2503014  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | MQTT payloads round-trip every field at its limits; sizes against the Week 6 JSON | FR9, Minimum-bit CR | host | `python3 -I tools/supervisor/payloads.py` | `ALL PASS`; every binary payload smaller than its JSON |
| S1-U02 | The C packers `payload_pack_telemetry()` and `payload_pack_scan()` give the bytes `payloads.pack()` gives, with every field's top bit set and a negative roll | FR9, Minimum-bit CR | host | `cc -std=c99 -Wall -Wextra -Werror -pedantic -O2 app_program/subsystem1/payload.c testing/subsystem1/host/test_payload.c -o /tmp/test_payload && /tmp/test_payload` | `ALL PASS` |
| S1-U03 | The dashboard decodes the bytes of S1-U02, with enumerated fields and faults by name; ignores a wrong size and a topic that is not ours; logs state changes, the run summary on a safe stop, barcodes and faults, and heartbeat gaps across the count's wrap and a restart | FR9, FR12, I12 | host | `cd tools/supervisor && pip install -r requirements.txt && python3 -m unittest -v test_dashboard` | `OK` |
| S1-U04 | The dashboard against a Mosquitto broker on the PC, without the car: payloads sent with `mosquitto_pub` show decoded on the page's `/state`; each button's request publishes its byte on `csc13/cmd` | FR9, I12 | host | Commands in `logs/2026-11-19_host_mqtt.txt` | `/state` holds the values of S1-U02; `mosquitto_sub` prints `01`, `02`, `04` for START, STOP, RESET |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…);
**host** = runs on the PC.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-31 | S1-U01 | `95d94ec` | commit2main | PASS | `logs/2026-10-31_payloads.txt` | Binary 10/12/8/2/1 B against JSON 275/182/141/50/101 B; periodic topics 172 B/s instead of 3245 B/s |
| 2026-10-31 | S1-U01 | `615a548` | commit2main | PASS | `logs/2026-10-31_payloads.txt` | Re-run after the scan sample angles changed to 85/95°; sizes unchanged |
| 2026-10-31 | S1-U01 | `42a79fa` | commit2main | PASS | `logs/2026-10-31_payloads.txt` | Adds the check against `motion_control_pack()`'s S2-U21 vector; sizes unchanged |
| 2026-10-31 | S1-U01 | `2f4eac0` | commit2main | PASS | `logs/2026-10-31_payloads.txt` | motion at 200 ms and cmd as event flag bits: periodic topics 112 B/s instead of 2335 B/s |
| 2026-10-31 | S1-U01 | `aedbf62` | commit2main | PASS | `logs/2026-10-31_payloads.txt` | Fault byte removed: telemetry 9 B; periodic topics 107 B/s instead of 2280 B/s |
| 2026-10-31 | S1-U01 | `3a71771` | commit2main | PASS | `logs/2026-10-31_payloads.txt` | Shared fault word added to telemetry (11 B), motion without its fault bits (92 bits, still 12 B): periodic topics 117 B/s instead of 2280 B/s |
| 2026-10-31 | S1-U01 | `af4be0d` | commit2main | PASS | `logs/2026-10-31_payloads_pr31.txt` | Re-run on PR #31; payload code unchanged |
| 2026-11-19 | S1-U01 | `3735a6a` | commit2main | PASS | `logs/2026-11-19_payloads_faults11.txt` | Fault field widened to 11 bits for the lifted fault (bit 10); telemetry is 83 bits, still 11 bytes |
| 2026-11-19 | S1-U01 | `94c5bc4` | commit2main | PASS | `logs/2026-11-19_host_mqtt.txt` | Re-run with the C packers' two vectors added to `payloads.py`; sizes unchanged. This row and the three below were run by the coding agent on the dev box, with no car attached |
| 2026-11-19 | S1-U02 | `94c5bc4` | commit2main | PASS | `logs/2026-11-19_host_mqtt.txt` | Faults 0x600, angles of 130° and 135°, pitch 900, roll −900. With the fault, pitch or `closest_deg` field packed one bit narrower the test fails |
| 2026-11-19 | S1-U03 | `94c5bc4` | commit2main | PASS | `logs/2026-11-19_host_mqtt.txt` | 4 tests. paho-mqtt came from `uv run --with 'paho-mqtt>=2,<3'` instead of pip |
| 2026-11-19 | S1-U04 | `94c5bc4` | commit2main | PASS | `logs/2026-11-19_host_mqtt.txt` | Mosquitto 2.1.2 on port 18830. A telemetry message of the wrong size was ignored. The page was also opened in a browser: it showed the values, and a click on STOP published `02` |
| 2026-11-19 | S1-U01 | `35c3cca` | commit2main | PASS | `logs/2026-11-19_supervisor.txt` | `payloads.py` moved to `supervisor/`, unchanged |
| 2026-11-19 | S1-U03 | `35c3cca` | commit2main | PASS | `logs/2026-11-19_supervisor.txt` | 4 tests, with the event log. The page was also viewed in a browser against Mosquitto and a scripted stand-in for the car: READY, FOLLOW, SAFE_STOP and the car going silent each showed as expected, and Start and Esc (Stop) changed the stand-in's state |
| 2026-11-19 | S1-U01 | `229ead5` | commit2main | PASS | `logs/2026-11-19_final_host.txt` | Final run on PR #43; payload layouts unchanged |
| 2026-11-19 | S1-U02 | `229ead5` | commit2main | PASS | `logs/2026-11-19_final_host.txt` | Final run on PR #43 |
| 2026-11-19 | S1-U03 | `229ead5` | commit2main | PASS | `logs/2026-11-19_final_host.txt` | Final run on PR #43: event log across gaps, reboots and the heartbeat wrap |
| 2026-11-19 | S1-U01 | `3e7007a` | commit2main | PASS | `logs/2026-11-19_exact_periods_host.txt` | Telemetry task woken by a cyclic handler; payload layouts unchanged |
| 2026-11-19 | S1-U02 | `3e7007a` | commit2main | PASS | `logs/2026-11-19_exact_periods_host.txt` | Same run |
| 2026-11-19 | S1-U03 | `3e7007a` | commit2main | PASS | `logs/2026-11-19_exact_periods_host.txt` | Run-summary text now says the highest hump of this run |

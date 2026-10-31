# Subsystem 1 — test log

**Subsystem:** Wi-Fi, command and telemetry  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | MQTT payloads round-trip every field at its limits; sizes against the Week 6 JSON | FR9, Minimum-bit CR | host | `python3 -I testing/subsystem1/host/payloads.py` | `ALL PASS`; every binary payload smaller than its JSON |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

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

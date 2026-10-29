# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | | | | | |
| INT-02 | Finite commands and coherent completion | I3/I4, FR4/NFR8 | proposed joint test | Controller issues accepted/rejected finite requests; reads running/final snapshots; tests concurrent cancellation | Correct units/direction, explicit rejection, coherent final state and no stale restart |
| INT-03 | Independent odometry consumers | I5, FR5/FR10/NFR8 | proposed joint test | Buddies 3/4/5 and Safety read independent snapshots during known movement; agree yaw derivation first | Counts/signs/mm/mm_s/timestamp consistent; one reader cannot consume another's data; yaw ownership resolved |
| INT-04 | Health, safety stop and restart arbitration | I10/I11, FR11/NFR5 | proposed joint test | Agree health transport and stop cause/generation; inject safe simulated faults, concurrent stop/command/reset; physical stage separately supervised | Stop invalidates old motion, latches cause, requires reset/revalidation/new Start; reset alone never moves |
| INT-05 | Line/barcode/avoidance motion handoff | I3/I4, FR3/FR7/FR8 | proposed joint test | Agree continuous versus finite target contract; exercise barcode turns/U-turn, line handoff and bounded bypass requests | Single motion owner, correct completion/cancellation, no stale target after handoff; system line recovery assessed separately |
| INT-06 | Motion telemetry and validity | Motion telemetry, FR9/NFR7 | proposed joint test | Buddy 1 packs/unpacks known boundary snapshots; agrees calibration-valid handling; runs publication/reconnection load | Exactly 36 bytes, correct signs/units/fault bits; reject malformed data; no RAM bit-field serialization or blocking motion |
| INT-07 | Shared GPIO dispatcher | IRQ13 / Buddy 5 | proposed joint test | Register callback at core-0 startup; exercise encoder and ultrasonic events together with bounded callback | No handler replacement, lost ownership or unintended flag acknowledgement; strict encoder faults retained |
| INT-08 | Integrated load and stop timing | NFR6/NFR7/A1 | proposed joint test | Agree limits and measurement methods; run sensing/Wi-Fi/reconnection load with both SMP builds as applicable | Record deadline/data-age violations, encoder invalids, CPU/stack/bandwidth/power and stop latency; no acceptance without agreed limits and evidence |


*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |

## Buddy 2 case prerequisites

INT-02–08 are draft cases from the Buddy 2 completion review, not executed
results or agreements. Both interface owners must review and sign off. INT-01
is the original unused placeholder and is preserved, not repurposed.
Before physical stages: validate the replacement encoder and record fresh
firmware/runtime calibration and power configuration, agree limits and arrange
supervision. No earlier hardware failure diagnosis is assumed.
Synthetic stages may validate interface logic without motors. Software stop
request timing is not physical stopping latency. Do not operate hardware as
part of documenting these cases.

| Case | Required reviewers | Status / evidence |
| --- | --- | --- |
| INT-02 | Buddy 2 + Controller owner | Proposed; unrun; unsigned |
| INT-03 | Buddy 2 + Buddies 3/4/5 + Safety owner | Proposed; unrun; unsigned |
| INT-04 | Buddy 2 + Safety/Controller owners | Proposed; health/generation contract unresolved |
| INT-05 | Buddy 2 + Buddy 3/5 + Controller owner | Proposed; target contract unresolved |
| INT-06 | Buddy 2 + Buddy 1 | Proposed; calibration validity contract unresolved |
| INT-07 | Buddy 2 + Buddy 5 | Proposed; unrun; unsigned |
| INT-08 | All affected owners | Proposed; limits/instrumentation and run evidence pending |

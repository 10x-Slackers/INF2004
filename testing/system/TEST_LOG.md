# System — test log

Tests of the whole car on the course against the requirements, one case per point of the Week 13 demo rubric plus a full run. Run the set before a demo or hand-in.

Methods and how to record results: `testing/README.md`. A system test whose result has to be seen (the car moving, stopping, rejoining the line) needs a video as well as the console log (`videos/README.md`).

## Test cases

Every case uses the normal build with the broker set in `config/wifi_secrets.h`, the dashboard open on the laptop (`tools/supervisor/README.md`), and the car on the course. Save the console log and the dashboard's **Download log** for each run.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | Detects a barcode | FR3 | manual | Car following the line towards a printed barcode beside it | The read shows on the console (`[barcode] ...`) and in the dashboard's events |
| SYS-02 | Decodes the barcode and reports the letter | FR3 | manual | As SYS-01, with a barcode of a known letter | The letter read matches the printed one, on the console and the dashboard's Line panel |
| SYS-03 | Measures the hump height | FR5, FR12, NFR4 | manual | Measure the hump's height by hand. Run the car over it, then stop the run | One hump reported; `[vc] max hump N mm` and the dashboard's safe-stop summary give a height close to the measured one. Record both |
| SYS-04 | Follows the line around the course | FR1, FR2 | manual | Start the car with GP20 or the dashboard's START and let it drive the course | Follows straights, curves and cross lines without leaving the line or needing a hand |
| SYS-05 | Carries out the barcode's instruction | FR3, FR8 | manual | Set the barcode's letter to an action on the dashboard (**Send to car**), then drive over it before a junction. Repeat for left, right, straight and U-turn | `[vc] barcode N pending`, then at the next junction the car takes the commanded branch and follows the new line |
| SYS-06 | Detects an obstacle | FR6, FR7 | manual | Place an obstacle on the line ahead of the car | The car stops before the obstacle and profiles it with the servo while standing still |
| SYS-07 | Measures the obstacle's width | FR6, NFR4 | manual | Measure the obstacle's width by hand; run as SYS-06 | The reported width is within ±10 cm of the measured one (closer is better). Record both |
| SYS-08 | Drives around the obstacle and rejoins the line | FR7, FR8, NFR5 | manual | As SYS-06, with open floor on at least one side | The car passes the obstacle without touching it, finds the line again and carries on following it |
| SYS-09 | Sends telemetry over MQTT for the whole run | FR9 | manual | Watch the dashboard for a whole run | Telemetry and heartbeat keep updating; state changes, barcodes, humps and bypasses show in the events; no `WIFI` fault |
| SYS-10 | Stops the motors when the car is picked up | FR10, FR11 | manual | While the car drives, pick it up and turn it upside down | The wheels stop within 0.5 s and stay stopped; the dashboard shows the safe stop and its summary |
| SYS-11 | A full course run | FR1, FR8, FR12 | manual | One run of the whole course with a barcode, a hump and an obstacle on it; end it by picking the car up | SYS-01 to SYS-10 all hold in the one run |

## Results

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|

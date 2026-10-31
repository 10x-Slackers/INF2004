# System — test log

Tests of the complete product against its requirements, run on the final hardware set-up. Run the full set before every demo and before hand-in.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | Static flash and RAM, in total and per subsystem | NFR7 | host | Build the normal image; `arm-none-eabi-size` on the `.elf` and on each subsystem's `.o` files under `build_make/mtkernel_3/app_program/` (commands in `logs/2026-10-31_static_memory.txt`) | `.data` + `.bss` under 50% of the 256 KB RAM; each subsystem's static RAM recorded |
| SYS-02 | No runtime allocation or unbounded queues in application code | NFR7 | host | `grep -rnE '\b(malloc\|calloc\|realloc\|free\|Kmalloc\|tk_get_mpl\|tk_get_mpf\|tk_cre_mbf\|tk_snd_mbx)\s*\(' app_program` | No matches; any hit must be in an `_init` function or a fixed-size, one-slot object |
| SYS-03 | Stack use of every task | NFR7 | auto | `TEST=1` image on the car; fill each task's stack with a known pattern at creation, run the full course, then print each task's bytes never written | Every task leaves at least 25% of its stack unused |
| SYS-04 | CPU load per core | NFR7 | auto | Full image with Wi-Fi and MQTT on the car; count idle time on each core over 10 s, once standing and once line following with telemetry | Each core busy under 70% while line following |
| SYS-05 | Telemetry bandwidth | NFR7 | manual | Full image on the car with MQTT; on the PC, `mosquitto_sub -v -t '#'` with timestamps for 60 s of line following; add up the payload bytes | Under 1 KB/s of payload on average; no topic published faster than designed |
| SYS-06 | Power demand | NFR7 | manual | USB power meter between the power bank and the car; read the current standing with Wi-Fi on, line following, and at the peak of a bypass turn | Peak below the power bank's rated output; average current recorded with the run time it gives |
| SYS-07 | Control deadlines under load | NFR6 | auto | Full image with Wi-Fi publishing and the ultrasonic scan running; each periodic task (motion, line, barcode, IMU, scan, safety) records its longest gap between runs and longest run, printed after the run | No gap longer than twice the task's period; motion `max_tick_us` under 5000 (as S2-U14) |
| SYS-08 | Data age at the point of use | NFR6, NFR5 | auto | Vehicle Controller and Safety Monitor record the largest age of each input they act on (line bits, barcode, odometry, forward range, IMU) during a full run | Each input's largest age is at most three periods of its producer; older data is treated as stale and stops the car |
| SYS-09 | End-to-end stop latency under load | NFR6, FR11 | manual | Full image, line following at the course speed with Wi-Fi publishing. (a) Place an obstacle in the path; (b) send `STOP` over MQTT. The firmware prints the time the trigger was seen (range under the stop distance, or the `STOP` message received) and the time `motion_control_stop()` returned; film the car. 5 runs each | Under 20 ms from trigger to brake in every run; the car never touches the obstacle |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | SYS-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-31 | SYS-01 | `9071c21` | commit2main | PASS | `logs/2026-10-31_static_memory.txt` | `.data` + `.bss` 24 784 B, 9.5% of 256 KB; flash 290 214 B of 2 MB. Application static RAM: S2 83 B, S4 32 B, S5 60 B, S1 and S3 stubs 0. Not included: the task stacks, which the kernel takes from its memory area at start-up (`APP_TASK_STACK_SIZE` 2048 B × 4 tasks in the normal image). Rerun once S1, S3 and the shared tasks exist |
| 2026-10-31 | SYS-02 | `9071c21` | commit2main | PASS | `logs/2026-10-31_static_memory.txt` | No matches in `app_program/`. Rerun once S1, S3 and the shared tasks exist |

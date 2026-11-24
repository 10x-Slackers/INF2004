# Team and subsystems

Who owns what: subsystems, pins, peripherals, task priorities, interfaces and
the kernel object budget. A **subsystem** is one student's part of the
product, with its own folder; it may have several tasks and own several
peripherals.

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | Wi-Fi, command and telemetry | 2503014 | | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motion control | 2503068 | xxcin | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | Line following and barcodes | 2502564 | mykacx | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU and terrain | 2503999 | commit2main | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Ultrasonic scanning and obstacle avoidance | 2503992 | CalebWee | `app_program/subsystem5/` | `testing/subsystem5/` |

Shared files (`app_program/app_main.c`, `app_program/app.h`, `app_program/app_fault.h`, `app_program/shared/`, this file)
are changed only by agreement of the team.

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it: two
subsystems on one pin build without error but fail once combined, and two
outputs driving each other can damage the pin.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | Ultrasonic (Buddy 5) | HC-SR04 TRIG (Grove 1). Unplug the sensor for `CRASH_REPORT=1` builds |
| GP1 | Ultrasonic (Buddy 5) | HC-SR04 ECHO, level shifted (Grove 1). Unplug the sensor for `CRASH_REPORT=1` builds |
| GP2 | Motion (Buddy 2) | right encoder phase A (Grove 2) |
| GP3 | Motion (Buddy 2) | right encoder phase B (Grove 2) |
| GP4 | IMU (Buddy 4) | GY-511 SDA, I2C0 (Grove 3) |
| GP5 | IMU (Buddy 4) | GY-511 SCL, I2C0 (Grove 3) |
| GP6 | Line following (Buddy 3) | right IR module DO (Grove 5) |
| GP7 | Line following (Buddy 3) | left IR module DO (Grove 7) |
| GP8 | Motion (Buddy 2) | left motor A, PWM4A |
| GP9 | Motion (Buddy 2) | left motor B, PWM4B |
| GP10 | Motion (Buddy 2) | right motor A, PWM5A |
| GP11 | Motion (Buddy 2) | right motor B, PWM5B |
| GP12 | Ultrasonic (Buddy 5) | SG90 servo signal (PWM slice 6 A) |
| GP13 | | |
| GP14 | | |
| GP15 | | |
| GP16 | Motion (Buddy 2) | left encoder phase A (Grove 4) |
| GP17 | Motion (Buddy 2) | left encoder phase B (Grove 4) |
| GP18 | | |
| GP19 | | |
| GP20 | Shared (Vehicle Controller); Ultrasonic (Buddy 5) in `TEST=1` | Robo Pico button: mission Start/Stop/reset in normal builds (`docs/VEHICLE_CONTROLLER.md`); steps the avoidance bench sequence in `TEST=1` builds |
| GP21 | Motion (Buddy 2) | Robo Pico button: a tap under 0.5 s starts the motion bench sequence; a hold of 1 s or more starts the line following bench run (Buddy 3); a press in between starts neither (`TEST=1` only) |
| GP22 | IMU (Buddy 4) | Robo Pico buzzer: beeps when calibration is done and on a fault (PWM slice 3 A) |
| GP26 | Line following (Buddy 3) | barcode IR module AO, ADC0 (Grove 6) |
| GP27 | | (ADC1 capable) |
| GP28 | | (ADC2 capable) |

## 3. Peripherals

**Each peripheral is driven by exactly one task, its *owning task*.** Every
other task asks the owning task through an interface (section 5) and never
touches the peripheral itself: tasks run on both cores at once, and the
drivers have no locking. A driver is code inside the owning subsystem,
`drv_<name>.c/.h` in its folder, never under `device/`. Switching on one of
the kernel's own drivers in `config/config_device.h` changes shared
configuration, so record it below first.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| I2C0 | IMU (Buddy 4) | `s4_imu` | GP4, GP5 (kernel driver `iica`, `DEVCNF_USE_IIC` = 1) |
| ADC | Line following (Buddy 3) | `s3_barcode` | GP26 (channel 0; `DEVCONF_ADC_PIN_INIT_0` = TRUE). The kernel's `adca` device is not opened |
| DMA channel 11 | Line following (Buddy 3) | `s3_barcode` | ADC FIFO to the sample ring. The Wi-Fi driver claims the lowest free channels (0, 1) at run time |
| PWM4, PWM5 | Motion (Buddy 2) | `s2_motion` (`motion_control_stop()` may brake from any task) | GP8–GP11 |
| IRQ13 (GPIO bank 0), core 0 | *shared* (`app_main.c`) | `app_gpio_isr` | Passes edges to the callbacks registered with `app_gpio_register()` in each `_init()`, not `tk_def_int(13)`. 2 slots |
| GPIO edge callback on IRQ13 | Motion (Buddy 2) | `drv_motion_edge`, registered with `app_gpio_register()` | GP2/3, GP16/17 encoders |
| PWM slice 3 | IMU (Buddy 4) | `s4_imu` | Slice 3 A: GP22 buzzer |
| PWM slice 6 | Ultrasonic (Buddy 5) | `s5_scan` | Slice 6 A: GP12 servo |
| GPIO edge callback on IRQ13 | Ultrasonic (Buddy 5) | `drv_hcsr04_edge`, registered with `app_gpio_register()` | GP1 echo |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

### Wi-Fi and MQTT

The radio, lwIP and MQTT belong to the kernel's Wi-Fi service task, `cyw43_utk_task`. **No application task may call `cyw43_...`, lwIP or MQTT functions**: they are not safe from any other task. Any task may read the link state with `cyw43_utk_get_status()`. The MQTT client (`lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`, `NET=lwip MQTT=1` builds) runs inside the service task: it publishes what the Telemetry task puts in a message buffer and passes commands back through an event flag (I12, `docs/TELEMETRY.md`). It is shared library code, so change it only by team agreement.

## 4. Task priorities

1 is the highest; 1–4 belong to the kernel (USB console 3, Wi-Fi 4). The bands follow the task order in `docs/ARCHITECTURE.md` (Tasks). Change a band by team agreement, and list every task.

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| Shared | 5, 10 | `safety` Safety Monitor (P1) – 5 – any (normal builds only); `vc` Vehicle Controller (P5) – 10 – any (normal builds only) |
| 2 | 6–7 | `s2_motion` (P2) – 6 – core 0; `s2_bench` – 7 – core 0 (`TEST=1` only) |
| 3 | 8–9 | `s3_barcode` (P3) – 8 – any; `s3_line` (P4) – 9 – any; `s3_bench` – 9 – any (`TEST=1` only) |
| 5 | 11–12 | `s5_avoid` (P6) – 11 – any; `s5_scan` (P7) – 12 – any; `s5_bench` – 12 – any (`TEST=1` only) |
| 4 | 18–19 | `s4_imu` (P8) – 18 – any |
| 1 | 20–21 | `s1_telemetry` Wi-Fi and MQTT Telemetry (P10) – 20 – any (`MQTT=1` builds only) |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every interface between subsystems. The **provider** owns the data and the
lock or kernel object behind it, and declares the calls in its own header;
users only call them.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| I3 | Controller, Buddies 3/5 → Motion | function calls under the motion mutex | `motion_control_move_*()`, `_turn_*()`, `_drive()`, `_stop()` | mm, degrees, mm/s; a new command replaces the running one | Buddy 2 contract |
| I4/I5 | Motion → any task | snapshot copy under the motion mutex | `motion_control_get_status()`; `motion_control_distance_um()` | state, speeds, counts, odometer, heading; distance rolled up to now in µm | Buddy 2 contract |
| I11 | Safety, Controller → Motion; Controller → Safety | function calls | `motion_control_stop()`, `_latch()`, `_latched()`; `safety_monitor_stop()`, `_clear()` | stop brakes both wheels before returning and cancels the command; while latched every command is refused; clear first clears an IMU sensor fault, and is refused while another IMU fault is set, or if motion failed to start; `docs/SAFETY_MONITOR.md` | Buddy 2 contract, shared |
| I6 | Controller ↔ Avoidance | function calls; status copy under the avoidance mutex | `obstacle_avoidance_start()`, `_cancel()`, `_line()`, `_get_status()` | state, bypass side and step, bypasses done, side clearances; `docs/OBSTACLE_AVOIDANCE.md` | Buddy 5 contract |
| I7/I8 | Any task ↔ Ultrasonic scan | function calls; result copy under the scan mutex | `ultrasonic_scan_request()`, `ultrasonic_scan_get()` | forward range and its time, obstacle profile; `docs/OBSTACLE_AVOIDANCE.md` | Buddy 5 contract |
| I1 | Line Following ↔ Controller, Avoidance | function calls; line state copy under the line mutex | `line_following_start()`, `_stop()`, `_get()` | follow at 1–500 mm/s until stopped, searching for a lost line; sensor bits, last side seen, junction flag, following, searching, line lost, sample count and time; `docs/LINE_FOLLOWING.md` | Buddy 3 contract |
| I2 | Barcode Decoder → Controller | status copy under the barcode mutex | `barcode_decoder_get()` | last validated command (left, right, straight, U-turn, per letter in `g_actions`, which `barcode_decoder_set_actions()` replaces from the supervisor's `map`), validated and rejected counts, odometer at the barcode; `barcode_decoder.h` | Buddy 3 contract |
| I9 | IMU → Controller, Telemetry | record copies under `gh_s4_mutex` | `s4_get_tilt()`, `_get_hump()`, `_get_motion()`; `s4_reset_hump_max()`, `_reset_sensor()` | calibration done, tilt, highest hump of the run (the controller resets it at each Start), motion class. The Safety Monitor reads only the IMU fault bits, and its clear calls `s4_reset_sensor()`: a sensor fault is cleared and the IMU calibrates again | Buddy 4 contract |
| I12 | Telemetry ↔ Wi-Fi service ↔ Controller | message buffer (`TMO_POL` both ends); event flag | `gh_s1_queue` (private to Subsystem 1, read through `mqtt_utk_app_take()`); `gh_s1_command_flag` | Topic number + binary payload, at most 13 bytes; START, STOP, RESET bits (`S1_CMD_...`); the barcode `map`, through `barcode_decoder_set_actions()`; `docs/TELEMETRY.md` | Buddy 1 contract |
| Faults | Every subsystem → any task | one kernel event flag (atomic set and clear) | `app_fault_set()`, `_clear()`, `app_faults()` (`app_fault.h`) | one bit per fault, each owner sets and clears only its own bits; `docs/DATA_REPRESENTATION.md` | shared |
| Motion telemetry | Motion → Buddy 1 | explicit bit packing | `motion_control_pack()` | 12 bytes; `docs/MOTION_CONTROL.md` | Buddy 2 contract |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`; raise one there if the team needs more.

| Object | Limit | Shared | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|---|
| Task | 32 | 2 | 1 | 2 | 3 | 1 | 3 |
| Semaphore | 16 | | | | | | 1 |
| Event flag | 16 | 1 | 1 | | | | |
| Mailbox | 12 | | | | | | |
| Mutex | 12 | | | 1 | 2 | 1 | 2 |
| Message buffer | 12 | | 1 | | | | |
| Fixed memory pool | 12 | | | | | | |
| Variable memory pool | 12 | | | | | | |
| Cyclic handler | 12 | 2 | 1 | 1 | 2 | 1 | |
| Alarm handler | 12 | | | | | | |

Shared is the Safety Monitor and Vehicle Controller (normal builds only) and the fault bitmask's event flag (`app_main.c`). (The kernel's USB and WiFi service tasks also use a few of these.)

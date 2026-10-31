# Team and subsystems

Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

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

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | Ultrasonic (Buddy 5) | HC-SR04 TRIG (Grove 1). Unplug the sensor for `CRASH_REPORT=1` builds |
| GP1 | Ultrasonic (Buddy 5) | HC-SR04 ECHO, level shifted (Grove 1). Unplug the sensor for `CRASH_REPORT=1` builds |
| GP2 | Motion (Buddy 2) | right encoder phase A (Grove 2) |
| GP3 | Motion (Buddy 2) | right encoder phase B (Grove 2) |
| GP4 | IMU (Buddy 4) | GY-511 SDA, I2C0 (Grove 3) |
| GP5 | IMU (Buddy 4) | GY-511 SCL, I2C0 (Grove 3) |
| GP6 | Line following (Buddy 3) | left IR module DO (Grove 5) |
| GP7 | Line following (Buddy 3) | right IR module DO (Grove 7) |
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
| GP20 | Ultrasonic (Buddy 5) | Robo Pico button: steps the avoidance bench sequence (`TEST=1` only) |
| GP21 | Motion (Buddy 2) | Robo Pico button: a tap under 0.5 s starts the motion bench sequence; a hold of 1 s or more starts the line following bench run (Buddy 3); a press in between starts neither (`TEST=1` only) |
| GP22 | IMU (Buddy 4) | Robo Pico buzzer: beeps when calibration is done and on a fault (PWM slice 3 A) |
| GP26 | Line following (Buddy 3) | barcode IR module AO, ADC0 (Grove 6) |
| GP27 | | (ADC1 capable) |
| GP28 | | (ADC2 capable) |

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | | | |
| I2C0 | IMU (Buddy 4) | `s4_imu` | GP4, GP5 (kernel driver `iica`, `DEVCNF_USE_IIC` = 1) |
| I2C1 | | | |
| SPI0 | | | |
| SPI1 | | | |
| ADC | Line following (Buddy 3) | `s3_barcode` | GP26 (channel 0; `DEVCONF_ADC_PIN_INIT_0` = TRUE). The kernel's `adca` device is not opened |
| DMA channel 11 | Line following (Buddy 3) | `s3_barcode` | ADC FIFO to the sample ring. The Wi-Fi driver claims the lowest free channels (0, 1) at run time |
| PWM4, PWM5 | Motion (Buddy 2) | `s2_motion` (`motion_control_stop()` may brake from any task) | GP8–GP11 |
| IRQ13 (GPIO bank 0), core 0 | Motion (Buddy 2) | `drv_motion_gpio_isr` | GP2/3, GP16/17. Other GPIO edge users register a callback with `drv_motion_gpio_register()` |
| PWM slices (list) | IMU (Buddy 4) | `s4_imu` | Slice 3 A: GP22 buzzer |
| PWM slices (list) | Ultrasonic (Buddy 5) | `s5_scan` | Slice 6 A: GP12 servo |
| GPIO edge callback on IRQ13 | Ultrasonic (Buddy 5) | `drv_hcsr04_edge`, registered with `drv_motion_gpio_register()` | GP1 echo |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students *can* do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
- **Send or receive data over the network** by adding code that *runs
  inside the service task*. The MQTT client is in
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. It publishes a heartbeat
  today. A "cloud / reporting" subsystem would add a kernel object
  (e.g. a message buffer) that application tasks fill. The MQTT code then
  reads it with `TMO_POL` (never blocking) and publishes it.
  Because this changes shared library code, **agree it with the team and
  the instructor first**, and record it as an interface in section 5.
  It needs the `NET=lwip MQTT=1` build (`make help`).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). The bands follow the task order in
`docs/ARCHITECTURE.md` (Tasks), so a lower `P` number always gets a higher priority. Change a band only by team
agreement. Write down every task you create.

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| Shared | 5, 10 | Safety Monitor (P1) – 5; Vehicle Controller (P5) – 10 |
| 2 | 6–7 | `s2_motion` (P2) – 6 – core 0; `s2_bench` – 7 – core 0 (`TEST=1` only) |
| 3 | 8–9 | `s3_barcode` (P3) – 8 – any; `s3_line` (P4) – 9 – any; `s3_bench` – 9 – any (`TEST=1` only) |
| 5 | 11–12 | `s5_avoid` (P6) – 11 – any; `s5_scan` (P7) – 12 – any; `s5_bench` – 12 – any (`TEST=1` only) |
| 4 | 18–19 | `s4_imu` (P8) – 18 – any |
| 1 | 20–21 | Wi-Fi and MQTT Telemetry (P10) |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| I3 | Controller, Buddies 3/5 → Motion | function calls under the motion mutex | `motion_control_move_*()`, `_turn_*()`, `_drive()`, `_stop()` | mm, degrees, mm/s; a new command replaces the running one | Buddy 2 contract |
| I4/I5 | Motion → any task | snapshot copy under the motion mutex | `motion_control_get_status()` | state, faults, speeds, counts, odometer, heading | Buddy 2 contract |
| I11 | Safety, Controller → Motion | function call | `motion_control_stop()` | brakes both wheels before returning; cancels the command | Buddy 2 contract |
| I6 | Controller ↔ Avoidance | function calls; status copy under the avoidance mutex | `obstacle_avoidance_start()`, `_cancel()`, `_get_status()` | state, faults, bypass side and step, bypasses done, side clearances; `docs/OBSTACLE_AVOIDANCE.md` | Buddy 5 contract |
| I7/I8 | Any task ↔ Ultrasonic scan | function calls; result copy under the scan mutex | `ultrasonic_scan_request()`, `ultrasonic_scan_get()` | forward range and its time, obstacle profile; `docs/OBSTACLE_AVOIDANCE.md` | Buddy 5 contract |
| I1 | Line Following ↔ Controller, Avoidance | function calls; line state copy under the line mutex | `line_following_start()`, `_stop()`, `_get()` | follow at 1–500 mm/s until stopped; sensor bits, last side seen, junction flag, following, sample count and time; `line_following.h` | Buddy 3 contract |
| I2 | Barcode Decoder → Controller | status copy under the barcode mutex | `barcode_decoder_get()` | last validated command (left, right, straight, U-turn for A to D), validated and rejected counts, odometer at the barcode; `barcode_decoder.h` | Buddy 3 contract |
| Motion telemetry | Motion → Buddy 1 | explicit bit packing | `motion_control_pack()` | 12 bytes; `docs/MOTION_CONTROL.md` | Buddy 2 contract |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. Fill in each subsystem's usage.

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | | 2 | 3 | 1 | 3 |
| Semaphore | 16 | | | | | 1 |
| Event flag | 16 | | | | | |
| Mailbox | 12 | | | | | |
| Mutex | 12 | | 1 | 2 | 1 | 2 |
| Message buffer | 12 | | | | | |
| Fixed memory pool | 12 | | | | | |
| Variable memory pool | 12 | | | | | |
| Cyclic handler | 12 | | 1 | 1 | 1 | |
| Alarm handler | 12 | | | | | |

(The kernel's USB and WiFi service tasks also use a few of these.)

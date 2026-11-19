# INF2004 Embedded Systems Programming Project

Autonomous Robotic Car Challenge based on µT-Kernel 3.0 (RP2040 dual-core SMP Port).
The car follows a line, acts on Code 39 barcodes at junctions, measures humps
with the IMU, profiles and drives around obstacles, publishes telemetry over
MQTT, and stops when it is picked up.

| Where | What |
|---|---|
| [AGENTS.md](AGENTS.md) | How to work in this repository: ownership, conventions, tests, checks |
| [docs/](docs/README.md) | Requirements, architecture, hardware and each subsystem |
| [TEAM.md](TEAM.md) | Who owns which subsystem, pin, peripheral, priority and interface |
| [testing/](testing/README.md) | Test cases and results |
| [tools/supervisor/](tools/supervisor/README.md) | The laptop dashboard that shows telemetry and sends Start, Stop and Reset |

## Getting Started (Dev Container)

### Prerequisites

- [Git](https://github.com/git-guides/install-git) (fully set-up)
- [Docker/Podman](https://docs.docker.com/engine/install/)
- [VS Code](https://code.visualstudio.com/download)
  - [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension

> [!WARNING]
> Do not use GitHub Desktop! All interactions (files, git, runtime, etc.) should be done through the Dev Container within VS Code.

### Installation

1. Clone the repo and open it in VS Code

   ```sh
   git clone git@github.com:10x-Slackers/INF2004.git
   code INF2004/
   ```

2. Click on the "Re-open in Dev Container" prompt. The container installs the compiler and `picotool`, clones the [Pico SDK](https://github.com/raspberrypi/pico-sdk) into `.pico-sdk/` with the USB, Wi-Fi and lwIP submodules, sets `PICO_SDK_PATH` and installs the pre-commit hooks, so section 1 below can be skipped.
3. For the MQTT build, copy `config/wifi_secrets.example.h` to `config/wifi_secrets.h` and fill in the Wi-Fi network and broker address (section 3).
4. Build from the repository root with `make -C build_make -j$(nproc)` (or the VS Code `Build` task), then flash with `picotool load -v -x build_make/mtk3pico_smp1_usb_cdc_wifi.uf2` (or the `Flash` task) while the Pico W is in BOOTSEL mode. Build options are in section 3.

### Developer Tooling

- Dev Containers
  - Standardised developer environment
- Pre-Commit
  - Formats `app_program/` with clang-format (`.clang-format`, Barr C layout) on every commit
- VS Code tasks
  - `Build`, `Build (tests)`, `Build (crash report)`, `Clean` and `Flash`

### Template upstream

This repository is based on the course's [INF2004_ProjectTemplate](https://github.com/sirfonzie/INF2004_ProjectTemplate). To pull in template updates:

```sh
git remote add upstream https://github.com/sirfonzie/INF2004_ProjectTemplate  # once
git pull upstream main
```

The rest of this file is the template's own guide.

---

## micro T-Kernel 3.0 project for Raspberry Pi Pico W

The firmware runs micro T-Kernel 3.0 on **both cores** of the Pico W's
RP2040, with a USB serial console, the Wi-Fi driver and, in the MQTT build,
lwIP and an MQTT client built in. It runs on a Cytron Robo Pico carrier
board; the wiring is in [docs/HARDWARE.md](docs/HARDWARE.md).

You need a **Raspberry Pi Pico W**, a USB cable, and a computer running
macOS, Linux, or Windows with **WSL (Ubuntu)**. On Windows, run every
command below inside the WSL Ubuntu terminal.

---

## 1. Set up the Pico C SDK and the compiler (not needed in the Dev Container)

You need two separate things:

| What | Why | Where it comes from |
|---|---|---|
| **Pico C SDK** | USB and WiFi driver source code | `git clone` (step 1.2) |
| **Arm GNU Toolchain** (`arm-none-eabi-gcc`) | the compiler | a separate download (step 1.3) |

> ⚠️ **The SDK does NOT include the compiler.** You must install the compiler
> on its own and put its `bin` folder on your `PATH` (step 1.4).

**1.1 Install the basic tools: git, make and a host C++ compiler**

```sh
# macOS
xcode-select --install

# Linux / WSL (Ubuntu)
sudo apt update && sudo apt install -y git make g++
```

**1.2 Download the Pico C SDK** (any folder works; this guide uses `~/pico`)

```sh
mkdir -p ~/pico && cd ~/pico
git clone --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip
```

The `submodule` line is required: it downloads the USB and WiFi drivers and
the lwIP network stack (used by the MQTT build).
(SDK 2.3.0 has also been tested and works.)

**1.3 Install the Arm GNU Toolchain (the compiler)**

- **macOS / Windows (WSL) / Linux:** download from
  <https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads>.
  Pick the **AArch32 bare-metal target (arm-none-eabi)** package for your
  computer. For WSL, choose the **Linux x86_64** `.tar.xz`.
  Extract it, e.g. to `~/tools/arm-gnu-toolchain-<version>/`.
- **Linux / WSL shortcut:** `sudo apt install -y gcc-arm-none-eabi`
  installs it straight onto your PATH. If you use this, skip the PATH line in 1.4.

Find the folder that contains `arm-none-eabi-gcc`. It is the toolchain's
**`bin`** folder, e.g. `~/tools/arm-gnu-toolchain-15.3.rel1/bin`.

**1.4 Tell your shell where the compiler and SDK are**

Add these two lines to the end of `~/.zshrc` (macOS) or `~/.bashrc` (Linux/WSL),
**changing both paths to match your computer**:

```sh
export PATH="$HOME/tools/arm-gnu-toolchain-15.3.rel1/bin:$PATH"   # folder that contains arm-none-eabi-gcc
export PICO_SDK_PATH="$HOME/pico/pico-sdk"                         # the SDK from step 1.2
```

Open a **new** terminal, then check both:

```sh
arm-none-eabi-gcc --version     # must print a version, not "command not found"
ls $PICO_SDK_PATH               # must list the SDK files
```

If `arm-none-eabi-gcc` is "not found", your `PATH` does not point to the
toolchain's `bin` folder. Fix that before going further.

---

## 2. Clone this repo and build the `.uf2` file

**2.1 Clone the repository**

```sh
cd ~/pico
git clone git@github.com:10x-Slackers/INF2004.git
cd INF2004
```

**2.2 Build**

```sh
cd build_make
make -j8
```

A successful build ends with a line like
`tools/elf2uf2 mtk3pico_smp1_usb_cdc_wifi.elf mtk3pico_smp1_usb_cdc_wifi.uf2`.
Your firmware is **`build_make/mtk3pico_smp1_usb_cdc_wifi.uf2`**.

If `make` stops with an error, it tells you which one is wrong:
`arm-none-eabi-gcc not found` means fix `PATH`, and
`PICO_SDK_PATH is not set` means fix `PICO_SDK_PATH` (step 1.4).

To build a single-core, UART-only or test version, see section 3.

**2.3 Flash it onto the Pico W**

1. Hold the **BOOTSEL** button and plug the Pico W into USB, then release.
   A drive called **RPI-RP2** appears.
2. Copy the `.uf2` file onto that drive. On WSL, run `explorer.exe .` in
   `build_make` to open the folder in Windows. The Pico reboots by itself.

With `picotool` installed (it is in the Dev Container), run
`picotool load -v -x build_make/mtk3pico_smp1_usb_cdc_wifi.uf2` instead of
copying.

**2.4 See the output**

Open the Pico's USB serial port at 115200 baud:

```sh
screen /dev/tty.usbmodem* 115200      # macOS   (quit: Ctrl-A then K)
screen /dev/ttyACM0 115200            # Linux
```

On Windows, use PuTTY or the VS Code *Serial Monitor* on the new COM port.
You should see the kernel banner and "all subsystems started", then
`[imu] calibrated` after about 2 s with the car still, and `[vc] INIT ->
READY`. Press GP20 on the Robo Pico to start a run and again to stop it
([docs/VEHICLE_CONTROLLER.md](docs/VEHICLE_CONTROLLER.md)).

**2.5 Write your own code**

The code lives in **`app_program/`**. Every `.c` file there, and in its
direct subfolders, is compiled automatically. Read [AGENTS.md](AGENTS.md)
first: it has the rules on ownership, the coding standard and tests.

- **`app_main.c` → `usermain()` is where the program starts.** It first calls
  every subsystem's `subsystemN_init()` (create kernel objects), then every
  `subsystemN_start()` (start tasks).
- **One folder per student:** `app_program/subsystem1/` … `subsystem5/`
  (telemetry, motion, line following and barcodes, IMU, obstacle
  avoidance), plus `app_program/shared/` for the Safety Monitor and the
  Vehicle Controller (the mission state machine).
- `subsystemN.h` is that subsystem's public interface; `app.h` and
  `app_fault.h` hold what the whole team shares.
- **Drivers go in the owning subsystem's folder**, as a `drv_<name>.c` /
  `drv_<name>.h` pair, e.g. `subsystem5/drv_hcsr04.c`. Do not add drivers
  under `device/`: that folder holds the kernel's own serial, I2C and ADC
  drivers.
- How to add a task, pass data between tasks and use the hardware is in
  [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md).

**Working as a team:** owners, pins, priorities and the interfaces between
subsystems are in **`TEAM.md`**; tests are recorded in **`testing/`** (see
`testing/README.md`). `make TEST=1` builds a test image that runs all the
subsystem tests at power-up, and each subsystem's bench sequence.

Pins: the pin map is in [docs/HARDWARE.md](docs/HARDWARE.md#pin-map); claim
a free pin in `TEAM.md` before using it. GP0/GP1 carry the HC-SR04, so
unplug it for `CRASH_REPORT=1` builds (section 3). Never use
GP23/GP24/GP25/GP29: they are wired to the Pico W radio. The Pico W's
on-board LED is also on the radio.

After editing, run `make -j8` again in `build_make` and copy the new `.uf2`.

---

## 3. Build options: `SMP`, `CONSOLE`, `WIFI`, `TEST`, `CRASH_REPORT`

`make` on its own uses the defaults. To build differently, put options
after `make`. They can be combined in any order:

```sh
make -j8                          # defaults: SMP=1 CONSOLE=usb_cdc WIFI=cyw43 TEST=0 CRASH_REPORT=0
make SMP=0 -j8                    # single-core build
make CONSOLE=uart WIFI=none -j8   # no USB console, radio off (no Pico SDK needed)
make TEST=1 -j8                   # firmware with the subsystem self-tests
make CRASH_REPORT=1 -j8           # also see crash reports, on GP0/GP1 (uses those pins)
```

To set several options at once, list them all, separated by spaces. For
example, to track down a crash in the test build while running on one core only:

```sh
make SMP=0 TEST=1 CRASH_REPORT=1 -j8
#  -> build_make/mtk3pico_smp0_usb_cdc_wifi_crash_test.uf2
```

Any option you leave out keeps its default (here `CONSOLE=usb_cdc` and
`WIFI=cyw43`). Options are not remembered: give them on every `make`, as
the next plain `make` goes back to the defaults.

| Option | Values (**default**) | What it does | Image name | C macro your code can test |
|---|---|---|---|---|
| `SMP` | `0`, **`1`** | `1` runs the kernel on both RP2040 cores; `0` uses core 0 only | `smp1` / `smp0` | `TK_SUPPORT_SMP` (1 or 0) |
| `CONSOLE` | **`usb_cdc`**, `uart` | where `tm_printf` output goes. `usb_cdc` = USB cable; `uart` = UART0 only (GP0 TX / GP1 RX, needs a USB-serial adapter) | `_usb_cdc` / `_uart` | `TM_CONSOLE_USB_CDC` |
| `WIFI` | **`cyw43`**, `none` | `cyw43` powers up the Pico W radio and its driver task; `none` leaves the radio off | `_wifi` / *(nothing)* | `TM_WIFI_CYW43` |
| `TEST` | **`0`**, `1` | `1` compiles every `subsystemN_test.c` and runs the tests at power-up | `_test` / *(nothing)* | `APP_TEST` (1 or 0) |
| `CRASH_REPORT` | **`0`**, `1` | `1` also copies the console to UART0 on **GP0 (TX) / GP1 (RX)**, 115200 baud, so you can see the last messages and the kernel's crash report (see below). **Uses GP0/GP1.** `CONSOLE=uart` always does this. | `_crash` / *(nothing)* | `TM_CONSOLE_UART` (1 or 0) |

**Why `CRASH_REPORT` exists.** USB output is not sent the moment you call
`tm_printf`: it waits in a buffer until the kernel's USB task sends it. If the
program crashes, no task runs again, so the last messages and the kernel's
crash report never reach your USB terminal. The terminal just goes quiet.
UART output *is* sent at once, even inside the crash handler. So when you
need to know why the Pico crashed or froze, build with `CRASH_REPORT=1`,
connect a USB-serial adapter (adapter RX → GP0, adapter GND → GND), and read
it at 115200 baud. Normal `tm_printf` output on USB works the same with
either setting. GP0/GP1 carry the HC-SR04 on this car: unplug it before
using `CRASH_REPORT=1`.

The options are part of the image name, e.g.
`mtk3pico_smp1_usb_cdc_wifi_test.uf2`, so you can always tell which build
you are flashing. Switching options needs no `make clean`: the makefile
notices and recompiles everything.

Using an option in C code:

```c
#if TK_SUPPORT_SMP
    ctsk.tskatr |= TA_ASSPRC;   /* core affinity only exists on the dual-core build */
#endif

#if APP_TEST
    /* code that only exists in the test build */
#endif
```

### Where the options live in the makefile

All five are handled in **`build_make/pico_rp2040.mk`**. To find each piece, search the file for the
names in the left column (e.g. `grep -n "TEST" build_make/pico_rp2040.mk`).
Each option goes through the same steps:

| Step | What to look for | Example for `TEST` |
|---|---|---|
| 1. Default value | `NAME ?= value` near the top of the file | `TEST ?= 0` |
| 2. Allowed values | an `$(error Unknown ...)` check just below it | `$(error Unknown TEST '$(TEST)'; use 0 or 1)` |
| 3. Image-name suffix | `NAME_SUFFIX :=` and the `EXE_FILE :=` line | `TEST_SUFFIX := _test` |
| 4. C macro for the code | a `CFLAGS += -D...` line | `CFLAGS += -DAPP_TEST=$(TEST)` |
| 5. Rebuild on change | the `PROFILE_ID :=` line | `...-test$(TEST)` |
| 6. Extra source files (`CONSOLE`, `WIFI` only) | an `ifeq ($(CONSOLE),usb_cdc)` or `ifeq ($(WIFI),cyw43)` block that adds the USB/radio files and SDK include paths | — |

- **Changing a default:** edit the `?=` line from step 1, e.g.
  `CONSOLE ?= uart`. Every `make` without that option then uses the new value.
- **Adding your own option** (say `DEBUG=1` to turn on extra prints): copy
  how `TEST` is done. Add steps 1 to 5 next to the `TEST` lines, then
  use `#if APP_DEBUG` in your code. **Step 5 matters:** without it, switching
  the option can quietly link files that were compiled with the old setting.
- **Kernel settings** such as the maximum number of tasks, semaphores or
  mailboxes are not make options. They are in `config/config.h` (`CNF_MAX_...`).

### Network and MQTT: `NET`, `MQTT`

The telemetry and supervisor commands need two more options, handled the
same way:

| Option | Values (**default**) | What it does | Image name | C macro |
|---|---|---|---|---|
| `NET` | **`none`**, `lwip` | `lwip` adds the lwIP TCP/IP stack, run by the Wi-Fi service task. Needs `WIFI=cyw43` | `_lwip` | `TM_NET_LWIP` |
| `MQTT` | **`0`**, `1` | `1` adds the MQTT client and Subsystem 1's Telemetry task. Needs `NET=lwip` | `_mqtt` | `TM_MQTT` |

They need the SDK's `lib/lwip` submodule and a `config/wifi_secrets.h` with
the Wi-Fi network and broker address (copy `config/wifi_secrets.example.h`;
the file is gitignored, never commit it). The car's full image for the demo:

```sh
make NET=lwip MQTT=1 -j8
#  -> build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2
```

Topics, payloads and the laptop dashboard are in
[docs/TELEMETRY.md](docs/TELEMETRY.md).

---

*micro T-Kernel 3.0 © Ken Sakamura / TRON Forum, distributed under the
T-License 2.2. This dual-core RP2040 port is derived from the official
μT-Kernel 3.0 BSP (`tron-forum/mtk3_bsp`, branch `pico_rp2040`).*

# INF2004 autonomous robotic car

Raspberry Pi Pico W + Cytron Robo Pico, C on dual-core µT-Kernel 3.0.
The [course template](https://github.com/sirfonzie/INF2004_ProjectTemplate)
provides the kernel, USB console and Wi-Fi port. Application subsystems live
in `app_program/`.

## Native build (macOS or Linux)

Install Git, GNU make, a host C/C++ compiler, Python 3 and the **Arm GNU
Toolchain** (`arm-none-eabi-gcc`, with newlib) on `PATH`. macOS's command-line
developer tools provide make and the host compiler. Docker is optional.

Use Pico SDK 2.2.0 with its USB and radio submodules:

```sh
git clone --depth 1 --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git .pico-sdk
git -C .pico-sdk submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip
make -j8
make test
```

An SDK elsewhere is supported with `export PICO_SDK_PATH=/path/to/pico-sdk`,
or gitignored `config/local.mk` containing `PICO_SDK_PATH := /path/to/pico-sdk`.
No generated Makefiles or object files belong in Git.

Default outputs (`.elf`, `.map`, `.uf2`):
`build/smp1-usb_cdc-cyw43-none-mqtt0-crash0-test0/mtk3pico_smp1_usb_cdc_wifi.*`

`make test` runs portable C99 motion tests, a full-width representation
reference and focused source checks. It does not move the robot.
`make TEST=1` builds on-target self-tests; `make clean` removes `build/` only.
Run `make help` for profile options. Different profiles have separate outputs.

## Flash and console

Lift both wheels before initial motor tests. Hold BOOTSEL while connecting
USB, then copy the UF2 onto `RPI-RP2`, or use:

```sh
picotool load -v -x build/smp1-usb_cdc-cyw43-none-mqtt0-crash0-test0/mtk3pico_smp1_usb_cdc_wifi.uf2
screen /dev/cu.usbmodemXXXX 115200
```

On Linux the console is usually `/dev/ttyACM0`. In `screen`, exit with
Ctrl-A then K. The motion console accepts `status`; `!` latches a stop.
Movement requires measured calibration, confirmation and explicit arming.
See [Buddy 2](#buddy-2--motion-control) below for commands.

## Buddy 2 — Motion Control

Controls both wheels using encoder feedback for forward, backward and turning moves.
LEFT: M1 motor GP8/9, encoder A/B GP16/17. RIGHT: M2 motor GP10/11,
encoder A/B GP2/3.

From the repository root, build Buddy 2 only:

```sh
make diagnostic -j8
```

Hold BOOTSEL while connecting USB, then copy the image to `RPI-RP2` (macOS):

```sh
cp build/smp1-usb_cdc-none-none-mqtt0-crash0-test1-s2only-s2diag/mtk3pico_smp1_usb_cdc_test_s2only_s2diag.uf2 /Volumes/RPI-RP2/
ls /dev/cu.usbmodem*
screen /dev/cu.usbmodemXXXX 115200
```

Replace `XXXX` with the listed port suffix. Issue console commands individually:

```text
config
encoder
status
config confirm 1
arm
forward 5
```

Check the loaded settings against `motion_defaults.h`; allow a control update
(about 10 ms) between confirmation and arming. After the wheels stop, try
`backward 5`, `left 450` or `right 450`, one at a time (mm and deci-degrees).
`stop` or `!` latches a safety stop; `disarm` cancels and removes permission.
Read `status` only after disarming; `reset` clears eligible faults but never arms.

Start with wheels secured off the ground and motor power isolation within reach;
USB stop delivery is not instantaneous. Every boot is disarmed and requires
confirmation before arming. Run `make test` for host tests or `make verify` for
sanitizers and builds. Teammate API details are in the
[integration reference](docs/MOTION_CONTROL.md).

## Project structure

| Path | Purpose |
| --- | --- |
| `app_program/subsystem1`–`subsystem5` | Team application code and drivers |
| `docs/`, `TEAM.md` | Requirements, hardware, ownership and interfaces |
| `testing/`, `videos/` | Tests, calibration evidence and video links |
| `build_make/`, `config/`, `etc/linker/` | Tracked firmware build inputs |
| `kernel/`, `include/`, `device/`, `lib/`, `tools/` | Kernel and board framework |
| `build/` | Ignored, reproducible firmware and host outputs |

Start with [docs/README.md](docs/README.md), [hardware](docs/HARDWARE.md),
[architecture](docs/ARCHITECTURE.md) and [testing](testing/README.md).
Application code follows BARR-C:2018; third-party sources retain their style.

µT-Kernel 3.0 © Ken Sakamura / TRON Forum, distributed under T-License 2.2.
This dual-core RP2040 port derives from the official µT-Kernel 3.0 BSP
(`tron-forum/mtk3_bsp`, branch `pico_rp2040`). Retain individual framework and
Pico SDK copyright and license notices when redistributing their sources.

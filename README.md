# INF2004 autonomous robotic car

Raspberry Pi Pico W + Cytron Robo Pico, C on dual-core µT-Kernel 3.0.
The [course template](https://github.com/sirfonzie/INF2004_ProjectTemplate)
provides the kernel, USB console and Wi-Fi port. Application subsystems live
in `app_program/`; Buddy 2's integration guide is [Motion control](docs/MOTION_CONTROL.md).

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
See the [bench procedure](docs/MOTION_CONTROL.md#bench-procedure).

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

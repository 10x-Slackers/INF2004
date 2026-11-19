# INF2004 Embedded Systems Programming Project

Autonomous robotic car on µT-Kernel 3.0 (RP2040 dual-core SMP port) for the Raspberry Pi Pico W.

- [AGENTS.md](AGENTS.md): how to work in this repository (ownership, conventions, tests)
- [docs/](docs/README.md): requirements, architecture, hardware and each subsystem
- [TEAM.md](TEAM.md): who owns which subsystem, pin, priority and interface
- [testing/](testing/README.md): test records

## Getting started (Dev Container)

Needs [Git](https://github.com/git-guides/install-git), [Docker or Podman](https://docs.docker.com/engine/install/) and [VS Code](https://code.visualstudio.com/download) with the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension. Do not use GitHub Desktop; work through the Dev Container.

1. Clone and open the repository:

   ```sh
   git clone git@github.com:10x-Slackers/INF2004.git
   code INF2004/
   ```

2. Choose "Reopen in Container". The container installs the compiler and `picotool` and clones the [Pico SDK](https://github.com/raspberrypi/pico-sdk) into `.pico-sdk/`, with `PICO_SDK_PATH` set.
3. Run `pre-commit install` once.
4. Build with `make -C build_make -j$(nproc)` (VS Code task `Build`) and flash with `picotool load -v -x build_make/mtk3pico_smp1_usb_cdc_wifi.uf2` (task `Flash`) while the Pico W is in BOOTSEL mode. More in [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md).

## Without the Dev Container

Install `git`, `make`, a host C++ compiler and the Arm GNU Toolchain (`arm-none-eabi-gcc`; on Ubuntu `sudo apt install -y git make g++ gcc-arm-none-eabi`, otherwise from [Arm's downloads](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads), AArch32 bare-metal). Then get the Pico SDK, which does not include the compiler:

```sh
git clone --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git ~/pico/pico-sdk
git -C ~/pico/pico-sdk submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip
export PICO_SDK_PATH="$HOME/pico/pico-sdk"   # add to ~/.bashrc or ~/.zshrc
```

`make` names what is missing if `arm-none-eabi-gcc` is not on the `PATH` or `PICO_SDK_PATH` is unset. Without `picotool`, hold BOOTSEL while plugging in the Pico W and copy the `.uf2` onto the `RPI-RP2` drive.

## Build options

`make` uses the defaults; options go after it, in any order, and are not remembered between builds. Each changes the image name, so you can tell which build you are flashing, and switching needs no `make clean`.

| Option | Values (**default**) | What it does | C macro |
|---|---|---|---|
| `SMP` | `0`, **`1`** | `1` runs the kernel on both cores; `0` on core 0 only | `TK_SUPPORT_SMP` |
| `CONSOLE` | **`usb_cdc`**, `uart` | Where `tm_printf` goes: the USB cable, or UART0 only (GP0 TX, GP1 RX) | `TM_CONSOLE_USB_CDC` |
| `WIFI` | **`cyw43`**, `none` | Power up the radio and its driver task, or leave it off (then no Pico SDK is needed) | `TM_WIFI_CYW43` |
| `TEST` | **`0`**, `1` | `1` compiles every `subsystemN_test.c` and runs the tests at power-up | `APP_TEST` |
| `CRASH_REPORT` | **`0`**, `1` | `1` also copies the console to UART0 at 115200 baud, so the last messages and the kernel's crash report survive a crash (USB output is lost then). Uses GP0/GP1, which carry the HC-SR04: unplug it first | `TM_CONSOLE_UART` |

For example `make -C build_make SMP=0 TEST=1 CRASH_REPORT=1 -j$(nproc)` gives `mtk3pico_smp0_usb_cdc_wifi_crash_test.uf2`. `NET=lwip MQTT=1` builds the MQTT telemetry ([docs/DEVELOPMENT.md](docs/DEVELOPMENT.md#building-with-mqtt)). The options are handled in `build_make/pico_rp2040.mk`; kernel limits such as the number of tasks are `CNF_MAX_...` in `config/config.h`.

## Template upstream

Based on the course's [INF2004_ProjectTemplate](https://github.com/sirfonzie/INF2004_ProjectTemplate). To pull in template updates:

```sh
git remote add upstream https://github.com/sirfonzie/INF2004_ProjectTemplate  # once
git pull upstream main
```

---

*micro T-Kernel 3.0 © Ken Sakamura / TRON Forum, distributed under the T-License 2.2. This dual-core RP2040 port is derived from the official μT-Kernel 3.0 BSP (`tron-forum/mtk3_bsp`, branch `pico_rp2040`).*

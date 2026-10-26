# Development Guide

How to build, flash and add code. Environment setup is in the [root README](../README.md#native-build-macos-or-linux). Build options (`SMP`, `CONSOLE`, `WIFI`, `TEST`, `CRASH_REPORT`, `NET`, `MQTT`)
are listed by `make help` and defined in `build_make/pico_rp2040.mk`.
`make test` runs host tests; `make TEST=1` builds the on-target test firmware.

## Build and test

Use the [root README](../README.md#native-build-macos-or-linux) for native
setup, firmware output paths, flashing and USB console commands. `make help`
lists profile options; the shared VS Code tasks call the same root Makefile.
Run both `make` and `make TEST=1` without new warnings after changes, and
`make test` for Buddy 2 host checks.

### Building with MQTT

1. Copy the credentials template and fill in the network and broker:

   ```sh
   cp config/wifi_secrets.example.h config/wifi_secrets.h
   ```

2. Build with `make NET=lwip MQTT=1 -j8`.

`config/wifi_secrets.h` is gitignored. Never commit it.

## Writing a task

`usermain()` in `app_program/app_main.c` is the entry point. It calls every `subsystemN_init()` (create kernel objects), then every `subsystemN_start()` (start tasks), then sleeps forever; it must not return.

Each subsystem lives in `app_program/subsystemN/`, with drivers named
`drv_<name>.c` / `.h`. The build discovers application C files at the root
and one subdirectory level automatically. Expose only the interfaces other
subsystems need; Buddy 2's public API is `motion_control.h`.

Start tasks with `app_start_task()` from `app.h`. Use priorities and hardware
ownership from [TEAM.md](../TEAM.md), not copied example values. Affinity is
`TP_PRC1` (core 0), `TP_PRC2` (core 1), or `0` for either core. A lower
priority number runs first.

### Things to know

- **Stacks are `TASK_STACK_SIZE` bytes** (`app.h`). Raise it for a task with deep call chains or large locals.
- **`tk_dly_tsk` is a relative delay.** The period becomes the delay plus the work time, so it drifts. For a fixed rate, sleep until the next absolute deadline or use a cyclic handler.
- **The tick is 10 ms** (`CNF_TIMER_PERIOD` in `config/config.h`). Delays are rounded to it.
- **There is no FPU.** Floating point is software-emulated; keep it out of fast loops.
- **Processor 1 is physical core 0** and runs the kernel's USB console and Wi-Fi service tasks.
- **Kernel object counts are capped** in `config/config.h`. The budget is tracked in `TEAM.md` section 6.

### Passing data between tasks

Record each synchronized interface in `TEAM.md` section 5. Use the kernel
objects below, or the documented motion APIs; do not share globals unguarded.

| Need                                 | Object            | Calls                      |
| ------------------------------------ | ----------------- | -------------------------- |
| Send a record to another task        | Message buffer    | `tk_snd_mbf`, `tk_rcv_mbf` |
| Wake a task on an event or condition | Event flag        | `tk_set_flg`, `tk_wai_flg` |
| Guard a shared value                 | Mutex             | `tk_loc_mtx`, `tk_unl_mtx` |
| Count or throttle                    | Semaphore         | `tk_sig_sem`, `tk_wai_sem` |
| Fixed-size blocks without a heap     | Fixed memory pool | `tk_get_mpf`, `tk_rel_mpf` |
| Run code at a fixed period           | Cyclic handler    | `tk_cre_cyc`, `tk_sta_cyc` |
| React to a pin edge                  | Interrupt handler | `tk_def_int`               |

Motion-specific locking, commands and data contracts are in
[MOTION_CONTROL.md](MOTION_CONTROL.md). The full API is in the [µT-Kernel 3.0 specification](https://tron-forum.github.io/mtkernel_3/). Both cores run tasks at once: `volatile` alone is not synchronization.

## Using hardware

`lib/libbsp` provides the GPIO and PWM helpers, declared in `include/bsp/sysdepend/cpu/rp2040/libbsp.h`:

| Function                      | Purpose                                      |
| ----------------------------- | -------------------------------------------- |
| `gpio_set_pin(no, mode)`      | Make a pin `GPIO_MODE_OUT` or `GPIO_MODE_IN` |
| `gpio_set_val(no, val)`       | Drive an output                              |
| `gpio_get_val(no)`            | Read an input                                |
| `pwm_set_pin(no)`             | Route a pin to its PWM slice                 |
| `pwm_set_wrap(no, wrap)`      | Set the counter wrap (period)                |
| `pwm_set_cc(no, cc)`          | Set the compare value (duty)                 |
| `pwm_set_enabled(no, enable)` | Start or stop the slice                      |

The kernel's I2C and ADC drivers in `device/` are switched on in `config/config_device.h`; see [boot-time pin setup](HARDWARE.md#boot-time-pin-setup) for the pins they claim. Each peripheral is driven by exactly one owning task (`TEAM.md` section 3).

The build does not use the Pico SDK's CMake system or runtime. It compiles selected SDK sources (TinyUSB, the CYW43 driver, lwIP) into the make build, so an arbitrary `pico_*` or `hardware_*` SDK call will not link unless its source is added to `build_make/pico_rp2040.mk`.

## Console output

Print with `tm_printf`:

```c
tm_printf((UB *)"[line] pos=%d\n", pos);
```

- `tm_printf` is atomic per call, not per line. Build a whole line with `tm_sprintf` into a buffer and emit it with one `tm_printf`, or another task's output can land in the middle.
- Over USB the console is a ring buffer drained by the kernel's USB task. Nothing is visible until the host has opened the port, and output written just before a crash is lost; build with `CRASH_REPORT=1` to see it on UART0.
- Do not print from interrupt handlers or tight control loops.

## Networking

The radio, lwIP and the MQTT client belong to the kernel's Wi-Fi service task. Application tasks only read its status with `cyw43_utk_get_status()` and hand data to it through a kernel object; the rules are in `TEAM.md` section 3 ("WiFi and MQTT are special"). The MQTT client is `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`.

## Conventions

### Engineering posture

This is coursework, not a product. Prefer fewer lines, readability and simplicity over production hardening: skip defensive checks, speculative abstraction and configuration nobody uses yet. See [AGENTS.md](../AGENTS.md).

### Coding standard

Application code follows the [Barr Group Embedded C Coding Standard](https://barrgroup.com/embedded-systems/books/embedded-c-coding-standard) (NFR2). The rules that come up most:

- Braces around every `if`, `else`, `for` and `while` body, each brace on its own line.
- Four-space indentation, no tabs, lines at most 80 characters.
- Fixed-width integer types (`uint8_t`, `int32_t`) rather than bare `int`.
- No magic numbers: name constants.
- Public functions and types carry their module's prefix.
- One header per module, exposing only what other modules need.

The layout rules (braces, indentation, line length, spacing) are applied automatically by clang-format through the pre-commit hook, using the root `.clang-format`. Run `pre-commit install` once after cloning. To format by hand:

```sh
pre-commit run clang-format --files app_program/*/*
```

clang-format only handles layout. Naming, types and magic numbers are still checked by review.

The hook covers `app_program/` only. The kernel sources in `kernel/`, `lib/`, `device/` and `include/` keep their upstream style. Do not reformat them.

### Design discipline

- Respect [hardware ownership](ARCHITECTURE.md#hardware-ownership). If your task needs another task's device, ask for the data through an interface.
- Only Wheel Motion Control writes motor pins.
- Claim pins, peripherals, priorities and interfaces in `TEAM.md` before coding them.
- Size application data to its range and record each field in [DATA_REPRESENTATION.md](DATA_REPRESENTATION.md).
- Automate unit tests in `subsystemN_test.c` with `TEST_CHECK` and record every test in `testing/` (see `testing/README.md`). Measurements and calibration values are assessed evidence (NFR8).

## Troubleshooting

| Symptom                               | Likely cause                                                                                                       |
| ------------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| `RPI-RP2` drive does not appear       | BOOTSEL was not held before power, or the cable is charge-only                                                     |
| `PICO_SDK_PATH is not set`            | The Dev Container did not clone the SDK. Delete `.pico-sdk/` and rebuild the Dev Container                         |
| Build succeeds but there is no `.uf2` | The host `elf2uf2` tool failed to compile. Look for a `g++` error earlier in the output                            |
| Nothing on the console                | The port has not been open long enough for the host to enumerate. Wait a second or two                             |
| Console stops mid-run                 | The firmware crashed or froze before the USB task could send the output. Rebuild with `CRASH_REPORT=1` (README §3) |

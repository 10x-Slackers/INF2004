# Development Guide

How to build, flash and add code. Environment setup is in the [root README](../README.md#getting-started-dev-container). Every build option (`SMP`, `CONSOLE`, `WIFI`, `TEST`, `CRASH_REPORT`, `NET`, `MQTT`) is explained in section 3 of the same README.

## Build, flash, console

Run from the repository root, inside the Dev Container.

| Step       | Command                                                                                                      |
| ---------- | ------------------------------------------------------------------------------------------------------------ |
| Build      | `make -C build_make -j$(nproc)`                                                                              |
| Test build | `make -C build_make TEST=1 -j$(nproc)`                                                                       |
| Clean      | `make -C build_make clean`                                                                                   |
| Flash      | Hold **BOOTSEL** while plugging in USB, then `picotool load -v -x build_make/mtk3pico_smp1_usb_cdc_wifi.uf2` |
| Console    | `screen /dev/ttyACM0 115200`                                                                                 |

The same steps exist as the VS Code tasks `Build`, `Build (tests)`, `Build (crash report)`, `Clean` and `Flash` (`Flash` runs the default `Build` first).

Check that both the normal and the `TEST=1` build still succeed without new warnings after every change.

### Building with MQTT

1. Copy the credentials template and fill in the network and broker:

   ```sh
   cp config/wifi_secrets.example.h config/wifi_secrets.h
   ```

2. Build with `make -C build_make NET=lwip MQTT=1 -j$(nproc)`.

`config/wifi_secrets.h` is gitignored. Never commit it.

## Repository layout

| Path                             | Contents                                                                 |
| -------------------------------- | ------------------------------------------------------------------------ |
| `app_program/`                   | **Application code.** `app_main.c`, `app.h` and one folder per subsystem |
| `TEAM.md`                        | Owners, pins, peripherals, task priorities, interfaces, object budget    |
| `testing/`                       | Test logs per subsystem, integration and system level                    |
| `videos/`                        | Links to test and demo videos                                            |
| `config/`                        | Kernel configuration (`config.h`, `config_device.h`) and `wifi_secrets`  |
| `build_make/`                    | Makefiles and build output                                               |
| `device/`                        | Kernel device drivers: serial, I2C, ADC                                  |
| `include/`                       | Kernel, BSP and T-Monitor headers                                        |
| `kernel/`                        | µT-Kernel itself and RP2040 start-up                                     |
| `lib/libbsp/`                    | Board helpers: GPIO and PWM                                              |
| `lib/libtm/`                     | Console (`tm_printf`)                                                    |
| `lib/libwifi/`, `lib/liblwip/`   | CYW43439 radio service, lwIP port and MQTT client                        |
| `docs/`                          | This documentation                                                       |

Every `.c` file in `app_program/` and in its subfolders is compiled automatically. No makefile edit is needed.

## Writing a task

`usermain()` in `app_program/app_main.c` is the entry point. It calls every `subsystemN_init()` (create kernel objects), then every `subsystemN_start()` (start tasks), then sleeps forever; it must not return.

Each subsystem lives in `app_program/subsystemN/`. `subsystemN.h` is its public interface (object IDs, message formats, priorities); everything else stays `LOCAL` in its `.c` files. Device drivers go in the owning subsystem's folder as `drv_<name>.c` / `drv_<name>.h`. A task is started with `app_start_task()` from `app.h`:

```c
#define S2_PRI_RANGE 14 /* from the subsystem's band in TEAM.md */
#define RANGE_MS     60

LOCAL void range_task(INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;

    while (1)
    {
        /* read sensor, publish result */
        tk_dly_tsk(RANGE_MS);
    }
}

EXPORT ER subsystem2_start(void)
{
    ID tid = app_start_task(range_task, S2_PRI_RANGE, 0, "range_task");
    return (tid < E_OK) ? tid : E_OK;
}
```

The third argument pins the task to a core: `TP_PRC1` (core 0), `TP_PRC2` (core 1) or `0` for any. Take each task's priority from the table in [ARCHITECTURE.md](ARCHITECTURE.md#tasks), inside the subsystem's band in `TEAM.md`. A lower number is a higher priority.

The subsystems in `app_program/` are the template's worked examples. Together they use every kernel object below; replace them with the car's subsystems.

### Things to know

- **Stacks are `TASK_STACK_SIZE` bytes** (`app.h`). Raise it for a task with deep call chains or large locals.
- **`tk_dly_tsk` is a relative delay.** The period becomes the delay plus the work time, so it drifts. For a fixed rate, sleep until the next absolute deadline or use a cyclic handler.
- **The tick is 10 ms** (`CNF_TIMER_PERIOD` in `config/config.h`). Delays are rounded to it.
- **There is no FPU.** Floating point is software-emulated; keep it out of fast loops.
- **Processor 1 is physical core 0** and runs the kernel's USB console and Wi-Fi service tasks.
- **Kernel object counts are capped** in `config/config.h`. The budget is tracked in `TEAM.md` section 6.

### Passing data between tasks

Use a kernel object for every interface in the architecture and record it in `TEAM.md` section 5; do not share globals unguarded.

| Need                                 | Object            | Calls                      |
| ------------------------------------ | ----------------- | -------------------------- |
| Send a record to another task        | Message buffer    | `tk_snd_mbf`, `tk_rcv_mbf` |
| Wake a task on an event or condition | Event flag        | `tk_set_flg`, `tk_wai_flg` |
| Guard a shared value                 | Mutex             | `tk_loc_mtx`, `tk_unl_mtx` |
| Count or throttle                    | Semaphore         | `tk_sig_sem`, `tk_wai_sem` |
| Fixed-size blocks without a heap     | Fixed memory pool | `tk_get_mpf`, `tk_rel_mpf` |
| Run code at a fixed period           | Cyclic handler    | `tk_cre_cyc`, `tk_sta_cyc` |
| React to a pin edge                  | Interrupt handler | `tk_def_int`               |

The full API is in the [µT-Kernel 3.0 specification](https://tron-forum.github.io/mtkernel_3/). Both cores run tasks at once: `volatile` is not synchronisation between cores, use a kernel object.

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
- Automate unit tests in `subsystemN_test.c` with `TEST_CHECK` and record every test in `testing/` (see `testing/README.md`). Measurements and calibration values are assessed evidence (NFR8).

## Troubleshooting

| Symptom                               | Likely cause                                                                                                       |
| ------------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| `RPI-RP2` drive does not appear       | BOOTSEL was not held before power, or the cable is charge-only                                                     |
| `PICO_SDK_PATH is not set`            | The Dev Container did not clone the SDK. Delete `.pico-sdk/` and rebuild the Dev Container                         |
| Build succeeds but there is no `.uf2` | The host `elf2uf2` tool failed to compile. Look for a `g++` error earlier in the output                            |
| Nothing on the console                | The port has not been open long enough for the host to enumerate. Wait a second or two                             |
| Console stops mid-run                 | The firmware crashed or froze before the USB task could send the output. Rebuild with `CRASH_REPORT=1` (README §3) |

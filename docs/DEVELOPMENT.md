# Development Guide

How to build, flash and add code. The rules for working in the repository (ownership, coding standard, tests) are in [AGENTS.md](../AGENTS.md); environment set-up and the build options `SMP`, `CONSOLE`, `WIFI`, `TEST` and `CRASH_REPORT` are in the [root README](../README.md).

## Build, flash, console

Run from the repository root, inside the Dev Container or with the SDK and compiler set up as in section 1 of the [root README](../README.md).

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

The broker set-up and the supervisor dashboard are in [TELEMETRY.md](TELEMETRY.md#supervisor-dashboard).

`config/wifi_secrets.h` is gitignored. Never commit it.

## Repository layout

| Path                             | Contents                                                                 |
| -------------------------------- | ------------------------------------------------------------------------ |
| `app_program/`                   | **Application code.** `app_main.c`, `app.h`, one folder per subsystem, and `shared/` (Safety Monitor, Vehicle Controller) |
| `TEAM.md`                        | Owners, pins, peripherals, task priorities, interfaces, object budget    |
| `testing/`                       | Test logs per subsystem and for integration, host tests                  |
| `config/`                        | Kernel configuration (`config.h`, `config_device.h`) and `wifi_secrets`  |
| `build_make/`                    | Makefiles and build output                                               |
| `device/`                        | Kernel device drivers: serial, I2C, ADC                                  |
| `include/`                       | Kernel, BSP and T-Monitor headers                                        |
| `kernel/`                        | µT-Kernel itself and RP2040 start-up                                     |
| `lib/libbsp/`                    | Board helpers: GPIO and PWM                                              |
| `lib/libtm/`                     | Console (`tm_printf`)                                                    |
| `lib/libwifi/`, `lib/liblwip/`   | CYW43439 radio service, lwIP port and MQTT client                        |
| `docs/`                          | This documentation                                                       |
| `tools/`                         | `elf2uf2` (built by make), the MQTT supervisor dashboard (`supervisor/`) and helper scripts |

Every `.c` file in `app_program/` and in its direct subfolders (one level down) is compiled automatically. No makefile edit is needed.

## Writing a task

`usermain()` in `app_program/app_main.c` is the entry point. It calls every `subsystemN_init()` (create kernel objects), then every `subsystemN_start()` (start tasks), then sleeps forever; it must not return.

Each subsystem lives in `app_program/subsystemN/`. `subsystemN.h` is its public interface; everything else stays `static` in its `.c` files. A periodic task is started with `app_start_task()` and woken by `app_start_cyclic()`, both from `app.h`:

```c
#define S5_PRI_RANGE 12 /* From the subsystem's band in TEAM.md. */
#define S5_RANGE_MS  60

static void subsystem5_range_task(INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR); /* Woken every S5_RANGE_MS. */
        /* Read the sensor, publish the result. */
    }
}

ER subsystem5_start(void)
{
    /* FP is the kernel's generic task pointer type. */
    return app_start_cyclic(app_start_task((FP)subsystem5_range_task,
                                           S5_PRI_RANGE, 0, "s5_range"),
                            S5_RANGE_MS);
}
```

The third argument of `app_start_task()` pins the task to a core: `TP_PRC1` (core 0), `TP_PRC2` (core 1) or `0` for any. Priorities come from the subsystem's band in `TEAM.md` section 4, in the task order of [ARCHITECTURE.md](ARCHITECTURE.md#tasks); a lower number is a higher priority.

### Things to know

- **Stacks are `APP_TASK_STACK_SIZE` bytes** (`app.h`). Raise it for a task with deep call chains or large locals.
- **`tk_dly_tsk` is a relative delay.** The period becomes the delay plus the work time, so it drifts. For a fixed rate, use a cyclic handler: `app_start_cyclic()` in `app.h` wakes a task every period.
- **The tick is 10 ms** (`CNF_TIMER_PERIOD` in `config/config.h`). A relative wait also gets one extra tick, so `tk_dly_tsk(10)` waits 10 to 20 ms, about 20 ms in a periodic loop.
- **There is no FPU.** Floating point is software-emulated; keep it out of fast loops.
- **Processor 1 is physical core 0** and runs the kernel's USB console and Wi-Fi service tasks.
- **Kernel object counts are capped** in `config/config.h`. The budget is tracked in `TEAM.md` section 6.

### Passing data between tasks

Most interfaces are functions in the provider's header that copy a record under the provider's mutex, so they are safe from any task on either core; record each in `TEAM.md` section 5. Do not share globals unguarded. The kernel objects available:

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

The radio, lwIP and the MQTT client belong to the kernel's Wi-Fi service task. Application tasks only read its status with `cyw43_utk_get_status()` and hand data to it through a kernel object; the rules are in `TEAM.md` section 3 ("Wi-Fi and MQTT"). The MQTT client is `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`.

## Troubleshooting

| Symptom                               | Likely cause                                                                                                       |
| ------------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| `RPI-RP2` drive does not appear       | BOOTSEL was not held before power, or the cable is charge-only                                                     |
| `PICO_SDK_PATH is not set`            | The Dev Container did not clone the SDK. Delete `.pico-sdk/` and rebuild the Dev Container                         |
| Build succeeds but there is no `.uf2` | The host `elf2uf2` tool failed to compile. Look for a `g++` error earlier in the output                            |
| Nothing on the console                | The port has not been open long enough for the host to enumerate. Wait a second or two                             |
| Console stops mid-run                 | The firmware crashed or froze before the USB task could send the output. Rebuild with `CRASH_REPORT=1` (root README, section 3) |

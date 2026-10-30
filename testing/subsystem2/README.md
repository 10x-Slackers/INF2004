# Buddy 2 motion control testing

Use C on Pico W/Robo Pico and µT-Kernel. The implementation contract and
requirement checklist are in [motion control](../../docs/MOTION_CONTROL.md).
Results, PID tuning and motion accuracy belong in [VERIFICATION](VERIFICATION.md);
unresolved calibration/integration decisions belong in [ISSUES](ISSUES.md).

## Builds

From the repository root with GNU make, ARM GCC and `.pico-sdk` or an exported
`PICO_SDK_PATH`:

```sh
make -j8                             # integrated production
make TEST=1 -j8                      # integrated boot tests
make diagnostic -j8                  # standalone Buddy 2, boot tests and banner
make test                            # host tests only; no hardware access
make verify                          # tests, sanitizers and all build entrypoints
```

The diagnostic image uses the same controller and driver as production,
with only Buddy 2 application startup, USB console and no Wi-Fi:

`build/smp1-usb_cdc-none-none-mqtt0-crash0-test1-s2only-s2diag/mtk3pico_smp1_usb_cdc_test_s2only_s2diag.uf2`

`make verify` overwrites `current.txt`, including tested-source SHA256 values.
Record failures in VERIFICATION before rerunning. The temporary full-width
reference is host/ARM representation evidence, not another firmware source or
capture backend. Test scripts never open serial devices, flash, arm or drive.

## Confirmed wiring

| Wheel | Motor connector | Encoder A/B | Supply |
| --- | --- | --- | --- |
| LEFT | pin 5 M1B, pin 6 M1A | pins 2/3 GP16/17, Grove 4 | pin 1 GND, pin 4 3.3 V |
| RIGHT | pin 5 M2B, pin 6 M2A | pins 2/3 GP2/3, Grove 2 | pin 1 GND, pin 4 3.3 V |

The left encoder is a same-model replacement with shortened leads. Neither
physical forward polarity nor CPR follows from the GPIO map or wire colours.
Do not carry over prior calibration or failure diagnoses.

## Console

Read `status`, `encoder`, `config` **while disarmed**. Status reports normalized
counts, mm, mm/s, faults and timing. Encoder reports raw counts, phases and
invalid totals. Before calibration, only raw counts/phases are meaningful;
zero mm/s is not proof of a stationary wheel when units are unconfigured.

Commands:

- `config circ N`, `track`, `cpr`, `cprl`, `cprr`, `kp`, `ki`, `kd`, `limit`,
  `timeout`, `stall`, `ml`, `mr`, `el`, `er`, `confirm`, each prefixed by
  `config` and followed by its numeric value. `ml/mr` are motor reversal;
  `el/er` encoder reversal. Gains are Q8 (256 = 1); geometry is mm;
  timeout/stall are ms; limits are permille. `cpr` clears per-wheel overrides.
- `arm`, `disarm`, `reset`; `stop` or immediate `!` latches a safety stop.
  Reset never arms or resumes. Every boot is disarmed.
- `speed N` mm/s; `forward N`/`backward N` mm; `left N`/`right N` deci-degrees.
- `motor W D T`: one wheel 0/1, signed duty permille, duration ms. It requires
  explicit arm and confirmed valid configuration; ±200 permille and 1000 ms
  maximum, further bounded by configured limits and normal fault checks.

Configuration report labels `elrev`/`errev` correspond to command keys
`el`/`er`; they are not accepted command names. For left reversal enabled and
right reversal disabled, while disarmed:

```text
config el 1
config er 0
config
```

The report then shows `elrev=1 errev=0`. Editing a setting clears confirmation;
it does not arm or move the car. A full valid configuration and a fresh control
sample are required before a later explicit arm. These edits are RAM-only.

Each accepted command returns `OK`, which means acceptance rather than success.
Inspect final state after completion/disarming. Changes are RAM-only; reviewed
calibration goes in `motion_defaults.h` to survive reboot. Confirmation asserts
operator review, not verified accuracy. The reviewed boot profile is documented
in [MOTION_CONTROL](../../docs/MOTION_CONTROL.md); every boot still requires
explicit confirmation and arming.

## Fresh physical verification procedure

1. Isolate motor power, including any USB-fed rail; lift and secure wheels.
   Record image SHA256, supply/load, wiring and every configuration field.
2. Manually flash the intended image and verify the diagnostic banner, boot
   tests, `armed=0`, zero outputs and rejected movement requests. Flashing
   remains an operator action; no automatic flash or serial commands are used.
3. Read `encoder`; mark and rotate each physical wheel forward and backward
   separately. Only its counts should change. Record raw signs and zero new
   invalids/overflow. Repeat several marked full revolutions each direction;
   calculate per-wheel x4 CPR and spread, rejecting ambiguous trials.
4. Measure loaded wheel circumference and wheel-centre track. Establish motor
   electrical forward using a supervised board/manual polarity test; do not
   infer it from encoder sign. Set reversal flags so normalized forward counts
   and requested physical forward agree. Verify input voltage/output type and
   signal integrity against the actual encoder datasheet or measurements.
5. Enter reviewed geometry, CPR, polarity, software limits and explicitly chosen
   initial PID settings. No numeric gain is prescribed. Only then confirm/arm
   for supervised bounded wheel trials. Stop on discrepancies; do not silently
   raise speed/range limits or disable encoder guards.
6. Tune each wheel under load: record requested/measured speed and external
   reference, response, steady error, settling, overshoot, saturation and drift.
   Gain tuning is a physical experiment, not inferred from host tests.
7. Agree accuracy tolerances; repeat forward/reverse known distances and
   left/right 90°/180° turns using an external ruler/protractor. Log each error,
   mean and spread. Encoder completion alone is not ground-truth accuracy.
8. Validate PWM, reversal brake, bounded pulses, stop/fault response and
   end-to-end physical stopping. Run peer integration and sensing/Wi-Fi load
   checks, measuring jitter, IRQ capture capacity, CPU, stack and power.

Keep fault findings and actual results in VERIFICATION. Hardware failure causes
must be established from fresh evidence. The notification timeout cannot stop
PWM if the CPU/kernel/motion task itself cannot run; use physical power isolation.

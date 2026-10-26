# Buddy 2 motion control

## Ownership and execution

Only Buddy 2 writes GP8/9 (left motor, PWM4 A/B) and GP10/11 (right,
PWM5 A/B). Left encoder A/B is **GP2/GP3**, right is GP16/GP17. GP4/GP5
remain Buddy 4's I2C0. Physical forward polarity must be measured; the
Robo Pico truth table is low/low = brake, high/low = forward, low/high =
reverse, high/high = coast. Software uses brake rather than coast.

PWM is 20 kHz: 125 MHz system clock, divider 1, TOP 6249. A whole-slice
CC write updates both channels; a direction reversal inserts a complete
10 ms braked cycle. Motor outputs start low before PWM is enabled.
Changing the system clock requires changing the PWM period.

Wheel Motion Control runs on core 0 at priority 6, with absolute 10 ms
release targets and tick-based sleeps. Safety has reserved priority 5.
The diagnostics console is priority 22, also core 0. USB and radio kernel
service priorities remain 3 and 4. The kernel tick is 10 ms; these are
requested periods, not measured deadlines. No heap use or console output
occurs in the control loop. Two 2048-byte task stacks are allocated at boot.

SIO hardware locks 3 (application state) and 4 (encoder snapshot) are
reserved. Task dispatch is disabled while lock 3 is held; encoder interrupts remain
enabled. Lock 4 masks local interrupts only across encoder updates/copies.
Both locks have memory barriers. Lock order is 3 then 4; the encoder ISR takes only 4. Public APIs
serialize short operations and never wait for motion completion. Their
lock acquisition can briefly spin; they are task-context APIs, not ISR APIs.
Measure the maximum interrupt blackout and encoder capture rate under load
before accepting the chosen encoder/speed calibration. The control tick
includes 64-bit integer division, so host timings cannot establish this limit.

IRQ13 (IO_IRQ_BANK0) has one core-0 dispatcher. It decodes four-edge
quadrature, saturates signed 24-bit counts, and faults on skipped phases or
counter exhaustion. Another GPIO user registers its pin mask and callback
with `drv_motion_gpio_register()` during its core-0 initialisation, before task start, sets
up its input pins, and receives an acknowledged edge pin mask. One additional
owner is supported (Buddy 5). The callback reads levels/timestamps itself;
it must be short, never sleep, and never acquire lock 3. Do not replace
IRQ13, enable these pins on core 1, or register a second competing handler.
Registration is part of single-threaded core-0 startup (including Buddy 5
initialisation after Buddy 2), not a runtime API. Pending edges on the newly
registered pins are cleared before enabling them; existing encoder enables
are preserved. Initialize callback state before registering it.

## Public API

Include `app_program/subsystem2/motion_control.h`. All command calls return
immediately after accepting/rejecting the request; no command queue exists.
There is one active manoeuvre, and a second is rejected. Boolean results
must be checked. Public packed configuration members must be populated only
with values in the representation table, before assigning the bit-fields.

| Function | Contract |
| --- | --- |
| `motion_control_configure(&config)` | Disarmed only; dimensions, gains, polarity, limits, confirmation; a fresh tick is required before arm |
| `motion_control_get_config(&config)` | Coherent calibration copy |
| `motion_control_set_speed(speed)` | 10–1000 mm/s; idle only; applies to the next command |
| `motion_control_arm()` | Requires valid, confirmed calibration, fresh status and no faults |
| `motion_control_move_forward(mm)` / `move_backward(mm)` | 1–10000 mm, also bounded by the signed encoder capacity |
| `motion_control_turn_left(ddeg)` / `turn_right(ddeg)` | 1–3600 deci-degrees; counter-rotating wheel arcs |
| `motion_control_stop()` | Cancel ordinary motion; preserves any existing fault latch |
| `motion_control_disarm()` | Cancel and remove permission to move |
| `motion_control_safety_stop(mask)` | Cancel, disarm, OR faults with the SAFETY bit; ordinary commands cannot clear it |
| `motion_control_reset_faults()` | Disarmed, fresh inputs, no encoder overflow/init failure; still requires a new arm and command |
| `motion_control_get_status(&status)` | Coherent counts, wheel speed, wheel/mean distances, state and timing |
| `motion_control_test_motor(wheel,duty,ms)` | Armed bench pulse, wheel 0/1, signed ±1–200 permille, 1–1000 ms; duty limit still applies |
| `motion_control_pack()` / `unpack()` | Explicit 36-byte integration representation; no compiler struct on a wire |

`motion_control_init()` is startup-only, invoked by `subsystem2_init()`;
reinitialising a running subsystem is unsupported. `tick()` and
`record_execution()` belong exclusively to the Wheel Motion Control task.
Output braking takes effect at its next release, plus at most one PWM
period. A stop invalidates the only active command under the same lock as
motor updates; there is no pending command that can resume after reset.
Measure actual end-to-end latency, including scheduler and USB/network load.
The immediate `!` receive hook and RX overflow also discard older queued
console commands, so a previously queued arm/reset cannot undo that stop.
This is the motion stop endpoint; the team's Safety Monitor must still call
it and arbitrate system-level faults and mission restart.

Status values: IDLE 0, RUNNING 1, COMPLETE 2, CANCELLED 3, FAILED 4.
Commands: NONE 0, FORWARD 1, BACKWARD 2, LEFT 3, RIGHT 4, BENCH 5.
A completed command remains visible until another command or stop.

Fault bits, independent and OR-combinable: stall 0, timeout 1, encoder 2,
control timing 3, numeric range 4, external safety stop 5, initialisation 6,
stale odometry 7. A fault disarms. Count overflow and initialisation failure
require reboot/revalidation; invalid-transition history can be acknowledged
by reset after correcting the wiring. Future events reassert the latch.

## Control and odometry

Counts represent all four quadrature edges per wheel revolution. Count zero
is boot; no count reset is performed while moving. Calibration polarity
normalises each wheel so forward counts and distance are positive. Distances
are computed from total counts, preserving fractional travel across samples;
reported millimetres truncate towards zero, rather than accumulating rounded
deltas. Average odometry is mean signed wheel distance; it is not path length
through a turn. Buddy 3 uses count deltas for precise barcode distances.
Buddy 4 uses signed wheel travel and track width to derive turn rate.

Speed is count delta times circumference divided by actual elapsed time.
Measurements outside ±2000 mm/s or ±8388608 mm fault rather than truncate.
Very low-resolution encoders can quantise a single edge into a large speed
spike: validate counts/revolution, sample noise and stall timeout on the car.
Consumers must reject status older than 30000 us using unsigned subtraction;
no valid zero reading should be inferred from a missing update. Timestamp
wrap is supported for intervals shorter than half the 32-bit timer range.

Separate wheel PID states use Q8 gains: output =
`(Kp*error + Ki*integral + Kd*(error-previous_error))/256` in permille.
Ki/Kd are **per 10 ms sample**, not per second. Integral is limited to
±65535 error-samples; integration freezes when it would drive saturation
further. Zero targets clear PID memory and brake that wheel. Gains are
shared calibration values but each wheel's feedback and state are independent.
Encoder progress mismatch adjusts wheel targets by at most 25% for straight
travel in either direction, and for matched turning arcs. Each wheel brakes
when its own distance completes. There is no acceleration-ramp planner.

Targets are whole encoder edges, rounded upwards. Turns use
`wheel_arc_mm = pi * track_mm * angle_ddeg / 3600`, pi approximated by
355/113. Bounds and 64-bit intermediates prevent arithmetic overflow;
unrepresentable moves are rejected. Opposite travel exceeding a two-mm
allowance plus two edges faults. Persistent commanded duty without edges
faults at the configured stall interval. Every move has a timeout. A running
control period over 15 ms faults; over 30 ms is classified as stale. An
execution lasting at least 10 ms also latches a timing fault. Execution instrumentation brakes immediately within the owning task when
it raises an overrun fault.

## Binary contract

Version 1 is identified by the enclosing topic/interface, not inferred from
struct size. Big-endian bit stream, first field starts at bit 0 (MSB of byte
0); widths in order:

| Field | Width | Bit offset |
| --- | --- | --- |
| state / armed / faults / command | 3 / 1 / 8 / 3 | 0 / 3 / 4 / 12 |
| left_count / right_count | 24 / 24 | 15 / 39 |
| left_mm / right_mm / average_mm | 24 / 24 / 24 | 63 / 87 / 111 |
| left_mm_s / right_mm_s | 12 / 12 | 135 / 147 |
| timestamp_us | 32 | 159 |
| max_execution_us / max_period_us / missed_periods | 32 each | 191 / 223 / 255 |
| reserved zero | 1 | 287 |

Counts and millimetres encode signed value + 8388608. Speeds encode signed
value + 2048; only −2000..2000 is valid. Helpers decode these offsets.
Unpack requires exactly 36 bytes, valid state/command/speed and a zero
reserved bit; failure leaves the output unchanged. The caller supplies
36-byte buffers. RAM structs are compiler-local and use unsigned bit-fields.
I5 consumers copy status independently; no single-consumer queue can steal
another consumer's odometry. Buddy 1 can serialize that snapshot without
accessing hardware or implementing MQTT here.

## Bench procedure

1. Build `make -j8`; flash the default UF2. Run `make test` on the host.
   For safe on-target checks build `make TEST=1 -j8` and use the `_test.uf2`
   in the corresponding `test1` profile directory. These tests never arm.
2. Lift wheels, secure the chassis, and keep the board power switch reachable.
   Confirm GPIO wiring and encoder supply/output voltage (3.3 V). Leave
   GP4/GP5 with the IMU. Use USB-only console; UART/crash reporting takes
   Buddy 5's GP0/GP1. Do not run an autonomous mission during bench pulses.
3. Open the USB terminal. `status` reports faults, signed counts, mm, wheel
   speed, timestamp, maximum tick execution/period and missed periods.
   Rotate each wheel manually through measured revolutions and obtain
   **four-edge counts per revolution** and encoder sign. Measure loaded
   wheel circumference and wheel-centre track width externally.
4. While disarmed, enter `config circ N`, `config track N`, `config cpr N`.
   Set `config ml 0|1`, `mr`, `el`, `er` for motor/encoder reversal. Optional
   keys: `kp`, `ki`, `kd` (Q8 integers 0–4095, Kp nonzero), `limit`
   (1–1000 permille), `timeout` (100–60000 ms), `stall` (100–10000 ms).
   `speed N` selects 10–1000 mm/s. Every config edit clears confirmation;
   finish with `config confirm 1`. Values are RAM-only and reset at boot.
5. `arm`, then `motor 0 100 200` or `motor 1 100 200` gives one low-duty
   200 ms pulse. Confirm direction/count sign with a suitably low power
   setting. Use `!` or `stop` for a latched stop. Correct polarity while
   disarmed; `reset`, reconfirm and arm before retrying. Increase duty only
   enough to overcome stiction. Stop testing on an encoder or range fault.
6. On clear level ground start with `forward 50`, `backward 50`, `left 300`,
   `right 300` (angles are deci-degrees). All commands are bounded, and a
   command in progress rejects new movement. Check `status` for completion.
7. Record PID step response and straight/reverse/turn accuracy externally,
   then tune Q8 gains and limits. Test stalls with a controlled bench method,
   faults, stop latency, timer jitter and encoder loss under Wi-Fi/IMU load.
   Capture logs and measurements in `testing/subsystem2/TEST_LOG.md`; use
   [videos](../videos/README.md) for demonstrations. Never call a simulation
   or host execution a robot measurement.

## BARR-C review boundaries

New modules use C99, unsigned bit-fields, biased signed storage, fixed-width
arithmetic, bounded loops, Allman braces, four-space indentation, 80-column
lines, module-prefixed functions and guarded headers. `g_`, `p_`, `pp_` mark
globals, pointers and pointers to pointers. Q8, PWM, ISR, IRQ, CC, CPR and
PID mean fixed-point / pulse width / interrupt handler / interrupt number /
compare register / counts per revolution / proportional-integral-derivative.

Explicit integration deviations:

- 4.1.b / 4.2.a: the existing `subsystem2_test.c` naming and per-subsystem
  shared header follow the course's auto-discovered test convention.
- 4.3.a: `ctrl_math.c` also implements the driver's pure quadrature/output
  planning helpers so host and firmware exercise the same code.
- 5.2: `unsigned int` is used only for ISO C bit-fields, as required for
  portable C99 bit-fields; arithmetic interfaces use fixed-width integers.
- 6.1.i / 7.1: `tm_console_receive_byte` is the generic USB port hook;
  `_task`/`_isr` callbacks use kernel `INT`, `UINT`, `ER`, `ID` and `FP`
  signatures. Fixed interrupt addresses belong to the RP2040 port.
- 1.6: conversions at biased-field assignments are justified by their
  range checks and the field table; task callback casts adapt the kernel's
  unprototyped `FP`, and console format casts adapt its `UB` string API.
- 6.2: short early returns reject commands and malformed packets. The
  integration parser/control step can exceed the preferred printed-page
  function length; splitting trivial branches would obscure the state flow.

Focused source checks are not certification of every standard rule. Kernel,
SDK and other buddies' code retain their existing conventions; in particular
Buddy 4's existing signed bit-fields are outside the new-code review scope.

# Buddy 2 integration reference

Operating instructions, wiring and boot settings are in the
[root README](../README.md#buddy-2--motion-control). This reference retains the
requirement mapping and detailed contract used by teammates.

## Requirements and boundaries

Sources: *Project Write-up.pdf*, pp. 1–3 and 5–6; *Project Briefing.pdf*,
slides 3 and 5; *Change Requirement 1.docx*; BARR-C:2018. The official brief
lists example API names, not exact C signatures, units, sampling periods,
numeric accuracy tolerances or calibration constants. The C contract below
preserves the team's existing interfaces. FR/NFR identifiers and the task,
lock and safety policies are team design decisions, not extra course wording.

| Source requirement | Implementation | Verification / deliverable |
| --- | --- | --- |
| Motor driver integration | `drv_motion.c`: M1 GP8/9, M2 GP10/11; bounded signed PWM | Actual output-register fixture; physical waveform/polarity check |
| Encoder integration | Same driver: left GP16/17, right GP2/3; four-edge quadrature IRQ | All transitions, invalid jumps, saturation, startup phases and W1C fixture; manual CPR check |
| Speed and distance estimation | `motion_control.c`: independent wheel CPR, mm and mm/s | Synthetic conversion/boundary/wrap tests; external distance/speed comparison |
| PID speed control | `ctrl_math.c`: independent wheel state, Q8 gains, elapsed-time scaling, anti-windup | C tests; loaded PID tuning report |
| Straight-line correction | Bounded correction from difference in physical wheel progress, forward and reverse | Unequal CPR/progress tests; track drift measurement |
| Encoder-based turns | Counter-rotating arcs for left/right angles, including U-turn | Turn signs, whole-edge goals/completion; external angle evaluation |
| Reusable motion library | Asynchronous forward/backward/left/right/stop API below | Acceptance, completion, rejection and cancellation tests |
| Integration | I3/I4 commands/status, I5 odometry, I11 stop; telemetry packer | Host contract tests plus joint peer tests |
| Stable motion, resource efficiency | Bounded duty, stall/timeout/range/timing faults; fixed storage | Host safety tests; target load/stack/CPU/power/stop measurements |
| Minimum-bit application fields | Unsigned bit-fields, combined fault mask, locked shared copies | [Field table](DATA_REPRESENTATION.md), RAM/message comparison and correctness evidence |
| Compact MQTT application data | Explicit 36-byte binary status packet for Buddy 1 | Known-bit and roundtrip tests; Buddy 1 owns MQTT transport |
| Calibration, tuning and accuracy reports | Measured geometry, CPR, polarity and gains | Coursework calibration, PID tuning and accuracy deliverables |

## Hardware and execution

The [Cytron motor truth table](https://www.farnell.com/datasheets/4381243.pdf)
is low/low brake, high/low one electrical direction, low/high the opposite,
high/high coast. Software uses low/low to brake. At the board's 125 MHz clock,
PWM divider 1 and TOP 6249 give 20 kHz. Each slice's A/B compare pair is written
in one register write. Opposite nonzero duty inserts a braked control update;
zero commands retain direction history. PWM zero applies at the next wrap
(nominally within 50 microseconds); it is not motor power isolation.

The GPIO dispatcher uses IRQ13 on core 0, edge flags and one GPIO_IN read per
wheel pair. A phase is A*2+B. Raw positive sequence is 00→01→11→10→00;
reverse gives negative counts. A two-bit jump is ambiguous: discard its delta,
increment invalid counters and resynchronize. Never reconstruct skipped edges.
GPIO flags can coalesce; software tests cannot establish capture bandwidth or
electrical signal quality. See the [RP2040 GPIO documentation](https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#hardware_gpio).
Encoder input electrical characteristics/pulls require the actual motor datasheet
or measurement; the driver uses the board input setup without guessing them.

The motion task runs at priority 6/core 0; the console at 22/core 0. A 10 ms
cyclic handler only sets a coalescing event flag. The task has a bounded 10 ms
notification wait; wait failure stops and terminates it. µT-Kernel's 10 ms tick
rounds the initial release and relative timeouts. Minimum accepted sample spacing
is 5 ms; shorter notifications preserve PID/count/time baselines, while encoder
faults are still checked. The actual locked count-sample time drives speed,
PID and freshness, rather than a potentially preempted task-entry timestamp.

Task state uses hardware lock 3 with dispatch disabled; encoder copies use
lock 4 with local interrupts masked. Both have memory barriers; lock order is
3 then 4, and ISR13 never acquires 3. Short public calls may spin for the lock;
they are task-context only. No heap allocation or printing occurs in control
or ISR paths. The two 2048-byte stacks are allocated during startup.

Buddy 5 may register one additional GPIO callback with
`drv_motion_gpio_register(uint32_t pins, drv_motion_gpio_callback_t p_callback_isr)`
on core 0 during startup, before tasks run. Configure its inputs and callback
state first. Do not replace IRQ13. The callback receives acknowledged edge pins,
must remain short and must not block or acquire lock 3. PWM4/5 belong to Buddy 2;
PWM6/GP12 remains available for the servo.

## Public contract

Include `motion_control.h`. All command acceptance functions return immediately:
`true` means accepted, not completed. Poll a coherent status snapshot for
RUNNING/COMPLETE/FAILED/CANCELLED. One command is active; no pending queue exists.
Populate bit-fields only after validating native input ranges to prevent
truncation before `configure()`. Each consumer owns its copied snapshot.

```c
void motion_control_init(void);  /* Startup only, normally via subsystem2_init. */
bool motion_control_configure(const motion_control_config_t * p_config);
void motion_control_get_config(motion_control_config_t * p_config);
bool motion_control_set_speed(uint32_t speed_mm_s);
bool motion_control_arm(void);
void motion_control_disarm(void);
void motion_control_stop(void);
void motion_control_safety_stop(uint32_t faults);
bool motion_control_reset_faults(void);
bool motion_control_move_forward(uint32_t distance_mm);
bool motion_control_move_backward(uint32_t distance_mm);
bool motion_control_turn_left(uint32_t angle_ddeg);
bool motion_control_turn_right(uint32_t angle_ddeg);
bool motion_control_test_motor(uint32_t wheel, int32_t duty,
                               uint32_t duration_ms);
void motion_control_get_status(motion_control_status_t * p_status);
void motion_control_pack(const motion_control_status_t * p_status,
                         uint8_t * p_packet);
bool motion_control_unpack(motion_control_status_t * p_status,
                           const uint8_t * p_packet, uint32_t length);
int32_t motion_control_decode_count(uint32_t encoded);
int32_t motion_control_decode_speed(uint32_t encoded);
```

- Distances: integer mm, 1–10000; angles: deci-degrees, 1–3600 (900 = 90°).
  Commands also must fit remaining signed count capacity. Rejected commands
  leave the active command unchanged.
- Speed: 10–1000 mm/s, set while idle; correction limited to 25% of speed.
- Bench: wheel 0 left / 1 right, signed duty ±1–200 permille, 1–1000 ms;
  normal arm/configuration, output limit and fault guards still apply.
- Configure only disarmed. Unconfirmed fields may be staged individually.
  Confirmation requires valid dimensions, CPR, Kp and software limits. A fresh
  sample must follow configuration before arming. Each boot remains disarmed.
- Stop cancels ordinary motion immediately under the output lock. Disarm also
  removes movement permission. Safety stop ORs faults with SAFETY, disarms and
  cancels. Reset requires fresh data, disarmed state and no sticky encoder
  overflow or init failure; it never arms or resumes a command.

`tick(uint32_t now_us)` and `record_execution(uint32_t elapsed_us)` are private
motion-task integration hooks despite their public prototypes. The entry-time
argument to tick is retained for compatibility; the driver supplies the sample
time. Do not call these from teammate tasks or ISR callbacks.

Fault bits: stall 0, timeout 1, encoder 2, control timing 3, numeric range 4,
external safety 5, init 6, stale odometry 7. States are IDLE 0, RUNNING 1,
COMPLETE 2, CANCELLED 3, FAILED 4; commands NONE 0, FORWARD 1, BACKWARD 2,
LEFT 3, RIGHT 4, BENCH 5. Faults disarm and write zero immediately in task
context. New invalid transitions fault while armed, including idle/bench;
manual disarmed invalids are counted. Overflow remains sticky until reboot.

The following are software policy limits, not official numeric requirements:
control gaps over 15 ms fault during motion (over 30 ms classified stale),
execution at least 10 ms faults, measured speeds outside ±2000 mm/s and
unrepresentable signed 24-bit counts/mm fault. Consumers reject samples older
than 30000 us using unsigned timer subtraction. Timestamps wrap at 32 bits;
comparisons require intervals below half the timer range. Stall/timeout and duty
limits need loaded validation. A stopped CPU or suspended motion task cannot
police its own PWM; physical stopping and end-to-end latency need measurement.

## Calibration and calculations

CPR means **four-edge counts per wheel-output revolution**, including gearbox
ratio. A nonzero left/right CPR overrides shared CPR; zero selects the shared
fallback. Both resolved CPR values must be 4–65535. Config is 28 bytes under
GNU ARM; all consumers must rebuild if its layout changes. Never transmit it
as raw RAM. Use zero or designated initializers.

Signed distance = total normalized counts × circumference / wheel CPR.
Speed = delta counts × circumference × 1000000 / (CPR × elapsed microseconds).
Division truncates toward zero; distances derive from total counts rather than
summing rounded increments. Average mm is signed mean wheel displacement,
not path length through a turn. Counts remain available before calibration;
mm/s and mm must not be treated as valid before calibration or during faults.
Consumers check configuration/confirmation, freshness and faults separately;
the existing packet has no extra validity bit.

Straight goals are ceil(distance × CPR / circumference). In-place turn arcs
are pi × track × angle_ddeg / 3600, with pi approximated by 355/113. Round each
wheel's goal upward independently. Each wheel brakes at its own goal.
Progress correction compares physical travel before whole-mm truncation.
Opposite travel over a two-mm allowance plus two counts faults. Calculations
use 64-bit intermediates and range checks before narrowing.

PID gains are Q8. Integral state is bounded to ±65535 nominal error-samples;
update by trunc(error × elapsed_us / 10000). Derivative is
trunc((error − previous_error) × 10000 / elapsed_us). Duty is
(Kp × error + Ki × integral + Kd × derivative) / 256, bounded by the configured
permille ceiling. Freeze integration when it would worsen output saturation;
zero targets reset state and brake. No floating point or acceleration planner
is used. Tune on the actual loaded robot.

## Binary contract

The enclosing interface/topic identifies version 1. Big-endian bit stream,
bit 0 = MSB of byte 0; never serialize compiler bit-field layout.

| Field | Width | Bit offset |
| --- | --- | --- |
| state / armed / faults / command | 3 / 1 / 8 / 3 | 0 / 3 / 4 / 12 |
| left_count / right_count | 24 / 24 | 15 / 39 |
| left_mm / right_mm / average_mm | 24 / 24 / 24 | 63 / 87 / 111 |
| left_mm_s / right_mm_s | 12 / 12 | 135 / 147 |
| timestamp_us | 32 | 159 |
| max_execution_us / max_period_us / missed_periods | 32 each | 191 / 223 / 255 |
| reserved zero | 1 | 287 |

Counts/mm store signed value + 8388608; speed stores signed value + 2048.
Unpack requires exactly 36 bytes, valid state/command/speed codes and reserved
zero; failure leaves its output unchanged. Pack requires a valid snapshot and
a 36-byte caller buffer. Transport never blocks the motion task.

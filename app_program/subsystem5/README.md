# Buddy 5 interface and calibration

Buddy 5 owns HC-SR04 GP0/GP1 and SG90 GP12. It produces recommendations;
only Buddy 2 writes GP8–GP11. Buddy 3 owns GP6/GP7. No rear-clearance source
is defined, so there is no reverse action. Width is explicitly unknown
(`width_valid=0`); angular edges and closest point remain available.

## Public integration contract

Include `subsystem5.h`. Init/start are already called by `app_main.c`.

- `s5_command(event)`: nonblocking copy into the internal input queue;
  check its return code. A full queue returns `E_TMOUT`; the controller must
  keep/command stop and retry later. Never treat enqueue success as execution.
- `s5_request_scan(FORWARD/COARSE/FINE)`: queues the corresponding request.
  FORWARD runs only in monitoring; detailed requests first request stop.
  Unsupported requests while busy are ignored; serialize requests and inspect
  `pending`, state and mode. There is only one outstanding scan.
- `s5_get_health(&copy)`: mutex-protected I6/I8/I10 snapshot; recomputes
  forward safety at read time. Fields are local ABI bit-fields, not wire data.
- Pure conversion/filter/profile/freshness/planning/transition functions are
  declared for deterministic tests. Drivers are scanner-only interfaces.

Inputs from the Vehicle Controller: ACTIVATE, CANCEL, RESET, scan requests.
Inputs forwarded from Buddy 2: MOTION_STOPPED, MOTION_MOVING,
MOTION_COMPLETE, MOTION_FAILED. These must refer to the current operation;
serialize them through the controller, discard old completions, report
MOVING before every motion and STOPPED only after measured standstill.
Inputs forwarded from Buddy 3: LINE_FOUND (valid line reacquisition),
RECOVERY_FAILED (one completed search failure). Forward SAFETY_STOP from
the Safety Monitor. Internal SCAN_DONE/TICK are not accepted by the API.

Outputs: action NONE, CONTINUE, STOP_AND_SCAN, BYPASS_LEFT, BYPASS_RIGHT,
RECOVERY, SAFE_STOP; state, retry counts, forward range, profile and faults.
The controller consumes changes in both action and recovery retry count;
each increment requests one new bounded line-search attempt. A search starts
with retry=0 and permits two retries. Buddy 5 never executes motion.

The controller must stop for STOP_AND_SCAN and SAFE_STOP, honour Safety
Monitor I11 independently of queue delivery, and request a fresh forward
range before resuming forward motion. SAFE_STOP latches: drain any pending
scan, confirm standstill, send RESET and then a new ACTIVATE. Activation
requests stop/coarse profiling first. With no external commands the subsystem
stays idle, centres the servo at startup and does not continuously sweep.

Only internal kernel objects are allocated: two tasks, two static message
buffers (four 104-byte input/result records and one 4-byte scan request),
and one inheritance mutex. Buffer backing storage is 432 + 8 bytes on Arm.
Each task has the existing 2048-byte kernel-allocated startup stack. No
allocation occurs in the control paths. The queues copy packed records;
avoidance owns control, scanner owns hardware/local samples, and both access
their snapshot fields under the same mutex. Shared object IDs stay private.

I6/I7/I8/I10 wiring and I1/I5/I11 adapters need team agreement. No competing
project-wide bus is introduced. Buddy 2 must supply stopped/completion/failure
status and later odometry/route-deviation costs. Buddy 3 must supply valid
line state/search completion. Vehicle Controller must translate a side
recommendation into bounded motion targets. Safety Monitor must consume
health and perform actual stop/latching. Missing sources are not fabricated.

## Scan and profile

FORWARD centres the servo and uses one bounded reading. COARSE uses
30/60/90/120/150 degrees, with a 200 ms settle on angle changes and three
readings per angle. The valid median is used (nearer reading for two valid
samples). All-invalid stays invalid. FINE adds seven 5-degree samples in a
30-degree window around the coarse closest point, clamped to 30–150 degrees.
It retains coarse side coverage, never repeats the whole sweep. A valid,
recent coarse candidate and continued confirmed standstill are required.
Detailed scans abort on cancellation or reported movement between samples.
Returning to centre includes settling before publishing completion.

Profiling sorts rays and replaces duplicate angles with their fine samples.
The nearest valid point selects the relevant contiguous obstacle group at
or below 350 mm. Its end angles are approximate edges, with uncertainty
from sample spacing and the sonar beam. Invalid rays break groups and never
prove clearance. Side range is the minimum of at least two valid rays beyond
the selected sector, including the outer scan endpoint. Any invalid ray
invalidates the full profile for autonomous planning. Width stays unknown:
angular returns from a broad sonar fan do not establish reliable metric
object width. Do not interpret zero unknown values as zero measurements.

A profile timestamp is the oldest contributing sample, not scan completion.
The initial maximum profile age is 6000 ms (coarse plus fine can take several
seconds). A forward range is usable only if valid, age <=200 ms, both measured
and current servo angles are within 5 degrees of forward, scanner idle and
state monitoring. Sideways profiles never authorize forward travel. All time
comparisons use unsigned subtraction across wrap. The controller must read
health immediately before movement; cached health is not authorization.

Valid forward range >350 mm permits CONTINUE. An obstacle requests stop and
profiling. <=100 mm is an emergency SAFE_STOP. Invalid scans permit at most
two retries, then stop. Stop confirmation has a 2000 ms deadline; each scan
8000 ms; bypass completion 10000 ms; each recovery attempt 3000 ms.
Timeouts, motion failure, safety override, no path, critical range, retry
exhaustion and cancellation produce latched SAFE_STOP. No reverse is inferred.

A valid, fresh detected profile can recommend a side with >=500 mm radial
range; if both qualify, left wins deterministically until odometry supplies
route costs. These rays are a clearance heuristic, not a swept-body path
proof. `S5_BYPASS_CALIBRATED=0` deliberately gates autonomous side
recommendations to SAFE_STOP until body width, sensor offset, track geometry
and safety margin are measured and this heuristic is validated. No chassis
dimensions are documented. Do not treat 500 mm as a measured safe margin.

## Drivers and calibration

USB CDC with `CRASH_REPORT=0` is required. UART builds refuse HC-SR04 init
without touching GP0/GP1. A classic 5 V HC-SR04 needs a divider/level shifter
on ECHO. The RCWL-9610A design/HC-SR04-R variant supports a 3.3 V supply;
use GPIO mode (M1=M2 open), TRIG=GP0 and ECHO=GP1. Match GPIO labels rather
than Grove wire colours. Common ground is required. Pico GPIO is not 5 V
tolerant; level-shift ECHO when using a module that outputs 5 V.

The driver explicitly starts the RP2040 1 MHz timer reference (12 cycles
of clk_ref), releases TIMER without resetting it, unpauses it and checks that
it advances. This make build uses core-clock SysTick for the RTOS and does
not execute Pico SDK timer startup. Timer initialization does not enable
watchdog resets, claim alarms, or change motor/servo PWM. An independent
RTOS deadline also bounds the spacing wait and backs up Echo polling if the
raw counter stops. The normal pulse measurement still uses the raw timer.

TRIG is SIO output, normally low, pulsed for at least 10 us using the shared
raw timer. `WaitUsec` is unsuitable here: this port implements it using the
calling core's SysTick, while core 1's SysTick is disabled. A migratable scan
task can otherwise stall with TRIG held high. The driver uses no per-core
microsecond delay and always lowers TRIG on its backup deadline. ECHO is
SIO input. TIMER_TIMERAWL is the free-running 1 MHz counter. Both rising-edge
and high-pulse waits are bounded at 25 ms each; unsigned subtraction handles
wrap. There is no console output or interrupt masking in the polling loops.
A tick-rounded delay enforces >=60 ms from the preceding pulse end before
another trigger. Conversion uses rounded echo_us*10/58 mm and accepts
20–4000 mm. Polling can be preempted: validate errors and CPU/timing under
load; this is not a measured latency guarantee.

SG90 uses hardware PWM slice 6A only (GP12). The local divider helper writes
125 in 8.4 format at 125 MHz, giving a 1 MHz counter; TOP=19999 gives 50 Hz.
Pulse widths 1100/1500/1900 us map initial 30/90/150-degree positions.
PWM reset is released with the RP2040 atomic CLEAR alias, never asserted;
existing motor slices 4/5 are not reset or configured. USE_PTMR stays off;
no shared BSP/config/kernel changes are needed. GP13 shares slice 6 and must
not independently change its frequency. Validate real angle direction:
this implementation assumes 30 degrees is right and 150 is left.

All physical targets are in `s5_limits.h`. Calibrate mechanical endpoints,
centre, pulse widths, settling, sonar error, detection/emergency thresholds,
clearance, maximum age and timeouts with the tests in the Buddy 5 test log.
Settle time, forward-age and scan deadlines are initial targets, not bench
measurements. Cancellation cannot interrupt the current bounded echo wait.
The Safety Monitor must stop motors independently; Buddy 5 owns no motors.

## Requirements review

FR6: forward/coarse/fine, closest point, angular sector and side ranges;
metric width explicitly invalid pending reliable geometry/measurements.
FR7: confirmed stop, recommendation, bounded recovery and safe stop; real
execution awaits motion/line/controller integration, route cost awaits I5.
NFR4/A5: deterministic geometry checks and physical accuracy plan; no physical
accuracy claim. NFR5/NFR6: calibration gate, freshness, bounded waits/retries;
stop latency needs measurement. NFR7: fixed arrays/queues, no runtime
allocation; CPU, stack high-water and power remain physical measurements.
NFR8: stable local contract, named targets and repeatable evidence plan.
Minimum-bit: documented fields, sizes, copy queues and protected snapshots.

Research: [RP2040 datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
(PWM, atomic aliases and timer), [HC-SR04 datasheet](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)
(trigger, useful range and cycle), and [TRON message-buffer specification](https://tron-forum.github.io/mtk3_spec_en/extended_synchronization_and_communication_functions.html).

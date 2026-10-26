# Buddy 2 PID tuning and motion accuracy report

Hardware measurements must be recorded here after running the
[bench procedure](../../docs/MOTION_CONTROL.md#bench-procedure).
No numerical entry below is an assumed measurement. Host fixtures use synthetic
200 mm wheels, 100 mm track and 1000 counts/revolution solely to test arithmetic.

## Calibration record

Record the tested commit/worktree fingerprint, robot identity, supply voltage,
payload, surface, wheel condition and evidence file with every session.

| Parameter | Method | Measured value | Evidence |
| --- | --- | --- | --- |
| Left/right wheel circumference, mm | Loaded rolling distance over several revolutions | Pending | |
| Track width, mm | Wheel-centre distance, refined by measured turns | Pending | |
| Left/right four-edge counts/revolution | Manually rotate each wheel through several full revolutions | Pending | |
| Motor and encoder polarities | Lifted-wheel bounded pulses, compare signs | Pending | |
| Maximum stable speed, mm/s | Loaded speed steps without lost edges or saturation | Pending | |
| Duty ceiling / stall timeout | Stiction, controlled stall and lowest-speed checks | Pending | |

The API uses one circumference and one counts/revolution value for both wheels.
Verify that the two wheels/encoders match sufficiently; feedback corrects motor
speed differences, not different wheel diameters or encoder ratios. A measured
mismatch requiring separate geometry is an integration decision, not a fabricated
average. Calibration is RAM-only; record final values for the Vehicle Controller
to supply at startup. Autonomous movement remains unconfirmed after reboot.

## PID tuning

Start with a low duty ceiling and speed; the initial Q8 gains are provisional,
not tuned. Test each wheel at several setpoints in both directions, then repeat
with the complete sensor/network workload. Capture time-series encoder speed
against an external reference, overshoot, settling time, steady-state error,
duty saturation and supply voltage. Ki/Kd are per 10 ms sample.

| Run / direction / target mm/s | Kp / Ki / Kd Q8 | Duty limit | Overshoot / settling / speed error | Evidence |
| --- | --- | --- | --- | --- |
| Pending hardware run | | | | |

Retain failed trials, the adjustment made, and the new trial rather than replacing
results. Confirm anti-windup by releasing a controlled stall and observing that
output does not remain saturated. Stop on unexpected wheel direction or faults.

## Distance and turn accuracy

Measure external displacement and heading; encoder-reported completion is not
ground truth. Test forward/reverse short and long moves, left/right 30°, 90°,
180° and 360° turns, with repeated trials on the intended course surface.
Report individual signed errors and mean/maximum absolute error, not only an
average that can cancel opposite errors. Record how wheel slip changes results.

| Run / command | Requested mm or degrees | External actual | Signed error | Evidence |
| --- | --- | --- | --- | --- |
| Pending hardware run | | | | |

Agree accuracy acceptance limits with the team before trials; the course brief
does not supply Buddy 2 numeric tolerances. Record the agreed limits beside the
results. Do not infer physical accuracy from the host tests.

## Timing, IRQ and stop verification

Under IMU and Wi-Fi load, capture control execution/period diagnostics and a
scope/logic-analyser trace of motor brake timing following `!` and the Safety
Monitor API. Include USB/network input delay separately from motion response.
Verify 20 kHz PWM, a full braked reversal interval, encoder pulse capture versus
an external counter, stale-data response, faults on disconnected encoders and
bounded bench-pulse completion. Test both wheel directions.

| Run / workload | Max control µs / period µs | Lost encoder edges | End-to-end stop µs | Evidence |
| --- | --- | --- | --- | --- |
| Pending hardware run | | | | |

The nominal loop is 10000 µs, with a running-period fault over 15000 µs.
Host times in the test log are simulations of software cost on the host CPU;
only this physical testing can establish RP2040 timing, stop latency and usable
encoder/speed limits. Live stack/RAM high-water marks and power draw likewise
require instrumentation; ELF section sizes measure only static allocation.

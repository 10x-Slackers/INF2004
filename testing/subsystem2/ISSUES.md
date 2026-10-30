# Buddy 2 open verification questions

This verification cycle uses the confirmed left GP16/17 and right GP2/3 wiring,
a replacement left encoder and shortened encoder leads. Earlier powered results,
CPR estimates, polarity settings and failure theories are not accepted evidence
for this cycle. No existing hardware defect is presumed.

## Calibration

The reviewed boot profile in [MOTION_CONTROL](../../docs/MOTION_CONTROL.md)
contains measured geometry, per-wheel CPR, verified polarity and working PI
settings. Defaults remain unconfirmed and cannot arm until explicitly confirmed.
Physical turn-angle accuracy, complete PID tuning and powered emergency-stop
interruption timing still require acceptance evidence; the boot profile alone
does not establish these results. Record measurements in
[VERIFICATION](VERIFICATION.md).

The exact motor/encoder part number and electrical output type are not supplied.
Confirm the actual datasheet or measure the signals before selecting external
pull resistors or changing the input electrical configuration. The fact that
the replacement is the same model does not establish its measured CPR/polarity.

## Team interfaces

- Preserve I3/I4 finite command/status semantics; consumers check acceptance and
  completion. Agree whether Buddy 3/controller also needs continuous independent
  wheel-speed targets; no such new contract is assumed here.
- I5 delivers per-consumer counts, mm, mm/s and timestamp. Agree yaw derivation
  and delivery of calibration/validity information: the preserved 36-byte packet
  has no separate validity flag. Consumers check configuration, faults and age.
- The safety-stop endpoint exists. I10 health delivery, the system safety task,
  system stop-cause/generation and reset/new-Start arbitration remain team work.
- Buddy 5 uses the one shared IRQ13 callback registration and keeps its handler
  bounded. Validate simultaneous sensor/encoder load on target.
- Buddy 1 owns binary MQTT topics/payload transport and networking task rules.
  The packer alone is not an executed MQTT integration test.

## Acceptance measurements

Official documents give qualitative accuracy/stability goals, not numeric
speed/distance/turn error tolerances or stop-latency targets. Agree these before
claiming physical acceptance. Measure target control jitter, encoder throughput,
interrupt blackout, CPU/stack usage, power and stop latency under sensor/Wi-Fi
load. Host timing and ELF static storage do not establish these measurements.
No software guard can guarantee shutdown if the CPU or motion task stops running.

BARR-C source checks cover only a subset of rules. Review port-specific types,
callback names, unsigned bit-fields and the documented module/layout boundaries
with the coursework reviewer; do not describe source-check success as full
standards certification.

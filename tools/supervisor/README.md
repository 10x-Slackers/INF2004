# Supervisor dashboard

A page on the laptop that shows what the car is doing and sends it Start, Stop and Reset over MQTT. It covers most of what the USB console shows during a run, so the car can run untethered. The protocol is in [docs/TELEMETRY.md](../../docs/TELEMETRY.md).

## Run it

1. On the laptop, start the broker: `mosquitto -c tools/supervisor/mosquitto.conf`. This config lets the car connect from the network; Mosquitto's default accepts only the laptop itself.
2. Put the laptop's address on that network in `MQTT_BROKER_IP` of `config/wifi_secrets.h` (copied from `wifi_secrets.example.h`), then build and flash with `make -C build_make NET=lwip MQTT=1`.
3. Start the dashboard and open <http://127.0.0.1:8000>:

   ```sh
   pip install -r tools/supervisor/requirements.txt   # once; paho-mqtt 2.x
   python3 tools/supervisor/dashboard.py
   ```

   `--broker` (default `127.0.0.1`) and `--broker-port` (1883) find the broker; `--port` (8000) is the page's.

Or run steps 1 and 3 in containers, with nothing to install but Podman or Docker: `podman compose up` (or `docker compose up`) in this folder.

## What it shows

- **The state**, large and coloured, with how long the car has been in it and what to do next. Start works only in READY, Reset only in SAFE_STOP; Stop in READY, FOLLOW, AVOID and TURN, and Esc sends it. The buttons are off while the car is offline.
- **Line** (FR2, FR3): the two IR sensors over the tape, junction, the last barcode and the count.
- **Motion** (FR4): wheel speeds, distance, heading, the current move and the encoder counts.
- **Terrain** (FR5, FR10): pitch, roll, motion class, the number of humps and the highest since the car was powered on.
- **Obstacle** (FR6, FR7): the range ahead and the last profile as a sketch (servo angles, 90° straight ahead, out to 0.6 m), with the avoidance state, side and clearances.
- **Faults** (FR11): every active `APP_FAULT_...` and the safety latch.
- **Events**: what changed, with the time, newest first. State changes, barcodes, humps, bypasses, faults set and cleared, profiled obstacles, missed heartbeats, a car restart, the commands sent, and on every safe stop a summary with the highest hump and the active faults (FR12). **Download log** saves it as text, for test evidence; **Clear log** empties it before a test.

The header turns red when no heartbeat has arrived for 2.5 s. The values dim when telemetry is over 1.5 s old: they are the last ones received. The car drops what it queued while the broker was away, so after a gap the page shows live data straight away, and the log reports how many heartbeats were missed.

After a telemetry gap of more than 2 s, the log does not report barcodes, humps or bypasses that happened during the gap, because a reboot also resets those counts. The page's counts are still right, and state changes and faults are still logged.

The console still has what telemetry does not carry, such as calibration details, the bench commands and the kernel's crash report.

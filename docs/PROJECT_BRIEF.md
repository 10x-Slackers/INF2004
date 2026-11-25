# Project Brief and Requirements

Condensed from the course project brief, the student briefing and the team's Week 6 design review (group CSC13).

## The challenge

Build a robotic car that completes a predefined course on its own, using only onboard sensors and embedded software. No remote steering or speed control.

The course contains:

- a black line track
- directional barcodes at selected locations
- speed humps of varying heights
- static obstacles on or near the line

### Mandatory constraints

- µT-Kernel RTOS on Raspberry Pi Pico W + Robo Pico
- C, using the Pico C SDK
- Follows the template GitHub repository
- Conforms to the Barr C Coding Standard
- Resource-efficient: prudent use of CPU, memory, communication and power

## Mission requirements

The car shall:

1. Follow a line track autonomously.
2. Detect and decode barcodes along the track.
3. Execute the navigation command a barcode encodes.
4. Detect humps along the route.
5. Measure and report the highest hump peak of the run.
6. Detect obstacles on or near the line.
7. Profile obstacle shape and position with ultrasonic sensing.
8. Navigate around obstacles without collision.
9. Reacquire the original line after a bypass.
10. Report telemetry over Wi-Fi.

### Barcode commands

| Barcode | Command     |
| ------- | ----------- |
| A       | Turn left   |
| B       | Turn right  |
| C       | Go straight |
| D       | U-turn      |

The letters are the brief's example mapping. The course's letters may differ, so the mapping is one table (`g_actions` in `barcode_decoder.c`), which the supervisor dashboard sets over MQTT. Turns must be executed with encoder-based motion control.

### Course markings

From the supplied barcode and line specification and sample barcodes:

| Item | Value |
| --- | --- |
| Line width | 18 mm, black on white |
| Barcode symbology | Code 39: one letter between `*` start and stop characters, for example `*A*` |
| Barcode length along the track | About 141 mm |
| Gap between barcode and line | 20 mm, beside the line |
| Narrow bar | About 3 mm (141 mm over the 47 narrow-bar widths of a three-character Code 39 symbol, wide = 3 × narrow) |

The sample barcodes use letters outside A to D (F, W, X, Y, Z), so the decoder must reject any letter not in the course's command mapping.

On the example course, the barcode beside the long straight between the two loops reads `*Z*`. It is the brief's 20 mm from the line, on the side that is the car's left heading towards the left loop. Its bars are about 7 mm narrow and 20 mm wide (measured from a photo, using the 18 mm line as the scale), and it ends about 150 mm before the crossing at the left loop. Z is set to a left turn there from the supervisor dashboard.

### Expected behaviour

**Line following** must tolerate minor lighting changes, slight track imperfections and speed variation.

**Hump detection** uses IMU data both to detect a hump and to estimate its height. The demonstration's hump is a ramp about the height of the smaller practice ramp; the instructor's intent is to use the IMU in a less conventional way to estimate the height the car reaches.

**Obstacle avoidance** follows this sequence:

1. Stop or slow down.
2. Profile the obstacle with the ultrasonic scanner.
3. Determine a safe bypass route.
4. Leave the line.
5. Navigate around the obstacle.
6. Search for and reacquire the original line.
7. Resume normal operation.

The bypass should deviate from the original route as little as possible.

## Subsystem ownership

Each buddy owns one subsystem, but the car is assessed as one integrated system. Everyone takes part in integration, system testing, debugging and the final demonstration, and is expected to understand the other subsystems' interfaces.

| Buddy | Subsystem                                                 | Hardware                 | Deliverables                                                                            |
| ----- | --------------------------------------------------------- | ------------------------ | --------------------------------------------------------------------------------------- |
| 1     | Wi-Fi communication, command and telemetry                | Pico W radio             | Communication API, telemetry framework, MQTT documentation, telemetry demonstration     |
| 2     | Motion control                                            | Motors, wheel encoders   | Motion-control library, PID tuning report, motion accuracy evaluation                   |
| 3     | Barcode decoding and IR line following                    | 3 × IR sensors           | Line-following module, barcode decoder, navigation command interface                    |
| 4     | IMU-based motion and terrain monitoring                   | IMU                      | IMU processing module, hump-detection algorithm, terrain analysis report                |
| 5     | Adaptive servo ultrasonic scanning and obstacle profiling | Ultrasonic sensor, servo | Scanning subsystem, obstacle profile generator, avoidance algorithm, recovery algorithm |

### Buddy 1: Wi-Fi, command and telemetry

Establish Wi-Fi, implement MQTT, design topics and message structures, publish telemetry, subscribe to commands, recover lost connections, and report a heartbeat.

Telemetry to cover: robot state, motor speed, encoder counts, line sensor status, barcode detected, hump measurements, obstacle scan results, distance travelled.

### Buddy 2: Motion control

Motor driver and encoder integration, speed and distance estimation, PID speed control, straight-line correction, encoder-based turning, calibration.

The rest of the team calls into this subsystem, so it must expose a reusable API. The brief's examples:

- `moveForward(distance)`
- `moveBackward(distance)`
- `turnLeft(angle)`
- `turnRight(angle)`
- `stop()`

Objectives: consistent speed, accurate distance, repeatable turns, stable motion.

### Buddy 3: Line following and barcodes

Sensor integration and calibration, line position estimation, the line-following algorithm, junction detection, barcode detection and decoding, navigation command generation.

Objectives: reliable tracking, accurate barcode recognition, robust operation at varying speeds.

### Buddy 4: IMU and terrain

IMU integration and calibration, tilt detection, hump detection, peak hump measurement, motion classification, collision detection, turn-rate monitoring, motion event reporting.

The instructor replaced collision detection, as a simplification: the IMU must instead detect that the car has been picked up and stop the motors at once. At the demonstration the car is lifted, in any manner, and turned upside down to end the run. No finer lift or overturn classification is needed.

Motion events to classify: stationary, accelerating, turning, climbing hump, descending hump, sudden impact. Sudden impact went with collision detection.

Objectives: reliable hump detection, accurate peak measurement, stable event classification.

### Buddy 5: Ultrasonic scanning and obstacle avoidance

| Stage                 | What it does                                                                 |
| --------------------- | ---------------------------------------------------------------------------- |
| 1. Coarse scan        | A few scan angles, for example 30°, 60°, 90°, 120°, 150°                     |
| 2. Fine scan          | Return to the obstacle region and scan at higher resolution                  |
| 3. Profiling          | Estimate location, width, closest point, left clearance, right clearance     |
| 4. Avoidance planning | Choose one of: stop, continue, turn left, turn right, reverse and reattempt  |
| 5. Line recovery      | Search for the line, reacquire tracking, hand control back to line following |

Objectives: accurate detection, effective profiling, reliable avoidance, successful line reacquisition.

## Assessment

| Area          | What is judged                                            |
| ------------- | --------------------------------------------------------- |
| Mission       | Objectives completed                                      |
| Navigation    | Accuracy, control, completion time                        |
| Perception    | Barcode recognition, hump measurement, obstacle profiling |
| Robustness    | Recovery and stable operation                             |
| Engineering   | Software design quality, resource efficiency              |
| Integration   | Interfaces and teamwork                                   |
| Demonstration | Clear evidence under test (Week 13 final run)             |

The final demonstration scores one point for each of: detect a barcode, decode it, measure hump height, follow the line, follow the barcode's instruction, detect an obstacle, measure its width, navigate around it and rejoin the line, send telemetry over MQTT, and stop the motors when the car is picked up, which ends the run. Each team gets up to three runs and the last run counts, even if it scores lower.

The briefing's advice on how to get there:

1. Define inputs, outputs and ownership.
2. Create repeatable subsystem tests.
3. Record measurements and calibration.
4. Integrate early through stable APIs.
5. Practise the demonstration.

Other assessed work:

- **Tribunals 1 and 2:** at each, the team is given a change in requirements and records how it affects the requirements and design, with a submission before and after. The changes from Tribunal 1 must be implemented and shown at the Week 13 demonstration, with evidence for the final implementation.
- **Change Requirement 1, Minimum-Bit Data Representation** (Tribunal 1): NFR9 below, evidence in [DATA_REPRESENTATION.md](DATA_REPRESENTATION.md#evidence).
- **Peer review** of the group project.

## Team requirements

These IDs come from the Week 6 design review, with the Tribunal 1 change added as NFR9. Use them in commits, tests and reports.

### Functional

| ID   | Requirement                                                                                                                                                                  | Realised by                       |
| ---- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------- |
| FR1  | Navigate autonomously with onboard sensing and software, without remote steering or speed control                                                                            | Vehicle Controller                |
| FR2  | Follow the black line with two IR sensors, estimate a discrete line position and detect junction candidates                                                                  | Line Following                    |
| FR3  | Detect and validate barcodes; execute left, right, straight and U-turn with encoder-based motion; reacquire the commanded outgoing branch                                    | Barcode Decoder, Line Following, Vehicle Controller |
| FR4  | Use encoder feedback and PID for independent wheel speeds, speed and distance estimation, straight-travel correction, and repeatable forward, reverse and turning manoeuvres | Wheel Motion Control, Encoder ISR |
| FR5  | Detect humps from calibrated GY-511 accelerometer data; estimate height from roll and track width (humps pass under the right wheel only); report the highest valid peak of the run                              | IMU and Terrain Monitor           |
| FR6  | Detect obstacles on or near the line; coarse scan, then fine scan outward from straight ahead; estimate location, width, closest point and left/right clearance                  | Ultrasonic Scan                   |
| FR7  | Stop for profiling; bypass on the more open side, checking the path before committing and trying the other side once if that path is not clear; end back on the original line. With no safe route, reverse a few centimetres (blind: no rear-clearance sensing) and reattempt once; still none means Safe Stop | Obstacle Avoidance                |
| FR8  | Coordinate navigation, line following, barcodes, wheel control, terrain monitoring, avoidance and recovery through one mission controller                                    | Vehicle Controller                |
| FR9  | Establish Wi-Fi/MQTT, publish telemetry and heartbeat, accept permitted supervisory commands, and recover connectivity without blocking local control                        | Wi-Fi and MQTT Telemetry          |
| FR10 | Classify steady, accelerating, braking, climbing and descending (turning is measured by the encoders, FR4); report calibrated tilt; latch a stop within 0.5 s of the car being turned upside down (lifted at the end of a run), and at once on a quick lift | IMU and Terrain Monitor           |
| FR11 | Latch a stop on critical faults or explicit Stop: invalidate pending motion, brake the motors, and require reset/revalidation plus a new Start before moving                 | Safety Monitor                    |
| FR12 | End a course on Stop: brake the motors and report final status and the run's maximum valid hump estimate. Courses run back to back, so there is no finish event              | Vehicle Controller                |

### Non-functional

| ID   | Requirement                                                                                                                        |
| ---- | ---------------------------------------------------------------------------------------------------------------------------------- |
| NFR1 | Run on Raspberry Pi Pico W + Robo Pico using µT-Kernel                                                                             |
| NFR2 | Use C and the Pico SDK, the template repository and the Barr C Coding Standard                                                     |
| NFR3 | Keep line and barcode operation working under defined lighting, mounting height and speed variation; demonstrate obstacle recovery |
| NFR4 | Measure distance, turn, hump-height and obstacle-profile errors against external ground truth                                      |
| NFR5 | Keep a tested clearance margin and bound every motion and recovery attempt; handle stale input and communication loss predictably  |
| NFR6 | Meet control deadlines and maximum data-age limits; measure end-to-end stop latency under network and sensing load                 |
| NFR7 | Measure CPU, stack/memory, telemetry bandwidth and power demand; no unbounded queues or runtime allocation in control paths        |
| NFR8 | Keep APIs stable and subsystem/integration tests repeatable, with owners, calibration data and recorded evidence                   |
| NFR9 | Store and send application data in only the bits its range needs, with faults as single bits, and show RAM and message sizes for the final code (Change Requirement 1) |

### Assumptions to validate

| ID  | Assumption                                                                                  | Risk if wrong                                               | How to validate                                                                           | Owner   |
| --- | ------------------------------------------------------------------------------------------- | ----------------------------------------------------------- | ----------------------------------------------------------------------------------------- | ------- |
| A1  | Wi-Fi and MQTT can run without significantly delaying control tasks                         | Networking interferes with navigation, or telemetry is lost | Run the car while publishing continuously; measure task timing and reconnection behaviour | Buddy 1 |
| A2  | Encoder counts are accurate enough for speed, distance and turns                            | Drift, overshot turns, inaccurate barcode manoeuvres        | Drive fixed distances and angles repeatedly; compare measured against actual              | Buddy 2 |
| A3  | IR sensors reliably tell the black line and barcodes from the track under expected lighting | Lost line or misread barcodes                               | Record readings over black and white under different lighting and speeds                  | Buddy 3 |
| A4  | IMU readings are stable enough to separate normal motion from humps, and the car turned over from both | False hump or lift stops, wrong height estimates      | Record on flat ground, over humps of known height, and while lifting and inverting the car fast and slowly | Buddy 4 |
| A5  | Ultrasonic scans give accurate enough distance and width while the servo moves              | Unsafe bypass chosen, or obstacles missed                   | Place objects at known distances and widths; compare computed profiles                    | Buddy 5 |

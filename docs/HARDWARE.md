# Hardware and Wiring Reference

Pin assignments, wiring and the datasheet facts needed to write drivers. Sources: the Week 6 design review, the supplied wiring slides, and the Cytron Robo Pico, Raspberry Pi Pico W, HC-SR04, GY-511 / LSM303DLHC, Waveshare Infrared Reflective Sensor, TCRT5000 and LM393 datasheets.

## Parts

| Part                                      | Role                                                           |
| ----------------------------------------- | -------------------------------------------------------------- |
| Raspberry Pi Pico W                       | Controller (RP2040, 2 × Cortex-M0+, CYW43439 radio)            |
| Cytron Robo Pico                          | Carrier board: motor driver, servo headers, Grove ports, power |
| 2 × DC gear motor with quadrature encoder | Drive wheels (6-pin PH2.0 connector)                           |
| SG90 servo                                | Pans the ultrasonic sensor                                     |
| HC-SR04                                   | Ultrasonic range sensor                                        |
| GY-511 (LSM303DLHC)                       | 3-axis accelerometer + 3-axis magnetometer                     |
| 3 × IR reflective sensor module           | 2 for line following, 1 for barcodes                           |
| 5 V power bank                            | Supply                                                         |

## Motor and encoder connector

The confirmed six-pin connections are identical in numbering on both motors:

| Motor connector pin | LEFT | RIGHT |
| --- | --- | --- |
| 1 | GND | GND |
| 2, encoder A | GP16 | GP2 |
| 3, encoder B | GP17 | GP3 |
| 4, encoder supply | Grove 4, 3.3 V | Grove 2, 3.3 V |
| 5, motor | M1B | M2B |
| 6, motor | M1A | M2A |

GPIO assignments do not establish physical motor/encoder polarity. Determine
forward signs, four-edge counts per wheel revolution and geometry by calibration.
Do not infer direction from wire colours.

## Pin map

The assignment agreed in the design review. The owner column is the only task allowed to touch the pin (see [ARCHITECTURE.md](ARCHITECTURE.md#hardware-ownership)).

| GPIO | Signal          | Connects to                 | Interface               | Owner task              |
| ---- | --------------- | --------------------------- | ----------------------- | ----------------------- |
| GP0  | `RANGE_TRIG`    | HC-SR04 TRIG                | GPIO out                | Ultrasonic Scan         |
| GP1  | `RANGE_ECHO`    | HC-SR04 ECHO, level shifted | Timed GPIO in           | Ultrasonic Scan         |
| GP2  | `ENC_R_A`       | Right motor pin 02           | GPIO in, edge interrupt | Wheel Motion Control    |
| GP3  | `ENC_R_B`       | Right motor pin 03           | GPIO in, edge interrupt | Wheel Motion Control    |
| GP4  | `IMU_SDA`       | GY-511 SDA                  | I2C0 SDA                | IMU and Terrain Monitor |
| GP5  | `IMU_SCL`       | GY-511 SCL                  | I2C0 SCL                | IMU and Terrain Monitor |
| GP6  | `LINE_LEFT`     | Left IR module DO           | GPIO in                 | Line Following          |
| GP7  | `LINE_RIGHT`    | Right IR module DO          | GPIO in                 | Line Following          |
| GP8  | `MOTOR_L_A`     | M1A (on board)              | PWM / GPIO out          | Wheel Motion Control    |
| GP9  | `MOTOR_L_B`     | M1B (on board)              | PWM / GPIO out          | Wheel Motion Control    |
| GP10 | `MOTOR_R_A`     | M2A (on board)              | PWM / GPIO out          | Wheel Motion Control    |
| GP11 | `MOTOR_R_B`     | M2B (on board)              | PWM / GPIO out          | Wheel Motion Control    |
| GP12 | `SONAR_PAN_PWM` | SG90 signal (servo header)  | Servo PWM               | Ultrasonic Scan         |
| GP16 | `ENC_L_A`       | Left motor pin 02          | GPIO in, edge interrupt | Wheel Motion Control    |
| GP17 | `ENC_L_B`       | Left motor pin 03          | GPIO in, edge interrupt | Wheel Motion Control    |
| GP26 | `BARCODE_AO`    | Auxiliary IR module AO      | ADC0                    | Barcode Decoder         |

Where each signal lands on the Robo Pico:

| Grove port | GPIOs      | Used for                            |
| ---------- | ---------- | ----------------------------------- |
| 1          | GP0, GP1   | HC-SR04                             |
| 2          | GP2, GP3   | Right encoder (also on Maker port)   |
| 3          | GP4, GP5   | GY-511                              |
| 4          | GP16, GP17 | Left encoder                       |
| 5          | GP6, GP26  | Left line sensor, barcode sensor    |
| 6          | GP26, GP27 | GP26 is the same pin as on port 5   |
| 7          | GP7, GP28  | Right line sensor                   |

GP26 appears on both Grove 5 and Grove 6. Never give it two independent signals.

### Unassigned and reserved pins

| GPIO                   | Status                                                                                   |
| ---------------------- | ---------------------------------------------------------------------------------------- |
| GP13, GP14, GP15       | Free. Spare servo headers                                                                |
| GP18                   | Robo Pico NeoPixel RGB LEDs                                                              |
| GP19                   | Free                                                                                     |
| GP20, GP21             | Robo Pico programmable buttons. Candidates for a Start/Stop input                        |
| GP22                   | Robo Pico piezo buzzer (has a mute switch)                                               |
| GP27                   | Free. ADC1                                                                               |
| GP28                   | Free. ADC2. Reads VBAT / 2 if the VBAT sense jumper on the back of the board is soldered |
| GP23, GP24, GP25, GP29 | **Reserved by the Pico W radio.** Do not use                                             |

The Pico W's on-board LED hangs off the radio chip (`WL_GPIO0`), not an RP2040 pin, so it cannot be driven with ordinary GPIO calls.

### Boot-time pin setup

By default the kernel leaves every application pin in its reset state. A few build options and driver settings claim pins on their own, so check them against the map above and record each one in `TEAM.md`.

| What | Where | Default |
| --- | --- | --- |
| UART0 console copy and crash report | `make CRASH_REPORT=1` or `CONSOLE=uart` | Off. When on, it takes GP0 and GP1 |
| Kernel I2C driver pins | `DEVCNF_USE_IIC` in `config/config_device.h`, pins in `device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h` | On. It registers I2C0 only (`iica`, in `kernel/sysdepend/pico_rp2040/devinit.c`) and muxes GP4, GP5 for the GY-511 |
| Kernel ADC driver pins | `DEVCONF_ADC_PIN_INIT_n` in `device/adc/sysdepend/rp2040/adc_cnf_sysdep.h` | None claimed |
| PWM block reset release | `kernel/sysdepend/pico_rp2040/hw_setting.c`, gated by `USE_PTMR` in `config/config.h` | Off |

Things that clash with this pin map:

- `CRASH_REPORT=1` takes GP0 and GP1, which carry the HC-SR04. Unplug the sensor before using a crash-report build.
- The GY-511 sits on Grove 3 (GP4, GP5) because those are the Robo Pico's I2C0 pins, the only I2C unit the kernel driver registers. The template's default I2C0 pins (GP8, GP9) are the left motor, so `DEVCNF_I2C0_SDA_PIN` and `DEVCNF_I2C0_SCL_PIN` must stay at 4 and 5.
- The ADC channel for the barcode sensor (GP26, channel 0) needs `DEVCONF_ADC_PIN_INIT_0` set to `TRUE`.

## Power

A 5 V power bank feeds the Robo Pico, through the VIN terminal or the Pico W's USB port.

| Limit                                    | Value                                                         |
| ---------------------------------------- | ------------------------------------------------------------- |
| Board input voltage (USB, LiPo or VIN)   | 3.6 V to 6 V                                                  |
| Motor and servo supply                   | Equal to the input supply. VUSB − 0.4 V when powered over USB |
| Motor current per channel                | 1 A continuous, 1.5 A peak for under 5 s                      |
| Motor driver PWM frequency               | 20 kHz maximum                                                |
| Total 3.3 V current from the Grove ports | 300 mA                                                        |
| Digital input high level                 | 2.0 V to 3.6 V. **GPIOs are not 5 V tolerant**                |
| Analog input                             | 0 V to 3.3 V                                                  |

The on/off switch cuts power to everything, including the Pico W. The reset button resets the Pico W only.

## Motors and encoders

### Motor driver

Each motor is driven by two GPIOs. Put PWM on one input and hold the other low to set speed and direction.

| Input A (GP8 / GP10) | Input B (GP9 / GP11) | Motor                |
| -------------------- | -------------------- | -------------------- |
| Low                  | Low                  | Brake                |
| High                 | Low                  | Forward              |
| Low                  | High                 | Backward             |
| High                 | High                 | Coast (outputs open) |

"Forward" in the truth table is an electrical direction. Keep the confirmed
wiring and calibrate each wheel's software reversal flag against physical forward. The motor test buttons on the board run a motor at full speed without any firmware, which is useful for checking wiring.

### Six-pin motor connector (PH2.0-6P)

| Pin | Function        | Connect to                           |
| --- | --------------- | ------------------------------------ |
| 01  | Encoder ground  | GND                                  |
| 02  | Encoder phase A | Left: GP16. Right: GP2               |
| 03  | Encoder phase B | Left: GP17. Right: GP3               |
| 04  | Encoder supply  | 3.3 V                                |
| 05  | Motor winding   | LEFT M1B; RIGHT M2B                  |
| 06  | Motor winding   | LEFT M1A; RIGHT M2A                  |

Check the plug's physical orientation against the motor's own markings before connecting pins 01 to 04. The wiring slides power the encoder from 3.3 V; if you power it from anything higher, its outputs must not exceed 3.3 V at the GPIO.

Encoder edges are captured by GPIO interrupt. Capture capacity and missed-edge
behavior must be measured at the intended speed and system load. Counts per wheel revolution are not in any supplied datasheet and must be measured (assumption A2).

## SG90 servo

Signal on GP12. V+ on the servo header is the board supply voltage.

No SG90 datasheet was supplied. The values below are the usual ones for this servo and must be calibrated on the actual part:

| Parameter   | Typical value                                          |
| ----------- | ------------------------------------------------------ |
| PWM period  | 20 ms (50 Hz)                                          |
| Pulse width | About 1 ms to 2 ms across the travel, 1.5 ms at centre |

Let the servo settle before taking a range reading at a new angle.

## HC-SR04 ultrasonic sensor

| Parameter                 | Value                        |
| ------------------------- | ---------------------------- |
| Supply                    | 5 V DC, 15 mA                |
| Range                     | 2 cm to 400 cm               |
| Accuracy                  | Up to 3 mm                   |
| Measuring angle           | 15°                          |
| Trigger                   | High pulse of at least 10 µs |
| Minimum measurement cycle | 60 ms                        |

To take a reading:

1. Drive TRIG high for at least 10 µs.
2. The module emits eight 40 kHz pulses and raises ECHO.
3. Time how long ECHO stays high.
4. Distance in cm = echo time in µs ÷ 58 (speed of sound 340 m/s, halved for the round trip).

**ECHO is a 5 V signal.** Bring it down to 3.3 V with a level shifter or a resistor divider (for example 1 kΩ in series, 2 kΩ to ground) before it reaches GP1. The datasheet specifies a 5 V supply; the wiring slide shows the module on 3.3 V, which is outside the datasheet rating.

Narrow, angled or soft objects reflect poorly. The datasheet's accuracy figure assumes a smooth target of at least 0.5 m².

## GY-511 (LSM303DLHC) IMU

Accelerometer and magnetometer only. **There is no gyroscope**, which is why turn rate is derived from odometry (FR10).

| Parameter                | Value                                                |
| ------------------------ | ---------------------------------------------------- |
| Module supply            | 3 V to 5 V. Use 3.3 V so SDA/SCL stay at GPIO levels |
| Bus                      | I2C, standard (100 kHz) or fast (400 kHz) mode       |
| Accelerometer address    | `0x19` (7-bit)                                       |
| Magnetometer address     | `0x1E` (7-bit)                                       |
| Accelerometer full scale | ±2 g, ±4 g, ±8 g or ±16 g                            |
| Magnetometer full scale  | ±1.3 to ±8.1 gauss                                   |

The two sensors are separate I2C devices with separate register maps.

### Accelerometer registers

| Register                   | Address          | Purpose                                                                          |
| -------------------------- | ---------------- | -------------------------------------------------------------------------------- |
| `CTRL_REG1_A`              | `0x20`           | Data rate, low-power enable, axis enables. Resets to `0x07`, which is power-down |
| `CTRL_REG4_A`              | `0x23`           | Block data update, endianness, full scale, high-resolution mode                  |
| `STATUS_REG_A`             | `0x27`           | Data-ready flags                                                                 |
| `OUT_X_L_A` to `OUT_Z_H_A` | `0x28` to `0x2D` | X, Y, Z output, low byte first                                                   |

A minimal setup:

1. Write `0x57` to `CTRL_REG1_A`: 100 Hz, normal mode, all three axes on.
2. Write `0x08` to `CTRL_REG4_A`: ±2 g, high-resolution mode.
3. Read six bytes starting at `0xA8`. Setting the top bit of the register address (`0x28 | 0x80`) turns on address auto-increment for a multi-byte read.

Each axis is a 16-bit two's complement value, left-justified. In high-resolution mode shift right by 4 to get a 12-bit value at 1 mg per LSB (±2 g range).

### Magnetometer registers

| Register                   | Address          | Purpose                                                       |
| -------------------------- | ---------------- | ------------------------------------------------------------- |
| `CRA_REG_M`                | `0x00`           | Data rate, temperature sensor enable                          |
| `CRB_REG_M`                | `0x01`           | Gain                                                          |
| `MR_REG_M`                 | `0x02`           | Mode. Resets to sleep; write `0x00` for continuous conversion |
| `OUT_X_H_M` to `OUT_Y_L_M` | `0x03` to `0x08` | Output, **high byte first, in the order X, Z, Y**             |

### Usage notes

- Pitch comes from the gravity vector, so it is only trustworthy when the car is not accelerating hard. Filter it, and calibrate a flat-ground baseline at start-up.
- The RP2040 has no FPU. Floating-point trigonometry runs in software and is slow; prefer integer or fixed-point maths in the sampling path.
- The module also offers a FIFO and two programmable interrupt generators (free-fall, motion, click). The design does not use them.

## IR reflective sensors

Three modules: two read digitally for line following, one read as analog for barcodes.

| Parameter | Value                                                                 |
| --------- | --------------------------------------------------------------------- |
| Module    | Waveshare Infrared Reflective Sensor (ST188 sensor, LM393 comparator) |
| Supply    | 3.0 V to 5.3 V. **Use 3.3 V** so AOUT cannot exceed the ADC limit     |
| Pins      | DOUT (digital), AOUT (analog), GND, VCC                               |

AOUT follows the amount of reflected infrared. DOUT is AOUT compared against a threshold by the LM393; the on-board potentiometer sets the threshold and the indicator LED shows the comparator state. Which level means "black" depends on the module, so confirm it on the real track before writing the line logic.

A TCRT5000 datasheet was also supplied. If the kit's modules use that sensor instead of the ST188, the points that matter are:

| Parameter                | TCRT5000 |
| ------------------------ | -------- |
| Peak operating distance  | 2.5 mm   |
| Emitter wavelength       | 950 nm   |
| Daylight blocking filter | Yes      |

Either way, mount the sensors a few millimetres above the surface and keep the height constant: reflective sensors are very sensitive to distance. Set the potentiometers with the sensors at their final height, over the actual track surface.

The Pico W ADC is 12-bit (0 to 4095) referenced to ADC_VREF, nominally 3.3 V. The barcode sampling rate depends on bar width and car speed and has to be established on the track (assumption A3).

## Raspberry Pi Pico W

| Item       | Detail                                        |
| ---------- | --------------------------------------------- |
| `VBUS`     | 5 V from the micro-USB port                   |
| `VSYS`     | Main input, 1.8 V to 5.5 V                    |
| `3V3`      | 3.3 V output. Keep external load under 300 mA |
| `ADC_VREF` | ADC reference, a filtered copy of 3.3 V       |
| GP23       | Radio power enable                            |
| GP24       | Radio SPI data / IRQ                          |
| GP25       | Radio SPI chip select                         |
| GP29       | Radio SPI clock, shared with ADC3 (VSYS / 3)  |
| `WL_GPIO0` | On-board LED, on the radio chip               |

For the full pinout see the [Pico W datasheet](https://datasheets.raspberrypi.com/picow/pico-w-datasheet.pdf).

## Pre-power checklist

1. HC-SR04 ECHO goes through a divider or level shifter.
2. IR modules and the GY-511 are powered from 3.3 V.
3. Encoder supply is 3.3 V, and the plug orientation matches the motor's markings.
4. Nothing is wired to GP23, GP24, GP25 or GP29.
5. GP26 carries one signal only.
6. The supply is between 3.6 V and 6 V.
7. The [boot-time pin setup](#boot-time-pin-setup) agrees with the pin map for every pin you have connected.

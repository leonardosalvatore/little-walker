# Little Walker — Motor Demo

ESP32-C6 firmware that drives **two hobby servos** (via the on-chip LEDC/PWM)
and **two DC motors** (via a Pololu **Motoron M3T453** triple I2C motor
controller), while showing live status on the on-board 1.47" LCD.

![Little Walker build: ESP32-C6-LCD-1.47 and Motoron M3T453 on a perfboard chassis between two geared wheels, the LCD showing the robot face](assets/little-walker-build.png)

The build above uses:

- **Waveshare** ESP32-C6-LCD-1.47 development board (MCU + display)
- **Pololu** Motoron M3T453 triple I2C motor controller
- 2x geared DC motors with 65 mm wheels
- A green prototyping/perfboard as the chassis and wiring base

See [Components](#components) for full specifications.

The display is split in two halves:

- **Top (~50%): an animated robot face** — two eyes that change expression as the
  demo runs (see [Robot face](#robot-face)).
- **Bottom (~50%): a 2x2 grid of status tiles** (Left Servo, Right Servo, Left
  Motor, Right Motor), highlighting the active one in blue. The firmware version
  is shown dimly in the face panel's corner.

The demo loops through this sequence, highlighting the active tile:

```
Left Servo : 0  -> 90 -> 180
Left Motor : Fwd -> Stop -> Back -> Stop
Right Servo: 0  -> 90 -> 180
Right Motor: Fwd -> Stop -> Back -> Stop
```

## Hardware

| Item                 | Detail                                                   |
| -------------------- | -------------------------------------------------------- |
| MCU board            | Waveshare **ESP32-C6-LCD-1.47** (172x320 ST7789)         |
| Motor controller     | Pololu **Motoron M3T453** (triple, I2C, addr `16`/`0x10`)|
| Motors               | 2x brushed DC motors (4.5–44 V, <= 0.8 A continuous)     |
| Servos               | 2x standard hobby servos (5 V)                           |
| Logic supply         | USB supply, selectable **3.3 V / 5 V**                   |
| Motor supply         | USB supply, **0–24 V** adjustable                        |
| Build base           | Breadboard with two power rails                          |

## Pin map (ESP32-C6)

| Function          | GPIO   | Notes                                  |
| ----------------- | ------ | -------------------------------------- |
| LCD MOSI / SCLK   | 6 / 7  | on-board display (do not reuse)        |
| LCD CS/DC/RST/BL  | 14/15/21/22 | on-board display                  |
| Servo LEFT signal | 0      | PWM 50 Hz                              |
| Servo RIGHT signal| 1      | PWM 50 Hz                              |
| I2C SDA           | 4      | to Motoron SDA                         |
| I2C SCL           | 5      | to Motoron SCL                         |

> The I2C pins (GPIO4/5) are defined at the top of `main/main.c`
> (`PIN_I2C_SDA` / `PIN_I2C_SCL`). Confirm they match a free pad on your board's
> header silkscreen and change them there if needed.

## Wiring overview

```
                    +5V rail            +3.3V rail            0-24V rail
                    (servo V+)          (logic VDD)           (motor VIN)
   USB PSU #2          |                    |                     |
  [3.3V / 5V] ---------+--------------------+                     |
   (set 5V out for     |                    |                     |
    servos; 3.3V out   |                    |                     |
    for logic)*        |                    |          USB PSU #1 |
                       |                    |          [0-24V] ---+
                       |                    |                     |
  =========================== COMMON GND RAIL ==========================
        |          |                |               |             |
        |          |                |               |             |
   ESP32-C6     Servo L          Servo R         Motoron        Motors
    GND          GND              GND             GND           (via M1/M2)


  ESP32-C6-LCD-1.47                         Motoron M3T453
  +-----------------+                       +--------------------+
  | GPIO0  (Servo L)|---------> Servo L sig | SDA  <----------+  |
  | GPIO1  (Servo R)|---------> Servo R sig | SCL  <--------+ |  |
  | GPIO4  (SDA)    |----------------------)| VDD  <------+ | |  |
  | GPIO5  (SCL)    |--------------------+ )| GND         | | |  |
  | 3V3             |------------------+ | )|             | | |  |
  | GND             |----------------+ | | )| VIN  <--- 0-24V    |
  +-----------------+                | | | (| M1A/M1B --> Motor L |
                                     | | | (| M2A/M2B --> Motor R |
        common GND <-----------------+ | | (+--------------------+
        ESP 3V3 ----> Motoron VDD ------+ | (
        GPIO5  ------> Motoron SCL --------+ (
        GPIO4  ------> Motoron SDA ----------+

  * The selectable USB PSU provides ONE voltage at a time. Use a unit with two
    independent outputs, OR feed logic (3.3 V) and servos (5 V) from separate
    regulated outputs. If you only have a single 5 V output available, power the
    ESP32-C6 over its USB-C port instead and use the PSU's 5 V only for servos.
```

### Servo connections (x2)

```
   Servo connector            ESP32-C6 / rails
   +-----------+
   | SIG  -----|-----------> GPIO0 (Left) / GPIO1 (Right)
   | V+   -----|-----------> +5V rail
   | GND  -----|-----------> COMMON GND rail
   +-----------+
```

### Motoron M3T453 connections

```
   Logic side (to ESP32-C6, 3.3 V)        Power / motor side
   +-------------------------+            +-------------------------+
   | VDD  <--- 3V3 (ESP32)   |            | VIN  <--- 0-24V (+)      |
   | GND  <--- COMMON GND    |            | GND  <--- COMMON GND     |
   | SDA  <--> GPIO4         |            | M1A/M1B --> Left Motor   |
   | SCL  <--> GPIO5         |            | M2A/M2B --> Right Motor  |
   +-------------------------+            | M3A/M3B --> (unused)     |
                                          +-------------------------+
   - Default 7-bit I2C address: 16 (0x10).
   - On-board 10k pull-ups on SDA/SCL tie to VDD (3.3 V) — no external
     pull-ups needed.
   - VDD logic range 3.0–5.5 V; VIN motor range 4.5–44 V.
```

## Power notes

- **Two USB power supplies on a breadboard:**
  - **PSU #1 (0–24 V):** feeds the Motoron **VIN** (motor power). Set it to your
    motors' rated voltage. Its negative output joins the common ground rail.
  - **PSU #2 (selectable 3.3 V / 5 V):** provides logic power. The ESP32-C6 runs
    at **3.3 V** and the Motoron **VDD** is also **3.3 V** (so the I2C levels
    match). Hobby servos want **5 V**.
- **Common ground is mandatory:** all grounds — ESP32-C6 GND, Motoron GND,
  both PSU negatives, servo grounds — must be tied to a single ground rail.
  Without it, I2C and PWM signals have no reference and nothing works.
- **Do not** power the servos or motor VIN from the ESP32-C6 3.3 V pin; it
  cannot supply that current.
- Keep motor (VIN) wiring on its own rail away from the logic rail; only the
  ground is shared.

## Robot face

The top panel renders two cyan rounded-rect eyes. Each eye is carved by two
background-coloured "eyelids" (top + bottom) that resize/rotate to form an
expression; some expressions also show a small text cue and a round black pupil.

| Expression | Eyes                              | Pupil | Text cue |
| ---------- | --------------------------------- | ----- | -------- |
| `sleep`    | thin bottom slit                  | no    | `z Z z`  |
| `surprise` | wide open                         | yes   | —        |
| `idle`     | short, centred band               | yes   | —        |
| `sad`      | slanted `/ \` (inner corners up)  | no    | —        |
| `happy`    | upward dome                       | no    | `Ahah`   |
| `furious`  | slanted `\ /` (inner corners down)| no    | `Grrr!`  |

The current expression is also printed to the serial log (`Expression: <name>`).
`demo_task` maps a face to each phase: `idle`/`surprise` while resting, `sleep`
during the left servo sweep, `furious` for the left motor, `sad` for the right
servo sweep, and `happy` for the right motor.

## Firmware behaviour

On boot the firmware:

1. Initializes the two servo PWM channels (centered at 90 deg).
2. Initializes the Motoron over I2C: `Reinitialize`, `Disable CRC`,
   `Clear reset flag`, and clears the Error mask so the 1.5 s command-timeout
   does not stop the motors during the servo phases.
3. Brings up the LCD + LVGL and draws the robot face (top) and 2x2 status grid
   (bottom).
4. Runs `demo_task`, stepping through the sequence above (~800 ms per step) and
   updating the face expression for each phase.

Motor speed is set by `MOTOR_SPEED` in `main/main.c` (0–800; default 400 for a
gentle demo). Forward = `+MOTOR_SPEED`, Back = `-MOTOR_SPEED`, Stop = `0`.

## Build & flash

```bash
idf.py set-target esp32c6
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

## Example boot log

On a correctly wired board the startup I2C scan finds the Motoron at address
`0x10` and the demo begins stepping through the servo/motor sequence:

```
I (333) main: Servos initialized: LEFT=GPIO0 (TIMER0) RIGHT=GPIO1 (TIMER1)
I (340) main: Initializing Motoron motor controller...
I (345) main: Scanning I2C bus (SDA=GPIO4 SCL=GPIO5)...
I (351) main:   found I2C device at 0x10 (16)
I (367) main: Motoron present at address 16.
I (374) main: Motoron initialized on I2C SDA=GPIO4 SCL=GPIO5 addr=16
I (518) main: Display initialized. Setting up LVGL...
I (607) main: UI ready. Starting motor demo...
I (607) main: Left servo: 0
I (1409) main: Left servo: 90
I (2210) main: Left servo: 180
I (6219) main: Right servo: 0
I (7020) main: Right servo: 90
I (7821) main: Right servo: 180
```

If the scan instead prints `no I2C devices found` or `Motoron NOT found at
address 16`, check the Motoron VDD/GND/SDA/SCL wiring, the common ground, and
that the controller is using its default address (16).

## Components

### Waveshare ESP32-C6-LCD-1.47

Waveshare ESP32-C6 development board with an integrated 1.47" TFT-LCD display.
The board is suitable for IoT projects, dashboards, and other applications that
need a graphical user interface. It supports **LVGL** (Light and Versatile
Graphics Library), an open-source library for building custom GUIs that also
handles input devices such as a mouse, buttons, etc.

| Spec            | Detail                                                              |
| --------------- | ------------------------------------------------------------------- |
| MCU             | ESP32-C6FH4 — 32-bit RISC-V, 160 MHz                                |
| SRAM            | 512 kB HP + 16 kB LP SRAM                                           |
| ROM             | 320 kB                                                              |
| Flash           | 4 MB                                                                |
| Wireless        | 2.4 GHz Wi-Fi 6 (802.11 b/g/n/ax) and Bluetooth 5 (LE)             |
| Antenna         | Integrated                                                          |
| Display         | 1.47" TFT-LCD, 172 × 320 px, 262k colors, SPI interface            |
| Connection      | USB-C port for power and programming                                |
| GPIO            | 2× 9-pin headers (not pre-soldered): 13 GPIOs with UART, I²C, PWM, 6 ADC channels, plus 5 V / 3.3 V / GND |
| Card slot       | MicroSD (offline image/data storage)                                |
| Buttons         | Reset and Boot                                                      |
| Dimensions      | approx. 36.4 × 20.3 mm                                              |
| Dev environments| ESP-IDF, Arduino IDE, CircuitPython                                 |
| Model           | Waveshare ESP32-C6-LCD-1.47                                         |

### Pololu Motoron M3T453 Triple I²C Motor Controller

An I²C-controlled motor controller for up to three DC motors. Each motor is
controlled independently, and multiple Motoron controllers can share the same
I²C bus for systems with more motors. Supplied without pin headers.

| Spec                     | Detail                                       |
| ------------------------ | -------------------------------------------- |
| Product                  | Motoron M3T453 Triple I2C Motor Controller   |
| Motors                   | Up to 3 DC motors                            |
| Interface                | I2C                                          |
| I2C clock frequency      | Up to 400 kHz                                |
| Motor voltage            | 4.5 to 44 V                                  |
| Current rating           | 0.8 A continuous per motor                   |
| Peak current             | 2 A per motor for < 1 s                      |
| Logic voltage            | 3.0 to 5.5 V                                 |
| PWM frequency            | 1 to 80 kHz                                  |
| Current sensing          | Motor channels 1 and 2                       |
| Reverse voltage protection| Down to −40 V on motor power supply         |
| Connections              | 2.54 mm pitch                                |
| Pin headers              | Not included                                 |
| Dimensions               | 17.8 × 22.9 mm                               |

### Geared DC motors with wheels (x2)

| Spec               | Detail                                              |
| ------------------ | --------------------------------------------------- |
| Supply voltage     | 3–9 V DC (optimal approx. 5 V)                      |
| Current (no-load)  | 160–200 mA depending on supply voltage              |
| Speed (no-load)    | 90–300 rpm (±10 %) depending on supply voltage      |
| Torque             | 800–1200 gf·cm depending on supply voltage          |
| Shaft              | 3.6 mm double shaft with 1.9 mm opening             |
| Wheel diameter     | 65 mm                                               |
| Wheel width        | 27 mm                                               |
| Motor dimensions   | 37.6 × 64.2 × 22.5 mm                               |
| Weight             | approx. 58 g                                        |

### Prototyping/perfboard chassis

The green board the electronics are mounted on is a **prototyping perfboard**
(also called a "perfboard", "protoboard", or solder-able experiment board): an
epoxy/FR-4 board with a grid of 2.54 mm-pitch plated through-holes. Unlike a
solderless breadboard, components and wires are **soldered** to the pads, giving
a permanent, vibration-resistant build — handy here since the board doubles as
the robot chassis carrying the two geared wheels and the USB-C cable. Each hole
is an isolated pad, so all connections between the ESP32-C6, Motoron, motors, and
power are made with soldered wire links on the underside.

# Little Walker — Motor Demo

ESP32-C6 firmware that drives **two hobby servos** (via the on-chip LEDC/PWM)
and **two DC motors** (via a Pololu **Motoron M3T453** triple I2C motor
controller), while showing live status on the on-board 1.47" LCD.

The display shows a `Motor Demo` title, the firmware version, and a 2x2 grid of
status tiles (Left Servo, Right Servo, Left Motor, Right Motor). The demo loops
through this sequence, highlighting the active tile:

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

## Firmware behaviour

On boot the firmware:

1. Initializes the two servo PWM channels (centered at 90 deg).
2. Initializes the Motoron over I2C: `Reinitialize`, `Disable CRC`,
   `Clear reset flag`, and clears the Error mask so the 1.5 s command-timeout
   does not stop the motors during the servo phases.
3. Brings up the LCD + LVGL and draws the 2x2 status grid.
4. Runs `demo_task`, stepping through the sequence above (~800 ms per step).

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

# Wheel-Based Soccer Robot: Microcontroller Firmware

STM32F407 firmware for a 4 omni-wheel holonomic soccer robot (KRSBI Beroda): wheel PID, odometry, sensors and a serial link to the vision PC.

<!-- TODO: add demo GIF/photo -->

## Overview

This is the low-level controller of R2C's wheeled soccer robot (codename *Warrior*) for **KRSBI-B** (Kontes Robot Sepak Bola Indonesia Beroda), the Indonesian national wheeled robot soccer contest. The robot uses four omni wheels in a holonomic layout and splits work between two computers:

- A **PC** runs camera vision and game strategy ([WheelBased-Soccer-Robot-PC](https://github.com/altilize/WheelBased-Soccer-Robot-PC)) and sends motion commands.
- This **STM32** turns those commands into wheel speeds, closes the speed loop on each motor, tracks the robot's position on the field and reports sensor data back.

## Features

- **Holonomic inverse kinematics** for a 4 omni-wheel base (translation and rotation at the same time).
- **Per-wheel PID speed control** using motor encoders sampled by a 10 Hz hardware timer (TIM5).
- **Odometry** from three dedicated quadrature encoder wheels, rotated into the field frame using IMU yaw, quantized into a 50 cm field grid.
- **Grid re-localization**: the PC can overwrite the grid position and heading offset at runtime.
- **Binary serial protocol** to the PC with header bytes and an inverted-sum checksum.
- **Sensors and I/O**: MPU6050 IMU (yaw), 2x VL53L1X time-of-flight distance sensors, IR ball/obstacle sensors, 16-channel multiplexed line sensor array, 4x5 keypad, SSD1306 OLED, kicker pins.
- **Line follower** using a weighted average over the vertical line sensor array with a clamped PID correction term (anti-windup on the integral).
- **Safety**: a hardware switch on PA0 disables the motor driver and resets all PID state when off.

## Architecture

```
 PC (vision + strategy)
        |  USB CDC serial, 115200 baud
        v
 +---------------------------- STM32F407 ----------------------------+
 |  UART_receive()  -> speed, heading, vz                            |
 |  playauto()      -> vx = v*sin(heading), vy = v*cos(heading)      |
 |  holonomic()     -> 4 wheel setpoints                             |
 |  pid()           -> PWM per wheel (setpoint vs encoder RPM)        |
 |  motorauto()     -> H-bridge pins                                 |
 |                                                                   |
 |  hitungposisi()  <- 3 odometry encoders + IMU yaw -> X, Y, grid   |
 |  UART_send()     -> compass, keypad, IR, grid, X/Y back to PC     |
 +-------------------------------------------------------------------+
        |
        v
   4 DC motors with encoders, omni wheels
```

## Holonomic Kinematics

The base has four omni wheels mounted at 45°, 135°, 225° and 315° around the robot center. For a desired body velocity `(vx, vy, ω)`, each wheel speed is:

```
v_i = -sin(θ_i) * vx + cos(θ_i) * vy + R * ω
```

With θ = 45°, 135°, 225°, 315°, `sin/cos(45°) ≈ 0.707`. The firmware folds in a 0.5 scale factor (0.707 × 0.5 ≈ 0.35) and a rotation gain of 0.25:

```c
motor_1 = (-0.35 * vx) + ( 0.35 * vy) + (0.25 * vz);
motor_2 = (-0.35 * vx) + (-0.35 * vy) + (0.25 * vz);
motor_3 = ( 0.35 * vx) + (-0.35 * vy) + (0.25 * vz);
motor_4 = ( 0.35 * vx) + ( 0.35 * vy) + (0.25 * vz);
```

The PC sends a speed magnitude and heading (0° forward, 90° right, -90° left), which `playauto()` converts to `vx = speed * sin(heading)` and `vy = speed * cos(heading)`.

Each wheel setpoint is compared to the measured RPM (encoder pulses per 100 ms window, 134 pulses per revolution) and an incremental PID accumulates the PWM output, clamped to 0 to 255. Direction is handled separately through a sign flag.

## Odometry

Three free-rolling encoder wheels (400 counts per revolution, 18.2 cm circumference) measure displacement independently of wheel slip on the drive motors:

```c
vx = Da*cos(60°) + Db*cos(60°) - Dc;
vy = Da*sin(30°) - Db*sin(30°);
```

The local displacement is rotated by the IMU yaw into field coordinates and accumulated into `Xpos, Ypos` (cm). The field is divided into 50 cm cells, and the grid cell is sent to the PC for navigation.

## Serial Protocol

**PC to STM32** (13 bytes, checksum is the bitwise NOT of the sum of bytes 1 to 11):

| Header | Meaning |
| --- | --- |
| `254` | Motion: enable, speed (16-bit), heading sign + value, vz sign + value (16-bit), handler, kick, kick mode |
| `250` | Set grid: grid X, grid Y, heading reset value and sign, line follower flag |

**STM32 to PC** (16 bytes, header `255`): compass sign and value, keypad button, IR sensors, grid X/Y, signed 16-bit X and Y position.

## Hardware and Software Stack

| Item | Details |
| --- | --- |
| MCU | STM32F407 (STM32duino core, SWD upload, USB CDC serial) |
| Drive | 4 DC motors with single-channel encoders, omni wheels, H-bridge drivers |
| Odometry | 3 quadrature encoder wheels |
| IMU | MPU6050 (`MPU6050_light`) |
| Distance | 2x VL53L1X ToF (`VL53L1X`, XSHUT address assignment) |
| Line sensor | Analog array through a 16:1 multiplexer |
| UI | SSD1306 128x64 OLED (`Adafruit_SSD1306`), 4x5 matrix keypad (`Keypad`) |
| Other libs | `ArduinoEigenDense` |
| Language | Arduino C++ |

## Repository Structure

```
├── WARRIOR-Goalkeeper-MCU.ino  # pin map, globals, library includes
├── init.ino                    # peripheral init: motors, encoders, timer, OLED, ToF, line sensor
├── main.ino                    # setup() and loop()
├── mode.ino                    # play/stop logic, command to (vx, vy, vz)
├── motion.ino                  # holonomic kinematics and motor output
├── PID.ino                     # per-wheel PID
├── interupsi.ino               # encoder ISRs, RPM timer callback
├── gyrodometri.ino             # odometry and grid computation
├── compass.ino                 # IMU yaw handling
├── UART.ino                    # serial protocol to/from PC
├── LineFollower.ino            # line sensor reading and PID
├── OLED.ino                    # status display
└── Debug.ino                   # serial debug helpers
```

## Build and Upload

1. Install the Arduino IDE and the [STM32duino core](https://github.com/stm32duino/Arduino_Core_STM32).
2. Install libraries: `MPU6050_light`, `VL53L1X` (Pololu), `Adafruit_SSD1306`, `Adafruit_GFX`, `Keypad`, `ArduinoEigen`.
3. The Arduino IDE requires the main sketch to match its folder name. Clone into a folder named `WARRIOR-Goalkeeper-MCU`, or rename the folder after cloning:
   ```bash
   git clone https://github.com/altilize/WheelBased-Soccer-Robot-Microcontroller.git WARRIOR-Goalkeeper-MCU
   ```
4. Board settings: STM32F407VETx, upload method SWD, USB support "CDC (generic Serial supersede U(S)ART)", U(S)ART support enabled.
5. On Linux, give your user access to the serial port:
   ```bash
   sudo usermod -a -G dialout $USER
   ```

## My Role

I (Moses Jaguar Ersaptiarso) was responsible for the firmware in this repository at R2C: holonomic motion control, per-wheel PID, odometry, the PC serial protocol, and integration of the IMU, ToF, line, IR sensors, keypad and OLED.

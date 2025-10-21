# Mr. Roboto

TODO

---

## Project Overview

This project implements:
- A
- B
- C

---

## Docker Setup
**Spin Up Container**: docker run --rm -it -v $(pwd):/workspace rpi-cc-img


**Create**: docker build --no-cache -t rpi-cc-img .


## Hardware Setup

### Microprocessor
- **Board:** Raspberry Pi 4

### Power
- **VSYS (for Pico):** Pin 2  
- **GND:** Pin 6

### I2C Encoder & IMU (Pico ↔ Pi)
| Signal | Pi Pin | Notes |
|---------|-----------|-------|
| SDA | GPIO 2 (Pin 3) | I2C1 SDA |
| SCL | GPIO 3 (Pin 5) | I2C1 SCL |

### UART PWM (Pi ↔ Pico)
| Function | Pi Pin | Notes |
|-----------|-----------|-------|
| UART0 TX (to Pico)   | GPIO 14 (Pin 8) | Sends PWM values |

---

## Software Components

- **Pose Graph Optimization** – Some Features, ...
- **Particle Filter** – Some Features, ...
- **Etc** – Some Features, ...

---

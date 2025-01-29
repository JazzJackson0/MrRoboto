

Hardware
--------

Raspberry Pi 4
Pins:
    Encoder & IMU IN [From Pico]
    + i2c1 sda: GPIO 2 (3)
    + i2c1 scl: GPIO 3 (5)
    ----------------------------
    PWM OUT [To Pico]
    + uart tx [UART0]: GPIO 14 (8)
    ----------------------------
    Power
    + vcc (For Pico): (2)
    + gnd: (6)



Cross-Compilation Fix
-------------------
PROBLEM: libopencv-dev for Docker is 4.5 and libopencv-dev for Raspberry Pi 4 is 4.6.0
SOLUTION: Created 4.5 symlinks to every 4.6.0 cmake library so executable will symlink and use 4.6.0 instead
---------------------------------------------------------------
for lib in /usr/lib/aarch64-linux-gnu/libopencv*.so.4.6*; do 
  sudo ln -s "$lib" "${lib/4.6.0/4.5}"; 
done
---------------------------------------------------------------

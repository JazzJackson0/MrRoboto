
Spin Up Container: docker run --rm -it -v $(pwd):/workspace rpi-cc-img


Create: docker build --no-cache -t rpi-cc-img .

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





Spin Up Container: docker run -it --rm --name rpi-cross-comp-container -v $(pwd):/workspace rpi4-cross-comp-img 

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




# The AAduino Zero

The AAduino Zero ([blog post](https://johan.kanflo.com/the-aaduino-zero/)) is the successor of the AAduino and is still the same size as an AA battery. This repo contains software for the AAduino Zero and will eventually hold the hardware schematics.

<p align="center">
<img src="https://raw.githubusercontent.com/kanflo/aaduino-zero/master/aadunio-zero.png" alt="Lots of AAduino Zeros!"/>
</p>

There's lots of [example code](https://github.com/kanflo/aaduino-zero/tree/master/examples) available, and a simple [serial tool](https://github.com/kanflo/aaduino-zero/tree/master/azctl) for flashing these.

### AAduino Zero specs

* STM32L052 micro controller with 32kb flash, 8kb RAM, 2kB EEPROM
* RFM68CW radio module
* TMP102 temperature sensor
* 4Mbit serial flash for sensor data logging and wireless firmware upgrades
* 32kHz oscillator for RTC
* Activity LED
* Reverse polarity protection
* 1x digital/analog I/O port
* UART port on 0.1” header
* Pre-programmed with a [serial boot loader](https://github.com/kanflo/aaduino-zero/tree/master/zeroboot) capable of handling firmware upgrades via radio or the UART.
* Minimum supply voltage: 1.8V
* Maximum supply voltage: 3.6V
* Minimum power consumption: 8μA. Yes, *eight microamps*.

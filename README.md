# The AAduino Zero

The AAduino Zero ([blog post](https://johan.kanflo.com/the-aaduino-zero/)) is the successor of the AAduino and is still the same size as an AA battery. This repo contains software for the AAduino Zero and will eventually hold the hardware schematics.

### Example code

The [transmitter](https://github.com/kanflo/aaduino-zero/tree/master/examples/transmitter) makes a temperature and battery voltage reading every 10 seconds and transmits this to a gateway or AAduino Zero running the receiver example:

```
[0] Temperature is 23.6°C, vcc is 3.40V
 Ack RSSI -26
```

The [receiver](https://github.com/kanflo/aaduino-zero/tree/master/examples/receiver) listens for incoming temperature/voltage reports and prints those:

```
From node 42: temperature:23.8°C, vcc:3.40V, rssi:-27
```

It also sends an ackmowledge to the transmitter to let it know the report was received. This is the ```ASCK RSSI``` print in the transmitter example.


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
* Pre-programmed with serial boot loader (this is a todo)
* Minimum supply voltage, 1.8V
* Maximum supply voltage, 3.6V
* Minimum power consumption is TBD


### Note

If you cloned this repo prior to November 16th 2018, please note I changed the libopencm3 remote as STM32L0 now is mature on the official master. If building fails, reclone this repo from scratch.

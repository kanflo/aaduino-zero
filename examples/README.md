# AAduino Zero Examples


### CLI

The [CLI example](https://github.com/kanflo/aaduino-zero/tree/master/examples/cli) demonstrates all the features of the AAduino Zero and it's supporting software. Build, flash, connect using 115200 8n1 and type `help\n`.

### crash

The [crash demo](https://github.com/kanflo/aaduino-zero/tree/master/examples/crash) demonstrates the crash dump feature. The goal of this feature is to have the device send a crash dump over the air allowing you to monitor the health of your nodes. The application will crash, save the crash dump in RAM and reboot. The bootloader will detect the crash dump and immediately start the application. One running fresh the application can deal with the crash dump. For now it only dumps it to the UART.

```
---
LSE ready (drive strength 1)


Welcome to the AAduino Zero Crash Example
Crashing!
---
LSE ready (drive strength 1)


Welcome to the AAduino Zero Crash Example


**** GURU MEDITATION DETECTED ****
r0    : 0x0000000a
r1    : 0x08003490
r2    : 0x20001f9c
r3    : 0x00000000
r12   : 0x20001f40
lr    : 0x080030dd
pc    : 0x08002b4e
psr   : 0x61000000

Halting
```

### lowpower

The [lowpower example](https://github.com/kanflo/aaduino-zero/tree/master/examples/lowpower) reads the temperature and supply voltage and sleeps for 30 seconds. While sleeping the device only consumes about 8μA. Please note that JTAG access during sleep will fail.

```
Welcome to the AAduino Zero Low Power Example
Found SPI flash Adesto AT45DB041E
TMP102 found, config is 0xe1a0 (12 bit)
RFM69CW found
[0] Temperature is 22.7°C, vcc is 3.30V
Sleeping 30s, we should consume ~8uA during sleep.
```

### receiver

The [receiver](https://github.com/kanflo/aaduino-zero/tree/master/examples/receiver) listens for incoming temperature/voltage reports and prints those:

```
Welcome to the AAduino Zero Receiver Example
TMP102 found, config is 0xe1a0 (12 bit)
RFM69CW found
Node 42 powered up
From node 42: temperature:22.6°C, vcc:3.40V, rssi:-27
```

It also sends an ack to the transmitter to let it know the report was received. This is the ```ASCK RSSI``` print in the transmitter example.

### transmitter

The [transmitter](https://github.com/kanflo/aaduino-zero/tree/master/examples/transmitter) makes a temperature and battery voltage reading every 30 seconds and transmits this to a gateway or AAduino Zero running the receiver example:

```
Welcome to the AAduino Zero Transmitter Example
TMP102 found, config is 0xe1a0 (12 bit)
RFM69CW found
Transmitting temperature and battery voltage every 30 seconds.
Sending powerup to gateway
 Ack RSSI -26
[0] Temperature is 22.6°C, vcc is 3.40V
 Ack RSSI -26
Sleeping
```

If building with `make CRASH=1` the firmware will crash after sending the temperature report. It will then restart and send a crash report to the gateway printing something like this:

```
Node 42 crashed at 0x08003b32 after 30 seconds, lr:0x080041f1 rssi:-27
```

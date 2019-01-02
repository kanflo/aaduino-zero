# AAduino Zero Bootloader ("zeroboot")

The bootloader is the first piece of software that starts in the AAdunio Zero and allows you to flash an application using a serial device (FTDI, CH340G, ...).

See the [zeroctl documentaton](https://github.com/kanflo/aaduino-zero/tree/master/azctl) documentaton for flashing applications.

The bootloader will stay active for 2.5 seconds at power on and flash the LED to let you know it's alive. If nothing happens on the UART it will start the application. If no applcation is found the LED will forever flash 4 times in rapid succession.


The bootloader is also used when upgrading the application over the air ("OTA"). While the application running on your AAduino Zero will download the OTA image over the radio interface, it will ask the bootloader to perform the upgrade.

Finally, the bootloader "forces" the CPU to stay awake for some time at power-on allowing you to connect your JTAG. The application will most likely take the STM32 CPU to deep sleep which prohibits JTAG access.

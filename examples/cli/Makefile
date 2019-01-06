BINARY = main

LOWPOWER := 0
ROOT = $(shell git rev-parse --show-toplevel)
COMMON = $(ROOT)/common
CFLAGS = -I. -I$(COMMON) -DCONFIG_COMMANDLINE -DCONFIG_RAM_VECTORS -DCONFIG_PAST_MINIMAL

ifeq ($(LOWPOWER),1)
  # Run in low power mode (experimental!)
  # This mode will make attaching via the BMP fail which can be fixed via
  # issuing the 'halt' command. 
  CFLAGS += -DCONFIG_LOW_POWER_MODE
endif

LDSCRIPT = $(COMMON)/stm32l052_app.ld

# Link with -Os or -O0
DEBUG := 0

OBJS = \
	$(COMMON)/past.o \
	$(COMMON)/cli.o \
	$(COMMON)/rtcdrv.o \
	$(COMMON)/sw_i2c.o \
	$(COMMON)/spi_driver.o \
	$(COMMON)/tmp102.o \
	$(COMMON)/rfm69.o \
	$(COMMON)/rflink.o \
	$(COMMON)/spiflash.o \
	$(COMMON)/hw.o \
	$(COMMON)/event.o \
	$(COMMON)/ringbuf.o \
	$(COMMON)/tick.o \
	$(COMMON)/dbg_printf.o \
	$(COMMON)/mini-printf.o \
	$(COMMON)/libopencm3-additions.o \
	$(COMMON)/lowpower.o


include $(ROOT)/libopencm3.target.mk

bmpflash:
	./bmp-flash.sh

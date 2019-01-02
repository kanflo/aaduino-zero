BINARY = main

ROOT = $(shell git rev-parse --show-toplevel)
COMMON = $(ROOT)/common
CFLAGS = -I. -I$(COMMON) -DCONFIG_RAM_VECTORS
LDSCRIPT = $(COMMON)/stm32l052_app.ld

OBJS = \
	$(COMMON)/sw_i2c.o \
	$(COMMON)/spi_driver.o \
	$(COMMON)/rtcdrv.o \
	$(COMMON)/hw.o \
	$(COMMON)/event.o \
	$(COMMON)/ringbuf.o \
	$(COMMON)/tick.o \
	$(COMMON)/crc16.o \
	$(COMMON)/bootcom.o \
	$(COMMON)/faulthandler.o \
	$(COMMON)/dbg_printf.o \
	$(COMMON)/mini-printf.o

include $(ROOT)/libopencm3.target.mk

bmpflash:
	./bmp-flash.sh

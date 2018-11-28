BINARY = main

ROOT = $(shell git rev-parse --show-toplevel)
COMMON = $(ROOT)/common
CFLAGS = -I. -I$(COMMON)
LDSCRIPT = $(COMMON)/stm32l052_app.ld

OBJS = \
	$(COMMON)/cli.o \
	$(COMMON)/sw_i2c.o \
	$(COMMON)/spi_driver.o \
	$(COMMON)/tmp102.o \
	$(COMMON)/rfm69.o \
	$(COMMON)/rfm69_link.o \
	$(COMMON)/hw.o \
	$(COMMON)/event.o \
	$(COMMON)/ringbuf.o \
	$(COMMON)/tick.o \
	$(COMMON)/dbg_printf.o \
	$(COMMON)/mini-printf.o

include $(ROOT)/libopencm3.target.mk

bmpflash:
	./bmp-flash.sh

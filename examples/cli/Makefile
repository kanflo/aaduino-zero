BINARY = main

ROOT = $(shell git rev-parse --show-toplevel)
COMMON = $(ROOT)/common
CFLAGS = -I. -I$(COMMON) -DCONFIG_COMMANDLINE

OBJS = \
	$(COMMON)/cli.o \
	$(COMMON)/spi_driver.o \
	$(COMMON)/tmp102.o \
	$(COMMON)/rfm69.o \
	$(COMMON)/spiflash.o \
	$(COMMON)/hw.o \
	$(COMMON)/event.o \
	$(COMMON)/ringbuf.o \
	$(COMMON)/tick.o \
	$(COMMON)/dbg_printf.o \
	$(COMMON)/mini-printf.o

#    past.o \

include $(ROOT)/libopencm3.target.mk

bmpflash:
	./bmp-flash.sh

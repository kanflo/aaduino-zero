BINARY = main

ROOT = $(shell git rev-parse --show-toplevel)
COMMON = $(ROOT)/common
CFLAGS = -I. -I$(COMMON) -DCONFIG_RAM_VECTORS
LDSCRIPT = $(COMMON)/stm32l052_app.ld

# Set node id with 'make NODEID=xx'
NODEID?=42

# Set gateway node id with 'make GATEWAYID=xx'
GATEWAYID?=1

# For testing the crash reporting feature
CRASH?=0

# RFM69 AES key must be 16 bytes
AESKEY=sampleEncryptKey

CFLAGS += -DCONFIG_NODEID=$(NODEID) -DCONFIG_AESKEY=\"$(AESKEY)\"
CFLAGS += -DCONFIG_GATEWAYID=$(GATEWAYID)
CFLAGS += -DCONFIG_CRASH=$(CRASH)

OBJS = \
	$(COMMON)/cli.o \
	$(COMMON)/sw_i2c.o \
	$(COMMON)/spi_driver.o \
	$(COMMON)/tmp102.o \
	$(COMMON)/rfm69.o \
	$(COMMON)/rflink.o \
	$(COMMON)/lowpower.o \
	$(COMMON)/spiflash.o \
	$(COMMON)/rtcdrv.o \
	$(COMMON)/hw.o \
	$(COMMON)/event.o \
	$(COMMON)/ringbuf.o \
	$(COMMON)/tick.o \
	$(COMMON)/faulthandler.o \
	$(COMMON)/bootcom.o \
	$(COMMON)/crc16.o \
	$(COMMON)/dbg_printf.o \
	$(COMMON)/mini-printf.o

include $(ROOT)/libopencm3.target.mk

bmpflash:
	./bmp-flash.sh

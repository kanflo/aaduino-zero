BINARY = zeroboot

GIT_VERSION := $(shell git describe --abbrev=4 --dirty --always --tags)
ROOT = $(shell git rev-parse --show-toplevel)
COMMON = $(ROOT)/common
CFLAGS = -I. -I$(COMMON) -DGIT_VERSION=\"$(GIT_VERSION)\" -DCONFIG_PAST_NO_GC
# Future optimisation: saves ~600 bytes but does not work for gcc <= 7
#CFLAGS += -flto
DEBUG = 1
CFLAGS += -ffunction-sections -fdata-sections -Wl,--gc-sections

OBJS = \
	hw.o \
	$(COMMON)/spi_driver.o \
	$(COMMON)/past.o \
	$(COMMON)/bootcom.o \
	$(COMMON)/spiflash.o \
	$(COMMON)/ringbuf.o \
	$(COMMON)/dbg_printf.o \
	$(COMMON)/mini-printf.o \
	$(COMMON)/crc16.o

# Bootloader linker script
LDSCRIPT = stm32l052_boot.ld

include ../libopencm3.target.mk

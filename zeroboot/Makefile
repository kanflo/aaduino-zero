BINARY = zeroboot

GIT_VERSION := $(shell git describe --abbrev=4 --dirty --always --tags)
ROOT = $(shell git rev-parse --show-toplevel)
COMMON = $(ROOT)/common
CFLAGS = -I. -I$(COMMON) -DGIT_VERSION=\"$(GIT_VERSION)\" -DCONFIG_PAST_NO_GC
# Future optimisation: saves ~600 bytes but does not work for gcc <= 7
#CFLAGS += -flto
DEBUG = 0
CFLAGS += -ffunction-sections -fdata-sections -Wl,--gc-sections

# When developing FWU support, it's convenient to download FWU images over the UART
#CFLAGS += -DCONFIG_BOOTLOADER_UART_FWU_SUPPORT

OBJS = \
	hw.o \
	$(COMMON)/spi_driver.o \
	$(COMMON)/past.o \
	$(COMMON)/bootcom.o \
	$(COMMON)/spiflash.o \
	$(COMMON)/ringbuf.o \
	$(COMMON)/fwupgrade.o \
	$(COMMON)/tick.o \
	$(COMMON)/uframe.o \
	$(COMMON)/crc16.o \
	$(COMMON)/libopencm3-additions.o
#	$(COMMON)/dbg_printf.o
#	$(COMMON)/mini-printf.o

# Bootloader linker script
LDSCRIPT = stm32l052_boot.ld

include ../libopencm3.target.mk

/* 
 * The MIT License (MIT)
 * 
 * Copyright (c) 2017 Johan Kanflo (github.com/kanflo)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#include "dbg_printf.h"
#include "spiflash.h"


/** @todo: this module is very much work in progress */

#ifdef CONFIG_SPIFLASH_DEBUG
 #define flash_printf(...) dbg_printf(...)
#else // CONFIG_SPIFLASH_DEBUG
 #define dbg_printf(...)
#endif // CONFIG_SPIFLASH_DEBUG

#define CMD_GETID            (0x9f)

#define CMD_PROGRAM_ERASE_BUFFER1  (0x83)
#define CMD_WRITE_BUFFER1          (0x84)
#define CMD_READ_STATUS            (0xd7)
#define CMD_READ_DATA              (0x03)



//#define CMD_PAGE_PROGRAM     (0x02)
//#define CMD_READ_DATA_HISPD  (0x0b)
//#define CMD_ERASE_SECTOR     (0xd8) // 4kb
//#define CMD_ERASE_SUBSECTOR  (0x20) // 64kb
//#define CMD_ERASE_CHIP       (0xc7)

#define STATUS_BUSY           (1 << 7)
#define STATUS_COMP           (1 << 6)
#define STATUS_DENSITY        (1 << 2)
#define STATUS_PROTECT        (1 << 1)
#define STATUS_PAGE_SIZE_256  (1 << 0)


typedef struct {
    uint8_t manufacturer;
    uint16_t jedec_id;
    uint32_t page_count;
    uint32_t page_size;
    char *description;
} spi_flash_t;


static void cs_assert(bool assert);
static uint8_t read_status(void);
static int8_t check_flash(uint8_t manufacturer, uint16_t jedec_id);
//static void flash_cmd(int8_t cmd);
static void set_page_size_256(void);

// Set when probing
static int8_t flash_idx = -1;

// List of supported flashes. Feel free to add the flash of your heart's desire
static const spi_flash_t flashes[] = {
    { 0x1f, 0x2400, 2048, 256, "Adesto AT45DB041E" },
    { 0, 0, 0, 0, 0 } // End marker
};


bool spiflash_probe(void)
{
    uint8_t info[5]; // 5 for AT45DB041E, others TBD
    cs_assert(true);
    (void) spi_xfer(SPI1, CMD_GETID);
    for (uint8_t i = 0; i < sizeof(info); i++) {
        info[i] = spi_xfer(SPI1, 0);
    }
    cs_assert(false);
    flash_idx = check_flash(info[0], info[1] << 8 | info[2]);
    if (flash_idx >= 0) {
        set_page_size_256();
    }
    return flash_idx >= 0;
}

const char *spiflash_get_desc(void)
{
    if (flash_idx >= 0) {
        return flashes[flash_idx].description;
    } else {
        return "No SPI flash found";
    }
}


bool spiflash_read(uint32_t address, uint32_t length, uint8_t *buffer)
{
    bool success = false;
    if (buffer && length) {
        dbg_printf("Reading %u bytes from 0x%08x\n", length, address);
        cs_assert(true);
        (void) spi_xfer(SPI1, CMD_READ_DATA);
        (void) spi_xfer(SPI1, (address >> 16) & 0xff);
        (void) spi_xfer(SPI1, (address >> 8) & 0xff);
        (void) spi_xfer(SPI1, address & 0xff);
        while(length--) {
            *(buffer++) = (uint8_t) spi_xfer(SPI1, 0);
        }
        cs_assert(false);
        success = true;
    }
    return success;
}

bool spiflash_write(uint32_t address, uint32_t length, uint8_t *buffer)
{
    bool success = false;
    if (flash_idx >= 0) {
        int32_t remain = length;
        dbg_printf("Writing %u bytes to 0x%08x\n", length, address);
        while(remain > 0) {
            uint32_t chunk_size = (((uint32_t) remain) > flashes[flash_idx].page_size) ? (flashes[flash_idx].page_size) : ((uint32_t) remain);

            cs_assert(true);
            dbg_printf("  %u bytes at 0x%08x (page %d)\n", chunk_size, address, address / flashes[flash_idx].page_size);
            (void) spi_xfer(SPI1, CMD_WRITE_BUFFER1);

            (void) spi_xfer(SPI1, (address >> 16) & 0xff);
            (void) spi_xfer(SPI1, (address >> 8) & 0xff);
            (void) spi_xfer(SPI1, address & 0xff);

            dbg_printf("    ");
            while(chunk_size--) {
                uint8_t ch = *(buffer++);
                dbg_printf(" %02x", ch);
                if (chunk_size % 32 == 0) {
                    dbg_printf("\n    ");
                }
                (void) spi_xfer(SPI1, ch);
            }
            cs_assert(false);

            cs_assert(true);
            dbg_printf("Writing to page %d\n", page_idx);
            (void) spi_xfer(SPI1, CMD_PROGRAM_ERASE_BUFFER1);
            (void) spi_xfer(SPI1, (address >> 16) & 0xff);
            (void) spi_xfer(SPI1, (address >> 8) & 0xff);
            (void) spi_xfer(SPI1, address & 0xff);
            cs_assert(false);


            uint32_t wait_counter = 0;
            while(!(read_status() & STATUS_BUSY)) {
                // TODO: Handle timeout
                wait_counter++;
                delay_ms(1);
            }
            dbg_printf("Duration %dms\n", wait_counter);
            address += flashes[flash_idx].page_size;
            remain -= flashes[flash_idx].page_size;
        }
        success = true;
    }
    return success;
}

bool spiflash_erase(uint32_t address, uint32_t length)
{
    bool success = false;
    (void) address;
    (void) length;
#if 0
    if (flash_idx >= 0) {
        uint32_t address_aligned = address & ~(SUBSECTOR_SIZE-1);
        uint32_t end_address = address + length;
        length += address & ~(SUBSECTOR_SIZE-1);
        dbg_printf("Erasing %u bytes at 0x%08x\n", length, address_aligned);
        while (end_address > address_aligned) {
            cs_assert(true);
            // TODO: Optimise by erasing sectors when possible (~2x erase speed)
            (void) spi_xfer(SPI1, CMD_ERASE_SUBSECTOR);
            dbg_printf("  Erasing subsector at 0x%08x\n", address_aligned);
            (void) spi_xfer(SPI1, address_aligned >> 16);
            (void) spi_xfer(SPI1, address_aligned >> 8);
            (void) spi_xfer(SPI1, address_aligned);
            cs_assert(false);
            delay_ms(70); // Subsector erase takes 70-150ms
            while(read_status() & STATUS_WIP) {
            // TODO: Handle timeout
                delay_ms(5);
            }
            dbg_printf("    Done\n");
            address_aligned += SUBSECTOR_SIZE;
        }
        success = true;
    }
#endif
    return success;
}

bool spiflash_chip_erase(void)
{
    bool success = false;
#if 0
    if (flash_idx >= 0) {
        dbg_printf("Erasing chip\n");
        uint8_t status;
        flash_cmd(CMD_ERASE_CHIP);
        while(1) {
            // TODO: Handle timeout
            status = read_status();
            if (!(status & STATUS_WIP)) {
                dbg_printf("Erase done\n");
                return true;
            }
            delay_ms(25);
        }
        success = true;
    }
#endif
    return success;
}

static void cs_assert(bool assert)
{
    if (assert) {
        gpio_clear(SPI1_FLASH_CS_PORT, SPI1_FLASH_CS_PIN);
    } else {
        gpio_set(SPI1_FLASH_CS_PORT, SPI1_FLASH_CS_PIN);
    }
}

static int8_t check_flash(uint8_t manufacturer, uint16_t jedec_id)
{
    uint8_t rover = 0;
    do {
        if (flashes[rover].manufacturer == manufacturer &&
            flashes[rover].jedec_id == jedec_id) {
            return rover;
        }
        rover++;
    } while(flashes[rover].manufacturer != 0);
    return -1;
}

static uint8_t read_status(void)
{
    uint8_t status = 0;
    cs_assert(true);
    (void) spi_xfer(SPI1, CMD_READ_STATUS);
    status = spi_xfer(SPI1, 0);
    cs_assert(false);
    return status;
}

#if 0
static void flash_cmd(int8_t cmd)
{
    cs_assert(true);
    (void) spi_xfer(SPI1, (uint8_t) cmd);
    cs_assert(false);
}
#endif

static void set_page_size_256(void)
{
    if (!(read_status() & STATUS_PAGE_SIZE_256)) {
        dbg_printf("Switching page size to 256 bytes\n");
        cs_assert(true);
        (void) spi_xfer(SPI1, 0x3d);
        (void) spi_xfer(SPI1, 0x2a);
        (void) spi_xfer(SPI1, 0x80);
        (void) spi_xfer(SPI1, 0xa6);
        cs_assert(false);
        while(!(read_status() & STATUS_BUSY)) {
            //delay_ms(1); // TODO: Handle timeout
        }
    }
}

/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Johan Kanflo (github.com/kanflo)
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

#include <flash.h>
#include "fwupgrade.h"
#include "pastunits.h"
#include "spiflash.h"
#include "crc16.h"
//#include "dbg_printf.h"
#include "libopencm3-additions.h"


/** Address in the serial flash where we store the images */
#define FWU_FLASH_START   (0)
/** Using 128kb for downloading gives some wear levelling */
#define FWU_FLASH_SIZE    (128*1024)

/** Firmware upgrade stats */
typedef enum {
    /** Idle, nothing going on */
    fwu_state_idle = 0,
    /** Currently downloading, past unit fwu_download_address points to the
     *  beginning of the download area. Goto fwu_state_ready when done.
     */
    fwu_state_downloading,
    /** We have a downloaded firmware in flash and are ready to run the upgrade.
     */
    fwu_state_ready,
    /** Currently flashing downloaded image to internal flash.
     *  Goto fwu_state_testing when dobe
     */
    fwu_state_upgrading,
    /** Currently running the upgraded image in test mode. From here we can
     *  go to fw_state_idle if image deemed ok or go to fwu_state_downgrading
     *  if we crash.
     */
    fwu_state_testing,
    /** Currently flashing backup image to internal flash. Goto fwu_state_idle
     *  when done.
     */
    fwu_state_downgrading,
    /** End marker */
    fwu_state_last,
} fwu_state_t;

/** From the linker file */
extern long flash_size, app_start, app_size, flash_block_size;

/** This struct is stored in the external serial flash with the image following
 *  directly after. We store two images, the downloaded one and the backup.
 *  Addresses are stored in past units fwu_download_address and
 *  fwu_backup_address.
 */
typedef struct {
    /** Expecting this one to be FWU_MAGIC */
    uint32_t magic;
    /** Downloaded firmware (1) or a backup (0) */
    uint32_t is_firmware;
    /** Size of image */
    uint32_t size;
    /** CRC16 of image (yes, could be a uint16_t but I wanted to get rid of
     *  compiler optimised packed structs)
     */
    uint32_t crc16;
} fwu_header_t;

/** Past provided by the application */
past_t *g_past = 0;

/** Size of a downloaded image (matches internal flash size according to
 *  linker file
 */
uint32_t g_image_size = 0;

/** Address in flash where we're currently downloading */
uint32_t g_dl_address = 0;

/** Size of image being downloaded */
uint32_t g_dl_size = 0;

/** Number of bytes downloaded */
uint32_t g_dl_count = 0;


/** State convenience funtions */
static fwu_state_t get_state(void);
static void set_state(fwu_state_t state);


/**
 * @brief      Initialize the firmware upgrade module
 *
 * @param      past  System parameter storage
 */
void fwu_init(past_t *past)
{
    g_past = past;
    g_image_size = (uint32_t) &flash_size;
}

/**
 * @brief      Check if we are downloading a new firmware image
 *
 * @return     true if we are
 */
bool fwu_is_downloading(void)
{
    return get_state() == fwu_state_downloading;
}

/**
 * @brief      Start a download
 *
 * @param[in]  size   Size of received image
 * @param[in]  crc16  16 bit CRC of image
 *
 * @return     true if download can commence
 */
bool fwu_start_download(uint16_t size, uint16_t crc16)
{
    bool success = false;
    do {
        if (size > (uint32_t) &app_size) {
            //dbg_printf("# DL size %d too large\n", size);
            break;
        }

        if (past_read_uint32(g_past, fwu_download_address, &g_dl_address)) {
            /** Download in next slot */
            g_dl_address += g_image_size;
            if (g_dl_address >= FWU_FLASH_START + FWU_FLASH_SIZE) {
                /** This should not happen */
                g_dl_address = FWU_FLASH_START;
            } else {
                if (g_dl_address + g_image_size > FWU_FLASH_START + FWU_FLASH_SIZE) {
                    /** To keep things simple, we do not deal with wrapping the
                     *  image in the remaining flash but rather start fresh at the
                     *  beginning
                     */
                    g_dl_address = FWU_FLASH_START;
                }
            }
        } else {
            /** Unit not found */
            g_dl_address = FWU_FLASH_START;
        }
        //dbg_printf("# Downloading to %d\n", g_dl_address);

        if (!past_write_uint32(g_past, fwu_download_address, g_dl_address)) {
            /** @todo: handle past failures */
            //dbg_printf("# Past failure at %d\n", __LINE__);
            break;
        }

        fwu_header_t header;
        header.magic = FWU_MAGIC;
        header.is_firmware = true;
        header.size = size;
        header.crc16 = crc16;

        /** Make room for the image */
        if (!spiflash_erase(g_dl_address, g_image_size)) {
            //dbg_printf("# Failed erasing %d bytes at 0x%08x\n", g_image_size, g_dl_address);
            /** @todo: handle flash failures */
            break;
        }

        /** Write image header */
        if (!spiflash_write(g_dl_address, sizeof(header), (uint8_t*) &header)) {
            //dbg_printf("# Failed writing header\n");
            /** @todo: handle flash failures */
            break;
        } else {
            g_dl_count = 0;
            g_dl_size = size;
            g_dl_address += sizeof(header);
#if 0
            /** @todo: handle persistance */
            set_state(fwu_state_downloading);
#endif
        }

        success = true;
    } while(0);

    return success;
}

/**
 * @brief      Called when we got a chunk of data
 *
 * @param      data  Pointer to data
 * @param[in]  size  Size of data
 */
void fwu_got_data(uint8_t *data, uint8_t size)
{
    if (!spiflash_write(g_dl_address, size, data)) {
        //dbg_printf("# Failed writing %d bytes at 0x%08x\n", size, g_dl_address);
        /** @todo: handle flash failures */
    } else {
        g_dl_count += size;
        g_dl_address += size;
    }

    if (fwu_download_complete()) {
        set_state(fwu_state_ready);
        //dbg_printf("# Download complete\n");
    }
}

/**
 * @brief      Check if download is complete meaning we have received all
 *             expected data.
 *
 * @return     true if we have
 */ 
bool fwu_download_complete(void)
{
    return g_dl_count >= g_dl_size;
}

/**
 * @brief      Run the upgrade
 *
 * @return     true if upgrade went well
 */
bool fwu_run_upgrade(void)
{
    uint32_t flash_start = (uint32_t) &app_start;
    uint32_t flash_end = (uint32_t) &app_start + (uint32_t) &app_size;
    uint32_t write_address = flash_start;
    uint32_t read_address = 0;
    uint32_t block_size = (uint32_t) &flash_block_size;
    fwu_header_t header;
    bool success = false;
    do {
        fwu_state_t state = get_state();
        if (state != fwu_state_ready && state != fwu_state_upgrading) {
            //dbg_printf("# Upgrade failed: wrong state %d\n", state);
            break;
        }

        if (!past_read_uint32(g_past, fwu_download_address, &read_address)) {
            //dbg_printf("# Upgrade failed: no dl address\n");
            break;
        }

        if (!spiflash_read(read_address, sizeof(header), (uint8_t*) &header)) {
            //dbg_printf("# Upgrade failed: failed to read header\n");
            break;
        }

        if (header.magic != FWU_MAGIC) {
            //dbg_printf("# Upgrade failed: wrong magic %08x\n", header.magic);
            break;
        }

        /** @todo: create backup of running firmware */

        /** Indicate the internal flash is about to be upgraded */
        set_state(fwu_state_upgrading);

        flash_unlock();
        //dbg_printf("# Erasing internal flash\n");
        for (uint32_t addr = flash_start; addr < flash_end; addr += block_size) {
            flash_erase_page(addr);
        }

        read_address += sizeof(header);
        uint8_t block[block_size];

        //dbg_printf("# Flashing\n");
        for (uint32_t i = 0; i < header.size; i+= block_size) {
            if (!spiflash_read(read_address, block_size, (uint8_t*) &block)) {
                //dbg_printf("# Upgrade failed: failed to read data\n");
                flash_lock();
                break;
            }
            for (uint32_t j = 0; j < block_size; j += 4) {
                uint32_t word = block[j+3] << 24 | block[j+2] << 16 | block[j+1] << 8 | block[j];
                flash_program_word(write_address + j, word);
            }

            read_address += block_size;
            write_address += block_size;

        }
        flash_lock();
        //dbg_printf("# Done!\n");

        uint16_t calc_crc = crc16((uint8_t*) flash_start, header.size);
        if (calc_crc != header.crc16) {
            //dbg_printf("# CRC mismatch: header:%04x flash:%04x\n", header.crc16, calc_crc);
            /** @todo: handle downgrade */
            break;
        }

        set_state(fwu_state_testing);
        success = true;
    } while(0);
    return success;
}

/**
 * @brief      Backup the running firmware
 *
 * @return     true if backup went well
 */
bool fwu_run_backup(void)
{
    return false;
}

/**
 * @brief      Restore the firmware in the backup
 *
 * @return     true if restore went well
 */
bool fwu_run_downgrade(void)
{
    return false;
}

static fwu_state_t get_state(void)
{
    fwu_state_t state = fwu_state_idle;
    uint32_t temp;
    if (past_read_uint32(g_past, fwu_state, &temp)) {
        if (temp < fwu_state_last) {
            state = (fwu_state_t) temp;
            //dbg_printf("# Read fwu state %d\n", state);
        } else {
            /** Should not happen */
            state = fwu_state_idle;
            //dbg_printf("# Read illegal fwu state %d\n", temp);
        }
    }
    //dbg_printf("# State is %d\n", state);
    return state;
}

static void set_state(fwu_state_t state)
{
    //dbg_printf("# Setting state to %d\n", state);
    uint32_t temp = (uint32_t) state;
    if (!past_write_uint32(g_past, fwu_state, temp)) {
        /** @todo: handle past failures */
    }
}

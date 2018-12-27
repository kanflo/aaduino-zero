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

#include "fwupgrade.h"
#include "pastunits.h"


/** Address in the serial flash where we store the images */
#define FWU_FLASH_START   (0)
/** Using 100kb for downloading gives some wear levelling */
#define FWU_FLASH_SIZE    (100*1024)

/** Firmware upgrade stats */
typedef enum {
    /** Currently we have no clue, need to check psat */
    fwu_state_unknown = 0,
    /** Idle, nothing going on */
    fwu_state_idle,
    /** Currently downloading, past unit fwu_download_address points to the
     *  beginning of the download area.
     */
    fwu_state_downloading,
    /** Currently running backup, past unit fwu_backup_address points to the
     *  beginning of the backup area.
     */
    fwu_state_backupping,
    /** Currently flashing downloaded image to internal flash */
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
} fwu_state_t;


#define FWU_MAGIC (0xf07ad07a)

/** This struct is stored in the external flash with the image following
 *  directly after. We store two images, the downloaded one and the backup.
 *  Addresses are stored in past units fwu_download_address and
 *  fwu_backup_address.
 */
typedef struct {
    /** Expecting this one to be FWU_MAGIC */
    uint32_t magic;
    /** Downloaded firmware (1) or a backup (0) */
    uint32_t is_download;
    /** Size of image */
    uint32_t size;
    /** CRC16 of image (yes, could be a uint16_t but I wanted to get rid of
     *  compiler optimised packed structs)
     */
    uint32_t crc16;
} fwu_header_t;

/** Completely clueless at boot */
fwu_state_t g_state = fwu_state_unknown;

/** Past provided by the application */
past_t *g_past = 0;


/**
 * @brief      Initialize the firmware upgrade module
 *
 * @param      past  System parameter storage
 */
void fwu_init(past_t *past)
{
    g_past = past;
}

/**
 * @brief      Check if we are downloading a new firmware image
 *
 * @return     true if we are
 */
bool fwu_is_downloading(void)
{
    return false;
}

/**
 * @brief      Start a download
 *
 * @param[in]  size   Size of received image
 * @param[in]  crc16  16 bit CRC of image
 */
void fwu_start_download(uint16_t size, uint16_t crc16)
{
    (void) size;
    (void) crc16;
}

/**
 * @brief      Called when we got a chunk of data
 *
 * @param      data  Pointer to data
 * @param[in]  size  Size of data
 */
void fwu_got_data(uint8_t *data, uint8_t size)
{
    (void) data;
    (void) size;
}

/**
 * @brief      Check if download is complete meaning we have received all
 *             expected data.
 *
 * @return     true if we have
 */ 
bool fwu_download_complete(void)
{
    return false;
}

/**
 * @brief      Run the upgrade
 *
 * @return     true if upgrade went well
 */
bool fwu_run_upgrade(void)
{
    return false;
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

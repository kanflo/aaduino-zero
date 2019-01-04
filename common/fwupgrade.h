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

#ifndef __FWUPGRADE_H__
#define __FWUPGRADE_H__

#include <stdint.h>
#include <stdbool.h>
#include "past.h"

#define FWU_MAGIC (0xf07ad07a)

/**
 * @brief      Initialize the firmware upgrade module
 *
 * @param      past  System parameter storage
 */
void fwu_init(past_t *past);

/**
 * @brief      Check if we are downloading a new firmware image
 *
 * @return     true if we are
 */
bool fwu_is_downloading(void);

/**
 * @brief      Start a download
 *
 * @param[in]  size   Size of received image
 * @param[in]  crc16  16 bit CRC of image
 *
 * @return     true if download can commence
 */
bool fwu_start_download(uint16_t size, uint16_t crc16);

/**
 * @brief      Called when we got a chunk of data
 *
 * @param      data  Pointer to data
 * @param[in]  size  Size of data
 */
void fwu_got_data(uint8_t *data, uint8_t size);

/**
 * @brief      Check if download is complete meaning we have received all
 *             expected data.
 *
 * @return     true if we have
 */
bool fwu_download_complete(void);

/**
 * @brief      Run the upgrade
 *
 * @return     true if upgrade went well
 */
bool fwu_run_upgrade(void);

/**
 * @brief      Backup the running firmware
 *
 * @return     true if backup went well
 */
bool fwu_run_backup(void);

/**
 * @brief      Restore the firmware in the backup
 *
 * @return     true if restore went well
 */
bool fwu_run_downgrade(void);

#endif // __FWUPGRADE_H__

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

#ifndef __SPIFLASH_H__
#define __SPIFLASH_H__

#include <stdbool.h>
#include <stdint.h>
#include <spi.h>
#include <gpio.h>
#include "dbg_printf.h"
#include "hw.h"
#include "tick.h"

/**
 * @brief      Probe for SPI flash
 *
 * @return     true if supported flash was found, false otherwise
 */
bool spiflash_probe(void);

/**
 * @brief      Get flash description
 *
 * @return     Flash description string
 */
const char *spiflash_get_desc(void);

/**
 * @brief      Read from serial flash
 *
 * @param[in]  address  Address to read from
 * @param[in]  length   Number of bytes to read
 * @param      buffer   Buffer to store data in
 *
 * @return     true if all went well
 */
bool spiflash_read(uint32_t address, uint32_t length, uint8_t *buffer);

/**
 * @brief      Write data to serial flash, will erase if needed
 *
 * @param[in]  address  Address to write from
 * @param[in]  length   Number of bytes to write
 * @param      buffer   Buffer holding data
 *
 * @return     true if all went well
 */
bool spiflash_write(uint32_t address, uint32_t length, uint8_t *buffer);

/**
 * @brief      Erase flash page/pages
 *
 * @param[in]  address  The address, must be page aligned
 * @param[in]  length   The length, must be page aligned
 *
 * @return     true if all went well
 */
bool spiflash_erase(uint32_t address, uint32_t length);

/**
 * @brief      Erase complete chip
 *
 * @return     true if all went well
 */
bool spiflash_chip_erase(void);

/**
 * @brief      Tell flash to go to sleep
 *
 * @return     void
 */
void spiflash_sleep(void);

/**
 * @brief      Tell flash to wake up
 *
 * @return     void
 */
void spiflash_wakeup(void);


#endif // __SPIFLASH_H__

/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Johan Kanflo (github.com/kanflo)
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

#ifndef __BOOTCOM_H__
#define __BOOTCOM_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * The bootcom modules allows for storing data in a small area of RAM not
 * cleared on reset or when the application starts. An application can use this
 * area for informing the bootloader it is time to run a firmware upgrade or
 * it can be used for an application to store a crash dump to be handled
 * by itself on following a system reset.
 *
 * The location and size of the bootcom area is specified in the linker file
 * and the bootcom module extects the symbols bootcom_start and bootcom_size,
 * the latter one being in bytes.
 *
 * Placing data in bootcom:
 *
 * bootcom_clear();
 * bootcom_put(0x12345678); // Application specific magic
 * bootcom_put(1);          // Application specific data
 * bootcom_put(2);          // Application specific data
 *
 * Obtaining data:
 *
 * uint32_t num_words = bootcom_size();
 * if (num_words) {
 *   magic = bootcom_get(0);
 *   data1 = bootcom_get(1);
 *   data2 = bootcom_get(2);
 * }
 *
 */

/**
  * @brief Clear bootcom area
  * @retval void
  */
void bootcom_clear(void);

/**
  * @brief Put data into bootcom buffer and update buffer crc
  * @param data data to place into next position of buffer. Places no data if
  *             bootcom is full.
  * @retval void
  */
void bootcom_put(uint32_t data);

/**
 * @brief Check if we have bootcom, return length in words
 *
 * @return number of words in bootcom. 0 if no data exists
 */
uint32_t bootcom_get_size(void);

/**
  * @brief Get data from bootcom buffer. Assumes user has validated using
  *        bootcom_size() so index is valid
  * @param index, index of data
  * @retval data at location
  */
uint32_t bootcom_get(uint8_t index);

#endif // __BOOTCOM_H__

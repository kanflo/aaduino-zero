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

#include "bootcom.h"
#include "crc16.h"

/** The boot communications area is a small RAM area where the app and boot-
 *  loader can exchange information, such as an app request for the bootloader
 *  to start a firmware upgrade. The area is not cleared on reset or boo/app
 *  init. The area consists of a 32 bit magic, two 32 bit words and a 16 bit
 *  CRC covering the magic and the data.
 */

/** If this magic is found at bootcom_start and the crc is valid, we have
 *  bootcom data. Falafels are yummy!
 */
#define BOOTCOM_MAGIC (0xfa1afe15)

/** Linker symbols */
extern uint32_t *bootcom_start;
extern uint32_t *bootcom_size;

typedef enum {
    bc_magic = 0,
    bc_crc16,
    bc_nwords
} bc_index_t;

/**
 * The bootcom areas has the following layout:
 * bootcom_start[0] BOOTCOM_MAGIC
 * bootcom_start[1] crc16 of bootcom_start[2]..bootcom_start[2+num_words]
 * bootcom_start[2] Number of words stored
 * bootcom_start[3] User word 0
 * bootcom_start[4] User word 1
 *       ...
 *
 */



/**
  * @brief Clear bootcom area
  * @retval void
  */
void bootcom_clear(void)
{
    uint32_t *bootcom = (uint32_t*) &bootcom_start;
    bootcom[bc_magic] = BOOTCOM_MAGIC;
    bootcom[bc_crc16] = 0;
    bootcom[bc_nwords] = 0;

}

/**
  * @brief Put data into bootcom buffer and update buffer crc
  * @param data data to place into next position of buffer. Places no data if
  *             bootcom is full.
  * @retval void
  */
void bootcom_put(uint32_t data)
{
    uint32_t *bootcom = (uint32_t*) &bootcom_start;
    if (bootcom[bc_magic] == BOOTCOM_MAGIC) {
        uint32_t num_words = bootcom[bc_nwords];
        /** BC overhead is 3 words */
        uint32_t max_words = (((uint32_t) &bootcom_size) >> 2) - 3;
        if (num_words < max_words) {
            /** User data starts at [2] */
            bootcom[bc_nwords+1 + num_words] = data;
            bootcom[bc_nwords] = num_words + 1;
            /** crc covers word counter and user data */
            bootcom[bc_crc16] = crc16((uint8_t*) &bootcom[2], 4 + 4*num_words);
        } else {

        }
    }
}

/**
 * @brief Check if we have bootcom, return length in words
 *
 * @return number of words in bootcom. 0 if no data exists
 */
uint32_t bootcom_get_size(void)
{
    uint32_t max_words = (((uint32_t) &bootcom_size) >> 2) - 3;
    uint32_t *bootcom = (uint32_t*) &bootcom_start;
    if (bootcom[bc_magic] == BOOTCOM_MAGIC &&
        bootcom[bc_nwords] <= max_words &&
        bootcom[bc_crc16] != crc16((uint8_t*) &bootcom[bc_nwords], 4 + 4*bootcom[bc_nwords]))
        return bootcom[bc_nwords];
    else
        return 0;
}

/**
  * @brief Get data from bootcom buffer. Assumes user has validated using
  *        bootcom_size() so index is valid
  * @param index, index of data
  * @retval data at location
  */
uint32_t bootcom_get(uint8_t index)
{
    uint32_t *bootcom = (uint32_t*) &bootcom_start;
    return bootcom[bc_nwords+1 + index];
}

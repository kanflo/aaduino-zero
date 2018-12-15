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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <rcc.h>
#include <scb.h>
#include <gpio.h>
#include <stdlib.h>
#include <flash.h>
#include "hw.h"
#include "ringbuf.h"
#include "past.h"
#include "bootcom.h"
#include "crc16.h"
#include "spiflash.h"


/**
 * This is the AAduino Zero bootloader that eventually will take care of
 * firmware upgrades either via the UART FOTA packages stored in the external
 * flash.
 * 
 * Right now we only flash the LED a couple of times and jump to the app though.
 *
 */


/** Linker file symbols */
extern long _app_start;
extern long _app_end;
extern long _past_start;
extern long _past_end;
extern long _bootcom_start;
extern long _bootcom_end;
extern long past_start, past_block_size;

static past_t g_past;

#define RX_BUF_SIZE  (16)
static ringbuf_t rx_buf;
static uint8_t rx_buffer[2*RX_BUF_SIZE];

/**
  * @brief Branch to main application
  * @retval false if app start failed
  */
static bool start_app(void)
{
    /** Branch to the address in the reset entry of the app's exception vector */
    uint32_t *app_start = (uint32_t*) (4 + (uint32_t) &_app_start);
    /** Is there something there we can branch to? */
    if (((*app_start) & 0xffff0000) == 0x08000000) {
        /** Initialize stack pointer of user app */
        volatile uint32_t *sp = (volatile uint32_t*) &_app_start;
        __asm (
            "mov sp, %0\n" : : "r" (*sp)
        );
        ((void (*)(void))*app_start)();
    }
    return false;
}

/**
 * @brief      Indicate system has halted
 */
static void halt(uint32_t count)
{
    while(1) {
        for (volatile int j = 0; j < 1500000; j++) ;
        for (uint32_t i = 0; i < count; i++) {
            hw_set_led(true);
            for (volatile int j = 0; j < 100000; j++) ;
            hw_set_led(false);
            for (volatile int j = 0; j < 100000; j++) ;
        }
    }
}

/**
  * @brief Ye olde main
  * @retval preferably none
  */
int main(void)
{
    ringbuf_init(&rx_buf, (uint8_t*) rx_buffer, sizeof(rx_buffer));
    hw_init(&rx_buf);
    hw_spi_init();
    dbg_printf("AAduino Zero Bootloader\n");
    if (!spiflash_probe()) {
        dbg_printf("SPI flash probe failed!\n");
        halt(2);
    }

    g_past.blocks[0] = (uint32_t) &past_start;
    g_past.blocks[1] = (uint32_t) &past_start + (uint32_t) &past_block_size;
    g_past._block_size = (uint32_t) &past_block_size;
    if (!past_init(&g_past)) {
        dbg_printf("Past init failed!\n");
        halt(3);
    }

    uint32_t foo, bar;
    if (bootcom_get(&foo, &bar)) {
        /** @todo: handle */
    }

#if 0
    /** Temporary LED blinking to show we're alive */
    for (int i = 0; i < 5; i++) {
        for (volatile int j = 0; j < 500000; j++) ;
        hw_set_led(true);
        for (volatile int j = 0; j < 100000; j++) ;
        hw_set_led(false);
    }
#endif

    hw_deinit();
    (void) start_app();
    dbg_printf("App returned :-/\n");

    /** Nothing to boot */
    halt(4);
    return 0;
}

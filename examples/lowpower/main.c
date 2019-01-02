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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <flash.h>
#include "bootcom.h"
#include "tick.h"
#include "spiflash.h"
#include "dbg_printf.h"
#include "rtcdrv.h"
#include "hw.h"
#include "tmp102.h"
#include "rfm69.h"
#include "lowpower.h"

static void blinken_halt(uint32_t blink_count)
{
    while(1) {
        for (uint32_t i = 0; i < blink_count; i++) {
            hw_set_led(true);
            delay_ms(100);
            hw_set_led(false);
            delay_ms(100);
        }
        delay_ms(1000);
    }
}

static void check_crash(void)
{
    if (bootcom_get_size() > 0) {
        if (bootcom_get(0) == 0xdeadc0de) {
            dbg_printf("\n\n**** GURU MEDITATION DETECTED ****\n");
            dbg_printf("r0    : 0x%08x\n", bootcom_get(1));
            dbg_printf("r1    : 0x%08x\n", bootcom_get(2));
            dbg_printf("r2    : 0x%08x\n", bootcom_get(3));
            dbg_printf("r3    : 0x%08x\n", bootcom_get(4));
            dbg_printf("r12   : 0x%08x\n", bootcom_get(5));
            dbg_printf("lr    : 0x%08x\n", bootcom_get(6));
            dbg_printf("pc    : 0x%08x\n", bootcom_get(7));
            dbg_printf("psr   : 0x%08x\n", bootcom_get(8));
            bootcom_clear();
            dbg_printf("\nHalting\n");
            blinken_halt(2);
        }
    }
}

/**
  * @brief Ye olde main
  * @retval preferably none
  */
int main(void)
{
    uint32_t counter = 0;
    uint32_t period = 30;
    hw_init(0);
    dbg_printf("\n\nWelcome to the AAduino Zero Low Power Example\n");

    check_crash();

    if (!spiflash_probe()) {
        dbg_printf("No SPI flash found\n");
    } else {
        dbg_printf("Found SPI flash %s\n", spiflash_get_desc());
    }

    (void) tmp102_init();
    rtcdrv_init();
    rtcdrv_set_wakeup(1);

    rfm69_setResetPin(RFM_RESET_PORT, RFM_RESET_PIN);
    rfm69_reset();
    if (!rfm69_init(SPI1_RFM_CS_PORT, SPI1_RFM_CS_PIN, false)) {
        dbg_printf("No RFM69CW found\n");
    } else {
        dbg_printf("RFM69CW found\n");
        rfm69_sleep(); // init RF module and put it to sleep
        rfm69_setPowerDBm(13); // // set output power, +13 dBm
        rfm69_setCSMA(true); // enable CSMA/CA algorithm
        rfm69_setAutoReadRSSI(true);
        (void) rfm69_setAESEncryption((void*) "sampleEncryptKey", 16);
    }

    while(1) {
        uint16_t vcc = vcc_measure();
        int32_t t = 1000 * tmp102_readTempC();
        int32_t frac = (t%1000)/100;
        if (frac < 0) {
            frac = -frac;
        }

        hw_set_led(true);
        dbg_printf("[%d] Temperature is %d.%d°C, vcc is %d.%02dV\n", counter, t/1000, frac, vcc/1000, (vcc%1000)/10);
        hw_set_led(false);

        dbg_printf("Sleeping %ds, we should consume ~8uA during sleep.\n", period);
        lp_sleep(period);
        dbg_printf("*yawn*\n");

        counter++;
    }

    blinken_halt(1);
    return 0;
}

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
#include "dbg_printf.h"
#include "hw.h"
#include "tick.h"
#include "tmp102.h"
#include "rfm69.h"
#include "spiflash.h"


#define RX_BUF_SIZE  (16)
static ringbuf_t rx_buf;
static uint8_t rx_buffer[2*RX_BUF_SIZE];

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

/**
  * @brief Ye olde main
  * @retval preferably none
  */
int main(void)
{
    uint32_t counter = 0;

    ringbuf_init(&rx_buf, (uint8_t*) rx_buffer, sizeof(rx_buffer));
    hw_init(&rx_buf);

    dbg_printf("\n\nWelcome to the AAduino Zero Hardware Test\n");

    if (!spiflash_probe()) {
        dbg_printf("No SPI flash found\n");
        blinken_halt(2);
    } else {
        dbg_printf("Found SPI flash %s\n", spiflash_get_desc());
        uint8_t buffer[256];

        for (uint32_t i = 0; i < sizeof(buffer); i++) {
            buffer[i] = i;
        }
        (void) spiflash_write(0, sizeof(buffer), (uint8_t*) buffer);

        for (uint32_t i = 0; i < sizeof(buffer); i++) {
            buffer[i] = ~i;
        }
        (void) spiflash_write(256, sizeof(buffer), (uint8_t*) buffer);

        memset(buffer, 0xcd, sizeof(buffer));
        (void) spiflash_read(0, sizeof(buffer), (uint8_t*) buffer);
        dbg_printf("Read page 0\n  ");
        for (uint32_t i = 0; i < sizeof(buffer); i++) {
            if (i && !(i % 32)) {
                dbg_printf("\n  ");
            }
            dbg_printf(" %02x", buffer[i]);
        }
        dbg_printf("\n");

        (void) spiflash_read(256, sizeof(buffer), (uint8_t*) buffer);
        dbg_printf("Read page 1\n  ");
        for (uint32_t i = 0; i < sizeof(buffer); i++) {
            if (i && !(i % 32)) {
                dbg_printf("\n  ");
            }
            dbg_printf(" %02x", buffer[i]);
        }
        dbg_printf("\n");
    }

    if (tmp102_init()) {
        uint32_t t = 1000 * tmp102_readTempC();
        dbg_printf("Temperature is %d.%d°C\n", t/1000, (t%1000)/100);
    }

//    rfm69_setResetPin(RFM_RESET_PORT, RFM_RESET_PIN);
//    rfm69_reset();
    if (!rfm69_init(SPI1_RFM_CS_PORT, SPI1_RFM_CS_PIN, false)) {
        dbg_printf("No RFM69CW found\n");
        blinken_halt(2);
    } else {
        dbg_printf("RFM69CW found\n");

        rfm69_sleep(); // init RF module and put it to sleep
        rfm69_setPowerDBm(13); // // set output power, +13 dBm
        rfm69_setCSMA(true); // enable CSMA/CA algorithm
        rfm69_setAutoReadRSSI(true);
        (void) rfm69_setAESEncryption((void*) "sampleEncryptKey", 16);
    }

    while(1) {
#if 0 // RX
        uint8_t packet[64];
        uint8_t len = rfm69_receive((char*) packet, sizeof(packet));
        if (len) {
            hw_set_led(true);
            dbg_printf("%02u : [ ", len);
            for (int i = 0; i < len; ++i) {
                dbg_printf("%02x ", packet[i]);
            }
            dbg_printf("] (%d)\n", rfm69_getRSSI());
            hw_set_led(false);
        }
#else // TX
        uint8_t testdata[] = {1, 42, 0x00, 'H', 'e', 'l', 'l', 'o', ' ', 'W', 'o', 'r', 'l', 'd', '!'};
        uint16_t vcc = vcc_measure();
        uint32_t t = 1000 * tmp102_readTempC();
        dbg_printf("Sending packet %d, temperature is %d.%d°C, vcc is %d.%02dV\n", counter++, t/1000, (t%1000)/100, vcc/1000, (vcc%1000)/10);
        hw_set_led(true);
        rfm69_send(testdata, sizeof(testdata));
        hw_set_led(false);
        rfm69_sleep();
#endif
        uint16_t b;
        while (ringbuf_get(&rx_buf, &b)) {
            dbg_printf("%c", b & 0xff);
        }

        delay_ms(10000);
    }
    return 0;
}

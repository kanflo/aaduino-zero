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
#include "cli.h"
#include "dbg_printf.h"
#include "hw.h"
#include "tick.h"
#include "tmp102.h"
#include "rfm69.h"
#include "rfm69_link.h"
#include "spiflash.h"

#define GATEWAY_ID                 (1) // Id of gateway
#define NODE_ID                   (42) // Id of node running this program
#define TEMPERATURE_PACKET_SIZE   (10) // Temperature packet size
#define MAX_PACKET_SIZE           (64) // Max size of RF packet

#define TEMPERATURE_FRAME_TYPE     (0) // Frame type for temperature packet


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
    hw_init(0);
    dbg_printf("\n\nWelcome to the AAduino Zero Transmitter Example\n");
    (void) tmp102_init();

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
        rfm69link_setNodeId(NODE_ID);
    }

    dbg_printf("Transmitting temperature and battery voltage every 2 seconds.\n");

#if 0
    while(1) {
        rfm69_link_frame_t frame;
        uint8_t src, dst, length;
        if (rfm69link_receiveFrame(&src, &dst, &frame, &length)) {
            hw_set_led(true);
            dbg_printf("%02d -> %02d (%d)\n", src, dst, length);
            hw_set_led(false);
        }
    }
#endif
    while(1) {
        // Send a frame formatted as
        //  <to:8> <from:8> <counter:8> <type:8> <temperature:32> <vcc:16>
        uint16_t vcc = vcc_measure();
        int32_t t = 1000 * tmp102_readTempC();

        rfm69_link_frame_t frame;
        uint8_t i = 0;
        frame.payload[i++] = TEMPERATURE_FRAME_TYPE;
        frame.payload[i++] = (t >> 24) & 0xff;
        frame.payload[i++] = (t >> 16) & 0xff;
        frame.payload[i++] = (t >>  8) & 0xff;
        frame.payload[i++] = (t      ) & 0xff;
        frame.payload[i++] = (vcc >> 8) & 0xff;
        frame.payload[i++] = (vcc    ) & 0xff;

        int32_t frac = (t%1000)/100;
        if (frac < 0) {
            frac = -frac;
        }
        dbg_printf("[%d] Temperature is %d.%d°C, vcc is %d.%02dV\n", counter, t/1000, frac, vcc/1000, (vcc%1000)/10);
        hw_set_led(true);
        if (rfm69link_sendFrame(GATEWAY_ID, &frame, i)) {
            dbg_printf(" Packet sent, ack RSSI %d\n", frame.rssi);
        } else {
            dbg_printf(" No response from gateway\n");
        }
        hw_set_led(false);
        // TODO: Use low power sleep ;)
        delay_ms(2000);
        counter++;
    }
    return 0;
}

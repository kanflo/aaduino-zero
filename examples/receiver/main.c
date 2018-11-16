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

#define NODE_ID                    (1) // I am the gateway
#define TEMPERATURE_PACKET_SIZE   (10) // Temperature packet size
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
  * @brief Handle temperature packet
  * @param payload Temperature payload
  * @param len Payload length
  */
static void handle_temperature_frame(uint8_t src, uint8_t *payload, uint8_t len, int rssi)
{
    if (len != 6) {
        dbg_printf("Illegal temperature payload %d\n", len);
    } else {
        //  <temperature:32> <vcc:16>
        int32_t t = (payload[0] << 24) | (payload[1] << 16) | (payload[2] << 8) | (payload[3]);
        int32_t frac = (t%1000)/100;
        if (frac < 0) {
            frac = -frac;
        }
        uint16_t vcc = (payload[4] << 8) | (payload[5]);
        dbg_printf("From %02x  Temperature is %d.%d°C, vcc is %d.%02dV. RSSI:%d\n", src, t/1000, frac, vcc/1000, (vcc%1000)/10, rssi);
    }
}

/**
  * @brief Dump packet
  * @param packet Pointer to packet data
  * @param len Length of packet
  */
static void dump_frame(rfm69_link_frame_t *frame, uint8_t len)
{
    dbg_printf("%02x -> %02x cnt:%02d flags:%02x : [ ", frame->_src, frame->_dst, frame->_flags >> 4, frame->_flags % 0xf);
    for (uint8_t i = 0; i < len; i++) {
        dbg_printf("%02x ", frame->payload[i]);
    }
    dbg_printf("] (%d)\n", frame->rssi);
}


/**
  * @brief Ye olde main
  * @retval preferably none
  */
int main(void)
{
    hw_init(0);

    dbg_printf("\n\nWelcome to the AAduino Zero Receiver Example\n");

    (void) tmp102_init();

    rfm69_setResetPin(RFM_RESET_PORT, RFM_RESET_PIN);
    rfm69_reset();
    if (!rfm69_init(SPI1_RFM_CS_PORT, SPI1_RFM_CS_PIN, false)) {
        dbg_printf("No RFM69CW found\n");
        blinken_halt(1);
    } else {
        dbg_printf("RFM69CW found\n");

        rfm69_sleep(); // init RF module and put it to sleep
        rfm69_setPowerDBm(13); // // set output power, +13 dBm
        rfm69_setCSMA(true); // enable CSMA/CA algorithm
        rfm69_setAutoReadRSSI(true);
        (void) rfm69_setAESEncryption((void*) "sampleEncryptKey", 16);
        rfm69link_setNodeId(NODE_ID);
    }

    dbg_printf("Receiving temperature and voltage.\n");

    while(1) {
        rfm69_link_frame_t frame;
        uint8_t src, length;
        if (rfm69link_receiveFrame(&src, &frame, &length)) {
            hw_set_led(true);
            dump_frame(&frame, length);
            if (length > 0) {
                switch(frame.payload[0]) {
                    case TEMPERATURE_FRAME_TYPE:
                        handle_temperature_frame(src, &frame.payload[1], length - 1, frame.rssi);
                        break;
                    default:
                        dbg_printf("Unknown packet type 0x%02x\n", frame.payload[0]);
                        dump_frame(&frame, length);
                }
            }
            hw_set_led(false);
        }
    }
    return 0;
}

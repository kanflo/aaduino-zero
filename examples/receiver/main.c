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
#include "rflink.h"
#include "spiflash.h"
#include "rtcdrv.h"
#include "bootcom.h"
#include "rfprotocol.h"

//#define CONFIG_RX_DEBUG

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
        dbg_printf("Illegal temperature payload length %d\n", len);
    } else {
        //  <temperature:32> <vcc:16>
        int32_t t = (payload[0] << 24) | (payload[1] << 16) | (payload[2] << 8) | (payload[3]);
        int32_t frac = (t%1000)/100;
        if (frac < 0) {
            frac = -frac;
        }
        uint16_t vcc = (payload[4] << 8) | (payload[5]);
        dbg_printf("From node %d: ", src);
        dbg_printf("temperature:%d.%d°C, ", t/1000, frac);
        dbg_printf("vcc:%d.%02dV, ", vcc/1000, (vcc%1000)/10);
        dbg_printf("rssi:%d\n", rssi);
    }
}


/**
  * @brief Handle fault packet
  * @param payload Fault payload
  * @param len Payload length
  */
static void handle_fault_frame(uint8_t src, uint8_t *payload, uint8_t len, int rssi)
{
    if (len != 3*4) {
        dbg_printf("Illegal fault payload length %d\n", len);
    } else {
        //  <pc:32> <lr:32> <uptime:32>
        int32_t pc = (payload[0] << 24) | (payload[1] << 16) | (payload[2] << 8) | (payload[3]);
        int32_t lr = (payload[4] << 24) | (payload[5] << 16) | (payload[6] << 8) | (payload[7]);
        int32_t uptime = (payload[8] << 24) | (payload[9] << 16) | (payload[10] << 8) | (payload[11]);
        dbg_printf("Node %d crashed at 0x%08x after %d seconds, ", src, pc, uptime);
        dbg_printf("lr:0x%08x rssi:%d\n", lr, rssi);
    }
}

/**
  * @brief Dump packet
  * @param packet Pointer to packet data
  * @param len Length of packet
  */
static void dump_frame(rflink_frame_t *frame, uint8_t len)
{
    dbg_printf("%02x -> %02x ", frame->_src, frame->_dst);
    dbg_printf("cnt:%02d flags:%x : [ ", FRAME_COUNTER(frame->_cntr_flags), FRAME_FLAGS(frame->_cntr_flags));
    for (uint8_t i = 0; i < len; i++) {
        dbg_printf("%02x ", frame->payload[i]);
    }
    dbg_printf("] (%d)\n", frame->rssi);
}


/**
 * @brief      Check for crash dump
 */
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
            dbg_printf("uptime  %ds\n", bootcom_get(9));
            bootcom_clear();
        }
    }
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
    rtcdrv_init();
    rtcdrv_set_wakeup(1);
    rfm69_setResetPin(RFM_RESET_PORT, RFM_RESET_PIN);
    rfm69_reset();
    if (!rfm69_init(SPI1_RFM_CS_PORT, SPI1_RFM_CS_PIN, false)) {
        dbg_printf("No RFM69CW found\n");
        blinken_halt(1);
    } else {
        dbg_printf("RFM69CW found, my node id is %d\n", CONFIG_NODEID);
        rfm69_sleep(); // init RF module and put it to sleep
        rfm69_setPowerDBm(13); // // set output power, +13 dBm
        rfm69_setCSMA(true); // enable CSMA/CA algorithm
        rfm69_setAutoReadRSSI(true);
        if (strlen(CONFIG_AESKEY) != 16) {
            dbg_printf("Error: AES key must be 16 bytes!\n");
        }
        (void) rfm69_setAESEncryption((void*) CONFIG_AESKEY, 16);
        rflink_setNodeId(CONFIG_NODEID);
    }

    check_crash();

    while(1) {
        rflink_frame_t frame;
        uint8_t src, length;
        if (rflink_receiveFrame(&src, &frame, &length)) {
            hw_set_led(true);
#ifdef CONFIG_RX_DEBUG
            dump_frame(&frame, length);
#endif // CONFIG_RX_DEBUG
            if (length > 0) {
                /** Chop off frame type from payload */
                uint8_t *payload = &frame.payload[1];
                uint8_t payload_len = length-1;
                switch(frame.payload[0]) {
                    case rf_temperature:
                        handle_temperature_frame(src, payload, payload_len, frame.rssi);
                        break;
                    case rf_powerup:
                        dbg_printf("Node %d powered up\n", src);
                        break;
                    case rf_hard_fault:
                        handle_fault_frame(src, payload, payload_len, frame.rssi);
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

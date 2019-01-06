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
#include "rtcdrv.h"
#include "lowpower.h"
#include "hw.h"
#include "tick.h"
#include "bootcom.h"
#include "tmp102.h"
#include "rfm69.h"
#include "rflink.h"
#include "spiflash.h"
#include "rfprotocol.h"

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
 * @brief      Send a powerup message to the gateway.
 */
static void send_powerup_frame(void)
{
        rflink_frame_t frame;
        uint8_t len = 0;
        frame.payload[len++] = rf_powerup;

        dbg_printf("Sending powerup to gateway\n");
        /** The powerup message has only a single frame type */
        /** @todo: Handle retransmission if this frame failed to deliver */
        /** @todo: Handle LAT */
        if (rflink_sendFrame(CONFIG_GATEWAYID, &frame, len) >= txstatus_ok) {
            dbg_printf(" Ack RSSI %d\n", frame.rssi);
        } else {
            dbg_printf(" No response from gateway\n");
        }

}

/**
 * @brief      Check for crash dump, send to gateway if found
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


            rflink_frame_t frame;
            uint8_t len = 0;
            frame.payload[len++] = rf_hard_fault;
            uint32_t pc = bootcom_get(7);
            uint32_t lr = bootcom_get(6);
            uint32_t uptime = bootcom_get(9);
            /** The crash report consists of a 1 byte frame type and the  pc and
             *  lr registers. 9 bytes in total.
             */
            frame.payload[len++] = (pc >> 24) & 0xff;
            frame.payload[len++] = (pc >> 16) & 0xff;
            frame.payload[len++] = (pc >>  8) & 0xff;
            frame.payload[len++] = (pc      ) & 0xff;
            frame.payload[len++] = (lr >> 24) & 0xff;
            frame.payload[len++] = (lr >> 16) & 0xff;
            frame.payload[len++] = (lr >>  8) & 0xff;
            frame.payload[len++] = (lr      ) & 0xff;
            frame.payload[len++] = (uptime >> 24) & 0xff;
            frame.payload[len++] = (uptime >> 16) & 0xff;
            frame.payload[len++] = (uptime >>  8) & 0xff;
            frame.payload[len++] = (uptime      ) & 0xff;
            dbg_printf("Sending crash report to gateway\n");
            /** @todo: Handle retransmission if this frame failed to deliver */
            /** @todo: Handle LAT */
            if (rflink_sendFrame(CONFIG_GATEWAYID, &frame, len) >= txstatus_ok) {
                dbg_printf(" Ack RSSI %d\n", frame.rssi);
            } else {
                dbg_printf(" No response from gateway\n");
            }
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
    uint32_t counter = 0;
    uint32_t period = 30;
    hw_init(0);

    dbg_printf("\n\nWelcome to the AAduino Zero Transmitter Example\n");

    (void) tmp102_init();
    rtcdrv_init();
    rtcdrv_set_wakeup(1);
    rfm69_setResetPin(RFM_RESET_PORT, RFM_RESET_PIN);
    rfm69_reset();
    if (!rfm69_init(SPI1_RFM_CS_PORT, SPI1_RFM_CS_PIN, false)) {
        dbg_printf("No RFM69CW found\n");
        blinken_halt(2);
    } else {
        dbg_printf("RFM69CW found, my node id is %d, talking to %d\n", CONFIG_NODEID, CONFIG_GATEWAYID);
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

    dbg_printf("Transmitting temperature and battery voltage every %d seconds.\n", period);

    send_powerup_frame();
    check_crash();

    while(1) {
        // Send a frame formatted as
        //  <to:8> <from:8> <counter:8> <type:8> <temperature:32> <vcc:16>
        uint16_t vcc = vcc_measure();
        int32_t t = 1000 * tmp102_readTempC();

        rflink_frame_t frame;
        uint8_t len = 0;
        frame.payload[len++] = rf_temperature;
        frame.payload[len++] = (t >> 24) & 0xff;
        frame.payload[len++] = (t >> 16) & 0xff;
        frame.payload[len++] = (t >>  8) & 0xff;
        frame.payload[len++] = (t      ) & 0xff;
        frame.payload[len++] = (vcc >> 8) & 0xff;
        frame.payload[len++] = (vcc    ) & 0xff;

        int32_t frac = (t%1000)/100;
        if (frac < 0) {
            frac = -frac;
        }
        hw_set_led(true);
        dbg_printf("[%d] Temperature is %d.%d°C, vcc is %d.%02dV\n", counter, t/1000, frac, vcc/1000, (vcc%1000)/10);
        hw_set_led(false);
        /** @todo: Handle LAT */
        if (rflink_sendFrame(CONFIG_GATEWAYID, &frame, len) >= txstatus_ok) {
            dbg_printf(" Ack RSSI %d\n", frame.rssi);
        } else {
            dbg_printf(" No response from gateway\n");
        }

        dbg_printf("Sleeping\n");
        lp_sleep(period);
        dbg_printf("*yawn*\n");
        counter++;
#if CONFIG_CRASH==1
        /** Try the crash-reports-over-the-air feature */
        uint32_t *a = 0;
        *a = 0;
#endif // CONFIG_CRASH
    }
    return 0;
}

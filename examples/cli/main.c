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
#include "spiflash.h"


static void blinken_halt(uint32_t blink_count);

#define RX_BUF_SIZE  (16)
static ringbuf_t rx_buf;
static uint8_t rx_buffer[2*RX_BUF_SIZE];

#define MAX_LINE_LENGTH  (80)

static void help_handler(uint32_t argc, char *argv[]);
static void halt_handler(uint32_t argc, char *argv[]);

cli_command_t commands[] = {
    {
        .cmd = "help",
        .handler = help_handler,
        .min_arg = 0, .max_arg = 0,
        .help = "Print help",
        .usage = "no usage"
    },
    {
        .cmd = "halt",
        .handler = halt_handler,
        .min_arg = 0, .max_arg = 0,
        .help = "Halt the system",
        .usage = "no usage"
    }
    // TODO: More commands to be added
};


static void help_handler(uint32_t argc, char *argv[])
{
    (void) argc;
    (void) argv;
    for (uint32_t i = 0; i < sizeof(commands) / sizeof(cli_command_t); i++) {
        dbg_printf("%s    %s\n", commands[i].cmd, commands[i].help);        
    }
}

static void halt_handler(uint32_t argc, char *argv[])
{
    (void) argc;
    (void) argv;
    dbg_printf("Halted\n");
    blinken_halt(2);
}

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
    uint32_t i = 0;
    char line[MAX_LINE_LENGTH];
    line[0] = 0;

    ringbuf_init(&rx_buf, (uint8_t*) rx_buffer, sizeof(rx_buffer));
    hw_init(&rx_buf);

    dbg_printf("\n\nWelcome to the AAduino Zero CLI\n");

    if (!spiflash_probe()) {
        dbg_printf("No SPI flash found\n");
    } else {
        dbg_printf("Found SPI flash %s\n", spiflash_get_desc());
    }

    if (tmp102_init()) {
        uint32_t t = 1000 * tmp102_readTempC();
        dbg_printf("Temperature is %d.%d°C\n", t/1000, (t%1000)/100);
    }

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

    dbg_printf("Try 'help <return>' for, well, help.\n");
    dbg_printf("%% ");
    while(1) {
        uint16_t b;
        while (ringbuf_get(&rx_buf, &b)) {
            if (b == '\r') {
                // Don't care
            } else if (b == '\n') {
                dbg_printf("\n");
                if (i > 0) {
                    line[i] = 0;
                    cli_run(commands, sizeof(commands) / sizeof(cli_command_t), line);
                    i = 0;
                    line[0] = 0;
                }
                dbg_printf("%% ");
            } else if (i < MAX_LINE_LENGTH-2) {
                dbg_printf("%c", b & 0xff);
                line[i++] = b;
            }
        }
    }
    return 0;
}

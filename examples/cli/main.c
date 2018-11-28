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
#include "cli.h"
#include "dbg_printf.h"
#include "hw.h"
#include "tick.h"
#include "tmp102.h"
#include "rfm69.h"
#include "spiflash.h"
#include "past.h"

static past_t g_past;

/** Linker symbols */
extern long past_start, past_block_size;


static void blinken_halt(uint32_t blink_count);
static void dump_mem(uint32_t address, uint32_t length);

#define RX_BUF_SIZE  (16)
static ringbuf_t rx_buf;
static uint8_t rx_buffer[2*RX_BUF_SIZE];

#define MAX_LINE_LENGTH  (80)

static void help_handler(uint32_t argc, char *argv[]);
static void halt_handler(uint32_t argc, char *argv[]);
static void past_format_handler(uint32_t argc, char *argv[]);
static void past_read_handler(uint32_t argc, char *argv[]);
static void past_write_handler(uint32_t argc, char *argv[]);
static void past_erase_handler(uint32_t argc, char *argv[]);
static void past_dump_handler(uint32_t argc, char *argv[]);

cli_command_t commands[] = {
    {
        .cmd = "help",
        .handler = help_handler,
        .min_arg = 0, .max_arg = 0,
        .help = "Print help",
        .usage = ""
    },
    {
        .cmd = "halt",
        .handler = halt_handler,
        .min_arg = 0, .max_arg = 64,
        .help = "Halt the system",
        .usage = "<arg> ... <arg>"
    },
    {
        .cmd = "pastformat",
        .handler = past_format_handler,
        .min_arg = 0, .max_arg = 0,
        .help = "Format past",
        .usage = ""
    },
    {
        .cmd = "pastread",
        .handler = past_read_handler,
        .min_arg = 1, .max_arg = 1,
        .help = "Read unit from past",
        .usage = "<unit>"
    },
    {
        .cmd = "pastwrite",
        .handler = past_write_handler,
        .min_arg = 2, .max_arg = 2,
        .help = "Write unit to past",
        .usage = "<unit> <data>"
    },
    {
        .cmd = "pasterase",
        .handler = past_erase_handler,
        .min_arg = 1, .max_arg = 1,
        .help = "Erase unit from past",
        .usage = "<unit>"
    },
    {
        .cmd = "pastdump",
        .handler = past_dump_handler,
        .min_arg = 0, .max_arg = 0,
        .help = "Dump past",
        .usage = ""
    },
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
    for (uint32_t i = 0; i < argc; i++) {
        dbg_printf("%d '%s'\n", i, argv[i]);
    }
    dbg_printf("Halted\n");
    blinken_halt(2);
}

static void past_format_handler(uint32_t argc, char *argv[])
{
    (void) argv;
    (void) argc;
    if (!past_format(&g_past)) {
        dbg_printf("Past formatting failed\n");
    }
    if (past_init(&g_past)) {
        dbg_printf("Past init success\n");
    } else {
        dbg_printf("Past init failed\n");
    }
}

static void past_read_handler(uint32_t argc, char *argv[])
{
    (void) argc;
    uint8_t *data;
    uint32_t length;
    uint32_t unit_id = atoi(argv[1]);
    if (past_read_unit(&g_past, unit_id, (const void**)&data, &length)) {
        dbg_printf("'%s' (%d bytes)\n", data, length);
    } else {
        dbg_printf("Unit %d not found\n", unit_id);
    }
}

static void past_write_handler(uint32_t argc, char *argv[])
{
    (void) argc;
    uint32_t unit_id = atoi(argv[1]);
    if (past_write_unit(&g_past, unit_id, (void*)argv[2], strlen(argv[2]) + 1)) {
        dbg_printf("Wrote unit %d (%d bytes)\n", unit_id, strlen(argv[2]) + 1);
    } else {
        dbg_printf("Failed to write unit %d\n", unit_id);
    }
}

static void past_erase_handler(uint32_t argc, char *argv[])
{
    (void) argc;
    uint32_t unit_id = atoi(argv[1]);
    if (past_erase_unit(&g_past, unit_id)) {
        dbg_printf("Erased unit %d\n", unit_id);
    } else {
        dbg_printf("Failed to erase unit %d\n", unit_id);
    }
}

static void past_dump_handler(uint32_t argc, char *argv[])
{
    (void) argc;
    (void) argv;
    dbg_printf("Past block 0:\n");
    dump_mem(g_past.blocks[0], 1024);
    dbg_printf("\nPast block 1:\n");
    dump_mem(g_past.blocks[1], 1024);

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

#define LINE_WIDTH  (16)

static void dump_mem(uint32_t address, uint32_t length)
{
    uint8_t *p = (uint8_t*) address;
    dbg_printf("%08x...%08x:", address, address + length - 1);
    for (uint32_t i = 0; i < length; i++) {
        if (i % LINE_WIDTH == 0) {
            dbg_printf("\n");
            dbg_printf("  %08x : ", address + i);
        }
        dbg_printf(" %02x", p[i]);
    }
    dbg_printf("\n");
}

#if 0
static void flash_test(void)
{
    dbg_printf("\n\n *** flash test ***\n");

    dump_mem(0x08007000, 16);
    flash_unlock();
    for (int i = 0; i < 1024; i+=4) {
        flash_program_word(0x08007000 + i, 0xffffffff);
    }
    flash_lock();

    dump_mem(0x08007000, 16);

    dbg_printf("\n>>> Erasing 0x08007000\n");
    flash_unlock();
    flash_erase_page(0x08007000);
    flash_lock();
    dump_mem(0x08007000, 16);

    dbg_printf("\n>>> Programming 0x08007000\n");
    flash_unlock();
    flash_program_word(0x08007000, 0x000000ff);
    flash_lock();
    dump_mem(0x08007000, 16);

    dbg_printf("\n>>> Programming 0x08007000\n");
    flash_unlock();
    flash_program_word(0x08007000, 0xff000000);
    flash_lock();
    dump_mem(0x08007000, 16);

    dbg_printf("\n>>> Programming 0x08007000\n");
    flash_unlock();
    flash_program_word(0x08007000, 0x00000000);
    flash_lock();
    dump_mem(0x08007000, 16);

    dbg_printf("\n---\ndone\n");
    while(1) ;
}
#endif

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

    g_past.blocks[0] = (uint32_t) &past_start;
    g_past.blocks[1] = (uint32_t) &past_start + (uint32_t) &past_block_size;
    g_past._block_size = (uint32_t) &past_block_size;

    if (!past_init(&g_past)) {
        dbg_printf("Error: past init failed!\n");
        dbg_printf("Past block 0:\n");
        dump_mem(g_past.blocks[0], 64);
        dbg_printf("Past block 1:\n");
        dump_mem(g_past.blocks[1], 64);
        blinken_halt(3);
    }

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
